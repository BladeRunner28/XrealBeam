package com.xman.xrealbeam;

import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.util.Log;

import java.util.Locale;
import java.util.zip.CRC32;

/**
 * XREAL Air IMU reader over raw USB HID (no vendor SDK).
 *
 * Protocol facts ported from thejackimonster/xreal-imu (MIT), via the vendored copy in
 * DannyDesert/XReal-Ultrawide: VID 0x3318 / PID 0x0424 (Air), IMU on interface 3 (MCU is 4).
 *
 * Outbound command frame (little-endian, 64-byte max report):
 *   [0] 0xAA | [1..4] CRC32 LE | [5..6] length LE | [7] msgid | [8..] data
 *   length = 3 + data.length ; CRC32 is computed over the length field onwards, for `length`
 *   bytes (i.e. length + msgid + data) ; total bytes written = 8 + data.length.
 *   The CRC is the standard reflected CRC-32 (init 0xFFFFFFFF, final xor) => java.util.zip.CRC32.
 *
 * The IMU stream is NOT free-running: device_imu_open() must stop, identify, drain calibration and
 * then start the stream before any packet arrives. That sequence is reproduced in handshake().
 *
 * Inbound 64-byte packets: signature 0x01 0x02 = sensor frame (0xAA 0x53 = INIT frame).
 * Offsets: temp i16 @2, timestamp u64 @4, gyro mult i16 @12 / div i32 @14,
 * gyro xyz 24-bit @18/21/24, accel mult @27 / div @29, accel xyz @33/36/39,
 * mag mult(swapped) @42 / div(swapped) @44, mag xyz @48/50/52.
 */
public class XrealImu {

    public static final int VID = 0x3318;
    public static final int PID_AIR = 0x0424;
    public static final int IMU_IFACE = 3;
    public static final int MCU_IFACE = 4;

    private static final String TAG = "XrealProbe";
    private static final int MAX_PACKET = 64;
    private static final int READ_TIMEOUT_MS = 300;
    private static final int WRITE_TIMEOUT_MS = 500;

    private static final int MSG_CAL_LEN = 0x14;
    private static final int MSG_CAL_NEXT = 0x15;
    private static final int MSG_START = 0x19;
    private static final int MSG_STATIC_ID = 0x1A;

    // ---- connection ----
    private UsbDeviceConnection conn;
    private UsbInterface iface;
    private UsbEndpoint epIn;
    private UsbEndpoint epOut;
    private volatile boolean running;
    private Thread reader;

    // ---- reported state (read by the UI thread) ----
    public volatile String state = "idle";
    public volatile String handshakeLog = "";
    public volatile String lastFrameInfo = "";
    public volatile String firstFramesHex = "";
    public volatile long frames, initFrames, badSig, readErrors, shortReads;
    public volatile double hz, avgHz;
    public volatile int staticId = -1, calLen = -1, calDrained, writeBytes = -1;
    private long streamStartWallMs;
    public volatile String writePath = "-";

    // ---- AHRS / drift ----
    private final MadgwickAhrs ahrs = new MadgwickAhrs(0.1f);
    public volatile float quatW = 1f, quatX, quatY, quatZ;   // for the render path
    /** Bias-corrected angular rate in the BODY frame (deg/s), before any mount/AHRS work.
     *  This is the clean input for deriving the mount from a guided gesture. */
    public volatile double gyroBodyX, gyroBodyY, gyroBodyZ;
    /** Latest accelerometer sample in g (body frame) — the gravity reference for mount calibration. */
    public volatile double accelBodyX, accelBodyY, accelBodyZ;
    // ---- short-window statistics for the guided mount calibration (~0.5 s window, published at
    // 2 Hz). The render/AHRS path does not read these; they exist so MountCal can ask "is the
    // wearer still, where is gravity pointing, and how much have they rotated this pose?" without
    // sampling the ~490 Hz stream from the UI thread.
    public volatile double avgAX, avgAY, avgAZ;   // window-mean accel, g
    public volatile double avgG;                  // window-mean |omega|, deg/s
    public volatile double spreadA;               // max|a| - min|a| inside the window, g
    public volatile double rotIntegralX, rotIntegralY, rotIntegralZ;  // bias-corrected gyro integral, deg
    public volatile long winSeq;                  // increments once per published window
    private static final double WIN_S = 0.5;
    private double winDt, wAX, wAY, wAZ, wG, winAMax = -1e9, winAMin = 1e9;
    public volatile double roll, pitch, yaw, driftDeg, driftDegPerMin, driftSeconds;
    public volatile double gyroBiasX, gyroBiasY, gyroBiasZ;
    public volatile boolean recentered;
    // ---- ZUPT / stillness ----
    public volatile boolean stillness;
    public volatile double stillSeconds, yawSuppressedDeg, rawDriftDeg;
    // ---- factory calibration blob captured from the device (0x15 segments) ----
    public volatile byte[] calData;
    private float refQ0, refQ1, refQ2, refQ3;
    private long refWallNs, streamStartNs;
    private boolean haveTs;
    private long prevTs;

    /** Zero the pose-calibration rotation integral (called when a calibration pose starts). */
    public void resetRotationIntegral() {
        rotIntegralX = 0; rotIntegralY = 0; rotIntegralZ = 0;
    }

    /** Re-zero the current attitude: the drift clock restarts and subsequent yaw is relative. */
    public void recenter() {
        refQ0 = ahrs.q0(); refQ1 = ahrs.q1(); refQ2 = ahrs.q2(); refQ3 = ahrs.q3();
        refWallNs = System.nanoTime();
        driftDeg = 0; driftDegPerMin = 0; driftSeconds = 0;
        recentered = true;
    }

    private long lastTs;
    private double lastGyroX, lastGyroY, lastGyroZ, lastAccX, lastAccY, lastAccZ, lastTemp;
    private int dumpedFrames;

    /** True if this USB device is the XREAL Air we know how to read. */
    public static boolean isXrealAir(UsbDevice d) {
        return d != null && d.getVendorId() == VID && d.getProductId() == PID_AIR;
    }

    /** Full interface/endpoint dump — makes the probe report self-documenting. */
    public static String describeDevice(UsbDevice d) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.US, "device: VID 0x%04X PID 0x%04X  name=%s%n",
                d.getVendorId(), d.getProductId(), d.getDeviceName()));
        sb.append(String.format(Locale.US, "  manufacturer=%s product=%s%n",
                String.valueOf(d.getManufacturerName()), String.valueOf(d.getProductName())));
        for (int i = 0; i < d.getInterfaceCount(); i++) {
            UsbInterface f = d.getInterface(i);
            sb.append(String.format(Locale.US, "  iface #%d id=%d class=0x%02X sub=0x%02X proto=0x%02X endpoints=%d%n",
                    i, f.getId(), f.getInterfaceClass(), f.getInterfaceSubclass(), f.getInterfaceProtocol(),
                    f.getEndpointCount()));
            for (int e = 0; e < f.getEndpointCount(); e++) {
                UsbEndpoint ep = f.getEndpoint(e);
                sb.append(String.format(Locale.US, "      ep addr=0x%02X dir=%s type=%d maxPacket=%d interval=%d%n",
                        ep.getAddress(), ep.getDirection() == UsbConstants.USB_DIR_IN ? "IN" : "OUT",
                        ep.getType(), ep.getMaxPacketSize(), ep.getInterval()));
            }
        }
        return sb.toString();
    }

    /** Attach to interface 3. Must be called after USB permission is granted. */
    public String connect(UsbDevice device) {
        StringBuilder log = new StringBuilder();
        log.append(describeDevice(device));

        iface = null;
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            if (device.getInterface(i).getId() == IMU_IFACE) {
                iface = device.getInterface(i);
                break;
            }
        }
        if (iface == null) {
            state = "FAILED: no interface " + IMU_IFACE;
            return log.append("no interface ").append(IMU_IFACE).append(" on this device\n").toString();
        }

        for (int e = 0; e < iface.getEndpointCount(); e++) {
            UsbEndpoint ep = iface.getEndpoint(e);
            if (ep.getDirection() == UsbConstants.USB_DIR_IN) {
                epIn = ep;
            } else {
                epOut = ep;
            }
        }
        log.append(String.format(Locale.US, "IMU iface id=%d class=0x%02X  IN ep=%s  OUT ep=%s%n",
                iface.getId(), iface.getInterfaceClass(),
                epIn == null ? "NONE" : String.format("0x%02X", epIn.getAddress()),
                epOut == null ? "NONE (writes will use HID SET_REPORT)" : String.format("0x%02X", epOut.getAddress())));

        handshakeLog = log.toString();
        return handshakeLog;
    }

    /** Used by the Activity, which owns the UsbManager. */
    public void attach(UsbDeviceConnection connection) {
        this.conn = connection;
    }

    /** Run the required handshake, then start the read loop. Returns a human-readable log. */
    public String start() {
        StringBuilder log = new StringBuilder(handshakeLog);
        if (conn == null || iface == null) {
            state = "FAILED: not connected";
            return log.append("not connected\n").toString();
        }

        boolean claimed = conn.claimInterface(iface, true); // force = detach kernel usbhid driver
        log.append("claimInterface(3, force=true) -> ").append(claimed).append('\n');
        if (!claimed) {
            state = "FAILED: HID interface not claimable";
            Log.w(TAG, "claim failed — Android/kernel refused HID interface 3");
            handshakeLog = log.toString();
            return handshakeLog;
        }

        byte[] buf = new byte[MAX_PACKET];
        // Clear the capture buffer HERE, before the drain. Resetting it later (as the first version
        // did) silently throws away the blob just read — that build logged "calDrained=38884" and
        // "blob captured: none" in the same breath.
        calData = null;

        // The device answers EVERY command with one 64-byte report whose msgid echoes that command.
        // Read after each send, or every reply lands one command late (first version of this probe
        // read them late: staticId came back 0 and calLen came back as the static id).
        writeBytes = write(MSG_START, new byte[]{0x00});
        log.append("cmd 0x19 signal=0x00 (stop stream) -> ").append(writeBytes)
           .append(" bytes via ").append(writePath).append('\n');
        int r = readRaw(buf, 600);
        log.append("   reply ").append(r).append(" bytes")
           .append(r >= 8 ? String.format(Locale.US, " msgid=0x%02X", buf[7] & 0xFF) : "").append('\n');

        writeBytes = write(MSG_STATIC_ID, new byte[0]);
        log.append("cmd 0x1A GET_STATIC_ID -> ").append(writeBytes).append(" bytes\n");
        r = readRaw(buf, 600);
        log.append("   reply ").append(r).append(" bytes");
        if (r >= 12) {
            int msgid = buf[7] & 0xFF;
            if (msgid == MSG_STATIC_ID) {
                staticId = le32(buf, 8);
                log.append(String.format(Locale.US, " msgid=0x%02X staticId=0x%08X", msgid, staticId));
            } else {
                log.append(String.format(Locale.US, " msgid=0x%02X (unexpected, ignoring payload)", msgid));
            }
        }
        log.append('\n');

        writeBytes = write(MSG_CAL_LEN, new byte[0]);
        log.append("cmd 0x14 GET_CAL_DATA_LENGTH -> ").append(writeBytes).append(" bytes\n");
        r = readRaw(buf, 600);
        log.append("   reply ").append(r).append(" bytes");
        if (r >= 12 && (buf[7] & 0xFF) == MSG_CAL_LEN) {
            calLen = le32(buf, 8);
            log.append(String.format(Locale.US, " msgid=0x%02X calLen=%d", buf[7] & 0xFF, calLen));
        } else if (r >= 8) {
            log.append(String.format(Locale.US, " msgid=0x%02X (unexpected, skipping drain)", buf[7] & 0xFF));
            calLen = 0;
        }
        log.append('\n');

        calDrained = 0;
        calSegments = 0;
        // The device reports a real calibration blob (measured: 38,884 bytes of JSON), so the cap has
        // to clear that; only absurd values are skipped, and the segment loop is bounded regardless.
        if (calLen > 0 && calLen <= 65536) {
            java.io.ByteArrayOutputStream cal = new java.io.ByteArrayOutputStream();
            while (calDrained < calLen) {
                if (write(MSG_CAL_NEXT, new byte[0]) < 0) break;
                int want = Math.min(56, calLen - calDrained);
                int got = readRaw(buf, 600);
                if (got <= 0) break;
                int payload = Math.max(0, Math.min(want, got - 8));
                if (payload > 0) cal.write(buf, 8, payload);
                calDrained += payload;
                if (++calSegments > 1200) break;
            }
            // Keep the raw blob: the reference driver and the macOS port both discard it, so what is
            // actually in here is unpublished — and it is the principled source for gyro bias and
            // axis misalignment, which is exactly the shape of our residual yaw drift.
            calData = cal.toByteArray();
        } else if (calLen > 65536) {
            log.append("calLen implausible — skipping calibration drain\n");
        }
        log.append("calibration drained ").append(calDrained).append('/').append(calLen).append(" bytes\n");

        // 0x19 signal 0x01 arms the stream. The reply to this one is often the INIT frame
        // (signature AA 53) rather than an echo, so do not require a matching msgid.
        writeBytes = write(MSG_START, new byte[]{0x01});
        log.append("cmd 0x19 signal=0x01 (START stream) -> ").append(writeBytes)
           .append(" bytes via ").append(writePath).append('\n');
        r = readRaw(buf, 400);
        log.append("   post-start report ").append(r).append(" bytes")
           .append(r >= 2 ? String.format(Locale.US, " sig=%02X %02X", buf[0] & 0xFF, buf[1] & 0xFF) : "")
           .append('\n');

        handshakeLog = log.toString();
        Log.i(TAG, handshakeLog);

        frames = initFrames = badSig = readErrors = shortReads = 0;
        dumpedFrames = 0;
        firstFramesHex = "";
        lastTs = 0;
        streamStartWallMs = System.currentTimeMillis();
        avgHz = 0;
        running = true;
        state = "streaming";
        ahrs.reset();
        haveTs = false;
        gyroBiasX = gyroBiasY = gyroBiasZ = 0;
        resetRotationIntegral();
        winDt = 0; wAX = wAY = wAZ = wG = 0; winAMax = -1e9; winAMin = 1e9;
        recentered = false;
        stillness = false;
        stillSeconds = 0;
        yawSuppressedDeg = 0;
        roll = pitch = yaw = driftDeg = driftDegPerMin = driftSeconds = 0;
        reader = new Thread(() -> readLoop(), "xreal-imu-reader");
        reader.setDaemon(true);
        reader.start();
        return handshakeLog;
    }

    private int calSegments;

    private void readLoop() {
        byte[] buf = new byte[MAX_PACKET];
        long windowStart = System.currentTimeMillis();
        long windowFrames = 0;
        while (running) {
            int r = readRaw(buf, READ_TIMEOUT_MS);
            if (r < 0) {
                readErrors++;
                // On detach the endpoint returns errors immediately and the loop would spin hot
                // (the first run logged ~1600 of these in 20 ms). Back off.
                if (readErrors <= 3 || readErrors % 100 == 0) {
                    Log.w(TAG, "read error count=" + readErrors);
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) { }
                if (readErrors > 50 && frames == 0) {
                    // No data at all after repeated errors: stream never started.
                    Log.w(TAG, "giving up: " + readErrors + " read errors, 0 frames");
                    running = false;
                }
                continue;
            }
            if (r == 0) continue;
            if (r != MAX_PACKET) {
                shortReads++;
                Log.w(TAG, "short report: " + r + " bytes");
                continue;
            }
            int sig0 = buf[0] & 0xFF, sig1 = buf[1] & 0xFF;
            if (sig0 == 0xAA && sig1 == 0x53) {
                initFrames++;
            } else if (sig0 == 0x01 && sig1 == 0x02) {
                frames++;
                windowFrames++;
                parseFrame(buf);
                if (dumpedFrames < 3) {
                    dumpedFrames++;
                    firstFramesHex += hex(buf) + "\n";
                }
            } else {
                badSig++;
                if (badSig <= 3) Log.w(TAG, "unexpected signature: " + hex(buf));
            }
            long now = System.currentTimeMillis();
            if (now - windowStart >= 1000) {
                hz = windowFrames * 1000.0 / (now - windowStart);
                windowFrames = 0;
                windowStart = now;
                // Rolling average over the whole streaming session — the 1-second figure goes bogus
                // on a partial window at detach, this one does not.
                double secs = (now - streamStartWallMs) / 1000.0;
                if (secs > 1) avgHz = frames / secs;
            }

            // Auto-recenter once the bias estimate has settled. Needed because DeX turns the phone
            // screen into a trackpad, so a human cannot reliably tap the button with glasses on.
            if (!recentered && frames > 6000) {
                recenter();
                Log.i(TAG, "auto-recenter at " + frames + " frames (drift clock started)");
            }
        }
    }

    private void parseFrame(byte[] b) {
        long ts = le64(b, 4);
        lastTs = ts;
        lastTemp = i16(b, 2) / 132.48 + 25.0;

        double gMult = i16(b, 12), gDiv = i32(b, 14);
        if (gDiv != 0) {
            lastGyroX = i24(b, 18) * gMult / gDiv;
            lastGyroY = i24(b, 21) * gMult / gDiv;
            lastGyroZ = i24(b, 24) * gMult / gDiv;
        }
        double aMult = i16(b, 27), aDiv = i32(b, 29);
        if (aDiv != 0) {
            lastAccX = i24(b, 33) * aMult / aDiv;
            lastAccY = i24(b, 36) * aMult / aDiv;
            lastAccZ = i24(b, 39) * aMult / aDiv;
        }

        // Sample interval from the DEVICE clock (nanoseconds), not wall time.
        double dt = 0;
        if (!haveTs) {
            haveTs = true;
            streamStartNs = ts;
        } else {
            long delta = ts - prevTs;
            if (delta > 0 && delta < 500_000_000L) dt = delta / 1e9;
        }
        prevTs = ts;

        // Slow gyro-bias tracker: only adapt while the glasses look stationary (gravity ≈ 1 g and
        // the residual is small). This is what FusionOffset does for the desktop driver; without it
        // yaw slew equals the raw bias, which is exactly the drift we are trying to measure.
        double aMag = Math.sqrt(lastAccX * lastAccX + lastAccY * lastAccY + lastAccZ * lastAccZ);
        double gx = lastGyroX - gyroBiasX;
        double gy = lastGyroY - gyroBiasY;
        double gz = lastGyroZ - gyroBiasZ;
        gyroBodyX = gx; gyroBodyY = gy; gyroBodyZ = gz;   // raw body rate for calibration
        double gMag = Math.sqrt(gx * gx + gy * gy + gz * gz);

        // Adapt the bias only when the glasses are genuinely still: gravity ≈ 1 g AND the corrected
        // gyro is near zero. Without the gyro gate a slow pan gets absorbed into the bias estimate.
        boolean stillNow = Math.abs(aMag - 1.0) < 0.05 && gMag < 2.0;
        if (stillNow) {
            gyroBiasX += (lastGyroX - gyroBiasX) * 0.0005;
            gyroBiasY += (lastGyroY - gyroBiasY) * 0.0005;
            gyroBiasZ += (lastGyroZ - gyroBiasZ) * 0.0005;
        }

        accelBodyX = lastAccX; accelBodyY = lastAccY; accelBodyZ = lastAccZ;

        // Short-window statistics for MountCal. Accumulated on the PRE-ZUPT bias-corrected rate:
        // the calibration wants the wearer's real rotation, not the drift governor's opinion of it.
        if (dt > 0) {
            winDt += dt;
            wAX += lastAccX * dt; wAY += lastAccY * dt; wAZ += lastAccZ * dt;
            wG += gMag * dt;
            if (aMag > winAMax) winAMax = aMag;
            if (aMag < winAMin) winAMin = aMag;
            rotIntegralX += gx * dt; rotIntegralY += gy * dt; rotIntegralZ += gz * dt;
            if (winDt >= WIN_S) {
                double inv = 1.0 / winDt;
                avgAX = wAX * inv; avgAY = wAY * inv; avgAZ = wAZ * inv;
                avgG = wG * inv;
                spreadA = winAMax - winAMin;
                winDt = 0; wAX = wAY = wAZ = wG = 0;
                winAMax = -1e9; winAMin = 1e9;
                winSeq++;
            }
        }

        // ZUPT (zero-velocity update) on the world-vertical axis.
        //
        // Why this form and not "slew yaw back to the reference": a head turned and then held still
        // is indistinguishable from accumulated drift, so pulling yaw to a fixed reference would
        // fight legitimate looks. Instead, while the glasses are still, the *rate* about the world
        // vertical must be zero — any residual there is pure bias error — so we remove exactly that
        // component and leave the current heading untouched. Legitimate turns are never affected
        // because a moving head is not "still".
        stillSeconds = stillNow ? stillSeconds + dt : 0;
        boolean zupt = stillSeconds > 1.0 && dt > 0;
        if (zupt) {
            // World up expressed in body coordinates: u = R(q)^T * (0,0,1).
            double qw = ahrs.q0(), qx = ahrs.q1(), qy = ahrs.q2(), qz = ahrs.q3();
            double ux = 2 * (qx * qz - qw * qy);
            double uy = 2 * (qy * qz + qw * qx);
            double uz = qw * qw - qx * qx - qy * qy + qz * qz;
            double dot = gx * ux + gy * uy + gz * uz;   // deg/s about the world vertical
            gx -= dot * ux;
            gy -= dot * uy;
            gz -= dot * uz;
            yawSuppressedDeg += dot * dt;               // signed: how much drift was prevented
        }
        stillness = zupt;

        if (dt > 0) {
            ahrs.update(gx, gy, gz, lastAccX, lastAccY, lastAccZ, dt);
            quatW = ahrs.q0(); quatX = ahrs.q1(); quatY = ahrs.q2(); quatZ = ahrs.q3();
            roll = ahrs.rollDeg();
            pitch = ahrs.pitchDeg();
            yaw = ahrs.yawDeg();
            if (recentered) {
                driftDeg = ahrs.relativeYawDeg(refQ0, refQ1, refQ2, refQ3);
                driftSeconds = (System.nanoTime() - refWallNs) / 1e9;
                driftDegPerMin = driftSeconds > 1 ? driftDeg / (driftSeconds / 60.0) : 0;
                // What yaw would have done WITHOUT the governor = observed change + what we removed.
                // This is the only honest way to score ZUPT's contribution on a given run.
                rawDriftDeg = driftDeg + yawSuppressedDeg;
            }
        }

        lastFrameInfo = String.format(Locale.US,
                "gyro  %7.2f %7.2f %7.2f deg/s   (mult=%.0f div=%.0f)%n"
                        + "accel %7.3f %7.3f %7.3f g       (mult=%.0f div=%.0f)%n"
                        + "temp  %6.1f C   ts=%d%n"
                        + "euler roll %7.2f  pitch %7.2f  yaw %7.2f deg%n"
                        + "drift %8.2f deg over %.1f s  =  %6.2f deg/min%n"
                        + "bias  %6.2f %6.2f %6.2f deg/s   q=%.4f %.4f %.4f %.4f",
                lastGyroX, lastGyroY, lastGyroZ, gMult, gDiv,
                lastAccX, lastAccY, lastAccZ, aMult, aDiv, lastTemp, ts,
                roll, pitch, yaw, driftDeg, driftSeconds, driftDegPerMin,
                gyroBiasX, gyroBiasY, gyroBiasZ,
                ahrs.q0(), ahrs.q1(), ahrs.q2(), ahrs.q3());
    }

    public void stop() {
        running = false;
        if (reader != null) {
            try { reader.join(1000); } catch (InterruptedException ignored) { }
        }
        if (conn != null && iface != null) {
            try { conn.releaseInterface(iface); } catch (Exception ignored) { }
        }
        state = "stopped";
    }

    // ---------------- low-level ----------------

    private int readRaw(byte[] buf, int timeoutMs) {
        if (epIn == null) return -1;
        return conn.bulkTransfer(epIn, buf, buf.length, timeoutMs);
    }

    /**
     * Send one framed command. Prefers the interface's OUT endpoint; if the HID interface has no
     * OUT endpoint (common for HID), falls back to a class-specific SET_REPORT control transfer.
     */
    private int write(int msgid, byte[] data) {
        int len = 3 + data.length;
        byte[] p = new byte[8 + data.length];
        p[0] = (byte) 0xAA;
        p[5] = (byte) (len & 0xFF);
        p[6] = (byte) ((len >> 8) & 0xFF);
        p[7] = (byte) msgid;
        System.arraycopy(data, 0, p, 8, data.length);
        CRC32 crc = new CRC32();
        crc.update(p, 5, len);
        long v = crc.getValue();
        p[1] = (byte) (v & 0xFF);
        p[2] = (byte) ((v >> 8) & 0xFF);
        p[3] = (byte) ((v >> 16) & 0xFF);
        p[4] = (byte) ((v >> 24) & 0xFF);

        if (epOut != null) {
            int n = conn.bulkTransfer(epOut, p, p.length, WRITE_TIMEOUT_MS);
            writePath = "OUT-endpoint";
            if (n >= 0) return n;
            Log.w(TAG, "OUT endpoint write failed (" + n + "), trying SET_REPORT");
        }
        int n = conn.controlTransfer(0x21, 0x09, 0x0200, iface.getId(), p, p.length, WRITE_TIMEOUT_MS);
        writePath = "SET_REPORT";
        return n;
    }

    private static String hex(byte[] b) {
        StringBuilder sb = new StringBuilder(b.length * 3);
        for (byte x : b) sb.append(String.format(Locale.US, "%02x ", x));
        return sb.toString().trim();
    }

    private static int le32(byte[] b, int o) {
        return (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8) | ((b[o + 2] & 0xFF) << 16) | ((b[o + 3] & 0xFF) << 24);
    }

    private static long le64(byte[] b, int o) {
        long v = 0;
        for (int i = 7; i >= 0; i--) v = (v << 8) | (b[o + i] & 0xFF);
        return v;
    }

    private static int i16(byte[] b, int o) {
        return (short) ((b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8));
    }

    private static int i32(byte[] b, int o) {
        return le32(b, o);
    }

    /** 24-bit little-endian signed, sign-extended. */
    private static int i24(byte[] b, int o) {
        int v = (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8) | ((b[o + 2] & 0xFF) << 16);
        if ((v & 0x800000) != 0) v |= 0xFF000000;
        return v;
    }
}
