package com.xman.xrealbeam;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.display.DisplayManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.util.TypedValue;
import android.view.Display;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.util.Locale;

/**
 * Raw-HID probe for the XREAL Air IMU on an Android host.
 *
 * Answers the one question that gates the whole "Beam-on-phone" project: can an Android app
 * claim HID interface 3 of VID 0x3318/PID 0x0424, run the xreal-imu handshake, and receive
 * 64-byte sensor frames — while the same cable drives DP video? Everything else (render, AHRS,
 * anchoring) is downstream of this.
 *
 * Tap the screen to re-run the handshake / restart the stream (no reinstall needed).
 * Full log: adb logcat -s XrealProbe
 */
public class MainActivity extends Activity {

    private static final String TAG = "XrealProbe";
    private static final String ACTION_USB_PERMISSION = "com.xman.xrealbeam.USB_PERMISSION";

    private UsbManager usbManager;
    private UsbDevice device;
    private UsbDeviceConnection connection;
    private XrealImu imu;

    private HeadPose pose;
    // Screen geometry + optics model: sizes/distances are in metres and the projection comes from
    // the glasses' field of view, so "150-inch at 15 feet" is literally reproducible.
    private final Geometry geom = new Geometry();
    // Mount calibration: derives the device->viewer mount from gravity in two still poses.
    private final MountCal mountCal = new MountCal();
    private boolean calApplied;
    private MountCal.Phase lastCalPhase = MountCal.Phase.IDLE;
    private long lastWinSeq = -1;
    private boolean autoLevelPending;
    private GlassesPresentation presentation;
    private GLSurfaceView embeddedGl;
    private DisplayManager displayManager;

    private TextView status;
    private TextView detail;
    private final Handler ui = new Handler(Looper.getMainLooper());

    private final DisplayManager.DisplayListener displayListener = new DisplayManager.DisplayListener() {
        @Override public void onDisplayAdded(int displayId) {
            log("display added: id=" + displayId);
            showOnGlasses();
        }
        @Override public void onDisplayRemoved(int displayId) {
            log("display removed: id=" + displayId);
            if (presentation != null) {
                try { presentation.dismiss(); } catch (Exception ignored) { }
                presentation = null;
            }
        }
        @Override public void onDisplayChanged(int displayId) { }
    };

    private final BroadcastReceiver permissionReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!ACTION_USB_PERMISSION.equals(intent.getAction())) return;
            UsbDevice granted = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            boolean ok = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            log("USB permission for " + (granted != null ? granted.getDeviceName() : "?") + " -> " + ok);
            if (ok && granted != null) {
                begin(granted);
            } else {
                setStatus("USB permission DENIED — cannot open the glasses");
            }
        }
    };

    private final BroadcastReceiver detachReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            UsbDevice gone = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            if (gone == null || XrealImu.isXrealAir(gone)) {
                log("USB detached — stopping reader");
                if (imu != null) imu.stop();
                setStatus("glasses disconnected");
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(32, 48, 32, 32);

        status = new TextView(this);
        status.setTextColor(Color.WHITE);
        status.setTextSize(TypedValue.COMPLEX_UNIT_SP, 16f);
        status.setText("starting…");
        root.addView(status);

        TextView hint = new TextView(this);
        hint.setTextColor(Color.parseColor("#88FFFFFF"));
        hint.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12f);
        hint.setPadding(0, 8, 0, 16);
        hint.setText("Tap anywhere to re-run the handshake. Log: adb logcat -s XrealProbe");
        root.addView(hint);

        pose = new HeadPose();
        // Both surfaces (embedded and glasses) hold this instance, so it must exist before the
        // renderers are constructed and must survive reconnects — see begin().
        imu = new XrealImu();

        HorizontalScrollView controls = new HorizontalScrollView(this);
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        controls.addView(row);
        root.addView(controls);

        addButton(row, "LEVEL", v -> {
            if (imu != null) {
                imu.recenter();                                   // drift clock
                pose.level(imu.quatW, imu.quatX, imu.quatY, imu.quatZ);  // render reference
                log("LEVEL: drift clock + render reference set");
            }
        });
        addButton(row, "CALIBRATE", v -> {
            calApplied = false;
            lastCalPhase = MountCal.Phase.IDLE;
            mountCal.start();
            log("calibration started: pose 1 = look straight ahead; pose 2 = tilt head back to look up");
        });
        // Escape hatch for a pose the automatic stillness detector will not accept (glasses wobbling
        // on the nose). Captures the current window mean as the current pose immediately.
        addButton(row, "pose now", v -> {
            mountCal.forcePose(imu.avgAX, imu.avgAY, imu.avgAZ,
                    imu.rotIntegralX, imu.rotIntegralY, imu.rotIntegralZ);
            log("pose now -> " + mountCal.status);
        });
        addButton(row, "flip", v -> {
            pose.nextVariant();
            log("sign variant -> " + pose.describe());
        });
        addButton(row, "RESET", v -> {
            mountCal.cancel();
            calApplied = false;
            lastCalPhase = MountCal.Phase.IDLE;
            pose.resetMount();
            log("mount reset to the measured default: " + pose.describe());
        });
        addButton(row, "mount X", v -> { pose.stepMount(0); log("mount: " + pose.describe()); });
        addButton(row, "mount Y", v -> { pose.stepMount(1); log("mount: " + pose.describe()); });
        addButton(row, "mount Z", v -> { pose.stepMount(2); log("mount: " + pose.describe()); });
        addButton(row, "lock/follow", v -> {
            pose.smoothFollow = !pose.smoothFollow;
            log("mode: " + pose.describe());
        });
        addButton(row, "screen", v -> { geom.cycleMode(); log(geom.describe()); });
        addButton(row, "size +", v -> { geom.scaleSize(1.1); log(geom.describe()); });
        addButton(row, "size -", v -> { geom.scaleSize(1.0 / 1.1); log(geom.describe()); });
        addButton(row, "nearer", v -> { geom.shiftDistance(-0.25); log(geom.describe()); });
        addButton(row, "farther", v -> { geom.shiftDistance(0.25); log(geom.describe()); });
        // Calibrating the optics assumption with the only instrument available: the wearer's eyes.
        addButton(row, "FOV check", v -> {
            geom.fovCheck = !geom.fovCheck;
            log(geom.fovDescribe() + " - brackets must sit ON the panel corners");
        });
        addButton(row, "FOV", v -> { geom.cycleFov(); log(geom.fovDescribe()); });
        addButton(row, "glasses", v -> showOnGlasses());

        // Embedded surface on the phone screen: a second view of the same pose path, so the motion
        // can be sanity-checked without wearing anything.
        embeddedGl = new GLSurfaceView(this);
        embeddedGl.setEGLContextClientVersion(2);
        embeddedGl.setRenderer(new ScreenRenderer(pose, imu, geom));
        embeddedGl.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        root.addView(embeddedGl, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 420));

        detail = new TextView(this);
        detail.setTextColor(Color.parseColor("#CCFFFFFF"));
        detail.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10f);
        detail.setTypeface(Typeface.MONOSPACE);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(detail);
        root.addView(scroll, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f));

        root.setOnClickListener((View v) -> retry());
        setContentView(root);

        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        if (usbManager == null) {
            setStatus("no USB host support on this device");
            return;
        }

        IntentFilter permFilter = new IntentFilter(ACTION_USB_PERMISSION);
        IntentFilter detachFilter = new IntentFilter(UsbManager.ACTION_USB_DEVICE_DETACHED);
        IntentFilter attachFilter = new IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(permissionReceiver, permFilter, Context.RECEIVER_NOT_EXPORTED);
            registerReceiver(detachReceiver, detachFilter, Context.RECEIVER_NOT_EXPORTED);
            registerReceiver(attachReceiver, attachFilter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(permissionReceiver, permFilter);
            registerReceiver(detachReceiver, detachFilter);
            registerReceiver(attachReceiver, attachFilter);
        }

        displayManager = (DisplayManager) getSystemService(Context.DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(displayListener, ui);
        }
        request();
        ui.post(refresh);
        ui.postDelayed(this::showOnGlasses, 1500);
    }

    private void addButton(LinearLayout row, String label, View.OnClickListener l) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setOnClickListener(l);
        row.addView(b);
    }

    /**
     * Put the virtual screen on the glasses. Tries a Presentation first; if the platform refuses
     * (DeX owning the display is the usual reason) the embedded surface in this activity is the
     * fallback, which DeX renders as a window on the glasses' desktop.
     */
    private void showOnGlasses() {
        if (displayManager == null || pose == null) return;
        Display target = null;
        for (Display d : displayManager.getDisplays()) {
            if (d.getDisplayId() != Display.DEFAULT_DISPLAY) {
                target = d;
                break;
            }
        }
        if (target == null) {
            log("no external display yet (presentation not shown)");
            return;
        }
        try {
            if (presentation != null) {
                try { presentation.dismiss(); } catch (Exception ignored) { }
                presentation = null;
            }
            presentation = new GlassesPresentation(this, target, pose, imu, mountCal, geom);
            presentation.show();
            log("presentation shown on display " + target.getDisplayId()
                    + " (" + target.getName() + ")");
        } catch (Exception e) {
            presentation = null;
            log("presentation FAILED on display " + target.getDisplayId() + ": " + e
                    + " — DeX may own the display; embedded surface remains visible");
        }
    }

    private final BroadcastReceiver attachReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            log("USB attached — requesting permission");
            request();
        }
    };

    /** Find the glasses and get permission (the permission intent needs FLAG_MUTABLE on API 31+). */
    private void request() {
        device = null;
        for (UsbDevice d : usbManager.getDeviceList().values()) {
            if (XrealImu.isXrealAir(d)) {
                device = d;
                break;
            }
        }
        if (device == null) {
            setStatus("XREAL Air (0x3318:0x0424) not attached\n\n"
                    + "connected USB devices:\n" + listDevices());
            return;
        }
        setStatus("found " + device.getProductName() + " — requesting USB permission…");
        if (usbManager.hasPermission(device)) {
            begin(device);
            return;
        }
        PendingIntent pi = PendingIntent.getBroadcast(this, 0,
                new Intent(ACTION_USB_PERMISSION).setPackage(getPackageName()),
                PendingIntent.FLAG_MUTABLE);
        usbManager.requestPermission(device, pi);
    }

    private void begin(UsbDevice d) {
        if (imu != null) imu.stop();
        connection = usbManager.openDevice(d);
        if (connection == null) {
            setStatus("openDevice() failed (permission or kernel driver)");
            return;
        }
        imu = (imu == null) ? new XrealImu() : imu;   // reuse: the GL renderers hold this reference
        imu.stop();
        imu.attach(connection);
        // The handshake blocks on USB I/O (bounded timeouts, but up to a few seconds) — keep it
        // off the main thread so a slow reply can't trip an ANR dialog.
        new Thread(() -> {
            log(imu.connect(d));
            log(imu.start());
            setStatus(imu.state);
            // Save the factory calibration blob so the Mac can inspect it after the replug.
            byte[] cal = imu.calData;
            if (cal != null && cal.length > 0) {
                try {
                    java.io.File f = new java.io.File(getExternalFilesDir(null), "xreal-calibration.json");
                    try (java.io.FileOutputStream out = new java.io.FileOutputStream(f, false)) {
                        out.write(cal);
                    }
                    log("calibration blob saved: " + cal.length + " bytes -> " + f.getAbsolutePath());
                } catch (Exception e) {
                    log("calibration save failed: " + e);
                }
            } else {
                log("no calibration blob captured (calDrained=" + imu.calDrained + ")");
            }
        }, "xreal-handshake").start();
    }

    private void retry() {
        if (device == null) {
            log("tap: no device, re-enumerating");
            request();
            return;
        }
        if (imu != null && connection != null) {
            new Thread(() -> {
                log("tap: restarting stream");
                log(imu.start());
                setStatus(imu.state);
            }, "xreal-restart").start();
        } else {
            log("tap: reconnecting");
            request();
        }
    }

    private String listDevices() {
        StringBuilder sb = new StringBuilder();
        for (UsbDevice d : usbManager.getDeviceList().values()) {
            sb.append(String.format(Locale.US, "  %04X:%04X %s (ifaces=%d)\n",
                    d.getVendorId(), d.getProductId(), d.getDeviceName(), d.getInterfaceCount()));
        }
        return sb.length() == 0 ? "  (none)\n" : sb.toString();
    }

    // ---- UI ----

    private int ticks = 0;

    private final Runnable refresh = new Runnable() {
        @Override
        public void run() {
            if (imu != null) {
                // Guided mount calibration. One finished IMU window per tick (the 0.5 s window
                // publishes at ~2 Hz), the rotation integral is reset at each pose change, and the
                // measured mount is installed the moment both poses are captured.
                MountCal.Phase ph = mountCal.phase();
                if (ph != lastCalPhase) {
                    if (ph == MountCal.Phase.FORWARD) imu.resetRotationIntegral();
                    if (ph == MountCal.Phase.DONE) {
                        log("CAL pose 2 captured — " + mountCal.note
                                + " (guard " + (mountCal.guardVerified()
                                ? String.format(Locale.US, "%.2f", mountCal.guardValue()) : "unverified") + ")");
                    } else if (ph == MountCal.Phase.FAILED) {
                        log("CAL failed — " + mountCal.note);
                    }
                    lastCalPhase = ph;
                }
                if (imu.winSeq != lastWinSeq) {
                    lastWinSeq = imu.winSeq;
                    mountCal.feed(imu.avgAX, imu.avgAY, imu.avgAZ, imu.avgG, imu.spreadA,
                            imu.rotIntegralX, imu.rotIntegralY, imu.rotIntegralZ);
                }
                if (mountCal.isDone() && !calApplied) {
                    calApplied = true;
                    float[] qm = mountCal.mountQuat();
                    pose.setMountMeasured(qm[0], qm[1], qm[2], qm[3]);
                    autoLevelPending = true;
                    log("MOUNT MEASURED — " + mountCal.note + "  [" + pose.describe() + "]");
                }
                // Auto-LEVEL: the calibration ends while the wearer is still looking up, so wait for
                // the next still moment (which is them back at a comfortable forward gaze) and set
                // the render reference plus the drift clock there. Removes the one button press that
                // is genuinely awkward to perform with the glasses on.
                if (autoLevelPending && "streaming".equals(imu.state)
                        && imu.spreadA <= 0.02 && imu.avgG <= 15.0 && imu.frames > 100) {
                    autoLevelPending = false;
                    imu.recenter();
                    pose.level(imu.quatW, imu.quatX, imu.quatY, imu.quatZ);
                    log("auto-LEVEL after mount calibration (screen now centred forward)");
                }
                String calLine = mountCal.phase() == MountCal.Phase.IDLE
                        ? "" : (mountCal.prompt() + "   [" + mountCal.status + "]\n");
                float[] ypr = pose.renderedYpr();
                double aMag = Math.sqrt(imu.accelBodyX * imu.accelBodyX
                        + imu.accelBodyY * imu.accelBodyY + imu.accelBodyZ * imu.accelBodyZ);
                status.setText(calLine + String.format(Locale.US,
                        "state: %s    frames: %d   %.0f Hz%n"
                                + "ahrs   yaw %7.1f  pitch %7.1f  roll %7.1f deg%n"
                                + "camera yaw %7.1f  pitch %7.1f  roll %7.1f deg%n"
                                + "accel %6.3f %6.3f %6.3f g   |a| %.3f   spread %.3f g%n"
                                + "DRIFT %7.2f deg / %5.0f s  = %7.2f deg/min%n"
                                + "ZUPT %-9s suppressed %7.2f deg%n"
                                + "render %s  %.0f fps  %s%n"
                                + "%s%n%s%n"
                                + "gyro bias %6.2f %6.2f %6.2f deg/s",
                        imu.state, imu.frames, imu.hz,
                        imu.yaw, imu.pitch, imu.roll,
                        ypr[0], ypr[1], ypr[2],
                        imu.accelBodyX, imu.accelBodyY, imu.accelBodyZ, aMag, imu.spreadA,
                        imu.driftDeg, imu.driftSeconds, imu.driftDegPerMin,
                        imu.stillness ? "ACTIVE" : "idle", imu.yawSuppressedDeg,
                        pose.isLeveled() ? "LEVELLED" : "not levelled",
                        pose.fps(), pose.describe(),
                        geom.describe(), geom.fovDescribe(),
                        imu.gyroBiasX, imu.gyroBiasY, imu.gyroBiasZ));
                detail.setText(imu.lastFrameInfo + "\n\n"
                        + "----- handshake -----\n" + imu.handshakeLog
                        + "\n----- first frames -----\n" + imu.firstFramesHex);
                if (++ticks % 8 == 0) dumpReport();   // ~every 2 s, so results survive a replug
            }
            ui.postDelayed(this, 250);
        }
    };

    private void setStatus(String s) {
        onUi(() -> {
            status.setText(s);
            Log.i(TAG, s.replace('\n', ' '));
        });
    }

    /** Snapshot the whole probe state into a file the Mac can read back over adb after replug. */
    private void dumpReport() {
        if (imu == null) return;
        String body = "state: " + imu.state
                + "\nframes: " + imu.frames + "   rate(1s window): " + imu.hz + " Hz   avg rate: "
                + imu.avgHz + " Hz"
                + "\ninitFrames: " + imu.initFrames + "   badSig: " + imu.badSig
                + "   readErrors: " + imu.readErrors + "   shortReads: " + imu.shortReads
                + "\nwriteBytes(last cmd): " + imu.writeBytes + "  via " + imu.writePath
                + "\nstaticId: " + imu.staticId + "   calLen: " + imu.calLen + "   calDrained: " + imu.calDrained
                + "\n=== AHRS (Madgwick, 6-axis, beta=0.1) ==="
                + "\nroll: " + imu.roll + "   pitch: " + imu.pitch + "   yaw: " + imu.yaw
                + "\ndrift: " + imu.driftDeg + " deg over " + imu.driftSeconds + " s  = "
                + imu.driftDegPerMin + " deg/min   (recentered=" + imu.recentered + ")"
                + "\ngyro bias estimate: " + imu.gyroBiasX + " " + imu.gyroBiasY + " " + imu.gyroBiasZ + " deg/s"
                + "\n=== SCREEN / OPTICS ==="
                + "\n" + geom.describe()
                + "\n" + geom.fovDescribe()
                + "\n=== MOUNT (body -> viewer) ==="
                + "\nselected: " + pose.describe()
                + "\nmount quat (w,x,y,z): " + f4(pose.mountQuat())
                + "\nsource: " + pose.mountSource + "   variant: " + pose.variantName()
                + "\ncal phase: " + mountCal.phase() + "   status: " + mountCal.status
                + "\ncal note: " + mountCal.note
                + "\nmeasured up (body, g): " + f3(mountCal.upInBody())
                + "\nmeasured fwd (body, g): " + f3(mountCal.forwardInBody())
                + "\ntilt over pose B: " + mountCal.tiltDeg() + " deg   sign guard: "
                + (mountCal.guardVerified() ? String.valueOf(mountCal.guardValue()) : "unverified")
                + "\naccel body now (g): " + imu.accelBodyX + " " + imu.accelBodyY + " " + imu.accelBodyZ
                + "\nIf a nod reads backwards, press flip once (v1 = 180 deg about viewer up)."
                + "\n=== ZUPT (world-vertical drift governor) ==="
                + "\nstillness: " + imu.stillness + "   stillSeconds: " + imu.stillSeconds
                + "\nyaw drift suppressed: " + imu.yawSuppressedDeg + " deg"
                + "\nyaw WITHOUT governor (observed + suppressed): " + imu.rawDriftDeg + " deg"
                + "\ncalData captured: " + (imu.calData == null ? "none" : imu.calData.length + " bytes")
                + "\n\n=== live frame ===\n" + imu.lastFrameInfo
                + "\n\n=== first frames (hex) ===\n" + imu.firstFramesHex
                + "\n=== handshake / device tree ===\n" + imu.handshakeLog
                + "\nwritten: " + new java.util.Date();
        try {
            java.io.File f = new java.io.File(getExternalFilesDir(null), "probe-report.txt");
            try (java.io.FileOutputStream out = new java.io.FileOutputStream(f, false)) {
                out.write(body.getBytes("UTF-8"));
            }
            Log.i(TAG, "report written: " + f.getAbsolutePath());
        } catch (Exception e) {
            Log.w(TAG, "report write failed: " + e);
        }
    }

    private static String f3(float[] v) {
        return String.format(Locale.US, "%.4f %.4f %.4f", v[0], v[1], v[2]);
    }

    private static String f4(float[] v) {
        return String.format(Locale.US, "%.5f %.5f %.5f %.5f", v[0], v[1], v[2], v[3]);
    }

    private void log(String s) {
        Log.i(TAG, s == null ? "null" : s);
        onUi(() -> {
            if (detail != null && s != null) detail.append(s + "\n");
        });
    }

    /** Run on the UI thread regardless of the calling thread (handshake runs off the main thread). */
    private void onUi(Runnable r) {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            r.run();
        } else {
            ui.post(r);
        }
    }

    @Override
    protected void onDestroy() {
        ui.removeCallbacks(refresh);
        if (displayManager != null) {
            try { displayManager.unregisterDisplayListener(displayListener); } catch (Exception ignored) { }
        }
        if (presentation != null) {
            try { presentation.dismiss(); } catch (Exception ignored) { }
            presentation = null;
        }
        if (embeddedGl != null) embeddedGl.onPause();
        if (imu != null) imu.stop();
        try {
            unregisterReceiver(permissionReceiver);
            unregisterReceiver(detachReceiver);
            unregisterReceiver(attachReceiver);
        } catch (Exception ignored) { }
        super.onDestroy();
    }
}
