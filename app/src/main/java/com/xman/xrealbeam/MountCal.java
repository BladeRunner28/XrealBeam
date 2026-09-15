package com.xman.xrealbeam;

import java.util.Locale;

/**
 * Derives the device -> viewer mount from GRAVITY, in two instructed still poses.
 *
 * Why not gestures: the previous version integrated head-turn and nod rates to find the axes. That
 * finds the axes but can never resolve their sign (a back-and-forth gesture sums to zero, so it was
 * accumulated as an outer product and came out sign-agnostic), which is why it shipped with four
 * variants for the wearer to cycle through — cycling four variants on your head is a bad procedure.
 *
 * Gravity is a direction with a *physical* sign: at rest the accelerometer reads +1 g along whichever
 * axis points up. So:
 *
 *   Pose A — worn, looking level, still.  accel = the world up in body coords  -> the viewer's +y row.
 *   Pose B — head tilted (instructed: back, looking up).  accel = up*cos(t) + forward*sin(t), so
 *            the component perpendicular to pose A's up is the forward axis.
 *
 *   up    = normalize(a_A)                                 (viewer +y expressed in body coords)
 *   f     = normalize(a_B - (a_B·up)·up)                    (viewer forward = -z, up to sign)
 *   right = up x (-f)                                       (viewer +x, keeps the matrix proper)
 *
 * The tilt angle only has to exceed ~40 deg: orthogonalising against `up` makes the recovered
 * direction exact at any tilt between 0 and 180 deg, with noise amplification ~1/sin(t).
 *
 * The one thing gravity CANNOT settle: `f` comes out with a sign ambiguity, and the two candidates
 * are exactly "the viewer rotated 180 deg about the up axis" (they differ by negating the right and
 * forward rows, det stays +1). Nothing in the accelerometer distinguishes them; the two differ in
 * the sign of pitch and roll — i.e. whether a nod reads as a nod or as a nod backwards.
 *
 * Resolution: the gyro integral over pose B, plus the instruction that the wearer was LOOKING UP.
 * Looking up is a positive rotation about the wearer's right axis, and the viewer's +x IS that
 * right axis, so for the correct candidate M the measured body rotation vector w satisfies
 * (M·w)·x > 0. The two candidates give opposite signs, so the test picks one.
 *
 * Honest limit: that resolves the ambiguity by TRUSTING THE INSTRUCTION, and no sensor data can do
 * better — the two candidates are indistinguishable from the accelerometer and gyro alone, being
 * related by a 180 deg rotation about the vertical, under which both the world-up reading and the
 * rotation vector are unchanged. If the wearer tilts their head forward when asked to look up, the
 * measured perpendicular component flips together with the motion and the outcome is the mirror
 * mount, which shows up immediately as a nod that reads backwards. One press of the flip button
 * (variant v1 = 180 deg about the viewer's up axis) is exactly that correction — the harness asserts
 * both halves of this: compliant -> correct mount, non-compliant -> exact mirror fixed by one press.
 *
 * If the tilt rotation seen by the gyro is too small to trust (< 10 deg), the mount is applied as
 * measured and the note says "guard unverified" — then a nod left/right or the variant button
 * resolves it, which is one binary instead of the old four-way hunt.
 *
 * Off-device verification: tools/PoseHarness.java runs this class and HeadPose against a synthetic
 * ground-truth mount. Noise-free input is recovered to 1e-9; with 0.01 g accelerometer noise the
 * recovered mount is within ~1 deg for any tilt over 60 deg, and the sign guard picks the correct
 * candidate in every trial including the deliberately-wrong-direction ones.
 */
public class MountCal {

    public enum Phase { IDLE, UP, FORWARD, DONE, FAILED }

    private static final int GOOD_WINDOWS = 3;        // consecutive still windows (~0.75 s at 4 Hz)
    private static final double MAX_SPREAD_G = 0.02;  // |a| spread inside one window
    private static final double MAX_GYRO_DPS = 15.0;  // mean |omega| inside one window
    private static final double MIN_TILT_DEG = 40.0;
    private static final double TILT_AXIS_MIN_DEG = 10.0;   // integral big enough to trust the sign

    private Phase phase = Phase.IDLE;
    private int good, nA, nB;
    private double sux, suy, suz, sfx, sfy, sfz;

    public volatile String note = "";
    public volatile String status = "idle";

    private final float[] up = new float[3];
    private final float[] fwd = new float[3];
    private final float[] mount = {1, 0, 0, 0};
    private volatile double tiltDeg, guardValue;
    private volatile boolean guardVerified;

    public Phase phase() {
        return phase;
    }

    public boolean isDone() {
        return phase == Phase.DONE;
    }

    public String prompt() {
        switch (phase) {
            case UP: return "CAL 1/2 — look straight ahead, hold still";
            case FORWARD: return "CAL 2/2 — tilt your head BACK to look straight up, hold still";
            case DONE: return "CAL done: " + note;
            case FAILED: return "CAL failed: " + note;
            default: return "calibration idle";
        }
    }

    public void start() {
        phase = Phase.UP;
        good = 0;
        nA = nB = 0;
        sux = suy = suz = sfx = sfy = sfz = 0;
        note = "";
        guardVerified = false;
        status = "pose A: looking level";
    }

    public void cancel() {
        phase = Phase.IDLE;
        status = "idle";
    }

    /**
     * One windowed snapshot from the IMU (call ~4 Hz). Accel in g, gyro in deg/s, spread = (max|a| -
     * min|a|) inside the window, rot = the body-frame gyro integral since the pose started (deg).
     */
    public void feed(double ax, double ay, double az, double gyroMag, double accelSpread,
                     double rotX, double rotY, double rotZ) {
        if (phase != Phase.UP && phase != Phase.FORWARD) {
            return;
        }
        double mag = Math.sqrt(ax * ax + ay * ay + az * az);
        boolean still = accelSpread <= MAX_SPREAD_G && gyroMag <= MAX_GYRO_DPS
                && Math.abs(mag - 1.0) < 0.15;
        if (!still) {
            good = 0;
            status = (phase == Phase.UP ? "pose A" : "pose B") + ": not still — |a|="
                    + fmt(mag) + " g, spread=" + fmt(accelSpread) + " g, |w|=" + fmt(gyroMag) + " deg/s";
            return;
        }
        good++;
        if (phase == Phase.UP) {
            sux += ax; suy += ay; suz += az; nA++;
            status = "pose A: still " + good + "/" + GOOD_WINDOWS;
            if (good >= GOOD_WINDOWS) finishUp();
        } else {
            sfx += ax; sfy += ay; sfz += az; nB++;
            status = "pose B: still " + good + "/" + GOOD_WINDOWS + "  (tilt seen "
                    + fmt(Math.sqrt(rotX * rotX + rotY * rotY + rotZ * rotZ)) + " deg)";
            if (good >= GOOD_WINDOWS) finishForward(rotX, rotY, rotZ);
        }
    }

    /**
     * Force-capture the current window as the current pose (the on-screen "pose now" button). The
     * automatic path needs three consecutive still windows; when a wearer is holding a pose but the
     * detector is unhappy (glasses wobbling on the nose), this takes the window mean as-is.
     */
    public void forcePose(double ax, double ay, double az, double rotX, double rotY, double rotZ) {
        if (phase != Phase.UP && phase != Phase.FORWARD) {
            return;
        }
        double mag = Math.sqrt(ax * ax + ay * ay + az * az);
        if (Math.abs(mag - 1.0) > 0.15) {
            status = "pose now ignored: |a| = " + fmt(mag) + " g (not a clean gravity reading)";
            return;
        }
        if (phase == Phase.UP) {
            sux = ax; suy = ay; suz = az; nA = 1;
            finishUp();
        } else {
            sfx = ax; sfy = ay; sfz = az; nB = 1;
            finishForward(rotX, rotY, rotZ);
        }
    }

    private void finishUp() {
        float[] a = normalize(new float[]{(float) (sux / nA), (float) (suy / nA), (float) (suz / nA)});
        System.arraycopy(a, 0, up, 0, 3);
        phase = Phase.FORWARD;
        good = 0;
        nB = 0;
        sfx = sfy = sfz = 0;
        status = "pose B: tilt head BACK, look straight up";
    }

    private void finishForward(double rotX, double rotY, double rotZ) {
        float[] a = new float[]{(float) (sfx / nB), (float) (sfy / nB), (float) (sfz / nB)};
        float dot = a[0] * up[0] + a[1] * up[1] + a[2] * up[2];
        float aNorm = (float) Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        tiltDeg = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, aNorm > 0 ? dot / aNorm : 1))));
        if (tiltDeg < MIN_TILT_DEG) {
            phase = Phase.FAILED;
            note = String.format(Locale.US,
                    "head tilted only %.0f deg - tilt back at least %.0f deg", tiltDeg, MIN_TILT_DEG);
            return;
        }

        float[] f = normalize(new float[]{a[0] - dot * up[0], a[1] - dot * up[1], a[2] - dot * up[2]});
        float[] q = mountFrom(up, f);
        if (q == null) {
            phase = Phase.FAILED;
            note = "measured axes were degenerate (up and forward nearly parallel)";
            return;
        }

        // Sign guard: looking up is a positive rotation about the wearer's right, and the viewer's
        // +x is that right axis, so (M * w).x must be positive for the correct candidate.
        double rotMag = Math.sqrt(rotX * rotX + rotY * rotY + rotZ * rotZ);
        String signNote;
        if (rotMag >= TILT_AXIS_MIN_DEG) {
            guardVerified = true;
            guardValue = rotateX(q, rotX, rotY, rotZ) / rotMag;
            if (guardValue < 0) {
                f = new float[]{-f[0], -f[1], -f[2]};
                q = mountFrom(up, f);
                guardValue = rotateX(q, rotX, rotY, rotZ) / rotMag;
                signNote = " guard flipped forward axis";
            } else {
                signNote = " guard ok";
            }
        } else {
            guardVerified = false;
            guardValue = 0;
            signNote = " guard UNVERIFIED (only " + fmt(rotMag) + " deg of tilt seen)";
        }

        System.arraycopy(f, 0, fwd, 0, 3);
        System.arraycopy(q, 0, mount, 0, 4);
        phase = Phase.DONE;
        status = "done";
        note = String.format(Locale.US, "up=(%s) fwd=(%s) tilt=%.0f deg%s",
                fmt3(up), fmt3(fwd), tiltDeg, signNote);
    }

    /** Mount from the measured up and forward axes: rows are the viewer axes in body coords. */
    private static float[] mountFrom(float[] up, float[] f) {
        float[] back = new float[]{-f[0], -f[1], -f[2]};
        float[] right = normalize(cross(up, back));
        return quatFromRows(right, up, back);
    }

    /** x-component of (M * w), i.e. the rotation's component about the viewer's right axis. */
    private static double rotateX(float[] q, double wx, double wy, double wz) {
        float w = q[0], x = q[1], y = q[2], z = q[3];
        double r00 = 1 - 2 * (y * y + z * z);
        double r01 = 2 * (x * y - w * z);
        double r02 = 2 * (x * z + w * y);
        return r00 * wx + r01 * wy + r02 * wz;
    }

    public float[] mountQuat() {
        return new float[]{mount[0], mount[1], mount[2], mount[3]};
    }

    public float[] upInBody() {
        return new float[]{up[0], up[1], up[2]};
    }

    public float[] forwardInBody() {
        return new float[]{fwd[0], fwd[1], fwd[2]};
    }

    public double tiltDeg() {
        return tiltDeg;
    }

    public double guardValue() {
        return guardValue;
    }

    public boolean guardVerified() {
        return guardVerified;
    }

    /** Quaternion whose rotation matrix has the given viewer axes as rows (viewer <- body). */
    private static float[] quatFromRows(float[] x, float[] y, float[] z) {
        float m00 = x[0], m01 = x[1], m02 = x[2];
        float m10 = y[0], m11 = y[1], m12 = y[2];
        float m20 = z[0], m21 = z[1], m22 = z[2];
        float tr = m00 + m11 + m22;
        float w, qx, qy, qz;
        if (tr > 0) {
            float s = (float) Math.sqrt(tr + 1.0) * 2f;
            w = 0.25f * s;
            qx = (m21 - m12) / s;
            qy = (m02 - m20) / s;
            qz = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            float s = (float) Math.sqrt(1.0 + m00 - m11 - m22) * 2f;
            w = (m21 - m12) / s;
            qx = 0.25f * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            float s = (float) Math.sqrt(1.0 + m11 - m00 - m22) * 2f;
            w = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25f * s;
            qz = (m12 + m21) / s;
        } else {
            float s = (float) Math.sqrt(1.0 + m22 - m00 - m11) * 2f;
            w = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25f * s;
        }
        float n = (float) Math.sqrt(w * w + qx * qx + qy * qy + qz * qz);
        if (n < 1e-6f) return null;
        return new float[]{w / n, qx / n, qy / n, qz / n};
    }

    private static float[] cross(float[] a, float[] b) {
        return new float[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    private static float[] normalize(float[] v) {
        float n = (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n < 1e-9f) return new float[]{0, 1, 0};
        return new float[]{v[0] / n, v[1] / n, v[2] / n};
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.2f", v);
    }

    private static String fmt3(float[] v) {
        return String.format(Locale.US, "%.2f,%.2f,%.2f", v[0], v[1], v[2]);
    }
}
