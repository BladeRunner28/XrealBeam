package com.xman.xrealbeam;

import java.util.Locale;

/**
 * Off-device verification harness for the pose pipeline. NOT part of the APK (it lives outside
 * app/src/main/java and is compiled only by tools/verify-pose.sh).
 *
 * It runs the *shipping* HeadPose and MountCal classes against synthetic ground truth, because the
 * one thing that cannot be checked on this Mac is whether head motion moves the screen the right way,
 * and a wrong mount is invisible in review but unmistakable on the head.
 *
 * Checks:
 *   1. HeadPose.DEFAULT_MOUNT equals the mount derived from the wearer's on-device axis report
 *      (nod rendered as yaw, tilt rendered as pitch -> 120 deg about (1,1,1), rows x->body+z,
 *      up->body+x, back->body+y), and renders every canonical head motion exactly.
 *   2. The four sign variants flip exactly the documented axis pairs (a single-axis flip would be a
 *      reflection, which is why the old "yaw sign" button misbehaved).
 *   3. The manual step buttons reach the same mount at (0,90,90) - i.e. the old (90,0,90) default was
 *      off by one 90 deg step about the wrong axes, which is what put motion on the wrong axes.
 *   4. MountCal recovers a random ground-truth mount from two noisy still poses, including the
 *      deliberately-wrong tilt direction (sign guard must self-correct), and reports the residual
 *      angle error.
 */
public class PoseHarness {

    // Canonical viewer frame: +x right, +y up, +z backward. Motions are named as the wearer feels
    // them; the axis is the true canonical rotation axis.
    private static final String[] MOTION_NAMES = {
            "turn LEFT (yaw)", "turn RIGHT (yaw)", "look UP (nod)", "look DOWN (nod)",
            "tilt RIGHT", "tilt LEFT"};
    private static final float[][] MOTION_AXES = {
            {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 0, -1}, {0, 0, 1}};

    /** The mount derived from the on-device report (see HeadPose class comment). */
    private static final float[][] EXPECTED_ROWS = {
            {0, 0, 1},   // viewer +x (right)  in body coords
            {1, 0, 0},   // viewer +y (up)     in body coords
            {0, 1, 0}};  // viewer +z (back)   in body coords

    private static boolean allOk = true;

    public static void main(String[] args) {
        testDefaultMount();
        testVariants();
        testStepsPath();
        testMountCal();
        System.out.println(allOk ? "\n=== ALL CHECKS PASSED ===" : "\n=== FAILURES PRESENT ===");
        if (!allOk) System.exit(1);
    }

    // ---------- 1. default mount ----------

    private static void testDefaultMount() {
        System.out.println("--- 1. default mount vs on-device derivation ---");
        float[][] rows = rowsOf(HeadPose.DEFAULT_MOUNT);
        float worst = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                worst = Math.max(worst, Math.abs(rows[i][j] - EXPECTED_ROWS[i][j]));
            }
        }
        report("default mount rows equal the derived rotation", worst < 1e-6f,
                String.format(Locale.US, "max row error %.2e", worst));

        float[] q = HeadPose.DEFAULT_MOUNT;
        double angle = 2 * Math.toDegrees(Math.acos(Math.min(1, Math.abs(q[0]))));
        report("default mount is the 120 deg cube rotation", Math.abs(angle - 120) < 1e-3,
                String.format(Locale.US, "angle %.3f deg about (%.3f,%.3f,%.3f)",
                        angle, Math.abs(q[1]), Math.abs(q[2]), Math.abs(q[3])));

        // render each motion through the shipping pipeline with the truth = the default mount
        HeadPose pose = new HeadPose();
        pose.level(1, 0, 0, 0);
        double worstDot = 1;
        for (int i = 0; i < MOTION_NAMES.length; i++) {
            float[] out = render(pose, q, MOTION_AXES[i], 30f);
            double dot = Math.abs(dot(out, axisQuat(MOTION_AXES[i], 30f)));
            worstDot = Math.min(worstDot, dot);
            System.out.printf(Locale.US, "      %-18s -> viewer axis %-14s |<q,q_intended>| = %.6f%n",
                    MOTION_NAMES[i], axisName(out), dot);
        }
        report("every motion renders exactly (axis AND sense)", worstDot > 1 - 1e-6,
                String.format(Locale.US, "worst |dot| = %.6f", worstDot));
    }

    // ---------- 2. sign variants ----------

    private static void testVariants() {
        System.out.println("\n--- 2. sign variants are proper 180 deg conjugations ---");
        HeadPose pose = new HeadPose();
        pose.level(1, 0, 0, 0);
        int[][] base = new int[MOTION_NAMES.length][4];
        for (int v = 0; v < 4; v++) {
            pose.resetMount();
            for (int i = 0; i < v; i++) pose.nextVariant();
            for (int i = 0; i < MOTION_NAMES.length; i++) {
                float[] out = renderFresh(pose, HeadPose.DEFAULT_MOUNT, MOTION_AXES[i], 30f);
                base[i][v] = sense(out);
            }
        }
        // documented: v1 flips yaw+roll, v2 flips pitch+roll, v3 flips yaw+pitch
        boolean[][] flip = new boolean[6][4];
        for (int i = 0; i < 6; i++) {
            for (int v = 1; v < 4; v++) flip[i][v] = base[i][v] != base[i][0];
        }
        // motion order: 0 turn L (yaw), 2 look up (pitch), 4 tilt R (roll)
        String[] want = {"none", "pitch+roll", "yaw+roll", "yaw+pitch"};
        for (int v = 1; v < 4; v++) {
            String got = (flip[0][v] ? "yaw+" : "") + (flip[2][v] ? "pitch+" : "") + (flip[4][v] ? "roll+" : "");
            got = got.isEmpty() ? "none" : got.substring(0, got.length() - 1);
            report("variant v" + v + " flips " + want[v], got.equals(want[v]), "observed: " + got);
        }
    }

    // ---------- 3. manual step buttons ----------

    private static void testStepsPath() {
        System.out.println("\n--- 3. manual step buttons vs the derived mount ---");
        HeadPose pose = new HeadPose();
        pose.mountStepsX = 0;
        pose.mountStepsY = 1;
        pose.mountStepsZ = 1;
        pose.applySteps();
        double dot = Math.abs(dot(pose.mountQuat(), HeadPose.DEFAULT_MOUNT));
        report("steps(0,90,90) == derived default", dot > 1 - 1e-6,
                String.format(Locale.US, "|<q,q_default>| = %.6f (quat %s)", dot, fmt4(pose.mountQuat())));
        pose.resetMount();
        report("RESET returns to the default mount",
                Math.abs(dot(pose.mountQuat(), HeadPose.DEFAULT_MOUNT)) > 1 - 1e-6
                        && HeadPose.SOURCE_DEFAULT.equals(pose.mountSource), pose.describe());
    }

    // ---------- 4. gravity calibration ----------

    private static void testMountCal() {
        System.out.println("\n--- 4. MountCal: recovery of a random ground-truth mount ---");
        java.util.Random rng = new java.util.Random(20260914L);

        System.out.printf(Locale.US, "   %-28s %-12s %-12s %s%n",
                "case", "tilt", "mount err", "guard");
        for (double tilt : new double[]{90, 75, 60, 45}) {
            double worst = 0;
            int guardUnverified = 0;
            for (int trial = 0; trial < 200; trial++) {
                float[] qTrue = randomMount(rng);
                float[] upTrue = applyT(qTrue, new float[]{0, 1, 0});          // up in body coords
                float[] fwdTrue = applyT(qTrue, new float[]{0, 0, -1});        // forward in body coords
                float[] rightTrue = applyT(qTrue, new float[]{1, 0, 0});       // right in body coords
                double t = Math.toRadians(tilt);
                double[] accA = noise(rng, upTrue);
                double[] accB = noise(rng, new double[]{
                        Math.cos(t) * upTrue[0] + Math.sin(t) * fwdTrue[0],
                        Math.cos(t) * upTrue[1] + Math.sin(t) * fwdTrue[1],
                        Math.cos(t) * upTrue[2] + Math.sin(t) * fwdTrue[2]});
                // gyro integral over pose B = "look up" = +tilt about the wearer's right
                double[] rot = {tilt * rightTrue[0], tilt * rightTrue[1], tilt * rightTrue[2]};

                MountCal cal = runCal(accA, accB, rot);
                if (cal == null) {
                    report("calibration completes (tilt " + tilt + ")", false, "phase not DONE");
                    return;
                }
                worst = Math.max(worst, mountError(cal.mountQuat(), qTrue));
                if (!cal.guardVerified()) guardUnverified++;
            }
            report(String.format(Locale.US, "tilt %.0f deg: mount error < 3 deg over 200 trials", tilt),
                    worst < 3.0, String.format(Locale.US, "worst %.3f deg, guard unverified %d",
                            worst, guardUnverified));
        }

        // Non-compliant tilt (wearer looks DOWN when asked to look up). Data cannot reveal this, so
        // the honest result is the mirror mount - and exactly one variant press must fix it.
        double worstMirror = 0, worstFixed = 0;
        for (int trial = 0; trial < 60; trial++) {
            float[] qTrue = randomMount(rng);
            float[] upTrue = applyT(qTrue, new float[]{0, 1, 0});
            float[] fwdTrue = applyT(qTrue, new float[]{0, 0, -1});
            float[] rightTrue = applyT(qTrue, new float[]{1, 0, 0});
            double t = Math.toRadians(75);
            double[] accA = noise(rng, upTrue);
            double[] accB = noise(rng, new double[]{
                    Math.cos(t) * upTrue[0] - Math.sin(t) * fwdTrue[0],
                    Math.cos(t) * upTrue[1] - Math.sin(t) * fwdTrue[1],
                    Math.cos(t) * upTrue[2] - Math.sin(t) * fwdTrue[2]});
            double[] rot = {-tiltRot(t) * rightTrue[0], -tiltRot(t) * rightTrue[1],
                    -tiltRot(t) * rightTrue[2]};
            MountCal cal = runCal(accA, accB, rot);
            if (cal == null) {
                report("non-compliant titration completes", false, "phase not DONE");
                return;
            }
            worstMirror = Math.max(worstMirror, mountError(cal.mountQuat(), mirror(qTrue)));
            HeadPose p = new HeadPose();
            p.setMountMeasured(cal.mountQuat()[0], cal.mountQuat()[1],
                    cal.mountQuat()[2], cal.mountQuat()[3]);
            p.nextVariant();     // v1 = 180 deg about the viewer's up axis
            worstFixed = Math.max(worstFixed, mountError(p.mountQuat(), qTrue));
        }
        report("backwards tilt -> exact mirror, one flip press fixes it (< 3 deg)",
                worstMirror < 3.0 && worstFixed < 3.0,
                String.format(Locale.US, "mirror err %.3f deg, after one press %.3f deg",
                        worstMirror, worstFixed));

        // noise-free recovery must be exact
        float[] qTrue = randomMount(new java.util.Random(1));
        float[] upTrue = applyT(qTrue, new float[]{0, 1, 0});
        float[] fwdTrue = applyT(qTrue, new float[]{0, 0, -1});
        float[] rightTrue = applyT(qTrue, new float[]{1, 0, 0});
        MountCal cal = new MountCal();
        cal.start();
        feedWindows(cal, new double[]{upTrue[0], upTrue[1], upTrue[2]}, 0, 0, 0, 0);
        feedWindows(cal, new double[]{fwdTrue[0], fwdTrue[1], fwdTrue[2]}, 0,
                90 * rightTrue[0], 90 * rightTrue[1], 90 * rightTrue[2]);
        report("noise-free recovery is exact (< 1e-4 deg)", mountError(cal.mountQuat(), qTrue) < 1e-4,
                String.format(Locale.US, "error %.3e deg", mountError(cal.mountQuat(), qTrue)));

        // too little tilt must fail loudly rather than produce a bad mount
        MountCal shy = new MountCal();
        shy.start();
        double[] a = {upTrue[0], upTrue[1], upTrue[2]};
        double t = Math.toRadians(20);
        feedWindows(shy, a, 0, 0, 0, 0);
        feedWindows(shy, new double[]{
                Math.cos(t) * upTrue[0] + Math.sin(t) * fwdTrue[0],
                Math.cos(t) * upTrue[1] + Math.sin(t) * fwdTrue[1],
                Math.cos(t) * upTrue[2] + Math.sin(t) * fwdTrue[2]}, 0, 0, 0, 0);
        report("a 20 deg tilt is rejected, not accepted", shy.phase() == MountCal.Phase.FAILED,
                shy.prompt());
    }

    // ---------- helpers ----------

    /** Three still windows at ~4 Hz = the capture condition. */
    private static void feedWindows(MountCal cal, double[] accel, double gyroMag,
                                    double rx, double ry, double rz) {
        for (int i = 0; i < 3; i++) {
            cal.feed(accel[0], accel[1], accel[2], gyroMag, 0.005, rx, ry, rz);
        }
    }

    /** Run a whole two-pose calibration; null if it did not reach DONE. */
    private static MountCal runCal(double[] accA, double[] accB, double[] rot) {
        MountCal cal = new MountCal();
        cal.start();
        feedWindows(cal, accA, 0, 0, 0, 0);
        feedWindows(cal, accB, 2.0, rot[0], rot[1], rot[2]);
        return cal.isDone() ? cal : null;
    }

    private static double tiltRot(double radians) {
        return Math.toDegrees(radians);
    }

    /** The mirror mount: the 180 deg conjugation about the viewer's up axis (variant v1). */
    private static float[] mirror(float[] q) {
        return qmul(axisQuat(new float[]{0, 1, 0}, 180), q);
    }

    private static double[] noise(java.util.Random rng, float[] v) {
        return new double[]{v[0] + rng.nextGaussian() * 0.01,
                v[1] + rng.nextGaussian() * 0.01,
                v[2] + rng.nextGaussian() * 0.01};
    }

    private static double[] noise(java.util.Random rng, double[] v) {
        return new double[]{v[0] + rng.nextGaussian() * 0.01,
                v[1] + rng.nextGaussian() * 0.01,
                v[2] + rng.nextGaussian() * 0.01};
    }

    private static float[] randomMount(java.util.Random rng) {
        // random unit quaternion (uniform-ish is fine for a smoke test)
        double w = rng.nextGaussian(), x = rng.nextGaussian(), y = rng.nextGaussian(), z = rng.nextGaussian();
        double n = Math.sqrt(w * w + x * x + y * y + z * z);
        return new float[]{(float) (w / n), (float) (x / n), (float) (y / n), (float) (z / n)};
    }

    /** Feed a physical motion to a fresh HeadPose whose mount is `mountUsed`; truth is the same. */
    private static float[] render(HeadPose pose, float[] mount, float[] axis, float deg) {
        return renderFresh(pose, mount, axis, deg);
    }

    private static float[] renderFresh(HeadPose pose, float[] mount, float[] axis, float deg) {
        float[] qView = axisQuat(axis, deg);                       // intended viewer-frame rotation
        float[] qBody = qmul(conj(mount), qmul(qView, mount));     // same rotation in body coords
        pose.level(1, 0, 0, 0);
        pose.update(qBody[0], qBody[1], qBody[2], qBody[3], 1.0 / 60.0);
        return pose.renderedQuat();
    }

    private static int sense(float[] q) {
        float[] ax = axisOf(q);
        int k = 0;
        for (int i = 1; i < 3; i++) if (Math.abs(ax[i]) > Math.abs(ax[k])) k = i;
        return ax[k] > 0 ? 1 : -1;
    }

    private static String axisName(float[] q) {
        float[] ax = axisOf(q);
        int k = 0;
        for (int i = 1; i < 3; i++) if (Math.abs(ax[i]) > Math.abs(ax[k])) k = i;
        return String.format(Locale.US, "%s%s (%.2f)", ax[k] > 0 ? "+" : "-", "xyz".charAt(k), ax[k]);
    }

    /** Rotation axis of a quaternion (sign = right-hand-rule sense). */
    private static float[] axisOf(float[] q) {
        float[] v = {q[1], q[2], q[3]};
        float n = (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n < 1e-6f) return new float[]{0, 0, 0};
        return new float[]{v[0] / n, v[1] / n, v[2] / n};
    }

    /**
     * R(q) as the rotation OPERATOR (v' = R v), so row i is the viewer axis i expressed in body
     * coordinates. Note HeadPose.getViewMatrix builds the transpose of this on purpose (view = R^T);
     * mixing the two conventions is exactly the kind of sign slip this harness exists to catch.
     */
    private static float[][] rowsOf(float[] q) {
        float w = q[0], x = q[1], y = q[2], z = q[3];
        return new float[][]{
                {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    }

    private static float[] applyT(float[] q, float[] v) {
        float[][] r = rowsOf(q);
        // R^T v : the body-frame representation of a viewer-frame vector
        return new float[]{
                r[0][0] * v[0] + r[1][0] * v[1] + r[2][0] * v[2],
                r[0][1] * v[0] + r[1][1] * v[1] + r[2][1] * v[2],
                r[0][2] * v[0] + r[1][2] * v[1] + r[2][2] * v[2]};
    }

    private static double mountError(float[] qa, float[] qb) {
        double d = Math.abs(dot(qa, qb));
        d = Math.min(1.0, d);
        return 2 * Math.toDegrees(Math.acos(d));
    }

    private static float dot(float[] a, float[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    private static float[] conj(float[] q) {
        return new float[]{q[0], -q[1], -q[2], -q[3]};
    }

    private static float[] qmul(float[] a, float[] b) {
        return new float[]{
                a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    }

    private static float[] axisQuat(float[] axis, float deg) {
        float n = (float) Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        float ax = axis[0] / n, ay = axis[1] / n, az = axis[2] / n;
        double h = Math.toRadians(deg) / 2;
        float s = (float) Math.sin(h), c = (float) Math.cos(h);
        return new float[]{c, ax * s, ay * s, az * s};
    }

    private static String fmt4(float[] q) {
        return String.format(Locale.US, "%.3f,%.3f,%.3f,%.3f", q[0], q[1], q[2], q[3]);
    }

    private static void report(String what, boolean ok, String detail) {
        if (!ok) allOk = false;
        System.out.printf(Locale.US, "   [%s] %-58s %s%n", ok ? "PASS" : "FAIL", what, detail);
    }
}
