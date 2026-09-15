package com.xman.xrealbeam;

/**
 * Turns the glasses' IMU quaternion into an OpenGL view matrix for a virtual screen.
 *
 * Three things make this non-obvious, and all three cost debugging time on 2026-09-14:
 *
 * 1. The Air's sensor frame is not a viewer frame. Measured on-device: lying flat on a desk puts
 *    gravity on the **+y** body axis, so the mount (body -> viewer) is not identity. The viewer
 *    frame used here is canonical OpenGL: +x right, +y up, +z backward (look direction = -z).
 *
 * 2. The mount must be applied as a CHANGE OF BASIS, q_view = q_mount * q_rel * conj(q_mount).
 *    A composition (q_mount * q_rel) stretches motion instead of rotating it.
 *
 * 3. **A mount that is wrong only by a permutation of axes is worse than an obviously wrong one**,
 *    because every motion still looks like clean head motion - just on the wrong axis. That is how
 *    this project spent a day: the wearer reported "nodding makes it yaw, tilting my head moves the
 *    frame up and down" and the app insisted on a "measured" MOUNT(90,0,90).
 *
 *    How the mount was *finally* established, because the wrong turn is worth recording: an earlier
 *    build shipped a mount derived by working backwards from that symptom sentence over the 24 proper
 *    axis-aligned mounts. That derivation silently assumed which mount the app had been using when
 *    the wearer saw the symptom - and with that prior state unknown, **every one of the 24 candidates
 *    explains the sentence**, so the answer was really just a guess dressed in algebra. It came out
 *    90 deg rolled (up = body +x), passed the off-device harness (which only checked that the pipeline
 *    agreed with its own assumption), and was falsified in about a minute on the head.
 *
 *    What is actually load-bearing is a direct measurement: the accelerometer reports the world up
 *    with a physical sign, so two still poses - level, then head tilted back - measure the viewer's
 *    up and forward rows outright ({@link MountCal}), and the wearer's eyes confirm it by checking the
 *    *shape* of each motion: yaw slides horizontally, pitch slides vertically, roll spins in place.
 *    "All three shapes right" is the acceptance test, and it is one sentence to report.
 *
 * Sign ambiguity: the four candidate mounts differ only in sign (each is the others conjugated by a
 * 180 deg rotation). Gravity fixes the axes; a single-axis sign flip is NOT physically realizable
 * (it is a reflection, not a rotation - the old "yaw sign" toggle was one, which is why it flipped
 * pitch and left yaw alone). So the sign choice is exposed as four *proper* variants, and
 * {@link MountCal} - which measures the up and forward axes from gravity in two still poses -
 * resolves it outright rather than by trial.
 *
 * "Level" defines the reference head orientation: everything rendered is relative to it, so the
 * screen starts centred in front of the wearer regardless of how the glasses sat at startup.
 *
 * Modes: world-lock (camera rotation = full head rotation, screen stays put in space) and
 * smooth-follow (the same rotation low-passed, ~0.4 s, so the screen lags and stays near centre).
 */
public class HeadPose {

    /**
     * Body -> viewer mount MEASURED on the wearer's head (Note 20 Ultra + XREAL Air, 2026-09-14).
     * Rows (viewer axes in body coords): x (right) -> body -x, y (up) -> body +z, z (back) -> body +y
     * - a 180 deg rotation about body (0,1,1)/sqrt(2), quaternion (0, 0, 0.707107, 0.707107).
     *
     * Raw inputs: still-pose accel up = (-0.004, 0.029, 1.000) g and forward =
     * (-0.039, -0.999, 0.029) g, i.e. within 2.8 deg of this clean axis alignment (the residual is
     * how level the wearer's head was, not the hardware). Confirmed on the head by the shape test
     * (yaw slides sideways, pitch slides vertically, roll spins) and reachable with the manual step
     * buttons at mount X 90 / Y 180. An earlier default of (0.5, 0.5, 0.5, 0.5) was 90 deg rolled -
     * see the class comment for why the derivation that produced it was not evidence.
     */
    public static final float[] DEFAULT_MOUNT = {0f, 0f, 0.70710678f, 0.70710678f};

    public static final String SOURCE_DEFAULT = "default";
    public static final String SOURCE_MEASURED = "autocal";
    public static final String SOURCE_STEPS = "steps";

    /** The four sign variants. v1 is the mirror (180 deg about the viewer's up axis) — see below. */
    public static final String[] VARIANT_NAMES = {
            "v0 none", "v1 flip pitch+roll", "v2 flip yaw+roll", "v3 flip yaw+pitch"};

    // ---- mount state (written from the UI thread, read from the render thread) ----
    public volatile int mountStepsX, mountStepsY, mountStepsZ;   // manual ×90° steps
    public volatile String mountSource = SOURCE_DEFAULT;
    public volatile int variant;
    private volatile float[] baseMount = DEFAULT_MOUNT.clone();
    private volatile float[][] variants = variantsOf(DEFAULT_MOUNT);

    public volatile boolean smoothFollow;

    /** Install a mount measured on-device (body -> viewer, unit quaternion). */
    public void setMountMeasured(float w, float x, float y, float z) {
        baseMount = normalize(new float[]{w, x, y, z});
        variants = variantsOf(baseMount);
        mountSource = SOURCE_MEASURED;
        variant = 0;
    }

    /** Build a mount from the manual ×90° step buttons (escape hatch, not the main path). */
    public void applySteps() {
        float[] q = {1f, 0f, 0f, 0f};
        q = qmul(q, stepQ(0, mountStepsX));
        q = qmul(q, stepQ(1, mountStepsY));
        q = qmul(q, stepQ(2, mountStepsZ));
        baseMount = normalize(q);
        variants = variantsOf(baseMount);
        mountSource = SOURCE_STEPS;
        variant = 0;
    }

    public void stepMount(int axis) {
        if (axis == 0) mountStepsX++;
        else if (axis == 1) mountStepsY++;
        else mountStepsZ++;
        applySteps();
    }

    /** Back to the measured/derived default mount — the escape hatch from an unknown state. */
    public void resetMount() {
        mountStepsX = 1;
        mountStepsY = 0;
        mountStepsZ = 1;
        baseMount = DEFAULT_MOUNT.clone();
        variants = variantsOf(baseMount);
        mountSource = SOURCE_DEFAULT;
        variant = 0;
    }

    public void nextVariant() {
        variant = (variant + 1) % 4;
    }

    /** The mount quaternion actually in use (body -> viewer). */
    public float[] mountQuat() {
        float[] v = variants[((variant % 4) + 4) % 4];
        return new float[]{v[0], v[1], v[2], v[3]};
    }

    public String variantName() {
        return VARIANT_NAMES[((variant % 4) + 4) % 4];
    }

    /**
     * The four proper sign combinations: the base mount conjugated by 0/180 deg about x, y, z.
     * Each variant flips exactly two axes (a single-axis flip is a reflection, not a rotation).
     * v1 (180 deg about the viewer's up axis) is the one the gravity calibration can land on if the
     * wearer tilts their head the wrong way, so it comes first: one press of the flip button fixes
     * a mirrored calibration, and the remaining two cover the other axis pairs.
     */
    private static float[][] variantsOf(float[] base) {
        float[][] out = new float[4][];
        out[0] = base.clone();
        out[1] = normalize(qmul(axisQuat(1, 180), base));   // flips pitch + roll (the "mirror")
        out[2] = normalize(qmul(axisQuat(0, 180), base));   // flips yaw + roll
        out[3] = normalize(qmul(axisQuat(2, 180), base));   // flips yaw + pitch
        return out;
    }

    private float refW = 1f, refX, refY, refZ;      // reference (level) orientation
    private volatile boolean haveRef;

    // current filtered relative quaternion, in the viewer frame
    private float cw = 1f, cx, cy, cz;
    private volatile boolean haveCanon;

    private float fps;
    private long frameCount, fpsWindowStart;

    /** Capture the current head orientation as "forward". */
    public void level(float qw, float qx, float qy, float qz) {
        refW = qw; refX = qx; refY = qy; refZ = qz;
        haveRef = true;
        haveCanon = false;
    }

    public boolean isLeveled() {
        return haveRef;
    }

    public float fps() {
        return fps;
    }

    /** Feed one IMU sample. Call at render rate; the smoothing is time-constant based. */
    public void update(float qw, float qx, float qy, float qz, double dt) {
        if (!haveRef) {
            level(qw, qx, qy, qz);
        }

        // q_rel = conj(q_ref) * q_cur  (rotation from the leveled pose to the current pose,
        // expressed in the body frame — which is exactly the frame the mount is defined in).
        float rw = refW * qw + refX * qx + refY * qy + refZ * qz;
        float rx = refW * qx - refX * qw - refY * qz + refZ * qy;
        float ry = refW * qy + refX * qz - refY * qw - refZ * qx;
        float rz = refW * qz - refX * qy + refY * qx - refZ * qw;

        float[] v = variants[((variant % 4) + 4) % 4];
        float mw = v[0], mx = v[1], my = v[2], mz = v[3];

        // Change of basis: q_view = q_mount * q_rel * conj(q_mount)
        float[] canon = qmul(new float[]{mw, mx, my, mz},
                qmul(new float[]{rw, rx, ry, rz}, new float[]{mw, -mx, -my, -mz}));
        float cwN = canon[0], cxN = canon[1], cyN = canon[2], czN = canon[3];

        if (!haveCanon) {
            cw = cwN; cx = cxN; cy = cyN; cz = czN;
            haveCanon = true;
        } else if (smoothFollow) {
            float alpha = dt > 0 ? (float) Math.min(1.0, dt / 0.4) : 0.5f;
            cw += (cwN - cw) * alpha;
            cx += (cxN - cx) * alpha;
            cy += (cyN - cy) * alpha;
            cz += (czN - cz) * alpha;
            float n = (float) Math.sqrt(cw * cw + cx * cx + cy * cy + cz * cz);
            if (n > 0) { cw /= n; cx /= n; cy /= n; cz /= n; }
        } else {
            cw = cwN; cx = cxN; cy = cyN; cz = czN;   // world-lock: no lag
        }

        long now = System.currentTimeMillis();
        frameCount++;
        if (fpsWindowStart == 0) fpsWindowStart = now;
        if (now - fpsWindowStart >= 1000) {
            fps = frameCount * 1000f / (now - fpsWindowStart);
            frameCount = 0;
            fpsWindowStart = now;
        }
    }

    /**
     * View matrix (column-major, OpenGL) = R(camera)^T. The model matrix places the screen at -Z.
     * The camera rotates *with* the head against the world, so a world-locked screen slides the
     * opposite way as the wearer turns — turn left and the screen must slide right.
     */
    public void getViewMatrix(float[] m) {
        float w = cw, x = cx, y = cy, z = cz;
        float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        m[0] = 1 - 2 * (yy + zz); m[4] = 2 * (xy + wz);     m[8] = 2 * (xz - wy);      m[12] = 0;
        m[1] = 2 * (xy - wz);     m[5] = 1 - 2 * (xx + zz); m[9] = 2 * (yz + wx);      m[13] = 0;
        m[2] = 2 * (xz + wy);     m[6] = 2 * (yz - wx);     m[10] = 1 - 2 * (xx + yy); m[14] = 0;
        m[3] = 0;                 m[7] = 0;                 m[11] = 0;                 m[15] = 1;
    }

    public String describe() {
        String mount = SOURCE_STEPS.equals(mountSource)
                ? "steps(" + mountStepsX * 90 + "," + mountStepsY * 90 + "," + mountStepsZ * 90 + ")"
                : mountSource;
        return "MOUNT " + mount + " " + variantName()
                + (smoothFollow ? " follow" : " world-lock");
    }

    /** The camera rotation currently being rendered (viewer frame) — for diagnostics and tests. */
    public float[] renderedQuat() {
        return new float[]{cw, cx, cy, cz};
    }

    /** Yaw / pitch / roll of the rendered camera rotation, in degrees (Y-X-Z convention). */
    public float[] renderedYpr() {
        float w = cw, x = cx, y = cy, z = cz;
        float r02 = 2 * (x * z + w * y);
        float r12 = 2 * (y * z - w * x);
        float r22 = 1 - 2 * (x * x + y * y);
        float r01 = 2 * (x * y - w * z);
        float r11 = 1 - 2 * (x * x + z * z);
        return new float[]{
                (float) Math.toDegrees(Math.atan2(r02, r22)),
                (float) Math.toDegrees(Math.asin(Math.max(-1f, Math.min(1f, -r12)))),
                (float) Math.toDegrees(Math.atan2(-r01, r11))};
    }

    // ---------------- quaternion helpers ----------------

    private static float[] stepQ(int axis, int steps) {
        return axisQuat(axis, 90.0 * (((steps % 4) + 4) % 4));
    }

    private static float[] axisQuat(int axis, double deg) {
        double s = ((deg % 360) + 360) % 360;
        if (s == 0) return new float[]{1, 0, 0, 0};
        double a = Math.toRadians(s) / 2.0;
        float c = (float) Math.cos(a), sn = (float) Math.sin(a);
        switch (axis) {
            case 0: return new float[]{c, sn, 0, 0};
            case 1: return new float[]{c, 0, sn, 0};
            default: return new float[]{c, 0, 0, sn};
        }
    }

    private static float[] qmul(float[] a, float[] b) {
        return qmul(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
    }

    private static float[] qmul(float aw, float ax, float ay, float az,
                                float bw, float bx, float by, float bz) {
        return new float[]{
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw};
    }

    private static float[] normalize(float[] q) {
        float n = (float) Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n < 1e-9f) return new float[]{1, 0, 0, 0};
        return new float[]{q[0] / n, q[1] / n, q[2] / n, q[3] / n};
    }
}
