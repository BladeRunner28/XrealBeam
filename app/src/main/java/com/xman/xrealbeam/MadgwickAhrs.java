package com.xman.xrealbeam;

/**
 * Madgwick AHRS — IMU-only (6-axis: gyroscope + accelerometer) variant.
 *
 * This is a Java port of the published reference implementation, Sebastian Madgwick's
 * open-source AHRS algorithm (x-io Technologies, MadgwickAHRS.c, IMU update), NOT a hand-rolled
 * integrator: the accelerometer provides the gravity reference that keeps roll/pitch bounded, and
 * the gradient-descent step is what stops the quaternion from diverging.
 *
 * Deliberate limitation: with no usable magnetometer there is no absolute yaw reference, so yaw
 * drifts at whatever the residual gyro-bias error is. That drift is the number the project needs
 * measured, not hidden — hence the drift readout in the host app. Porting a magnetometer path for
 * the Air is not worth it: the reference driver treats its magnetic fields as unreliable
 * ("bizarre" byte order, swapped mult/div), and the SDK itself documents the Air as 3DoF
 * rotation-only.
 *
 * Units: gyro in deg/s (as the device reports), accelerometer in g, dt in seconds.
 */
public class MadgwickAhrs {

    private float q0 = 1f, q1 = 0f, q2 = 0f, q3 = 0f;
    private final float beta;          // filter gain: 2*proportional gain on the measured error
    private long samples;

    public MadgwickAhrs() {
        this(0.1f);
    }

    public MadgwickAhrs(float beta) {
        this.beta = beta;
    }

    public void reset() {
        q0 = 1f; q1 = 0f; q2 = 0f; q3 = 0f;
        samples = 0;
    }

    /** Madgwick IMU update. dt <= 0 is ignored (first sample has no interval). */
    public void update(double gxDps, double gyDps, double gzDps,
                       double axG, double ayG, double azG, double dt) {
        if (dt <= 0 || dt > 1.0) return;

        final float DEG2RAD = (float) (Math.PI / 180.0);
        float gx = (float) (gxDps * DEG2RAD);
        float gy = (float) (gyDps * DEG2RAD);
        float gz = (float) (gzDps * DEG2RAD);

        float ax = (float) axG, ay = (float) ayG, az = (float) azG;

        // Rate of change of quaternion from gyroscope.
        float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        final float norm = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm > 0.0001f) {
            float recipNorm = 1f / norm;
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            // Gradient step: objective function J^T * f, from the reference implementation.
            float _2q0 = 2f * q0, _2q1 = 2f * q1, _2q2 = 2f * q2, _2q3 = 2f * q3;
            float _4q0 = 4f * q0, _4q1 = 4f * q1, _4q2 = 4f * q2;
            float _8q1 = 8f * q1, _8q2 = 8f * q2;
            float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

            float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            float s1 = _4q1 * q3q3 - _2q3 * ax + 4f * q0q0 * q1 - _2q0 * ay
                    - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            float s2 = 4f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                    - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            float s3 = 4f * q1q1 * q3 - _2q1 * ax + 4f * q2q2 * q3 - _2q2 * ay;

            float sNorm = (float) Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            if (sNorm > 0.0001f) {
                qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0 / sNorm;
                qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1 / sNorm;
                qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2 / sNorm;
                qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3 / sNorm;
            }
        }

        q0 += qDot1 * (float) dt;
        q1 += qDot2 * (float) dt;
        q2 += qDot3 * (float) dt;
        q3 += qDot4 * (float) dt;

        float recipNorm = 1f / (float) Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
        samples++;
    }

    public long samples() { return samples; }
    public float q0() { return q0; }
    public float q1() { return q1; }
    public float q2() { return q2; }
    public float q3() { return q3; }

    public float rollDeg() {
        return (float) Math.toDegrees(Math.atan2(2f * (q0 * q1 + q2 * q3),
                1f - 2f * (q1 * q1 + q2 * q2)));
    }

    public float pitchDeg() {
        float v = 2f * (q0 * q2 - q3 * q1);
        v = Math.max(-1f, Math.min(1f, v));
        return (float) Math.toDegrees(Math.asin(v));
    }

    public float yawDeg() {
        return (float) Math.toDegrees(Math.atan2(2f * (q0 * q3 + q1 * q2),
                1f - 2f * (q2 * q2 + q3 * q3)));
    }

    /** Rotation of v by this quaternion's conjugate — used to get orientation relative to a reference. */
    public float relativeYawDeg(float refW, float refX, float refY, float refZ) {
        // q_rel = conj(q_ref) * q_cur
        float aw = refW, ax = -refX, ay = -refY, az = -refZ;   // conjugate of reference
        float bw = q0, bx = q1, by = q2, bz = q3;
        float rw = aw * bw - ax * bx - ay * by - az * bz;
        float rx = aw * bx + ax * bw + ay * bz - az * by;
        float ry = aw * by - ax * bz + ay * bw + az * bx;
        float rz = aw * bz + ax * by - ay * bx + az * bw;
        return (float) Math.toDegrees(Math.atan2(2f * (rw * rz + rx * ry),
                1f - 2f * (ry * ry + rz * rz)));
    }
}
