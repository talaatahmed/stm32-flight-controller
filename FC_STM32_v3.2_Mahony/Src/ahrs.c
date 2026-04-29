/* ─────────────────────────────────────────────────────────────────
 *  AHRS — Mahony Filter implementation
 *
 *  Replaces the complementary filter from v2.2.
 *
 *  Reference:
 *    Mahony et al., "Nonlinear Complementary Filters on the
 *    Special Orthogonal Group", IEEE TAC 2008.
 *    Implementation mirrors the approach used in ArduPilot's
 *    AP_AHRS_DCM (PI correction on gravity error vector).
 *
 *  Data flow each call:
 *    1. Normalize accelerometer reading
 *    2. Estimate gravity direction from current quaternion
 *    3. Compute error = cross(accel_measured, gravity_estimated)
 *    4. Apply PI correction to gyro rates
 *    5. Integrate corrected rates into quaternion
 *    6. Normalize quaternion
 *    7. Extract roll/pitch from quaternion
 * ─────────────────────────────────────────────────────────────── */

#include "ahrs.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG   57.2957795f
#define DEG_TO_RAD   0.01745329f

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Init
 * ────────────────────────────────────────────────────────────── */
void AHRS_Init(AHRS_t *ahrs)
{
    if (!ahrs) return;

    memset(ahrs, 0, sizeof(*ahrs));

    /* Identity quaternion = vehicle is level, facing forward */
    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;

    ahrs->Kp = AHRS_DEFAULT_KP;
    ahrs->Ki = AHRS_DEFAULT_KI;

    ahrs->initialized = 1;
}

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Reset — return to level, zero integral accumulators.
 * ────────────────────────────────────────────────────────────── */
void AHRS_Reset(AHRS_t *ahrs)
{
    if (!ahrs) return;

    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;

    ahrs->ix = 0.0f;
    ahrs->iy = 0.0f;
    ahrs->iz = 0.0f;

    ahrs->roll_deg  = 0.0f;
    ahrs->pitch_deg = 0.0f;
}

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Update — Mahony filter, one step.
 *
 *  Step 1 — Normalize accelerometer:
 *    The accel vector points in the direction of gravity.
 *    We only care about the direction (unit vector), not magnitude.
 *    If norm ≈ 0 (free-fall / sensor fault), skip accel correction
 *    this cycle and integrate gyro only.
 *
 *  Step 2 — Estimated gravity from quaternion:
 *    The current quaternion encodes our best estimate of orientation.
 *    We can rotate the world gravity vector [0,0,1] into body frame
 *    using the rotation matrix derived from the quaternion.
 *    The result is what the accelerometer SHOULD read if our
 *    quaternion is correct.
 *
 *  Step 3 — Error (cross product):
 *    error = cross(accel_measured, gravity_estimated)
 *    This gives a rotation axis + magnitude indicating HOW WRONG
 *    our current quaternion is, and in which direction to correct.
 *    When the filter is converged, the two vectors are parallel
 *    and the cross product → zero.
 *
 *  Step 4 — PI correction applied to gyro:
 *    Proportional term: immediate correction each step.
 *    Integral term:     accumulates over time to cancel gyro bias.
 *    Together they push the quaternion toward the accel-derived
 *    true orientation, exactly like AP_AHRS_DCM's PI loop.
 *
 *  Step 5 — Quaternion integration:
 *    q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
 *    Integrated with first-order Euler: q += q_dot * dt
 *
 *  Step 6 — Normalize:
 *    Integration introduces floating-point drift.
 *    Must renormalize every step to keep q a unit quaternion.
 *
 *  Step 7 — Euler extraction:
 *    roll  = atan2(2*(q0*q1 + q2*q3),  1 - 2*(q1² + q2²))
 *    pitch = asin (2*(q0*q2 - q3*q1))
 *    asin argument clamped to [-1, 1] to prevent NaN at ±90°.
 * ────────────────────────────────────────────────────────────── */
void AHRS_Update(AHRS_t *ahrs, const IMU_Sample_t *imu, float dt)
{
    if (!ahrs || !imu || !ahrs->initialized) return;

    /* ── Local copies of quaternion and gains ── */
    float q0 = ahrs->q[0];
    float q1 = ahrs->q[1];
    float q2 = ahrs->q[2];
    float q3 = ahrs->q[3];

    /* ── Convert gyro to rad/s ── */
    float gx = imu->gyro_x_dps * DEG_TO_RAD;
    float gy = imu->gyro_y_dps * DEG_TO_RAD;
    float gz = imu->gyro_z_dps * DEG_TO_RAD;

    float ax = imu->accel_x_g;
    float ay = imu->accel_y_g;
    float az = imu->accel_z_g;

    /* ── Steps 1–4: Accelerometer correction (skipped if invalid) ── */
    float norm = sqrtf(ax*ax + ay*ay + az*az);

    if (norm >= 0.001f) {

        /* Step 1: normalize to unit vector */
        float inv_norm = 1.0f / norm;
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        /* Step 2: estimated gravity direction from quaternion
         *
         * This is the third column of the rotation matrix R(q),
         * which maps world Z = [0,0,1] into the body frame.
         *
         *   vx = 2*(q1*q3 - q0*q2)
         *   vy = 2*(q0*q1 + q2*q3)
         *   vz = q0² - q1² - q2² + q3²
         *
         * When q=[1,0,0,0]: vx=0, vy=0, vz=1 → matches flat accel ✓ */
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* Step 3: error = cross(accel_measured, gravity_estimated) */
        float ex = ay*vz - az*vy;
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx;

        /* Step 4a: integral term — accumulates gyro bias estimate */
        ahrs->ix += ahrs->Ki * ex * dt;
        ahrs->iy += ahrs->Ki * ey * dt;
        ahrs->iz += ahrs->Ki * ez * dt;

        /* Step 4b: apply PI correction to gyro rates */
        gx += ahrs->Kp * ex + ahrs->ix;
        gy += ahrs->Kp * ey + ahrs->iy;
        gz += ahrs->Kp * ez + ahrs->iz;
    }

    /* ── Step 5: integrate quaternion (first-order Euler) ──
     *
     * q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
     *
     * Save qa, qb, qc first — we need the OLD values for all
     * four updates (using already-updated q0 would corrupt q1). */
    float qa = q0, qb = q1, qc = q2;

    q0 += (-qb*gx - qc*gy - q3*gz) * (0.5f * dt);
    q1 += ( qa*gx + qc*gz - q3*gy) * (0.5f * dt);
    q2 += ( qa*gy - qb*gz + q3*gx) * (0.5f * dt);
    q3 += ( qa*gz + qb*gy - qc*gx) * (0.5f * dt);

    /* ── Step 6: renormalize quaternion ── */
    float inv_qnorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    ahrs->q[0] = q0 * inv_qnorm;
    ahrs->q[1] = q1 * inv_qnorm;
    ahrs->q[2] = q2 * inv_qnorm;
    ahrs->q[3] = q3 * inv_qnorm;

    /* ── Step 7: extract Euler angles from normalized quaternion ── */
    q0 = ahrs->q[0];
    q1 = ahrs->q[1];
    q2 = ahrs->q[2];
    q3 = ahrs->q[3];

    ahrs->roll_deg = atan2f(2.0f*(q0*q1 + q2*q3),
                            1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;

    /* Clamp to [-1, 1] before asinf — floating-point drift can
     * push this slightly outside range, causing NaN at ±90° pitch */
    float sinp = 2.0f*(q0*q2 - q3*q1);
    if      (sinp >  1.0f) sinp =  1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    ahrs->pitch_deg = asinf(sinp) * RAD_TO_DEG;
}
