/* ─────────────────────────────────────────────────────────────────
 *  AHRS — Attitude and Heading Reference System (implementation)
 *
 *  Implements a complementary filter that fuses:
 *    - Accelerometer-derived tilt angle (absolute, noisy)
 *    - Gyroscope-integrated angle (smooth, drifts over time)
 *
 *  Result: smooth attitude that does not drift.
 *
 *  This is the same math that lived inside inertial_sensor.c in v2.1.
 *  Moved here in v2.2 to mirror ArduPilot's AP_InertialSensor /
 *  AP_AHRS separation.
 * ─────────────────────────────────────────────────────────────── */

#include "ahrs.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG  57.2957795f

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Init — set everything to safe defaults.
 * ────────────────────────────────────────────────────────────── */
void AHRS_Init(AHRS_t *ahrs)
{
    if (!ahrs) return;

    memset(ahrs, 0, sizeof(*ahrs));
    ahrs->complementary_alpha = AHRS_DEFAULT_ALPHA;
    ahrs->initialized = 1;
}

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Reset — zero the attitude back out.
 *
 *  Useful when the vehicle is known to be level on the ground and
 *  you want to clear any accumulated drift before takeoff.
 * ────────────────────────────────────────────────────────────── */
void AHRS_Reset(AHRS_t *ahrs)
{
    if (!ahrs) return;
    ahrs->roll_deg  = 0.0f;
    ahrs->pitch_deg = 0.0f;
}

/* ────────────────────────────────────────────────────────────────
 *  AHRS_Update — run the complementary filter once.
 *
 *  Math:
 *
 *  1. Accelerometer angles (absolute reference, noisy):
 *        roll_acc  = atan2(ay, az)
 *        pitch_acc = atan2(-ax, sqrt(ay² + az²))
 *
 *     These come from the gravity vector. When the vehicle is still,
 *     gravity points straight down in the body frame; tilt angles
 *     can be recovered from how the 1g gravity vector decomposes
 *     across the X/Y/Z axes.
 *
 *  2. Gyro integration (smooth, drifts):
 *        roll_gyro  = previous_roll  + gyro_x * dt
 *        pitch_gyro = previous_pitch + gyro_y * dt
 *
 *     Integrating angular rate over time gives angle. But integrators
 *     accumulate any small bias as drift, so this alone is unusable
 *     long-term.
 *
 *  3. Blend:
 *        roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc
 *        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
 *
 *     Alpha close to 1.0 (e.g. 0.98) → trust gyro for short-term,
 *     use accel only as a slow correction against drift.
 *
 *  The filter is naturally stable from a cold start: the very first
 *  call has previous_roll = 0, so roll_gyro = gyro_x * dt (tiny),
 *  and the output is dominated by (1-alpha) * roll_acc anyway.
 * ────────────────────────────────────────────────────────────── */
void AHRS_Update(AHRS_t *ahrs, const IMU_Sample_t *imu, float dt)
{
    if (!ahrs || !imu || !ahrs->initialized) return;

    const float alpha = ahrs->complementary_alpha;

    /* ─── 1. Accelerometer angle (absolute, noisy) ─── */
    float roll_acc  = atan2f(imu->accel_y_g, imu->accel_z_g) * RAD_TO_DEG;
    float pitch_acc = atan2f(-imu->accel_x_g,
                              sqrtf(imu->accel_y_g * imu->accel_y_g +
                                    imu->accel_z_g * imu->accel_z_g)) * RAD_TO_DEG;

    /* ─── 2. Gyro integration (smooth, drifts) ─── */
    float roll_gyro  = ahrs->roll_deg  + imu->gyro_x_dps * dt;
    float pitch_gyro = ahrs->pitch_deg + imu->gyro_y_dps * dt;

    /* ─── 3. Blend ─── */
    ahrs->roll_deg  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    ahrs->pitch_deg = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
}
