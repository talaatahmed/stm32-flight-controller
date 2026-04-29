#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include "stm32f4xx_hal.h"
#include "inertial_sensor_backend.h"

/* ─────────────────────────────────────────────────────────────────
 *  AHRS — Mahony Filter (replaces complementary filter v2.2)
 *
 *  Algorithm:
 *    Maintains a unit quaternion representing vehicle orientation.
 *    Uses PI feedback from accelerometer cross-product error to
 *    correct gyro drift — the same principle as AP_AHRS_DCM
 *    but operating in quaternion space (no gimbal lock).
 *
 *  Advantages over complementary filter:
 *    - No gimbal lock at ±90° pitch
 *    - PI feedback adapts correction strength (vs fixed alpha blend)
 *    - Numerically stable at large angles
 *
 *  Public API is UNCHANGED — main.c requires no modifications.
 * ─────────────────────────────────────────────────────────────── */

typedef struct {

    /* ─── Output: attitude in degrees ─────────────────────────── */
    float roll_deg;
    float pitch_deg;

    /* ─── Mahony filter state ──────────────────────────────────
     * Unit quaternion representing current orientation.
     * q[0] = scalar (w), q[1]=x, q[2]=y, q[3]=z
     * Initialized to [1, 0, 0, 0] = identity = level. */
    float q[4];

    /* Integral feedback accumulators (one per axis).
     * Accumulate the gyro bias estimate over time.
     * Reset to zero on AHRS_Reset(). */
    float ix, iy, iz;

    /* ─── Gains ────────────────────────────────────────────────
     * Kp: proportional gain — how aggressively to correct gyro
     *     from the accelerometer error each step.
     *     Higher Kp → faster convergence, but more accel noise.
     *
     * Ki: integral gain — how fast to estimate and remove gyro bias.
     *     Set to 0 to disable bias correction (pure proportional). */
    float Kp;
    float Ki;

    /* ─── Internal ─────────────────────────────────────────────── */
    uint8_t initialized;

} AHRS_t;

/* ─── Default gains ─────────────────────────────────────────── */
#define AHRS_DEFAULT_KP   2.0f      /* standard starting point */
#define AHRS_DEFAULT_KI   0.005f    /* light integral correction */

/* ─── Public API (unchanged from v2.2) ─────────────────────── */
void AHRS_Init  (AHRS_t *ahrs);
void AHRS_Update(AHRS_t *ahrs, const IMU_Sample_t *imu, float dt);
void AHRS_Reset (AHRS_t *ahrs);

#endif /* INC_AHRS_H_ */
