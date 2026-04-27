#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include "stm32f4xx_hal.h"
#include "inertial_sensor_backend.h"  /* for IMU_Sample_t */

/* ─────────────────────────────────────────────────────────────────
 *  AHRS — Attitude and Heading Reference System
 *
 *  This is the equivalent of ArduPilot's AP_AHRS library.
 *
 *  Responsibility:
 *    Take in clean gyro/accel data from the IMU layer,
 *    compute and maintain the vehicle's attitude (roll & pitch).
 *
 *  Algorithm (v2.2):
 *    Complementary filter — fuses gyro integration (fast but drifts)
 *    with accelerometer angle (absolute but noisy).
 *
 *  Why a separate file?
 *    Because attitude estimation is independent of which IMU chip
 *    is used. The same complementary filter (or, later, the same
 *    EKF) works for MPU6050, ICM-42688, BMI270, etc. Keeping it
 *    separate also makes it trivial to swap the algorithm later
 *    (e.g., upgrade complementary → DCM → EKF) without touching
 *    the IMU driver code.
 *
 *  Future evolution:
 *    Replace this complementary filter with:
 *      - Mahony / Madgwick filter (better, still cheap)
 *      - DCM (ArduPilot's pre-EKF approach)
 *      - EKF (ArduPilot's current approach, requires GPS+mag+baro)
 * ─────────────────────────────────────────────────────────────── */

typedef struct {
    /* ─── Output: vehicle attitude in degrees ─── */
    float roll_deg;
    float pitch_deg;
    /* yaw_deg would go here later — needs magnetometer to be useful */

    /* ─── Filter configuration ─── */
    float complementary_alpha;   /* blend: 1.0 = all gyro, 0.0 = all accel */

    /* ─── Internal state ─── */
    uint8_t initialized;

} AHRS_t;

/* ─── Defaults ──────────────────────────────────────────────── */
#define AHRS_DEFAULT_ALPHA   0.98f   /* 98% gyro / 2% accel — standard */

/* ─── Public API ────────────────────────────────────────────── */

/* Initialize the AHRS state. Sets attitude to zero and uses the
 * default complementary filter alpha. */
void AHRS_Init(AHRS_t *ahrs);

/* Run the attitude estimator with one new IMU sample.
 *   imu : freshest sample from IS_Update() (already filtered)
 *   dt  : time since last update, in seconds
 *
 * Updates ahrs->roll_deg and ahrs->pitch_deg. */
void AHRS_Update(AHRS_t *ahrs, const IMU_Sample_t *imu, float dt);

/* Reset attitude back to zero (e.g., after a re-level on the ground). */
void AHRS_Reset(AHRS_t *ahrs);

#endif /* INC_AHRS_H_ */
