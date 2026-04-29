/* ─────────────────────────────────────────────────────────────────
 *  AttitudeControl — v2 implementation
 *
 *  v2 additions vs v1:
 *
 *  1. THROTTLE SCALING
 *     Mirrors AC_AttitudeControl_Multi::rate_controller_run().
 *
 *     Problem: at low throttle, motors spin slowly → less airflow →
 *     less control authority per PWM tick. The same PID output
 *     that corrects 5° at hover does almost nothing at 20% throttle.
 *
 *     Fix: scale PID output by throttle before sending to Motor_Mix.
 *       scaled_output = raw_output * (throttle / hover_throttle)
 *       clamped to [-1, +1]
 *
 *     hover_throttle = 0.5 (assumes hover at 50% — tune for your build)
 *     At hover:     scale = 1.0 → no change
 *     Below hover:  scale < 1.0 → less authority (physically correct)
 *     Above hover:  scale > 1.0 → more authority (motors spinning fast)
 *
 *  2. YAW RATE CONTROL
 *     Mirrors AC_AttitudeControl_Multi inner loop for yaw.
 *
 *     No outer angle P for yaw — without a magnetometer we cannot
 *     measure absolute heading, so we cannot close an angle loop.
 *     Instead we run a rate-only inner loop:
 *       yaw_rate_error = yaw_rate_target - gyro_z
 *       out_yaw = PID(yaw_rate_error)
 *
 *     At rest, yaw_rate_target = 0 → controller holds zero yaw rate.
 *     This prevents heading drift from gyro bias (within Ki capability).
 *
 *  Yaw PID gains (ArduPilot ATC_RAT_YAW defaults):
 *     Kp  = 0.180
 *     Ki  = 0.018
 *     Kd  = 0.000   (yaw has no D — too noisy, no benefit)
 *     FLTD alpha = 0 (no D filter needed when Kd=0)
 * ─────────────────────────────────────────────────────────────── */

#include "attitude_control.h"

/* ── Throttle scaling ── */
#define HOVER_THROTTLE          0.5f   /* tune: throttle at which quad hovers */
#define THROTTLE_SCALE_MIN      0.1f   /* never scale below this              */

/* ── Inner loop filter + limits ── */
#define RATE_D_LPF_ALPHA        0.557f   /* 20Hz @ 100Hz */
#define RATE_INTEGRAL_LIMIT     0.3f
#define RATE_OUTPUT_LIMIT       1.0f

#define YAW_INTEGRAL_LIMIT      0.3f
#define YAW_OUTPUT_LIMIT        1.0f

/* ── Rate deadband ── */
#define RATE_DEADBAND_DPS       2.0f

/* ────────────────────────────────────────────────────────────────
 *  AttCtrl_Init
 * ────────────────────────────────────────────────────────────── */
void AttCtrl_Init(AttitudeControl_t *ac)
{
    /* Outer loop */
    AC_P_Init(&ac->angle_roll_p,  4.5f, 200.0f);
    AC_P_Init(&ac->angle_pitch_p, 4.5f, 200.0f);

    /* Inner loop — roll & pitch */
    PID_Init(&ac->rate_roll_pid,
             0.135f, 0.135f, 0.0036f,
             RATE_D_LPF_ALPHA,
             RATE_INTEGRAL_LIMIT, RATE_OUTPUT_LIMIT);

    PID_Init(&ac->rate_pitch_pid,
             0.135f, 0.135f, 0.0036f,
             RATE_D_LPF_ALPHA,
             RATE_INTEGRAL_LIMIT, RATE_OUTPUT_LIMIT);

    /* Inner loop — yaw (rate only, no D) */
    PID_Init(&ac->rate_yaw_pid,
             0.180f, 0.018f, 0.000f,
             0.0f,                    /* no D filter needed */
             YAW_INTEGRAL_LIMIT, YAW_OUTPUT_LIMIT);

    ac->roll_target_deg      = 0.0f;
    ac->pitch_target_deg     = 0.0f;
    ac->yaw_rate_target_dps  = 0.0f;
}

/* ────────────────────────────────────────────────────────────────
 *  AttCtrl_SetTarget
 * ────────────────────────────────────────────────────────────── */
void AttCtrl_SetTarget(AttitudeControl_t *ac,
                       float roll_deg,
                       float pitch_deg,
                       float yaw_rate_dps)
{
    ac->roll_target_deg     = roll_deg;
    ac->pitch_target_deg    = pitch_deg;
    ac->yaw_rate_target_dps = yaw_rate_dps;
}

/* ────────────────────────────────────────────────────────────────
 *  AttCtrl_Update
 * ────────────────────────────────────────────────────────────── */
void AttCtrl_Update(AttitudeControl_t  *ac,
                    const AHRS_t       *ahrs,
                    const IMU_Sample_t *imu,
                    float               dt,
                    float               throttle,
                    float              *out_roll,
                    float              *out_pitch,
                    float              *out_yaw)
{
    /* ── Outer loop: angle error → rate setpoint ─────────────── */
    float roll_rate_target  = AC_P_Update(&ac->angle_roll_p,
                                 ac->roll_target_deg  - ahrs->roll_deg);
    float pitch_rate_target = AC_P_Update(&ac->angle_pitch_p,
                                 ac->pitch_target_deg - ahrs->pitch_deg);

    /* Reset integrators when in deadband (relax_attitude_controllers) */
    if (roll_rate_target  == 0.0f) ac->rate_roll_pid.integral  = 0.0f;
    if (pitch_rate_target == 0.0f) ac->rate_pitch_pid.integral = 0.0f;

    /* ── Rate deadband on gyro measurement ───────────────────── */
    float gyro_x = imu->gyro_x_dps;
    float gyro_y = imu->gyro_y_dps;
    float gyro_z = imu->gyro_z_dps;

    if      (gyro_x >  RATE_DEADBAND_DPS) gyro_x -= RATE_DEADBAND_DPS;
    else if (gyro_x < -RATE_DEADBAND_DPS) gyro_x += RATE_DEADBAND_DPS;
    else     gyro_x = 0.0f;

    if      (gyro_y >  RATE_DEADBAND_DPS) gyro_y -= RATE_DEADBAND_DPS;
    else if (gyro_y < -RATE_DEADBAND_DPS) gyro_y += RATE_DEADBAND_DPS;
    else     gyro_y = 0.0f;

    if      (gyro_z >  RATE_DEADBAND_DPS) gyro_z -= RATE_DEADBAND_DPS;
    else if (gyro_z < -RATE_DEADBAND_DPS) gyro_z += RATE_DEADBAND_DPS;
    else     gyro_z = 0.0f;

    /* ── Inner loop: rate error → raw output ────────────────── */
    float raw_roll  = PID_Update(&ac->rate_roll_pid,
                                  roll_rate_target, gyro_x, dt);
    float raw_pitch = PID_Update(&ac->rate_pitch_pid,
                                  pitch_rate_target, gyro_y, dt);
    float raw_yaw   = PID_Update(&ac->rate_yaw_pid,
                                  ac->yaw_rate_target_dps, gyro_z, dt);

    /* ── Throttle scaling ────────────────────────────────────────
     *
     * scale = throttle / hover_throttle
     *
     * At hover (throttle=0.5): scale=1.0 → gains unchanged
     * At low   (throttle=0.2): scale=0.4 → reduced authority
     *                          (correct — motors are slow)
     * At high  (throttle=0.8): scale=1.6 → increased authority
     *                          (correct — motors are fast)
     *
     * Clamped to THROTTLE_SCALE_MIN to prevent near-zero scaling
     * from making the controller unresponsive on the bench. */
    float scale = throttle / HOVER_THROTTLE;
    if (scale < THROTTLE_SCALE_MIN) scale = THROTTLE_SCALE_MIN;

    /* Apply scale and clamp to [-1, +1] */
    float sr = raw_roll  * scale;
    float sp = raw_pitch * scale;
    float sy = raw_yaw   * scale;

    if      (sr >  1.0f) sr =  1.0f;
    else if (sr < -1.0f) sr = -1.0f;
    if      (sp >  1.0f) sp =  1.0f;
    else if (sp < -1.0f) sp = -1.0f;
    if      (sy >  1.0f) sy =  1.0f;
    else if (sy < -1.0f) sy = -1.0f;

    *out_roll  = sr;
    *out_pitch = sp;
    *out_yaw   = sy;
}

/* ────────────────────────────────────────────────────────────────
 *  AttCtrl_Reset
 * ────────────────────────────────────────────────────────────── */
void AttCtrl_Reset(AttitudeControl_t *ac)
{
    PID_Reset(&ac->rate_roll_pid);
    PID_Reset(&ac->rate_pitch_pid);
    PID_Reset(&ac->rate_yaw_pid);
}
