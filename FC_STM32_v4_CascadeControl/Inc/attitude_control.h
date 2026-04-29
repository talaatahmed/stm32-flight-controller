#ifndef INC_ATTITUDE_CONTROL_H_
#define INC_ATTITUDE_CONTROL_H_

#include "ac_p_controller.h"
#include "pid.h"
#include "ahrs.h"
#include "inertial_sensor_backend.h"

/* ─────────────────────────────────────────────────────────────────
 *  AttitudeControl — cascade angle + rate controller
 *
 *  v2 additions (mirrors ArduPilot AC_AttitudeControl_Multi):
 *    - Yaw rate control (inner loop on gyro_z)
 *    - Throttle scaling on all PID outputs
 * ─────────────────────────────────────────────────────────────── */

typedef struct {

    /* ── Outer loop (angle → rate setpoint) ── */
    AC_P_t  angle_roll_p;
    AC_P_t  angle_pitch_p;

    /* ── Inner loop (rate error → output) ── */
    PID_t   rate_roll_pid;
    PID_t   rate_pitch_pid;
    PID_t   rate_yaw_pid;     /* v2: yaw rate control */

    /* ── Targets ── */
    float   roll_target_deg;
    float   pitch_target_deg;
    float   yaw_rate_target_dps;   /* v2: commanded yaw rate */

} AttitudeControl_t;

/* Initialize with ArduPilot-equivalent default gains */
void AttCtrl_Init(AttitudeControl_t *ac);

/* Set desired attitude + yaw rate.
 *   roll_deg, pitch_deg     : angle targets (level = 0)
 *   yaw_rate_dps            : commanded yaw rate (0 = hold heading) */
void AttCtrl_SetTarget(AttitudeControl_t *ac,
                       float roll_deg,
                       float pitch_deg,
                       float yaw_rate_dps);

/* Run both loops.
 *   throttle : 0.0–1.0 — used to scale PID output authority.
 *              Mirrors AC_AttitudeControl_Multi::rate_controller_run().
 *   out_roll, out_pitch, out_yaw : [-1, +1] for Motor_Mix */
void AttCtrl_Update(AttitudeControl_t  *ac,
                    const AHRS_t       *ahrs,
                    const IMU_Sample_t *imu,
                    float               dt,
                    float               throttle,
                    float              *out_roll,
                    float              *out_pitch,
                    float              *out_yaw);

/* Reset all integrators — call on disarm */
void AttCtrl_Reset(AttitudeControl_t *ac);

#endif /* INC_ATTITUDE_CONTROL_H_ */
