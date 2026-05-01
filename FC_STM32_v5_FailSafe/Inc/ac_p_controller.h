#ifndef INC_AC_P_H_
#define INC_AC_P_H_

/* ─────────────────────────────────────────────────────────────────
 *  AC_P — Proportional controller (outer / angle loop)
 *
 *  Mirrors ArduPilot's AC_P (libraries/AC_PID/AC_P.h).
 *  Converts angle error (degrees) into a rate setpoint (deg/s).
 *
 *  Used as the OUTER loop of the cascade:
 *    angle_error_deg → AC_P → rate_setpoint_dps → PID (inner loop)
 * ─────────────────────────────────────────────────────────────── */

typedef struct {
    float kP;
    float rate_max_dps;   /* output clamp — 0 = unlimited */
} AC_P_t;

void  AC_P_Init  (AC_P_t *p, float kP, float rate_max_dps);
float AC_P_Update(AC_P_t *p, float error_deg);

#endif /* INC_AC_P_H_ */
