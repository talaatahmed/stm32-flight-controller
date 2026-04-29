#ifndef INC_PID_H_
#define INC_PID_H_

/* ─────────────────────────────────────────────────────────────────
 *  PID — Rate controller (inner loop)
 *
 *  Mirrors ArduPilot's AC_PID (libraries/AC_PID/AC_PID.h).
 *
 *  Change from v1:
 *    Added D-term low-pass filter (d_lpf_alpha + filtered_derivative)
 *    to match AC_PID's FLTD parameter (default 20Hz in ArduPilot).
 *    Without this filter, Kd amplifies high-frequency gyro noise.
 * ─────────────────────────────────────────────────────────────── */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;

    /* D-term low-pass filter — matches AC_PID FLTD */
    float d_lpf_alpha;          /* IIR alpha (0=no pass, 1=no filter) */
    float filtered_derivative;  /* filter state — persists between calls */

    float integral_limit;
    float output_limit;
} PID_t;

void  PID_Init  (PID_t *pid,
                 float Kp, float Ki, float Kd,
                 float d_lpf_alpha,
                 float integral_limit,
                 float output_limit);

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt);

void  PID_Reset (PID_t *pid);

#endif /* INC_PID_H_ */
