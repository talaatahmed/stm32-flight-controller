/* ─────────────────────────────────────────────────────────────────
 *  PID — implementation
 *  Mirrors ArduPilot's AC_PID (libraries/AC_PID/AC_PID.cpp)
 *
 *  Key change from v1:
 *    D-term passes through a first-order IIR before Kd is applied.
 *    Matches AC_PID's FLTD (D-term filter frequency) parameter.
 * ─────────────────────────────────────────────────────────────── */

#include "pid.h"

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float d_lpf_alpha,
              float integral_limit,
              float output_limit)
{
    pid->Kp                  = Kp;
    pid->Ki                  = Ki;
    pid->Kd                  = Kd;
    pid->integral            = 0.0f;
    pid->prev_error          = 0.0f;
    pid->d_lpf_alpha         = d_lpf_alpha;
    pid->filtered_derivative = 0.0f;
    pid->integral_limit      = integral_limit;
    pid->output_limit        = output_limit;
}

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    float error = setpoint - measurement;

    /* ── Proportional ── */
    float P = pid->Kp * error;

    /* ── Integral with anti-windup ── */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    /* ── Derivative with low-pass filter (matches AC_PID FLTD) ──
     *
     * Raw derivative is noisy — Kd amplifies high-frequency content.
     * ArduPilot filters it with a first-order IIR at FLTD Hz before
     * applying Kd. We use alpha=0.557 → 20Hz cutoff at 100Hz rate.
     *
     *   raw   = (error - prev_error) / dt
     *   filt  = alpha * raw + (1-alpha) * prev_filt
     *   D     = Kd * filt                               */
    float raw_deriv           = (error - pid->prev_error) / dt;
    pid->filtered_derivative  = pid->d_lpf_alpha * raw_deriv
                              + (1.0f - pid->d_lpf_alpha) * pid->filtered_derivative;
    pid->prev_error           = error;
    float D                   = pid->Kd * pid->filtered_derivative;

    /* ── Sum and clamp ── */
    float output = P + I + D;
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral            = 0.0f;
    pid->prev_error          = 0.0f;
    pid->filtered_derivative = 0.0f;
}
