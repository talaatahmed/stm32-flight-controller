#include"pid.h"

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float integral_limit,
              float output_limit)
{
    pid->Kp             = Kp;
    pid->Ki             = Ki;
    pid->Kd             = Kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
}

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    /* ── Error ── */
    float error = setpoint - measurement;

    /* ── Proportional ── */
    float P = pid->Kp * error;

    /* ── Integral with anti-windup clamp ── */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit)
             pid->integral =  pid->integral_limit;
    else if  (pid->integral < -pid->integral_limit)
             pid->integral = -pid->integral_limit;
    float I = pid->Ki * pid->integral;

    /* ── Derivative (on measurement, not error — prevents kick) ── */
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error  = error;
    float D = pid->Kd * derivative;

    /* ── Sum and clamp output ── */
    float output = P + I + D;
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
