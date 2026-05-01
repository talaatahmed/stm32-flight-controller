/* ─────────────────────────────────────────────────────────────────
 *  AC_P — implementation
 *  Mirrors ArduPilot's AC_P (libraries/AC_PID/AC_P.cpp)
 * ─────────────────────────────────────────────────────────────── */
#include "ac_p_controller.h"

/* Angle errors smaller than this are treated as zero.
 * Prevents tiny sensor noise from generating rate commands.
 * ArduPilot equivalent: input shaping dead zone. */
#define ANGLE_DEADBAND_DEG   0.5f

void AC_P_Init(AC_P_t *p, float kP, float rate_max_dps)
{
    p->kP           = kP;
    p->rate_max_dps = rate_max_dps;
}

float AC_P_Update_old(AC_P_t *p, float error_deg)
{
    float output = p->kP * error_deg;

    if (p->rate_max_dps > 0.0f) {
        if      (output >  p->rate_max_dps) output =  p->rate_max_dps;
        else if (output < -p->rate_max_dps) output = -p->rate_max_dps;
    }

    return output;
}

float AC_P_Update(AC_P_t *p, float error_deg)
{
    /* Apply deadband — zero out small errors */
    if (error_deg >  ANGLE_DEADBAND_DEG) error_deg -= ANGLE_DEADBAND_DEG;
    else if (error_deg < -ANGLE_DEADBAND_DEG) error_deg += ANGLE_DEADBAND_DEG;
    else error_deg = 0.0f;

    float output = p->kP * error_deg;

    if (p->rate_max_dps > 0.0f) {
        if      (output >  p->rate_max_dps) output =  p->rate_max_dps;
        else if (output < -p->rate_max_dps) output = -p->rate_max_dps;
    }

    return output;
}
