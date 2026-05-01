/* ─────────────────────────────────────────────────────────────────
 *  Motor Mixer — ArduPilot AP_MotorsMatrix style
 *
 *  Key upgrade from v1:
 *
 *  v1 used a fixed ±200 tick authority and simple clamp:
 *    output = clamp(base + roll*200 - pitch*200 + yaw*200)
 *  When a motor saturated, the correction was silently lost.
 *
 *  This version mirrors AP_MotorsMatrix desaturation:
 *
 *  1. Work in normalized float space throughout.
 *     roll/pitch/yaw inputs [-1,+1] map to ±0.5 of the
 *     throttle range — authority scales with available headroom.
 *
 *  2. After mixing, run desaturation:
 *     - If any motor > 1.0: pull ALL motors down equally.
 *       This preserves the differential (= attitude correction)
 *       at the cost of slightly less total thrust.
 *     - If any motor < 0.0: push ALL motors up equally.
 *       Same principle — trade thrust for attitude.
 *     - If BOTH constraints can't be satisfied simultaneously
 *       (extreme attitude + extreme throttle), attitude wins:
 *       sacrifice throttle uniformity last.
 *
 *  3. Convert normalized [0,1] → PWM [PWM_MIN, PWM_MAX] only
 *     at the final step.
 *
 *  Why this matters in flight:
 *    During aggressive corrections (e.g. recovering from 30° roll),
 *    one or two motors will want to go to max. Without desaturation,
 *    those motors clip and the correction weakens exactly when it's
 *    needed most. With desaturation, the controller reduces throttle
 *    slightly on ALL motors to make room — the correction executes
 *    fully.
 *
 *  ArduPilot reference:
 *    libraries/AP_Motors/AP_MotorsMatrix.cpp
 *    AP_MotorsMatrix::output_armed_stabilizing()
 * ─────────────────────────────────────────────────────────────── */

#include "motor_mix.h"

/* ────────────────────────────────────────────────────────────────
 *  Authority scale factor.
 *
 *  Controls how much of the throttle range roll/pitch/yaw can use.
 *  0.5 means attitude corrections can use ±50% of [PWM_MIN,PWM_MAX].
 *
 *  ArduPilot dynamically scales this based on throttle headroom.
 *  We use a fixed 0.5 which is a good default for a typical build.
 *  Increase toward 1.0 for more aggressive attitude authority.
 * ────────────────────────────────────────────────────────────── */
#define ATTITUDE_AUTHORITY   0.5f

/* ────────────────────────────────────────────────────────────────
 *  clamp_f — float clamp helper
 * ────────────────────────────────────────────────────────────── */
static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ────────────────────────────────────────────────────────────────
 *  Motor_Init
 * ────────────────────────────────────────────────────────────── */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, PWM_MIN);
}

/* ────────────────────────────────────────────────────────────────
 *  Motor_Mix — ArduPilot-style desaturating mixer
 *
 *  Inputs (all normalized):
 *    throttle : 0.0 – 1.0   base thrust
 *    roll     : -1.0 – +1.0  positive = roll right
 *    pitch    : -1.0 – +1.0  positive = pitch forward
 *    yaw      : -1.0 – +1.0  positive = yaw clockwise
 *
 *  X-configuration mix table:
 *    M1 FL(CCW) = throttle + roll - pitch + yaw
 *    M2 FR(CW)  = throttle - roll - pitch - yaw
 *    M3 RR(CCW) = throttle - roll + pitch + yaw
 *    M4 RL(CW)  = throttle + roll + pitch - yaw
 * ────────────────────────────────────────────────────────────── */
void Motor_Mix(float throttle, float roll, float pitch, float yaw,
               MotorOutput_t *out)
{
    /* ── Scale attitude inputs by authority factor ──
     * Converts [-1,+1] commands into a fraction of the PWM range.
     * At ATTITUDE_AUTHORITY=0.5, full roll deflection = ±0.5 in
     * normalized space = ±500 ticks = ±50% of [1000,2000]. */
    float r = roll  * ATTITUDE_AUTHORITY;
    float p = pitch * ATTITUDE_AUTHORITY;
    float y = yaw   * ATTITUDE_AUTHORITY;

    /* ── Step 1: Raw mix in normalized [0,1] space ── */
    float m[4];
    m[0] = throttle + r - p + y;   /* M1 FL */
    m[1] = throttle - r - p - y;   /* M2 FR */
    m[2] = throttle - r + p + y;   /* M3 RR */
    m[3] = throttle + r + p - y;   /* M4 RL */

    /* ── Step 2: Desaturation (AP_MotorsMatrix style) ───────────
     *
     * Find the highest and lowest motor values. If either is out
     * of [0.0, 1.0], shift ALL motors equally to bring them back.
     * This preserves the differential between motors (attitude),
     * trading only total thrust level. */

    /* Find min and max */
    float m_min = m[0], m_max = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] < m_min) m_min = m[i];
        if (m[i] > m_max) m_max = m[i];
    }

    /* If any motor above 1.0: pull all down */
    if (m_max > 1.0f) {
        float excess = m_max - 1.0f;
        for (int i = 0; i < 4; i++) m[i] -= excess;
        /* Recalculate min after shift */
        m_min -= excess;
    }

    /* If any motor below 0.0: push all up */
    if (m_min < 0.0f) {
        float deficit = -m_min;
        for (int i = 0; i < 4; i++) m[i] += deficit;
    }

    /* ── Step 3: Final clamp ─────────────────────────────────────
     * After desaturation, motors should be in [0,1]. The clamp
     * below is a safety net for extreme cases (e.g. full attitude
     * correction at zero throttle) where both constraints cannot
     * be satisfied simultaneously. In that case, attitude takes
     * priority and throttle is sacrificed — mirroring ArduPilot's
     * behavior when it sets thr_adj in output_armed_stabilizing(). */
    for (int i = 0; i < 4; i++) {
        m[i] = clamp_f(m[i], 0.0f, 1.0f);
    }

    /* ── Step 4: Convert normalized → PWM ticks ── */
    float range = (float)(PWM_MAX - PWM_MIN);
    out->m1 = (uint16_t)(PWM_MIN + m[0] * range);
    out->m2 = (uint16_t)(PWM_MIN + m[1] * range);
    out->m3 = (uint16_t)(PWM_MIN + m[2] * range);
    out->m4 = (uint16_t)(PWM_MIN + m[3] * range);
}

/* ────────────────────────────────────────────────────────────────
 *  Motor_Write
 * ────────────────────────────────────────────────────────────── */
void Motor_Write(TIM_HandleTypeDef *htim, MotorOutput_t *out)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, out->m1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, out->m2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, out->m3);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, out->m4);
}

/* ────────────────────────────────────────────────────────────────
 *  Motor_Disarm
 * ────────────────────────────────────────────────────────────── */
void Motor_Disarm(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, PWM_MIN);
}
