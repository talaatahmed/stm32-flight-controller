#include "motor_mix.h"

/* ── Clamp helper ── */
static uint16_t clamp_pwm(float val)
{
    if (val < PWM_MIN) return PWM_MIN;
    if (val > PWM_MAX) return PWM_MAX;
    return (uint16_t)val;
}

/* ── Start all 4 PWM channels ── */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

    /* Hold at minimum — required before ESC arms */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, PWM_MIN);
}

/* ── X-configuration motor mixing ──
 *
 *  throttle : 0.0 – 1.0  (base thrust)
 *  roll     : -1.0 – +1.0  (+= right)
 *  pitch    : -1.0 – +1.0  (+= forward)
 *  yaw      : -1.0 – +1.0  (+= clockwise)
 *
 *  Layout:
 *    M1(FL,CCW)  M2(FR,CW)
 *    M4(RL,CW)   M3(RR,CCW)
 *
 *  Mix table (X config standard):
 *    M1 = throttle + roll - pitch + yaw   (FL)
 *    M2 = throttle - roll - pitch - yaw   (FR)
 *    M3 = throttle - roll + pitch + yaw   (RR)
 *    M4 = throttle + roll + pitch - yaw   (RL)
 */
void Motor_Mix(float throttle, float roll, float pitch, float yaw,
               MotorOutput_t *out)
{
    /* Scale throttle to PWM range */
    float base  = PWM_MIN + throttle * (PWM_MAX - PWM_MIN);

    /* Scale roll/pitch/yaw — authority = ±200 ticks = ±20% of range */
    float r = roll  * 200.0f;
    float p = pitch * 200.0f;
    float y = yaw   * 200.0f;

    out->m1 = clamp_pwm(base + r - p + y);
    out->m2 = clamp_pwm(base - r - p - y);
    out->m3 = clamp_pwm(base - r + p + y);
    out->m4 = clamp_pwm(base + r + p - y);
}

/* ── Write computed values to timer compare registers ── */
void Motor_Write(TIM_HandleTypeDef *htim, MotorOutput_t *out)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, out->m1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, out->m2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, out->m3);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, out->m4);
}

/* ── Safe disarm — set all to minimum ── */
void Motor_Disarm(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, PWM_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, PWM_MIN);
}
