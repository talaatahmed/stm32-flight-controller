#ifndef MOTOR_MIX_H
#define MOTOR_MIX_H

#include "stm32f4xx_hal.h"

/* PWM pulse width limits (in timer ticks = microseconds at 1MHz) */
#define PWM_MIN       1000    /* ESC minimum — motor armed, not spinning */
#define PWM_MAX       2000    /* ESC maximum — full throttle             */
#define THROTTLE_IDLE 1100    /* Safe idle for simulation                */

typedef struct {
    uint16_t m1;   /* Front Left  — CCW */
    uint16_t m2;   /* Front Right — CW  */
    uint16_t m3;   /* Rear Right  — CCW */
    uint16_t m4;   /* Rear Left   — CW  */
} MotorOutput_t;

void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_Mix(float throttle, float roll, float pitch, float yaw,
               MotorOutput_t *out);
void Motor_Write(TIM_HandleTypeDef *htim, MotorOutput_t *out);
void Motor_Disarm(TIM_HandleTypeDef *htim);

#endif
