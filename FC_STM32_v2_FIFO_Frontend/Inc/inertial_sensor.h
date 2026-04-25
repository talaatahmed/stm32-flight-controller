#ifndef INC_INERTIAL_SENSOR_H_
#define INC_INERTIAL_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "inertial_sensor_backend.h"

/* ─────────────────────────────────────────────────────────────────
 *  InertialSensor frontend — the public face of the IMU subsystem.
 *
 *  The application code (main.c) talks ONLY to this header.
 *  It never sees mpu6050.h, never knows about FIFOs, never touches
 *  I2C registers.
 *
 *  In ArduPilot this is class AP_InertialSensor.
 * ─────────────────────────────────────────────────────────────── */

typedef struct {
    /* ─── Latest sensor readings (updated by IS_Update) ─── */
    IMU_Sample_t imu;

    /* ─── Computed attitude (filled by IS_Update) ─── */
    float roll_deg;
    float pitch_deg;

    /* ─── Internal state ─── */
    ISBackend_t backend;
    float       complementary_alpha;
    uint8_t     initialized;
} InertialSensor_t;

/* ─── Configuration ─────────────────────────────────────────── */
#define IS_DEFAULT_ALPHA   0.98f   /* complementary filter blend */

/* ─── Public API ────────────────────────────────────────────── */

/* Probe each known backend in order until one works. Equivalent
 * to ArduPilot's ADD_BACKEND() chain in AP_InertialSensor.cpp. */
HAL_StatusTypeDef IS_Init(InertialSensor_t *ins, I2C_HandleTypeDef *hi2c);

/* One-time bias calibration — keep board still during this call.
 * Typical: 500 samples ≈ 1 second. */
HAL_StatusTypeDef IS_Calibrate(InertialSensor_t *ins, uint16_t num_samples);

/* Pull new samples from backend, run complementary filter,
 * update ins->imu and ins->roll_deg / ins->pitch_deg.
 *
 * Returns HAL_OK + true if new data is available
 *         HAL_OK + false if FIFO had nothing (normal — just retry next loop)
 *         HAL_ERROR on bus failure or overflow */
HAL_StatusTypeDef IS_Update(InertialSensor_t *ins, float dt, uint8_t *new_data);

#endif /* INC_INERTIAL_SENSOR_H_ */
