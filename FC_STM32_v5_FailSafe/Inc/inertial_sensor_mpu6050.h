#ifndef INC_INERTIAL_SENSOR_MPU6050_H_
#define INC_INERTIAL_SENSOR_MPU6050_H_

#include "inertial_sensor_backend.h"

/* ─────────────────────────────────────────────────────────────────
 * Try to detect & initialize an MPU6050 on the given I2C bus.
 *
 * If the sensor responds correctly:
 *   - fills `backend` with init/read function pointers + context
 *   - returns HAL_OK
 * If the sensor is missing or wrong:
 *   - returns HAL_ERROR (frontend will try the next backend)
 *
 * This is the ArduPilot "probe" pattern — try each candidate
 * until one succeeds.
 * ─────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_Backend_Probe(ISBackend_t       *backend,
                                         I2C_HandleTypeDef *hi2c);

/* Calibration — frontend passes the opaque context back to us.
 * Cast happens inside the .c file. */
HAL_StatusTypeDef MPU6050_Backend_Calibrate(void *ctx, uint16_t n_samples);

#endif /* INC_INERTIAL_SENSOR_MPU6050_H_ */
