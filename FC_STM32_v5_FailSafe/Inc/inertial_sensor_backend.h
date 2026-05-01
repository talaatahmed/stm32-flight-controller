#ifndef INC_INERTIAL_SENSOR_BACKEND_H_
#define INC_INERTIAL_SENSOR_BACKEND_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────
 * IMU sample — what every backend must produce.
 *
 * Units are SI / standard:
 *   gyro  → degrees per second
 *   accel → g (1.0 = gravity)
 * ─────────────────────────────────────────────────────────────── */
typedef struct {
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
} IMU_Sample_t;

/* ─────────────────────────────────────────────────────────────────
 * Backend interface — pattern equivalent to a C++ abstract class.
 *
 * Each concrete backend (mpu6050, icm42688, etc.) provides:
 *   - context: opaque pointer to device-specific state
 *   - init   : one-time setup (registers, FIFO, etc.)
 *   - read   : pulls latest averaged sample into IMU_Sample_t
 *
 * The frontend never knows what's inside `context`.
 * ─────────────────────────────────────────────────────────────── */
typedef struct ISBackend ISBackend_t;

struct ISBackend {
    /* Opaque device-specific state. Cast inside each backend's .c file. */
    void *context;

    /* Initialize the underlying hardware. Returns HAL_OK on success. */
    HAL_StatusTypeDef (*init)(void *context);

    /* Read & average all available samples. Sets *samples_read = 0
     * if no new data. Returns HAL_OK even when no samples are available
     * (that's not an error — it's a normal poll). */
    HAL_StatusTypeDef (*read)(void *context,
                              IMU_Sample_t *out,
                              uint8_t      *samples_read);
};

#endif /* INC_INERTIAL_SENSOR_BACKEND_H_ */
