#ifndef INC_INERTIAL_SENSOR_H_
#define INC_INERTIAL_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "inertial_sensor_backend.h"

/* ─────────────────────────────────────────────────────────────────
 *  InertialSensor frontend — the public face of the IMU subsystem.
 *
 *  v2.2 CHANGE — ArduPilot-style architectural split:
 *  This layer ONLY produces clean gyro/accel vectors. It does NOT
 *  compute attitude (roll/pitch). Attitude estimation has moved to
 *  a separate `ahrs.c` layer, exactly like ArduPilot separates
 *  AP_InertialSensor (raw IMU) from AP_AHRS (attitude estimation).
 *
 *  The application code (main.c) talks to this header to GET DATA,
 *  and to ahrs.h to GET ATTITUDE. The two are independent.
 * ─────────────────────────────────────────────────────────────── */

typedef struct {
    /* Latest filtered sensor readings (updated by IS_Update).
     * Units: gyro = degrees/sec, accel = g.
     * Already passed through:
     *   - hardware DLPF (42 Hz)
     *   - FIFO averaging (~10 samples per loop)
     *   - bias subtraction (after IS_Calibrate)
     *   - software IIR low-pass filter (alpha=0.80) */
    IMU_Sample_t imu;

    /* Internal state */
    ISBackend_t backend;
    uint8_t     initialized;

} InertialSensor_t;

/* ─── Public API ────────────────────────────────────────────── */

/* Probe each known backend in order until one works. Equivalent
 * to ArduPilot's ADD_BACKEND() chain in AP_InertialSensor.cpp. */
HAL_StatusTypeDef IS_Init(InertialSensor_t *ins, I2C_HandleTypeDef *hi2c);

/* One-time bias calibration — keep board still during this call.
 * Typical: 500 samples ≈ 1 second. */
HAL_StatusTypeDef IS_Calibrate(InertialSensor_t *ins, uint16_t num_samples);

/* Pull new samples from backend, update ins->imu.
 *
 * v2.2 NOTE: dt parameter removed — this layer no longer integrates
 * anything time-dependent. Pass dt to AHRS_Update() instead.
 *
 * Returns HAL_OK + true if new data is available
 *         HAL_OK + false if FIFO had nothing (normal — just retry next loop)
 *         HAL_ERROR on bus failure or overflow */
HAL_StatusTypeDef IS_Update(InertialSensor_t *ins, uint8_t *new_data);

#endif /* INC_INERTIAL_SENSOR_H_ */
