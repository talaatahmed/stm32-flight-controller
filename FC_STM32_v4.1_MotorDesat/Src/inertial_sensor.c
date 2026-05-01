/* ─────────────────────────────────────────────────────────────────
 *  InertialSensor frontend — implementation.
 *
 *  v2.2 CHANGE — ArduPilot-style architectural split:
 *  Removed run_complementary_filter() entirely. This file is now
 *  ONLY a thin orchestration layer:
 *    1. Probe each available backend until one works.
 *    2. Forward init/calibrate calls to the chosen backend.
 *    3. On each Update, copy the latest IMU sample from backend.
 *
 *  No attitude math here anymore. That work moved to ahrs.c.
 * ─────────────────────────────────────────────────────────────── */

#include "inertial_sensor.h"
#include "inertial_sensor_mpu6050.h"
#include <string.h>

/* ────────────────────────────────────────────────────────────────
 *  IS_Init — try each known backend, stop at the first that works.
 *
 *  This is the heart of the frontend/backend split. Today we only
 *  have MPU6050; tomorrow you'd ADD a probe call for ICM-42688
 *  here, and the rest of the system would never know.
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef IS_Init(InertialSensor_t *ins, I2C_HandleTypeDef *hi2c)
{
    if (!ins || !hi2c) return HAL_ERROR;

    /* Zero everything — including the imu sample (starts at 0,0,0)
     * and initialized flag. */
    memset(ins, 0, sizeof(*ins));

    /* ─── Probe chain ──
     * Try each backend in priority order. First one to respond wins.
     * Add new backends here, e.g.:
     *   if (ICM42688_Backend_Probe(&ins->backend, hi2c) == HAL_OK) goto found;
     */
    if (MPU6050_Backend_Probe(&ins->backend, hi2c) == HAL_OK) goto found;

    /* No backend responded */
    return HAL_ERROR;

found:
    /* Backend probed successfully — now run its hardware init */
    if (ins->backend.init(ins->backend.context) != HAL_OK)
        return HAL_ERROR;

    ins->initialized = 1;
    return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────
 *  IS_Calibrate — delegate to backend's calibration routine.
 *
 *  Note: in ArduPilot, calibration values are stored as parameters
 *  in EEPROM and applied at the frontend. Our simpler model stores
 *  them in the backend's static context (in RAM only — they're lost
 *  on reset, requiring re-calibration each boot).
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef IS_Calibrate(InertialSensor_t *ins, uint16_t num_samples)
{
    if (!ins || !ins->initialized) return HAL_ERROR;
    return MPU6050_Backend_Calibrate(ins->backend.context, num_samples);
}

/* ────────────────────────────────────────────────────────────────
 *  IS_Update — called every loop iteration.
 *
 *  v2.2: This is now a pure data pipeline — no filtering or
 *  attitude math. Just pulls the latest averaged + IIR-filtered
 *  sample from the backend into ins->imu.
 *
 *  After this returns with new_data=1, main.c should pass ins.imu
 *  to AHRS_Update() to compute roll/pitch.
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef IS_Update(InertialSensor_t *ins, uint8_t *new_data)
{
    if (!ins || !ins->initialized || !new_data) return HAL_ERROR;

    uint8_t samples = 0;
    HAL_StatusTypeDef status = ins->backend.read(ins->backend.context,
                                                  &ins->imu, &samples);
    if (status != HAL_OK) {
        *new_data = 0;
        return status;
    }

    if (samples == 0) {
        *new_data = 0;
        return HAL_OK;   /* not an error — just no data this poll */
    }

    *new_data = 1;
    return HAL_OK;
}
