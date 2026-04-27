/* ─────────────────────────────────────────────────────────────────
 *  InertialSensor frontend — implementation.
 *
 *  This file is the orchestrator. It:
 *    1. Probes each available backend until one works.
 *    2. Calls backend->read() each loop.
 *    3. Runs the complementary filter on the result.
 *    4. Exposes a simple API (roll_deg / pitch_deg) to the app.
 *
 *  No I2C addresses, register names, or scale factors live here.
 *  All of that is the backend's responsibility.
 * ─────────────────────────────────────────────────────────────── */

#include "inertial_sensor.h"
#include "inertial_sensor_mpu6050.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG  57.2957795f

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

    memset(ins, 0, sizeof(*ins));
    ins->complementary_alpha = IS_DEFAULT_ALPHA;

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
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef IS_Calibrate(InertialSensor_t *ins, uint16_t num_samples)
{
    if (!ins || !ins->initialized) return HAL_ERROR;
    return MPU6050_Backend_Calibrate(ins->backend.context, num_samples);
}

/* ────────────────────────────────────────────────────────────────
 *  Internal: complementary filter
 *  Same math as your original — fused accel + gyro → attitude.
 * ────────────────────────────────────────────────────────────── */

static void run_complementary_filter(InertialSensor_t *ins, float dt)
{
    const IMU_Sample_t *s = &ins->imu;
    const float alpha     = ins->complementary_alpha;

    /* Accelerometer angle (absolute reference, noisy) */
    float roll_acc  = atan2f(s->accel_y_g, s->accel_z_g) * RAD_TO_DEG;
    float pitch_acc = atan2f(-s->accel_x_g,
                              sqrtf(s->accel_y_g * s->accel_y_g +
                                    s->accel_z_g * s->accel_z_g)) * RAD_TO_DEG;

    /* Gyro integration (fast, drifts) */
    float roll_gyro  = ins->roll_deg  + s->gyro_x_dps * dt;
    float pitch_gyro = ins->pitch_deg + s->gyro_y_dps * dt;

    /* Blend */
    ins->roll_deg  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    ins->pitch_deg = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
}

/* ────────────────────────────────────────────────────────────────
 *  IS_Update — called every loop iteration.
 *
 *  Pulls latest averaged sample from backend, runs filter, updates
 *  attitude. Returns *new_data = 0 if FIFO was empty (normal).
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef IS_Update(InertialSensor_t *ins, float dt, uint8_t *new_data)
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

    run_complementary_filter(ins, dt);
    *new_data = 1;
    return HAL_OK;
}
