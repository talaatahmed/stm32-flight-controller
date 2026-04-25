/* ─────────────────────────────────────────────────────────────────
 * MPU6050 backend — implementation
 *
 * This file owns ALL MPU6050-specific knowledge:
 *   - I2C addresses
 *   - register map
 *   - scale factors
 *   - FIFO packet format
 *   - calibration logic
 *
 * Everything outside this file (frontend, application code) sees
 * only the generic IMU_Sample_t struct. To swap in an ICM-42688
 * later, you'd write a parallel inertial_sensor_icm42688.c — and
 * NOTHING else changes.
 * ─────────────────────────────────────────────────────────────── */

#include "inertial_sensor_mpu6050.h"
#include <string.h>

/* ─── I2C address ────────────────────────────────────────────── */
#define MPU6050_I2C_ADDR          (0x68 << 1)

/* ─── Register map ───────────────────────────────────────────── */
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_FIFO_EN        0x23
#define MPU6050_REG_USER_CTRL      0x6A
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_FIFO_COUNT_H   0x72
#define MPU6050_REG_FIFO_R_W       0x74
#define MPU6050_REG_WHO_AM_I       0x75

/* ─── Scale factors (datasheet) ──────────────────────────────── */
#define ACCEL_SCALE_2G             16384.0f   /* LSB/g    @ ±2g    */
#define GYRO_SCALE_250             131.0f     /* LSB/dps  @ ±250°/s */

/* ─── FIFO packet: 6 bytes accel + 6 bytes gyro ─────────────── */
#define FIFO_PACKET_SIZE           12
#define FIFO_OVERFLOW_THRESHOLD    1008       /* near full → reset */

/* ─────────────────────────────────────────────────────────────────
 * Private context — holds everything specific to one MPU6050.
 *
 * The frontend sees this as `void *context` and never touches
 * its internals. Only this .c file casts it back to MPU6050_Ctx_t*.
 * ─────────────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;

    /* Static bias from one-time calibration — subtracted from
     * every read after IS_Calibrate() is called. */
    float gyro_bias_dps[3];   /* X, Y, Z */
    float accel_bias_g[3];    /* X, Y, Z (Z already gravity-corrected) */
    uint8_t calibrated;       /* 0 until IS_Calibrate completes */
} MPU6050_Ctx_t;

/* ─── Static instance — fine for embedded, no malloc needed ── */
static MPU6050_Ctx_t s_mpu6050_ctx;

/* ────────────────────────────────────────────────────────────────
 *  I2C HELPERS
 * ────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c,
                                    uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c,
                                    uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}

/* ────────────────────────────────────────────────────────────────
 *  HARDWARE INIT
 *  Sets up MPU6050 for 1kHz FIFO-based sampling.
 * ────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef mpu6050_hw_init(void *ctx)
{
    MPU6050_Ctx_t *c = (MPU6050_Ctx_t *)ctx;
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0;

    HAL_Delay(100);  /* sensor power-up time */

    /* WHO_AM_I check — confirms we have an MPU6050 */
    status = read_regs(c->hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (status != HAL_OK) return status;
    if (who_am_i != 0x68) return HAL_ERROR;

    /* Wake from sleep, use PLL with X-gyro reference (most stable) */
    if ((status = write_reg(c->hi2c, MPU6050_REG_PWR_MGMT_1, 0x01)) != HAL_OK)
        return status;
    HAL_Delay(50);

    /* SMPLRT_DIV = 0 → sample rate = 1kHz / (1 + 0) = 1kHz */
    if ((status = write_reg(c->hi2c, MPU6050_REG_SMPLRT_DIV, 0x00)) != HAL_OK)
        return status;

    /* DLPF_CFG = 1 → BW=184Hz, gyro_rate=1kHz
     * (Wide BW → leave heavy filtering to software) */
    if ((status = write_reg(c->hi2c, MPU6050_REG_CONFIG, 0x01)) != HAL_OK)
        return status;

    /* GYRO  ±250 °/s, ACCEL ±2 g — most sensitive, best for stabilize */
    if ((status = write_reg(c->hi2c, MPU6050_REG_GYRO_CONFIG,  0x00)) != HAL_OK)
        return status;
    if ((status = write_reg(c->hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00)) != HAL_OK)
        return status;

    /* FIFO_EN: enable accel XYZ + gyro XYZ → 0x78 (bits 6,5,4,3) */
    if ((status = write_reg(c->hi2c, MPU6050_REG_FIFO_EN, 0x78)) != HAL_OK)
        return status;

    /* USER_CTRL: FIFO_EN=1, FIFO_RESET=1 → 0x44 (start fresh) */
    if ((status = write_reg(c->hi2c, MPU6050_REG_USER_CTRL, 0x44)) != HAL_OK)
        return status;

    HAL_Delay(10);  /* let FIFO start filling */
    return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────
 *  CALIBRATION (declared in header, callable from frontend)
 *
 *  Collects N samples while the board is still and computes mean
 *  gyro/accel offsets. Z accel keeps gravity (=1g) implicitly.
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef MPU6050_Backend_Calibrate(void *ctx, uint16_t n_samples)
{
    MPU6050_Ctx_t *c = (MPU6050_Ctx_t *)ctx;
    if (!c) return HAL_ERROR;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t collected = 0;
    uint8_t buf[14];

    /* Read directly from sensor data registers (NOT FIFO) for cal —
     * simpler and we control timing exactly. */
    for (uint16_t i = 0; i < n_samples; i++) {
        if (read_regs(c->hi2c, 0x3B, buf, 14) != HAL_OK) return HAL_ERROR;

        int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
        int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

        sum_ax += ax / ACCEL_SCALE_2G;
        sum_ay += ay / ACCEL_SCALE_2G;
        sum_az += az / ACCEL_SCALE_2G;
        sum_gx += gx / GYRO_SCALE_250;
        sum_gy += gy / GYRO_SCALE_250;
        sum_gz += gz / GYRO_SCALE_250;

        collected++;
        HAL_Delay(2);
    }

    if (collected == 0) return HAL_ERROR;

    c->accel_bias_g[0] = (float)(sum_ax / collected);
    c->accel_bias_g[1] = (float)(sum_ay / collected);
    c->accel_bias_g[2] = (float)(sum_az / collected) - 1.0f;  /* remove gravity */
    c->gyro_bias_dps[0] = (float)(sum_gx / collected);
    c->gyro_bias_dps[1] = (float)(sum_gy / collected);
    c->gyro_bias_dps[2] = (float)(sum_gz / collected);

    c->calibrated = 1;

    /* Reset FIFO so we don't read stale samples after calibration */
    write_reg(c->hi2c, MPU6050_REG_USER_CTRL, 0x44);
    HAL_Delay(10);

    return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────
 *  READ + AVERAGE (the function pointer used by the frontend)
 *
 *  Drains all packets from FIFO, averages them, applies bias,
 *  scales to SI units, and writes IMU_Sample_t.
 * ────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef mpu6050_read_avg(void *ctx,
                                          IMU_Sample_t *out,
                                          uint8_t      *samples_read)
{
    MPU6050_Ctx_t *c = (MPU6050_Ctx_t *)ctx;
    uint8_t  cnt_buf[2];
    uint16_t fifo_count;
    uint8_t  packet[FIFO_PACKET_SIZE];

    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    *samples_read = 0;

    /* 1. How many bytes in FIFO? */
    if (read_regs(c->hi2c, MPU6050_REG_FIFO_COUNT_H, cnt_buf, 2) != HAL_OK)
        return HAL_ERROR;
    fifo_count = ((uint16_t)cnt_buf[0] << 8) | cnt_buf[1];

    /* 2. Overflow → reset and bail */
    if (fifo_count >= FIFO_OVERFLOW_THRESHOLD) {
        write_reg(c->hi2c, MPU6050_REG_USER_CTRL, 0x44);
        return HAL_ERROR;
    }

    /* 3. Drain & accumulate every complete packet available */
    uint16_t packets = fifo_count / FIFO_PACKET_SIZE;

    for (uint16_t i = 0; i < packets; i++) {
        if (read_regs(c->hi2c, MPU6050_REG_FIFO_R_W,
                      packet, FIFO_PACKET_SIZE) != HAL_OK)
            return HAL_ERROR;

        sum_ax += (int16_t)((packet[0]  << 8) | packet[1]);
        sum_ay += (int16_t)((packet[2]  << 8) | packet[3]);
        sum_az += (int16_t)((packet[4]  << 8) | packet[5]);
        sum_gx += (int16_t)((packet[6]  << 8) | packet[7]);
        sum_gy += (int16_t)((packet[8]  << 8) | packet[9]);
        sum_gz += (int16_t)((packet[10] << 8) | packet[11]);

        (*samples_read)++;
    }

    /* No new data — not an error, just nothing to do this loop */
    if (*samples_read == 0) return HAL_OK;

    /* 4. Average → scale → bias correction → output */
    float n = (float)(*samples_read);

    out->accel_x_g = (sum_ax / n) / ACCEL_SCALE_2G;
    out->accel_y_g = (sum_ay / n) / ACCEL_SCALE_2G;
    out->accel_z_g = (sum_az / n) / ACCEL_SCALE_2G;
    out->gyro_x_dps = (sum_gx / n) / GYRO_SCALE_250;
    out->gyro_y_dps = (sum_gy / n) / GYRO_SCALE_250;
    out->gyro_z_dps = (sum_gz / n) / GYRO_SCALE_250;

    if (c->calibrated) {
        out->accel_x_g  -= c->accel_bias_g[0];
        out->accel_y_g  -= c->accel_bias_g[1];
        out->accel_z_g  -= c->accel_bias_g[2];
        out->gyro_x_dps -= c->gyro_bias_dps[0];
        out->gyro_y_dps -= c->gyro_bias_dps[1];
        out->gyro_z_dps -= c->gyro_bias_dps[2];
    }

    return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────
 *  PROBE — public entry point (the only thing exposed in the .h)
 *
 *  ArduPilot pattern: try to talk to the device, and if it
 *  responds correctly, register init/read pointers in the backend.
 * ────────────────────────────────────────────────────────────── */

HAL_StatusTypeDef MPU6050_Backend_Probe(ISBackend_t *backend,
                                         I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;

    /* Quick existence check before committing to this backend */
    HAL_Delay(50);
    if (read_regs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1) != HAL_OK)
        return HAL_ERROR;
    if (who_am_i != 0x68)
        return HAL_ERROR;

    /* Found one — set up the context */
    memset(&s_mpu6050_ctx, 0, sizeof(s_mpu6050_ctx));
    s_mpu6050_ctx.hi2c = hi2c;
    s_mpu6050_ctx.calibrated = 0;

    /* Wire up the function pointers */
    backend->context = &s_mpu6050_ctx;
    backend->init    = mpu6050_hw_init;
    backend->read    = mpu6050_read_avg;

    return HAL_OK;
}
