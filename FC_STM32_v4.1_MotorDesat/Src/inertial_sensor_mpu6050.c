/* ─────────────────────────────────────────────────────────────────
 * MPU6050 backend — implementation
 *
 * This file owns ALL MPU6050-specific knowledge:
 *   - I2C addresses
 *   - register map
 *   - scale factors
 *   - FIFO packet format
 *   - calibration logic
 *   - hardware DLPF (anti-aliasing) configuration    ← NEW (v2.1)
 *   - software IIR low-pass filter                   ← NEW (v2.1)
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
 * [CHANGE 1 of 3] IIR Low-Pass Filter alpha coefficient
 *
 * Formula applied each loop:
 *   filtered = alpha * new_sample + (1 - alpha) * previous_filtered
 *
 * Why 0.80?
 *   Our output rate is 100Hz → Nyquist = 50Hz.
 *   Hardware DLPF already blocks above 42Hz (anti-aliasing).
 *   The IIR here is a second smoothing stage, not an anti-alias filter.
 *   0.80 gives a light touch — fast enough for PID, smooth enough to
 *   reduce residual quantization noise after averaging.
 *
 * Tuning guide:
 *   Raise alpha (toward 1.0) → faster response, more noise through
 *   Lower alpha (toward 0.0) → smoother, slower to track motion
 *   Start at 0.80, observe plotter, adjust in steps of 0.05.
 * ─────────────────────────────────────────────────────────────── */
#define IIR_ALPHA   0.80f

/* ─────────────────────────────────────────────────────────────────
 * [CHANGE 2 of 3] Private context — IIR state fields added
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

    /* ── IIR Low-Pass Filter state ────────────────────────────
     * These hold the previous filtered output for each axis.
     * The filter formula needs the last output to compute the next one,
     * so we must persist it across loop iterations inside the context.
     *
     * iir_initialized: 0 on first call → seed filter with raw value
     *   so we don't start from zero (which would cause a spike).
     * ─────────────────────────────────────────────────────── */
    float   iir_gyro[3];      /* previous filtered gyro  output: [0]=X [1]=Y [2]=Z */
    float   iir_accel[3];     /* previous filtered accel output: [0]=X [1]=Y [2]=Z */
    uint8_t iir_initialized;  /* 0 until first sample seeds the filter */

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
 * [CHANGE 3 of 3] HARDWARE INIT — DLPF register updated
 *
 * The only change from v2 is in the CONFIG register write.
 * Everything else is identical.
 *
 * WHY we changed DLPF_CFG from 1 to 3:
 *
 *   Our loop reads the FIFO at 100Hz (every 10ms).
 *   Nyquist theorem: to avoid aliasing, we must cut off all signal
 *   above 50Hz BEFORE averaging (i.e., at the hardware level).
 *
 *   DLPF_CFG = 1 → BW = 184Hz  ← too wide, aliasing can occur
 *   DLPF_CFG = 3 → BW =  42Hz  ← correct anti-alias for 100Hz output
 *
 *   This is exactly what ArduPilot does: it sets the hardware DLPF
 *   to match the intended output rate, then applies a software filter
 *   on top. The hardware filter is not just "noise reduction" —
 *   it is an anti-aliasing filter that protects the averaging step.
 *
 *   SMPLRT_DIV stays 0x00 (1kHz into FIFO) — we want many samples
 *   to average, we just don't want high-frequency content in them.
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

    /* SMPLRT_DIV = 0x00 → sample rate = 1kHz / (1 + 0) = 1kHz
     * The FIFO fills at 1kHz, giving us ~10 samples to average per loop.
     * This is unchanged — we still want the high sample rate into FIFO. */
    if ((status = write_reg(c->hi2c, MPU6050_REG_SMPLRT_DIV, 0x00)) != HAL_OK)
        return status;

    /* CONFIG register (0x1A) — sets Digital Low Pass Filter (DLPF)
     *
     * ┌──────────┬──────────────┬──────────────┬──────────────┐
     * │ DLPF_CFG │ Accel BW(Hz) │ Gyro  BW(Hz) │ Gyro Rate    │
     * ├──────────┼──────────────┼──────────────┼──────────────┤
     * │    0     │     260      │     256      │    8 kHz     │
     * │    1     │     184      │     188      │    1 kHz     │
     * │    2     │      94      │      98      │    1 kHz     │
     * │    3     │      44      │      42      │    1 kHz ← ✓ │
     * │    4     │      21      │      20      │    1 kHz     │
     * │    5     │      10      │      10      │    1 kHz     │
     * │    6     │       5      │       5      │    1 kHz     │
     * └──────────┴──────────────┴──────────────┴──────────────┘
     *
     * We use DLPF_CFG = 3 (value 0x03):
     *   - Accel BW = 44Hz  (just under our 50Hz Nyquist limit)
     *   - Gyro  BW = 42Hz  (same)
     *   - Gyro sample rate stays 1kHz → FIFO still fills at 1kHz ✓
     *
     * PREVIOUSLY: 0x01 (184Hz) — too wide for 100Hz output rate
     * NOW:        0x03 ( 42Hz) — correct anti-alias for 100Hz output  */
    if ((status = write_reg(c->hi2c, MPU6050_REG_CONFIG, 0x03)) != HAL_OK)
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
 *  CALIBRATION (unchanged from v2)
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
 *  READ + AVERAGE + IIR FILTER
 *  (the function pointer used by the frontend)
 *
 *  Data path (3 stages):
 *
 *  Stage 1 — FIFO drain + accumulate:
 *    Read all available 12-byte packets from FIFO and sum them.
 *
 *  Stage 2 — Average + scale + bias:
 *    Divide sums by packet count → raw average.
 *    Scale from LSB to physical units (g, dps).
 *    Subtract calibration bias.
 *
 *  Stage 3 — IIR Low-Pass Filter:   ← NEW (v2.1)
 *    Apply single-pole IIR to each axis to smooth residual noise.
 *    Formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *    State persists across calls inside MPU6050_Ctx_t.
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

    /* ── Stage 1: FIFO drain ───────────────────────────────── */

    /* 1a. How many bytes are in the FIFO right now? */
    if (read_regs(c->hi2c, MPU6050_REG_FIFO_COUNT_H, cnt_buf, 2) != HAL_OK)
        return HAL_ERROR;
    fifo_count = ((uint16_t)cnt_buf[0] << 8) | cnt_buf[1];

    /* 1b. Overflow guard — if FIFO is nearly full (≥1008 bytes),
     *     it means we fell behind. Reset it and bail this cycle.
     *     The next call will start fresh. */
    if (fifo_count >= FIFO_OVERFLOW_THRESHOLD) {
        write_reg(c->hi2c, MPU6050_REG_USER_CTRL, 0x44);
        return HAL_ERROR;
    }

    /* 1c. Read and accumulate every complete 12-byte packet */
    uint16_t packets = fifo_count / FIFO_PACKET_SIZE;

    for (uint16_t i = 0; i < packets; i++) {
        if (read_regs(c->hi2c, MPU6050_REG_FIFO_R_W,
                      packet, FIFO_PACKET_SIZE) != HAL_OK)
            return HAL_ERROR;

        /* Packet layout (big-endian, signed 16-bit):
         *   bytes  0-1  : accel X
         *   bytes  2-3  : accel Y
         *   bytes  4-5  : accel Z
         *   bytes  6-7  : gyro  X
         *   bytes  8-9  : gyro  Y
         *   bytes 10-11 : gyro  Z */
        sum_ax += (int16_t)((packet[0]  << 8) | packet[1]);
        sum_ay += (int16_t)((packet[2]  << 8) | packet[3]);
        sum_az += (int16_t)((packet[4]  << 8) | packet[5]);
        sum_gx += (int16_t)((packet[6]  << 8) | packet[7]);
        sum_gy += (int16_t)((packet[8]  << 8) | packet[9]);
        sum_gz += (int16_t)((packet[10] << 8) | packet[11]);

        (*samples_read)++;
    }

    /* No new data this poll — not an error, just nothing to do */
    if (*samples_read == 0) return HAL_OK;

    /* ── Stage 2: Average → scale → bias correction ────────── */

    float n = (float)(*samples_read);

    /* Divide accumulated sum by sample count → average raw LSB value.
     * Then divide by scale factor → physical units. */
    out->accel_x_g   = (sum_ax / n) / ACCEL_SCALE_2G;
    out->accel_y_g   = (sum_ay / n) / ACCEL_SCALE_2G;
    out->accel_z_g   = (sum_az / n) / ACCEL_SCALE_2G;
    out->gyro_x_dps  = (sum_gx / n) / GYRO_SCALE_250;
    out->gyro_y_dps  = (sum_gy / n) / GYRO_SCALE_250;
    out->gyro_z_dps  = (sum_gz / n) / GYRO_SCALE_250;

    /* Subtract calibration bias if IS_Calibrate() has been called */
    if (c->calibrated) {
        out->accel_x_g  -= c->accel_bias_g[0];
        out->accel_y_g  -= c->accel_bias_g[1];
        out->accel_z_g  -= c->accel_bias_g[2];
        out->gyro_x_dps -= c->gyro_bias_dps[0];
        out->gyro_y_dps -= c->gyro_bias_dps[1];
        out->gyro_z_dps -= c->gyro_bias_dps[2];
    }

    /* ── Stage 3: IIR Low-Pass Filter ──────────────────────────
     *
     * This is a single-pole (first-order) IIR filter.
     * It is applied AFTER averaging (stage 2), so it acts as a
     * second smoothing layer on top of the box-filter averaging.
     *
     * On the very first call, we "seed" the filter state with the
     * current sample instead of starting from 0. If we started from
     * 0, the filter output would ramp up from zero over several
     * samples, causing a large transient spike in the PID output
     * at startup. Seeding prevents that.
     * ─────────────────────────────────────────────────────── */
    if (!c->iir_initialized) {

        /* First call: copy current values directly into filter state.
         * Output is unchanged this cycle — no filtering applied yet. */
        c->iir_gyro[0]  = out->gyro_x_dps;
        c->iir_gyro[1]  = out->gyro_y_dps;
        c->iir_gyro[2]  = out->gyro_z_dps;
        c->iir_accel[0] = out->accel_x_g;
        c->iir_accel[1] = out->accel_y_g;
        c->iir_accel[2] = out->accel_z_g;

        c->iir_initialized = 1;

    } else {

        /* Every subsequent call: apply IIR formula to all 6 axes.
         *
         *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
         *
         *   x[n]   = new averaged sample (from stage 2)
         *   y[n-1] = previous filtered output (stored in context)
         *   y[n]   = new filtered output (written to out + stored)
         *
         * Higher alpha → tracks input faster (less smoothing)
         * Lower  alpha → smoother but slower to follow real motion  */

        c->iir_gyro[0] = IIR_ALPHA * out->gyro_x_dps + (1.0f - IIR_ALPHA) * c->iir_gyro[0];
        c->iir_gyro[1] = IIR_ALPHA * out->gyro_y_dps + (1.0f - IIR_ALPHA) * c->iir_gyro[1];
        c->iir_gyro[2] = IIR_ALPHA * out->gyro_z_dps + (1.0f - IIR_ALPHA) * c->iir_gyro[2];

        c->iir_accel[0] = IIR_ALPHA * out->accel_x_g  + (1.0f - IIR_ALPHA) * c->iir_accel[0];
        c->iir_accel[1] = IIR_ALPHA * out->accel_y_g  + (1.0f - IIR_ALPHA) * c->iir_accel[1];
        c->iir_accel[2] = IIR_ALPHA * out->accel_z_g  + (1.0f - IIR_ALPHA) * c->iir_accel[2];

        /* Write filtered values back into output struct so the
         * frontend (complementary filter) receives clean data */
        out->gyro_x_dps = c->iir_gyro[0];
        out->gyro_y_dps = c->iir_gyro[1];
        out->gyro_z_dps = c->iir_gyro[2];
        out->accel_x_g  = c->iir_accel[0];
        out->accel_y_g  = c->iir_accel[1];
        out->accel_z_g  = c->iir_accel[2];
    }

    return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────
 *  PROBE — public entry point (unchanged from v2)
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

    /* Found one — set up the context.
     * memset zeroes everything including iir_initialized = 0,
     * which is correct — filter seeds on first read. */
    memset(&s_mpu6050_ctx, 0, sizeof(s_mpu6050_ctx));
    s_mpu6050_ctx.hi2c = hi2c;
    s_mpu6050_ctx.calibrated = 0;

    /* Wire up the function pointers */
    backend->context = &s_mpu6050_ctx;
    backend->init    = mpu6050_hw_init;
    backend->read    = mpu6050_read_avg;

    return HAL_OK;
}
