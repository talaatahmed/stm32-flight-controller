#include <math.h>
#include "mpu6050.h"
/* ─────────────────────────────────────────
 * Internal helper — write one register byte
 * ───────────────────────────────────────── */
static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2, HAL_MAX_DELAY);
}

/* ─────────────────────────────────────────
 * Internal helper — read N bytes from register
 * ───────────────────────────────────────── */
static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c,
                                           uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    /* Send register address */
    status = HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    /* Read data */
    return HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buf, len, HAL_MAX_DELAY);
}

/* ─────────────────────────────────────────
 * Initialize MPU-6050
 * ───────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0;

    /* Step 1: Verify device identity */
    status = MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (status != HAL_OK)   return status;
    if (who_am_i != 0x68)   return HAL_ERROR;  /* Wrong device */

    /* Step 2: Wake up — clear sleep bit (default is sleep mode) */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) return status;
    HAL_Delay(100);  /* Allow oscillator to stabilize */

    /* Step 3: SMPLRT_DIV = 0 → Sample Rate = 1000/(1+0) = 1kHz */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (status != HAL_OK) return status;

    /* Step 4: DLPF_CFG = 1 → BW=184Hz, gyro_rate=1kHz */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, 0x01);
    if (status != HAL_OK) return status;

    /* Step 5: Gyro range → ±250°/s (most sensitive, best for Phase 1) */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (status != HAL_OK) return status;

    /* Step 6: Accel range → ±2g (most sensitive, best for Phase 1) */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (status != HAL_OK) return status;

    // 7) Enable FIFO sources (Accel + Gyro)
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_FIFO_EN, 0x78);// FIFO_EN
    if (status != HAL_OK) return status;

    // 8) Enable FIFO + Reset (only one time)
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x44); // USER_CTRL
    if (status != HAL_OK) return status;

    HAL_Delay(1);

    // 9) سيب FIFO شغال بدون reset
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, MPU6050_USERCTRL_FIFO_EN);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/* ─────────────────────────────────────────
 * Read raw 16-bit sensor data
 * Registers 0x3B–0x48 = accel(6) + temp(2) + gyro(6)
 * ───────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data)
{
    uint8_t buf[14];  /* 14 bytes: 6 accel + 2 temp + 6 gyro */
    HAL_StatusTypeDef status;

    status = MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return status;

    /* Combine high and low bytes — MPU6050 is big-endian */
    data->accel_x_raw = (int16_t)(buf[0]  << 8 | buf[1]);
    data->accel_y_raw = (int16_t)(buf[2]  << 8 | buf[3]);
    data->accel_z_raw = (int16_t)(buf[4]  << 8 | buf[5]);
    /* buf[6] and buf[7] = temperature — skip for now */
    data->gyro_x_raw  = (int16_t)(buf[8]  << 8 | buf[9]);
    data->gyro_y_raw  = (int16_t)(buf[10] << 8 | buf[11]);
    data->gyro_z_raw  = (int16_t)(buf[12] << 8 | buf[13]);

    return HAL_OK;
}

/* ─────────────────────────────────────────
 * Convert raw integers to physical units
 * ───────────────────────────────────────── */
void MPU6050_ScaleData(MPU6050_RawData_t *raw, MPU6050_ScaledData_t *scaled)
{
    scaled->accel_x = raw->accel_x_raw / ACCEL_SCALE_2G;
    scaled->accel_y = raw->accel_y_raw / ACCEL_SCALE_2G;
    scaled->accel_z = raw->accel_z_raw / ACCEL_SCALE_2G;

    scaled->gyro_x  = raw->gyro_x_raw  / GYRO_SCALE_250;
    scaled->gyro_y  = raw->gyro_y_raw  / GYRO_SCALE_250;
    scaled->gyro_z  = raw->gyro_z_raw  / GYRO_SCALE_250;
}

/*─────────────────────────────────────────
 * Calibration
 ─────────────────────────────────────────*/
HAL_StatusTypeDef MPU6050_Calibrate(I2C_HandleTypeDef *hi2c,
                                     MPU6050_Bias_t *bias,
                                     uint16_t num_samples)
{
    MPU6050_RawData_t    raw;
    MPU6050_ScaledData_t scaled;

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        if (MPU6050_ReadRaw(hi2c, &raw) != HAL_OK)
            return HAL_ERROR;

        MPU6050_ScaleData(&raw, &scaled);

        sum_ax += scaled.accel_x;
        sum_ay += scaled.accel_y;
        sum_az += scaled.accel_z;
        sum_gx += scaled.gyro_x;
        sum_gy += scaled.gyro_y;
        sum_gz += scaled.gyro_z;

        HAL_Delay(2);  /* 2ms between samples = ~500Hz collection rate */
    }

    bias->accel_x_bias = (float)(sum_ax / num_samples);
    bias->accel_y_bias = (float)(sum_ay / num_samples);
    bias->accel_z_bias = (float)(sum_az / num_samples) - 1.0f; /* remove gravity */
    bias->gyro_x_bias  = (float)(sum_gx / num_samples);
    bias->gyro_y_bias  = (float)(sum_gy / num_samples);
    bias->gyro_z_bias  = (float)(sum_gz / num_samples);

    return HAL_OK;
}

void MPU6050_ApplyBias(MPU6050_ScaledData_t *data,
                       MPU6050_Bias_t *bias)
{
    data->accel_x -= bias->accel_x_bias;
    data->accel_y -= bias->accel_y_bias;
    data->accel_z -= bias->accel_z_bias;
    data->gyro_x  -= bias->gyro_x_bias;
    data->gyro_y  -= bias->gyro_y_bias;
    data->gyro_z  -= bias->gyro_z_bias;
}

/*─────────────────────────────────────────
 ComplementaryFilter
 ─────────────────────────────────────────*/
void MPU6050_ComplementaryFilter(MPU6050_ScaledData_t *data,
                                  MPU6050_Attitude_t   *attitude,
                                  float dt,
                                  float alpha)
{
    /* ── Accelerometer angle (absolute reference, noisy) ── */
    float roll_acc  = atan2f(data->accel_y, data->accel_z)
                      * RAD_TO_DEG;

    float pitch_acc = atan2f(-data->accel_x,
                              sqrtf(data->accel_y * data->accel_y +
                                    data->accel_z * data->accel_z))
                      * RAD_TO_DEG;

    /* ── Gyro integration (fast tracking, drifts) ── */
    float roll_gyro  = attitude->roll  + data->gyro_x * dt;
    float pitch_gyro = attitude->pitch + data->gyro_y * dt;

    /* ── Complementary blend ── */
    attitude->roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    attitude->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
}

/*─────────────────────────────────────────
ReadFromFIFO
 ─────────────────────────────────────────*/
//HAL_StatusTypeDef MPU6050_ReadFromFIFO(I2C_HandleTypeDef *hi2c,
//                                        MPU6050_RawData_t *out,
//                                        uint8_t *samples_read)
//{
//    uint8_t cnt_buf[2];
//    uint16_t fifo_count;
//    uint8_t packet[MPU6050_FIFO_PACKET_SIZE];
//
//    *samples_read = 0;
//
//    /* 1. Read FIFO_COUNT (2 bytes, high byte first) */
//    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_COUNT_H,
//                          I2C_MEMADD_SIZE_8BIT, cnt_buf, 2, 50) != HAL_OK)
//        return HAL_ERROR;
//
//    fifo_count = ((uint16_t)cnt_buf[0] << 8) | cnt_buf[1];
//
//    /* 2. Overflow check — FIFO is 1024 bytes. If we hit 1008+, reset. */
//    if (fifo_count >= 1008) {
//        uint8_t reset = 0x44;  /* FIFO_EN=1, FIFO_RESET=1 */
//        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_USER_CTRL,
//                          I2C_MEMADD_SIZE_8BIT, &reset, 1, 50);
//        return HAL_ERROR;  /* signal overflow to caller */
//    }
//
//    /* 3. Not enough for a full packet yet? Return quietly. */
//    if (fifo_count < MPU6050_FIFO_PACKET_SIZE)
//        return HAL_OK;  /* no data yet, but not an error */
//
//    /* 4. Read ONE packet — raw bytes, big-endian */
//    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_R_W,
//                          I2C_MEMADD_SIZE_8BIT, packet,
//                          MPU6050_FIFO_PACKET_SIZE, 50) != HAL_OK)
//        return HAL_ERROR;
//
//    /* 5. Parse big-endian into signed 16-bit */
//    out->accel_x_raw = (int16_t)((packet[0]  << 8) | packet[1]);
//    out->accel_y_raw = (int16_t)((packet[2]  << 8) | packet[3]);
//    out->accel_z_raw = (int16_t)((packet[4]  << 8) | packet[5]);
//    out->gyro_x_raw  = (int16_t)((packet[6]  << 8) | packet[7]);
//    out->gyro_y_raw  = (int16_t)((packet[8]  << 8) | packet[9]);
//    out->gyro_z_raw  = (int16_t)((packet[10] << 8) | packet[11]);
//
//    *samples_read = 1;
//    return HAL_OK;
//}
//
//HAL_StatusTypeDef MPU6050_ReadFromFIFO(I2C_HandleTypeDef *hi2c,
//                                        MPU6050_RawData_t *out,
//                                        uint8_t *samples_read)
//{
//    uint8_t cnt_buf[2];
//    uint16_t fifo_count;
//    uint8_t packet[MPU6050_FIFO_PACKET_SIZE];
//
//    *samples_read = 0;
//
//    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_COUNT_H,
//                          I2C_MEMADD_SIZE_8BIT, cnt_buf, 2, 50) != HAL_OK)
//        return HAL_ERROR;
//
//    fifo_count = ((uint16_t)cnt_buf[0] << 8) | cnt_buf[1];
//
//    /* Overflow → reset and bail */
//    if (fifo_count >= 1008) {
//        uint8_t reset = 0x44;
//        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_USER_CTRL,
//                          I2C_MEMADD_SIZE_8BIT, &reset, 1, 50);
//        return HAL_ERROR;
//    }
//
//    /* ── DRAIN ALL: read packets until FIFO is < 1 packet ──
//     * Keep only the LAST sample (most recent → freshest for PID). */
//    while (fifo_count >= MPU6050_FIFO_PACKET_SIZE) {
//        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_R_W,
//                              I2C_MEMADD_SIZE_8BIT, packet,
//                              MPU6050_FIFO_PACKET_SIZE, 50) != HAL_OK)
//            return HAL_ERROR;
//
//        (*samples_read)++;
//        fifo_count -= MPU6050_FIFO_PACKET_SIZE;
//    }
//
//    /* Parse only the last packet we read */
//    if (*samples_read > 0) {
//        out->accel_x_raw = (int16_t)((packet[0]  << 8) | packet[1]);
//        out->accel_y_raw = (int16_t)((packet[2]  << 8) | packet[3]);
//        out->accel_z_raw = (int16_t)((packet[4]  << 8) | packet[5]);
//        out->gyro_x_raw  = (int16_t)((packet[6]  << 8) | packet[7]);
//        out->gyro_y_raw  = (int16_t)((packet[8]  << 8) | packet[9]);
//        out->gyro_z_raw  = (int16_t)((packet[10] << 8) | packet[11]);
//    }
//
//    return HAL_OK;
//}

HAL_StatusTypeDef MPU6050_ReadFromFIFO(I2C_HandleTypeDef *hi2c,
                                        MPU6050_RawData_t *out,
                                        uint8_t *samples_read)
{
    uint8_t  cnt_buf[2];
    uint16_t fifo_count;
    uint8_t  packet[MPU6050_FIFO_PACKET_SIZE];

    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    *samples_read = 0;

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_COUNT_H,
                          I2C_MEMADD_SIZE_8BIT, cnt_buf, 2, 50) != HAL_OK)
        return HAL_ERROR;

    fifo_count = ((uint16_t)cnt_buf[0] << 8) | cnt_buf[1];

    if (fifo_count >= 1008) {
        uint8_t reset = 0x44;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_USER_CTRL,
                          I2C_MEMADD_SIZE_8BIT, &reset, 1, 50);
        return HAL_ERROR;
    }

    /* ── DRAIN + ACCUMULATE ALL SAMPLES ── */
    uint16_t packets_available = fifo_count / MPU6050_FIFO_PACKET_SIZE;

    for (uint16_t i = 0; i < packets_available; i++) {
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_FIFO_R_W,
                              I2C_MEMADD_SIZE_8BIT, packet,
                              MPU6050_FIFO_PACKET_SIZE, 50) != HAL_OK)
            return HAL_ERROR;

        sum_ax += (int16_t)((packet[0]  << 8) | packet[1]);
        sum_ay += (int16_t)((packet[2]  << 8) | packet[3]);
        sum_az += (int16_t)((packet[4]  << 8) | packet[5]);
        sum_gx += (int16_t)((packet[6]  << 8) | packet[7]);
        sum_gy += (int16_t)((packet[8]  << 8) | packet[9]);
        sum_gz += (int16_t)((packet[10] << 8) | packet[11]);

        (*samples_read)++;
    }

    /* ── AVERAGE → output raw (still integer, divide in scale step) ── */
    if (*samples_read > 0) {
        out->accel_x_raw = (int16_t)(sum_ax / *samples_read);
        out->accel_y_raw = (int16_t)(sum_ay / *samples_read);
        out->accel_z_raw = (int16_t)(sum_az / *samples_read);
        out->gyro_x_raw  = (int16_t)(sum_gx / *samples_read);
        out->gyro_y_raw  = (int16_t)(sum_gy / *samples_read);
        out->gyro_z_raw  = (int16_t)(sum_gz / *samples_read);
    }

    return HAL_OK;
}
