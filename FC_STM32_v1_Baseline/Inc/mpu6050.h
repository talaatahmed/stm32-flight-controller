#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

/* I2C Address — AD0 tied to GND → 0x68 shifted left by 1 for HAL */
#define MPU6050_ADDR        (0x68 << 1)

/* Register Map */
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_WHO_AM_I      0x75
/* FIFO + Control Registers */
#define MPU6050_REG_FIFO_EN        0x23
#define MPU6050_REG_USER_CTRL      0x6A
#define MPU6050_REG_FIFO_COUNT_H   0x72
#define MPU6050_REG_FIFO_R_W       0x74
/* USER_CTRL bits */
#define MPU6050_USERCTRL_FIFO_EN     0x40
#define MPU6050_USERCTRL_FIFO_RESET  0x04

/* Scale Factors */
#define ACCEL_SCALE_2G   16384.0f   /* LSB/g  for ±2g  range */
#define GYRO_SCALE_250   131.0f     /* LSB/(°/s) for ±250°/s */

/* Packet = 6 bytes accel + 6 bytes gyro = 12 bytes */
#define MPU6050_FIFO_PACKET_SIZE  12

#define RAD_TO_DEG  57.2957795f

/* Raw data struct */
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} MPU6050_RawData_t;

/* Scaled data struct */
typedef struct {
    float accel_x;   /* in g  */
    float accel_y;
    float accel_z;
    float gyro_x;    /* in °/s */
    float gyro_y;
    float gyro_z;
} MPU6050_ScaledData_t;

typedef struct {
    float accel_x_bias;
    float accel_y_bias;
    float accel_z_bias;   /* = avg_az - 1.0f  (remove gravity) */
    float gyro_x_bias;
    float gyro_y_bias;
    float gyro_z_bias;
} MPU6050_Bias_t;

typedef struct {
    float roll;    /* degrees — rotation around X axis */
    float pitch;   /* degrees — rotation around Y axis */
} MPU6050_Attitude_t;

/* Function prototypes */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *data);
void              MPU6050_ScaleData(MPU6050_RawData_t *raw, MPU6050_ScaledData_t *scaled);

HAL_StatusTypeDef MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Bias_t *bias, uint16_t num_samples);

void 			  MPU6050_ApplyBias(MPU6050_ScaledData_t *data, MPU6050_Bias_t *bias);
void 			  MPU6050_ComplementaryFilter(MPU6050_ScaledData_t *data, MPU6050_Attitude_t *attitude, float dt, float alpha);
HAL_StatusTypeDef MPU6050_ReadFromFIFO(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *out, uint8_t *samples_read);

#endif
