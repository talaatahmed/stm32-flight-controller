# STM32 IMU Learning Pipeline

A step-by-step implementation of an IMU-based flight control system  
built from scratch on STM32 (F411RE) using the MPU6050 sensor.

## Purpose
This project is a learning pipeline showing progression from raw sensor data to full control.

## Structure
- IMU_Phase1
- IMU_Phase3_ComplementaryFilter
- IMU_Phase4_PID
- IMU_Phase5_Visualizer
- IMU_Phase6_MAVLINK
- IMU_Phase7_PWM

## Flow
MPU6050 → Filter → Attitude → PID → Motor Mix → PWM
