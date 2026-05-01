# FC_STM32_v3.2_Mahony

### STM32F411 Flight Controller — Mahony AHRS + Single-Loop PID

---

## Overview

This is version 3.1 of a custom quadcopter flight controller firmware built on the **STM32F411RE (NUCLEO-F411RE)** using STM32 HAL libraries.

The key architectural achievement of this version is the **ArduPilot-style frontend/backend split** for the IMU subsystem, combined with a dedicated **AHRS layer** using a **Mahony complementary filter** for attitude estimation.

The attitude output (roll, pitch) feeds a **single-loop PID controller** that drives the motor mixer directly.

---

## Hardware

| Component | Details                                  |
| --------- | ---------------------------------------- |
| MCU       | STM32F411RETx (NUCLEO-F411RE)            |
| IMU       | MPU-6050 via I2C1 (PB8=SCL, PB9=SDA)     |
| Motors    | 4× ESC via TIM3 PWM (PA6, PA7, PB0, PB1) |
| Telemetry | MAVLink over USART2 (PA2=TX, PA3=RX)     |
| Clock     | 84 MHz (HSI + PLL)                       |
| Loop rate | 100 Hz                                   |

---

## Architecture

```
MPU-6050 Hardware
      │  I2C1
      ▼
MPU6050 Backend         ← inertial_sensor_mpu6050.c
  - FIFO drain
  - ~10 sample averaging
  - IIR low-pass filter (α=0.80)
  - Bias calibration
      │
      ▼
InertialSensor Frontend ← inertial_sensor.c
  - Backend probe chain
  - Thin orchestration layer
  - No attitude math
      │  IMU_Sample_t
      ▼
AHRS — Mahony Filter    ← ahrs.c
  - Unit quaternion state
  - PI feedback from accelerometer
  - Outputs: roll_deg, pitch_deg, yaw_deg
      │
      ▼
PID Controller          ← pid.c
  - Single loop: angle error → motor command
  - Anti-windup integral clamp
  - D-term on error
      │
      ▼
Motor Mixer             ← motor_mix.c
  - X-configuration
  - PWM [1000–2000 µs]
      │  TIM3
      ▼
4× ESC / Motors
```

---

## Key Design Decisions

### Mahony Filter (replaces complementary filter)

Previous versions used a simple complementary filter:

```
angle = alpha * gyro_integrated + (1-alpha) * accel_angle
```

v3.1 upgrades to a **Mahony filter** operating in quaternion space:

- Maintains a unit quaternion `q[4]` representing orientation
- Uses PI feedback (Kp, Ki) from the accelerometer cross-product error
- No gimbal lock at ±90° pitch
- Numerically stable at large angles
- Same mathematical principle as ArduPilot's `AP_AHRS_DCM`

**Default gains:**

| Parameter | Value | Description                      |
| --------- | ----- | -------------------------------- |
| Kp        | 2.0   | Proportional — convergence speed |
| Ki        | 0.005 | Integral — gyro bias correction  |

### ArduPilot-style IMU Split

Mirrors ArduPilot's `AP_InertialSensor` / `AP_AHRS` separation:

- `inertial_sensor.c` — raw data pipeline only, no attitude math
- `ahrs.c` — attitude estimation only, sensor-agnostic
- Adding a new IMU (e.g. ICM-42688) requires only a new backend — zero changes to AHRS or application code

### MAVLink Telemetry

Streams two message types to a GCS (Mission Planner / QGroundControl):

| Message   | Rate   | Content                              |
| --------- | ------ | ------------------------------------ |
| HEARTBEAT | 1 Hz   | Vehicle type, autopilot, armed state |
| ATTITUDE  | 100 Hz | roll, pitch, gyro rates (in radians) |

---

## File Structure

```
Inc/
  ahrs.h                    — Mahony filter struct + API
  inertial_sensor.h         — Frontend public API
  inertial_sensor_backend.h — IMU_Sample_t + ISBackend_t interface
  inertial_sensor_mpu6050.h — MPU6050 backend probe API
  mavlink_telemetry.h       — MAVLink send functions
  motor_mix.h               — MotorOutput_t + Motor_* API
  pid.h                     — PID_t struct + API
  main.h                    — Pin definitions

Src/
  main.c                    — 100Hz main loop, system init
  ahrs.c                    — Mahony filter implementation
  inertial_sensor.c         — Frontend orchestration
  inertial_sensor_mpu6050.c — MPU6050 driver (FIFO, IIR, calibration)
  mavlink_telemetry.c       — MAVLink message packing + UART TX
  motor_mix.c               — X-config motor mixing
  pid.c                     — PID controller
  stm32f4xx_hal_msp.c       — HAL MSP peripheral init (I2C, TIM, UART)
  stm32f4xx_it.c            — Interrupt handlers (HardFault LED blink)
```

---

## Main Loop (100 Hz)

```
1. IS_Update()        — pull latest IMU sample from MPU6050 backend
2. AHRS_Update()      — run Mahony filter → roll_deg, pitch_deg
3. PID_Update()       — roll/pitch error → motor commands
4. Motor_Mix()        — commands → M1–M4 PWM values
5. Motor_Write()      — write to TIM3 compare registers
6. MAVLink_SendAttitude()  — stream attitude to GCS (100Hz)
7. MAVLink_SendHeartbeat() — heartbeat at 1Hz
```

---

## Motor Layout (X-Configuration)

```
M1 FL (CCW)  ●───────● M2 FR (CW)
              \     /
               \   /
                \ /
                / \
               /   \
              /     \
M4 RL (CW)   ●───────● M3 RR (CCW)
```

| Motor | Position    | Direction | Channel        |
| ----- | ----------- | --------- | -------------- |
| M1    | Front Left  | CCW       | TIM3_CH1 (PA6) |
| M2    | Front Right | CW        | TIM3_CH2 (PA7) |
| M3    | Rear Right  | CCW       | TIM3_CH3 (PB0) |
| M4    | Rear Left   | CW        | TIM3_CH4 (PB1) |

---

## PWM Configuration

| Parameter     | Value                |
| ------------- | -------------------- |
| Timer         | TIM3                 |
| PWM_MIN       | 1000 µs              |
| PWM_MAX       | 2000 µs              |
| THROTTLE_IDLE | 1100 µs              |
| Frequency     | 50 Hz (standard ESC) |

---

## Debugging

- **HardFault**: PA5 (LD2) blinks rapidly — check I2C wiring or stack overflow
- **No MAVLink data**: verify USART2 baud = 115200, PA2=TX connected
- **IMU not found**: check I2C address (0x68 default), SDA/SCL pullups

---

## Limitations

- Yaw angle drifts over time — no magnetometer correction
- PID gains are starting values — require in-flight tuning
- No EEPROM — calibration data lost on reset, re-calibrate each boot
- Single-loop PID is less responsive than cascade at large angles

---

## Next Version

v4.0 upgrades the control architecture to a **cascade (two-loop) controller** mirroring ArduPilot's `AC_AttitudeControl_Multi`, with a dedicated yaw rate loop and throttle-scaled outputs.
