# FC_STM32_v3.1_AHRS

**A flight controller build that separates attitude estimation from
the IMU driver layer, adopting the same architectural boundary that
ArduPilot draws between `AP_InertialSensor` and `AP_AHRS`.**

This version builds on v3 (hardware DLPF + IIR filter) and adds a
clean architectural boundary: the IMU layer produces sensor data,
the AHRS layer computes attitude, and neither knows about the other.

---

## Hardware

| Component | Detail                                        |
| --------- | --------------------------------------------- |
| MCU       | STM32F411RE (NUCLEO-F411RE board)             |
| IMU       | MPU6050 (I2C address 0x68, fast mode 400 kHz) |
| Motors    | 4× brushless via ESC, PWM on TIM3 CH1–CH4     |
| Telemetry | USART2 → USB (ST-Link virtual COM) + MAVLink  |

---

## What Changed From v3

### The Core Idea

In v3, `inertial_sensor.c` (the frontend) still ran the complementary
filter and produced `roll_deg` / `pitch_deg` directly. This mixed two
responsibilities that ArduPilot keeps strictly separate:

| Responsibility      | ArduPilot           | v3                         | v3.1                |
| ------------------- | ------------------- | -------------------------- | ------------------- |
| IMU data pipeline   | `AP_InertialSensor` | `inertial_sensor.c`        | `inertial_sensor.c` |
| Attitude estimation | `AP_AHRS`           | inside `inertial_sensor.c` | **`ahrs.c`** ← new  |

### New File: `ahrs.c` / `ahrs.h`

A new `AHRS_t` struct and three functions:

```c
void AHRS_Init(AHRS_t *ahrs);
void AHRS_Update(AHRS_t *ahrs, const IMU_Sample_t *imu, float dt);
void AHRS_Reset(AHRS_t *ahrs);
```

The complementary filter math moved here unchanged. The algorithm is
identical to v3 — only its location in the codebase changed.

### Changes to `inertial_sensor.h` / `.c`

- `roll_deg` and `pitch_deg` fields removed from `InertialSensor_t`
- `complementary_alpha` field removed
- `IS_Update()` no longer takes a `dt` parameter
- `run_complementary_filter()` removed entirely
- The frontend is now a pure data pipeline: probe → init → calibrate → read

### New File: `mavlink_telemetry.c` / `.h`

MAVLink message construction and UART transmission moved out of
`main.c` into a dedicated telemetry layer:

```c
void MAVLink_Init(UART_HandleTypeDef *huart);
void MAVLink_SendHeartbeat(void);
void MAVLink_SendAttitude(float roll_deg, float pitch_deg,
                           float gyro_x_dps, float gyro_y_dps);
```

`main.c` no longer includes `mavlink.h` directly. It only calls
these three functions — the same pattern used in ArduPilot's
`GCS_MAVLink` module.

### `main.c` — Before vs. After

**v3:**

```c
InertialSensor_t ins;   /* held roll_deg and pitch_deg */

IS_Update(&ins, DT, &new_data);
PID_Update(&pid_roll, SETPOINT, ins.roll_deg, DT);
send_mavlink_attitude(ins.roll_deg, ins.pitch_deg, ...);
```

**v3.1:**

```c
InertialSensor_t ins;   /* gyro/accel data only */
AHRS_t           ahrs;  /* attitude lives here now */

IS_Update(&ins, &new_data);
AHRS_Update(&ahrs, &ins.imu, DT);
PID_Update(&pid_roll, SETPOINT, ahrs.roll_deg, DT);
MAVLink_SendAttitude(ahrs.roll_deg, ahrs.pitch_deg, ...);
```

---

## Data Flow (v3.1)

```
MPU6050 FIFO
    ↓
inertial_sensor_mpu6050.c   ← hardware DLPF (42Hz) + averaging + IIR
    ↓  ins.imu (gyro_dps, accel_g)
inertial_sensor.c           ← frontend: thin orchestration only
    ↓  IMU_Sample_t
ahrs.c                      ← complementary filter → roll_deg, pitch_deg
    ↓  ahrs.roll_deg / ahrs.pitch_deg
pid.c                       ← roll + pitch control
    ↓  out_roll / out_pitch
motor_mix.c                 ← X-config mixing → M1..M4
    ↓
mavlink_telemetry.c         ← ATTITUDE @ 100Hz, HEARTBEAT @ 1Hz
```

Each layer has one responsibility. Each can be replaced independently.

---

## Connection to ArduPilot Source

| This project                | ArduPilot equivalent                |
| --------------------------- | ----------------------------------- |
| `inertial_sensor.h/.c`      | `AP_InertialSensor.h/.cpp`          |
| `inertial_sensor_backend.h` | `AP_InertialSensor_Backend.h`       |
| `inertial_sensor_mpu6050.c` | `AP_InertialSensor_Invensense.cpp`  |
| `ahrs.h/.c`                 | `AP_AHRS.h/.cpp` (DCM/EKF frontend) |
| `mavlink_telemetry.c`       | `GCS_MAVLink` module                |
| `pid.c`                     | `AC_PID` library                    |
| `motor_mix.c`               | `AP_MotorsMatrix` library           |

---

## File Structure

```
Core/
├── Inc/
│   ├── inertial_sensor.h           IMU frontend API (modified)
│   ├── inertial_sensor_backend.h   Abstract backend interface (unchanged)
│   ├── inertial_sensor_mpu6050.h   MPU6050 backend header (unchanged)
│   ├── ahrs.h                      ← NEW: attitude estimator API
│   ├── mavlink_telemetry.h         ← NEW: telemetry layer API
│   ├── pid.h                       (unchanged)
│   └── motor_mix.h                 (unchanged)
└── Src/
    ├── main.c                      Simplified (modified)
    ├── inertial_sensor.c           Pure data pipeline (modified)
    ├── inertial_sensor_mpu6050.c   Backend, DLPF + IIR (unchanged from v3)
    ├── ahrs.c                      ← NEW: complementary filter
    ├── mavlink_telemetry.c         ← NEW: MAVLink message construction
    ├── pid.c                       (unchanged)
    └── motor_mix.c                 (unchanged)
```

---

## Key Numbers (inherited from v3, unchanged)

| Parameter                 | Value   | Reason                       |
| ------------------------- | ------- | ---------------------------- |
| Sensor sample rate        | 1000 Hz | SMPLRT_DIV = 0x00            |
| Hardware DLPF BW          | 42 Hz   | Anti-alias for 100 Hz output |
| Samples averaged per loop | ~10     | Drain-all FIFO strategy      |
| IIR alpha                 | 0.80    | Light software smoothing     |
| Complementary alpha       | 0.98    | 98% gyro / 2% accel          |
| Loop rate                 | 100 Hz  | 10 ms HAL_Delay              |
| I2C clock                 | 400 kHz | Fast mode                    |

---

## What Comes Next

**v3.2 — Mahony Filter:**
Replace the complementary filter in ahrs.c with a Mahony filter
operating in quaternion space — no gimbal lock, PI feedback from
accelerometer, numerically stable at large angles.
No other files change — this is exactly why the AHRS layer was
separated in this version.
