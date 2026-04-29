# FC_STM32_v4_CascadeControl

### STM32F411 Flight Controller — Mahony AHRS + ArduPilot-style Cascade Controller

---

## Overview

This is version 4.0 of a custom quadcopter flight controller firmware built on the **STM32F411RE (NUCLEO-F411RE)** using STM32 HAL libraries.

This version upgrades the control architecture from a **single-loop PID** (v3.1) to a full **ArduPilot-style cascade controller**, mirroring the structure of `AC_AttitudeControl_Multi` and `AC_PID` from ArduPilot's libraries.

The AHRS layer (Mahony filter) and IMU subsystem are carried forward unchanged from v3.1.

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
      │
      ▼
InertialSensor Frontend ← inertial_sensor.c
      │  IMU_Sample_t (gyro_dps, accel_g)
      ▼
AHRS — Mahony Filter    ← ahrs.c
      │  roll_deg, pitch_deg, yaw_deg
      ▼
AttitudeControl         ← attitude_control.c
  ┌──────────────────────────────────────┐
  │  OUTER LOOP (AC_P)                   │
  │    angle_error → rate_setpoint       │
  │    roll:  Kp=4.5, max=200 dps        │
  │    pitch: Kp=4.5, max=200 dps        │
  │                                      │
  │  INNER LOOP (PID + D-LPF)            │
  │    rate_error → motor_command        │
  │    roll/pitch: Kp=0.135, Ki=0.135,   │
  │                Kd=0.0036, LPF=20Hz   │
  │    yaw (rate only): Kp=0.180,        │
  │                     Ki=0.018, Kd=0   │
  │                                      │
  │  THROTTLE SCALING                    │
  │    output *= throttle / hover_throttle│
  └──────────────────────────────────────┘
      │  roll_cmd, pitch_cmd, yaw_cmd [-1,+1]
      ▼
Motor Mixer             ← motor_mix.c
      │  TIM3
      ▼
4× ESC / Motors
```

---

## Control Architecture

### Why Cascade?

The single-loop PID in v3.1 mapped angle error directly to motor commands. This works, but has limitations:

- D-term acts on angle, which is slow and noisy
- No separation between attitude and rate dynamics
- Less responsive during fast maneuvers

The cascade architecture separates control into two loops running at different conceptual speeds:

```
                    Outer Loop (AC_P)
angle_target ──► [ × Kp ] ──► rate_setpoint
                                    │
                                    ▼
                    Inner Loop (PID)
                [ rate_error → PID ] ──► motor_command
                         ▲
                    gyro_x/y/z (raw, not from AHRS)
```

This mirrors exactly how ArduPilot's `AC_AttitudeControl_Multi::input_euler_angle_roll_pitch()` and `rate_controller_run()` work.

---

## New Modules vs v3.1

| Module                | File                 | ArduPilot Equivalent       |
| --------------------- | -------------------- | -------------------------- |
| Angle P controller    | `ac_p.c`             | `AC_P`                     |
| Cascade orchestration | `attitude_control.c` | `AC_AttitudeControl_Multi` |

### AC_P (Outer Loop)

Simple proportional controller with output rate clamp:

```
rate_setpoint = Kp × angle_error
rate_setpoint = clamp(rate_setpoint, ±rate_max_dps)
```

Includes a **0.5° deadband** — angle errors below this threshold produce zero rate setpoint, preventing sensor noise from generating unnecessary motor commands. Mirrors ArduPilot's input shaping dead zone.

### PID (Inner Loop) — Upgraded from v3.1

Added **D-term low-pass filter** matching ArduPilot's `FLTD` parameter:

```
raw_derivative   = (error - prev_error) / dt
filtered_deriv   = α × raw_derivative + (1-α) × prev_filtered
D                = Kd × filtered_deriv
```

`α = 0.557` → 20 Hz cutoff at 100 Hz loop rate. Prevents gyro noise amplification through the D-term.

### Throttle Scaling

At low throttle, motors spin slowly → less aerodynamic authority per PWM tick. The same PID output that corrects attitude at hover does almost nothing at 20% throttle.

Solution (mirrors `AC_AttitudeControl_Multi::rate_controller_run()`):

```
scale  = throttle / hover_throttle    (hover_throttle = 0.5)
output = raw_pid_output × scale
output = clamp(output, [-1, +1])
```

| Throttle    | Scale | Effect              |
| ----------- | ----- | ------------------- |
| 0.5 (hover) | 1.0   | Unchanged           |
| 0.2 (low)   | 0.4   | Reduced authority   |
| 0.8 (high)  | 1.6   | Increased authority |

### Yaw Rate Control

No outer angle loop for yaw — without a magnetometer, absolute heading cannot be measured. Instead, a **rate-only inner loop** runs on `gyro_z`:

```
yaw_rate_error = yaw_rate_target - gyro_z
out_yaw        = PID(yaw_rate_error)
```

At rest, `yaw_rate_target = 0` → controller actively resists any rotational disturbance on the yaw axis. Equivalent to ArduPilot's `ACRO` mode yaw behavior.

> **Note:** This stabilizes yaw _rate_, not yaw _angle_. If the vehicle is rotated 30° and released, the controller will stop further rotation but will not return to 0°. A magnetometer is required for full heading hold.

---

## Integral Management

Two mechanisms prevent integral windup:

**1. Anti-windup clamp** (in PID):

```
integral = clamp(integral, ±integral_limit)
```

**2. Integrator reset when in deadband** (in AttCtrl_Update):

```c
if (roll_rate_target  == 0.0f) rate_roll_pid.integral  = 0.0f;
if (pitch_rate_target == 0.0f) rate_pitch_pid.integral = 0.0f;
```

Mirrors ArduPilot's `relax_attitude_controllers()` — prevents accumulated integral from holding motors at non-zero output after returning to level.

---

## Default Gains

### Outer Loop (AC_P)

| Axis  | Kp  | Rate Max |
| ----- | --- | -------- |
| Roll  | 4.5 | 200 dps  |
| Pitch | 4.5 | 200 dps  |

### Inner Loop (PID)

| Axis  | Kp    | Ki    | Kd     | LPF   |
| ----- | ----- | ----- | ------ | ----- |
| Roll  | 0.135 | 0.135 | 0.0036 | 20 Hz |
| Pitch | 0.135 | 0.135 | 0.0036 | 20 Hz |
| Yaw   | 0.180 | 0.018 | 0.000  | —     |

All gains match ArduPilot's default `ATC_*` parameter values for a small quadcopter.

---

## File Structure

```
Inc/
  ac_p.h                    — AC_P struct + API
  attitude_control.h        — AttitudeControl_t struct + API
  ahrs.h                    — Mahony filter struct + API
  inertial_sensor.h         — Frontend public API
  inertial_sensor_backend.h — IMU_Sample_t + ISBackend_t interface
  inertial_sensor_mpu6050.h — MPU6050 backend probe API
  mavlink_telemetry.h       — MAVLink send functions
  motor_mix.h               — MotorOutput_t + Motor_* API
  pid.h                     — PID_t struct + API (with D-LPF)
  main.h                    — Pin definitions

Src/
  main.c                    — 100Hz main loop, system init
  ac_p.c                    — Proportional outer loop
  attitude_control.c        — Cascade orchestration
  ahrs.c                    — Mahony filter (roll, pitch, yaw)
  inertial_sensor.c         — Frontend orchestration
  inertial_sensor_mpu6050.c — MPU6050 driver
  mavlink_telemetry.c       — MAVLink message packing + UART TX
  motor_mix.c               — X-config motor mixing
  pid.c                     — PID controller with D-term LPF
  stm32f4xx_hal_msp.c       — HAL MSP peripheral init
  stm32f4xx_it.c            — Interrupt handlers

Tools/
  plotter.py                — Live Python visualizer (roll, pitch, yaw + PWM)
```

---

## Main Loop (100 Hz)

```
1. IS_Update()             — pull latest IMU sample
2. AHRS_Update()           — Mahony → roll, pitch, yaw
3. AttCtrl_SetTarget()     — set desired roll=0, pitch=0, yaw_rate=0
4. AttCtrl_Update()        — cascade → roll_cmd, pitch_cmd, yaw_cmd
5. Motor_Mix()             — commands → M1–M4 PWM
6. Motor_Write()           — write TIM3 compare registers
7. MAVLink_SendAttitude()  — 100Hz stream to GCS
8. MAVLink_SendHeartbeat() — 1Hz
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

## Live Telemetry Visualizer

`plotter.py` connects via serial and plots in real time:

- **Top panel**: Roll, Pitch, Yaw angles (degrees)
- **Bottom panel**: M1–M4 PWM values (µs)

**Requirements:**

```
pip install pyserial matplotlib
```

**UART format (7 comma-separated fields):**

```
+RR.rr,+PP.pp,+YY.yy,M1,M2,M3,M4\r\n
```

Example: `+0.00,+0.00,+0.00,1300,1300,1300,1300`

---

## Validated Behavior

| Test                           | Result                                                   |
| ------------------------------ | -------------------------------------------------------- |
| Level hover (0°, 0°)           | All motors = 1300, zero oscillation                      |
| Pitch disturbance (+10°, +39°) | Front motors increase, rear decrease — correct direction |
| Roll disturbance (+7°, +31°)   | Left/right differential — correct direction              |
| Return to level                | Motors return to 1300 exactly, no integral residual      |
| Motor saturation at 31° roll   | M1 hits PWM_MIN=1000 — expected at throttle=0.3          |

---

## Limitations

- Yaw angle drifts — magnetometer required for heading hold
- `hover_throttle = 0.5` is an estimate — requires tuning for actual frame
- Gains are ArduPilot defaults — require in-flight autotune for specific build
- No EEPROM — calibration lost on reset

---

## Roadmap

| Priority | Feature                                                     |
| -------- | ----------------------------------------------------------- |
| High     | FreeRTOS task separation (IMU, control, telemetry)          |
| High     | Magnetometer (HMC5883L) for yaw angle hold                  |
| Medium   | Feed-forward term (Kff) in inner loop                       |
| Medium   | Motor saturation handling (reduce throttle before clipping) |
| Low      | Arm/Disarm state machine                                    |
| Low      | Parameter storage in flash                                  |
