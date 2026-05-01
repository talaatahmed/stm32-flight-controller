# STM32 Flight Controller

A modular, bare-metal quadcopter flight controller built from scratch on the **STM32F411RE (NUCLEO-F411RE)** using the MPU-6050 IMU. The project evolves version by version, mirroring the architecture of ArduPilot at each stage — from a simple polling loop to a full cascade attitude controller with safety failsafes.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | STM32F411RETx (NUCLEO-F411RE) |
| IMU | MPU-6050 via I2C1 (PB8=SCL, PB9=SDA) |
| Motors | 4× ESC via TIM3 PWM (PA6, PA7, PB0, PB1) |
| Telemetry | MAVLink over USART2 (PA2=TX, PA3=RX) |
| Toolchain | STM32CubeIDE, bare-metal HAL, no RTOS |

---

## Repository Structure

Each folder is a self-contained STM32CubeIDE project. Every version builds on the previous one with a focused, documented change.

```
FC_STM32_v1_Baseline/
FC_STM32_v2_FIFO_Frontend/
FC_STM32_v3_IIR_Filter/
FC_STM32_v3.1_AHRS/
FC_STM32_v3.2_Mahony/
FC_STM32_v4_CascadeControl/
FC_STM32_v4.1_MotorDesat/
FC_STM32_v5_FailSafe/
```

---

## Version History

### v1 — Baseline
Single-file IMU driver, complementary filter, single-loop PID, X-config motor mixing, MAVLink telemetry, Python live visualizer. All seven foundational phases implemented and verified.

### v2 — FIFO Frontend
ArduPilot-style frontend/backend IMU split. MPU-6050 backend uses FIFO drain + averaging (~10 samples per loop at 1kHz sensor rate). Frontend is a thin orchestration layer with no hardware knowledge. Adding a new IMU requires only a new backend file.

### v3 — IIR Filter
Two-stage filtering pipeline: hardware DLPF corrected from 184Hz to 42Hz (proper anti-aliasing for 100Hz output rate), plus a software first-order IIR filter (α=0.80) as a second smoothing stage after FIFO averaging. Only the MPU-6050 backend changed.

### v3.1 — AHRS
Attitude estimation extracted from the IMU frontend into a dedicated `ahrs.c` layer — mirroring ArduPilot's `AP_InertialSensor` / `AP_AHRS` separation. MAVLink telemetry moved to its own `mavlink_telemetry.c` module. `main.c` no longer contains any sensor or protocol knowledge.

### v3.2 — Mahony
Complementary filter replaced with a **Mahony filter** in quaternion space. PI feedback from accelerometer cross-product error corrects gyro drift. No gimbal lock at ±90° pitch. Numerically stable at large angles. Adds `yaw_deg` output. Public API unchanged — `main.c` required no modification.

### v4 — CascadeControl
Full **ArduPilot-style cascade controller** replacing the single-loop PID:
- Outer loop: `AC_P` — angle error → rate setpoint (mirrors `AC_P`)
- Inner loop: `PID` with D-term LPF at 20Hz (mirrors `AC_PID` with `FLTD`)
- Yaw rate control (inner loop only, no magnetometer required)
- Throttle scaling: `output *= throttle / hover_throttle`
- Angle deadband (0.5°) and rate deadband (2.0 dps) suppress sensor noise
- Integrator reset when in deadband (mirrors `relax_attitude_controllers()`)
- All default gains match ArduPilot `ATC_*` parameter defaults

### v4.1 — MotorDesat
Motor mixer rewritten to match **`AP_MotorsMatrix::output_armed_stabilizing()`**. Works in normalized float space throughout. On saturation, shifts all motors equally to preserve the attitude differential — trades total thrust for full correction authority. Replaces the previous fixed ±200 tick clamp which silently discarded corrections on saturation. `Motor_Mix()` API unchanged.

### v5 — FailSafe
Tilt and yaw failsafes added for real flight safety:
- `TILT_FAILSAFE_DEG = 45°` — immediate motor cutoff if roll or pitch exceeded
- `YAW_FAILSAFE_DEG = 90°` — immediate motor cutoff if yaw drifts beyond threshold
- Hard stop: `Motor_Disarm()` + LED blink at 2.5Hz + infinite loop
- Requires manual power cycle to resume — no auto-recovery
- Mirrors ArduPilot `AP_Arming` angle limit enforcement

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

---

## Data Flow (v5 — current)

```
MPU-6050 @ 1kHz
    ↓  FIFO drain + averaging + IIR
inertial_sensor_mpu6050.c  (backend)
    ↓  IMU_Sample_t
inertial_sensor.c          (frontend)
    ↓  gyro_dps, accel_g
ahrs.c                     (Mahony filter)
    ↓  roll_deg, pitch_deg, yaw_deg
attitude_control.c         (AC_P outer + PID inner)
    ↓  roll_cmd, pitch_cmd, yaw_cmd
motor_mix.c                (desaturating X-config mixer)
    ↓  M1–M4 PWM [1000–2000µs]
4× ESC / Motors via TIM3
```

---

## Tools

`plotter.py` — Live Python visualizer over UART. Plots roll, pitch, yaw angles and M1–M4 PWM values in real time at 100Hz.

```
pip install pyserial matplotlib
```

---

## Next Steps

| Feature | Description |
|---|---|
| Altitude hold | VL53L1X laser rangefinder via I2C — altitude PID loop |
| Yaw heading hold | HMC5883L magnetometer — Mahony with mag correction |
| RC Input | PWM/PPM receiver — pilot-controlled STABILIZE mode |
| FreeRTOS | IMU task at 1kHz, control task at 400Hz |
