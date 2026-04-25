# FC_STM32_v1_Baseline

**A stabilizing flight controller built from scratch on STM32F411RE + MPU6050.**

This is the baseline project — Phase 1 through Phase 7 of a complete
flight controller implementation. Every concept was built manually,
without abstraction layers, so that the fundamentals are fully visible.
It is the reference point that v2 was refactored from.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | STM32F411RE (NUCLEO-F411RE board) |
| IMU | MPU6050 (I2C, address 0x68) |
| Motors | 4× brushless via ESC, PWM on TIM3 CH1–CH4 |
| Telemetry | USART2 → USB (ST-Link virtual COM) |

---

## What Was Built — Phase by Phase

### Phase 1 & 2 — IMU Driver (`mpu6050.c / mpu6050.h`)

**What was learned:**
- How to communicate with an I2C sensor using STM32 HAL
  (`HAL_I2C_Master_Transmit` / `HAL_I2C_Master_Receive`)
- The MPU6050 register map: `WHO_AM_I`, `PWR_MGMT_1`, `SMPLRT_DIV`,
  `CONFIG`, `GYRO_CONFIG`, `ACCEL_CONFIG`, `ACCEL_XOUT_H`
- How raw 16-bit signed integers from the sensor become physical units:
  - Accel: `raw / 16384.0` → g (at ±2g range)
  - Gyro:  `raw / 131.0`   → °/s (at ±250°/s range)
- Static bias calibration: collect N samples at rest, compute mean,
  subtract from every future reading. Z-axis accel corrected for gravity.

**Key code:**
```c
MPU6050_Init()        // wake up sensor, configure registers
MPU6050_ReadRaw()     // read 14 bytes (accel + temp + gyro) in one burst
MPU6050_ScaleData()   // raw int16 → float in SI units
MPU6050_Calibrate()   // 500-sample average → bias struct
MPU6050_ApplyBias()   // subtract bias from every reading
```

---

### Phase 3 — Complementary Filter (`mpu6050.c`)

**What was learned:**
- Why neither sensor alone is sufficient:
  - Accelerometer gives absolute angle but is noisy and disturbed by vibration
  - Gyroscope gives clean rate but drifts over time when integrated
- The complementary filter blends both:
  ```
  angle = α × (angle + gyro_rate × dt) + (1 − α) × accel_angle
  ```
  with α = 0.98 (trust gyro 98% short-term, correct drift via accel long-term)
- How to derive roll and pitch from raw accel using `atan2f`

**Key insight:** The filter has no memory of the past beyond the last
angle estimate. It is stateless except for `attitude.roll` and
`attitude.pitch`. This is both its strength (simplicity) and its
weakness (no sensor fusion, no noise model).

---

### Phase 4 — PID Controller (`pid.c / pid.h`)

**What was learned:**
- The three terms of a PID controller and their physical meaning:
  - **P** (Proportional): corrects in proportion to current error
  - **I** (Integral): eliminates steady-state offset over time
  - **D** (Derivative): damps oscillation by reacting to rate of change
- Integral windup and how to prevent it with an `i_limit` clamp
- Output saturation via `out_limit`
- Why tuning order matters: P first, then D, then I last

**Starting gains:** Kp=1.0, Ki=0.0, Kd=0.1 (Ki intentionally zero
until P+D are stable).

---

### Phase 5 — Visualizer

**What was learned:**
- Sending structured serial data over USART2 at 115200 baud using
  `HAL_UART_Transmit`
- Formatting float values as fixed-point strings without `printf`
  (avoid heavy stdlib on embedded)
- Reading the attitude and PID output in real-time to verify the
  filter and controller are behaving correctly

---

### Phase 6 — MAVLink Telemetry (`main.c`)

**What was learned:**
- What MAVLink is: a lightweight binary message protocol used by
  ArduPilot, PX4, and most modern autopilots
- How to pack and send two message types:
  - `HEARTBEAT`: identifies the system to a ground station
  - `ATTITUDE`: sends roll, pitch, yaw + angular rates
- The MAVLink packet structure: start byte, length, sequence,
  system ID, component ID, message ID, payload, CRC
- How to view MAVLink output using MAVProxy or Mission Planner

---

### Phase 7 — Motor Mixing + PWM Output (`motor_mix.c`, `MX_TIM3_Init`)

**What was learned:**
- How PWM works for ESC control:
  - Period = 20ms (50Hz), pulse width 1000–2000µs
  - TIM3 configured: Prescaler=83, Period=19999 → 1µs resolution at 84MHz
- Quadcopter motor layout (X-frame):
  ```
  M1(FL, CCW)   M2(FR, CW)
  M4(BL, CW)    M3(BR, CCW)
  ```
- The mixing equations that translate throttle + PID output into
  four independent motor commands:
  ```
  M1 = throttle + roll + pitch
  M2 = throttle - roll + pitch
  M3 = throttle - roll - pitch
  M4 = throttle + roll - pitch
  ```
- Why motor outputs must be clamped to [1000, 2000] to stay within
  valid ESC range

---

## Main Loop Structure

```
while(1):
    loop_start = HAL_GetTick()

    MPU6050_ReadRaw()               // ~1.4ms I2C blocking read
    MPU6050_ScaleData()
    MPU6050_ApplyBias()
    MPU6050_ComplementaryFilter()   // attitude update
    PID_Update(roll)
    PID_Update(pitch)
    MotorMix_Update()               // write PWM to ESCs
    send_mavlink_attitude()         // USART2 telemetry

    HAL_Delay(10 - elapsed)         // pace loop to 100Hz
```

**Loop rate:** 100Hz (`LOOP_MS = 10`)
**Sensor rate:** same 100Hz — one read per loop (polling)

---

## Known Limitations (Fixed in v2)

| Limitation | Impact |
|---|---|
| Sensor polled once per loop | 98.75% of the sensor's data discarded (MPU6050 can output 1kHz) |
| I2C read blocks the CPU | Loop timing jitter if read takes longer than expected |
| `mpu6050.c` owns everything | To swap sensor, must rewrite all driver code |
| No separation of concerns | Application code knows I2C addresses and register names |
| Single complementary filter | No noise model, no multi-sensor fusion capability |

---

## Files

```
Core/
├── Inc/
│   ├── mpu6050.h       IMU driver interface
│   ├── pid.h           PID controller interface
│   └── motor_mix.h     Motor mixing interface
└── Src/
    ├── main.c          System init, main loop, MAVLink
    ├── mpu6050.c       IMU driver (init, read, calibrate, filter)
    ├── pid.c           PID controller
    └── motor_mix.c     PWM output + motor mixing
```

---

## Key Numbers to Remember

| Parameter | Value | Why |
|---|---|---|
| Loop rate | 100 Hz | `LOOP_MS = 10` |
| Sensor sample rate | 100 Hz | `SMPLRT_DIV = 0x07` → 125Hz (close enough) |
| I2C clock | 100 kHz | Standard mode |
| DLPF cutoff | 44 Hz | `CONFIG = 0x03` |
| Gyro range | ±250 °/s | `GYRO_CONFIG = 0x00` |
| Accel range | ±2 g | `ACCEL_CONFIG = 0x00` |
| Complementary α | 0.98 | 98% gyro, 2% accel per sample |
| PWM frequency | 50 Hz | Standard ESC protocol |
| Calibration samples | 500 | ~1 second at rest |
