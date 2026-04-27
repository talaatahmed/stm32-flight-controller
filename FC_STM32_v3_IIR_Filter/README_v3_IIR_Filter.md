# FC_STM32_v3_IIR_Filter

**A flight controller build that adds a two-stage hardware + software
filtering pipeline to the IMU data path, inspired by ArduPilot's
anti-aliasing and signal conditioning approach.**

This version builds directly on v2's FIFO + frontend/backend
architecture and adds the missing signal processing layer between
raw sensor data and the attitude estimator.

---

## Hardware

| Component | Detail                                        |
| --------- | --------------------------------------------- |
| MCU       | STM32F411RE (NUCLEO-F411RE board)             |
| IMU       | MPU6050 (I2C address 0x68, fast mode 400 kHz) |
| Motors    | 4× brushless via ESC, PWM on TIM3 CH1–CH4     |
| Telemetry | USART2 → USB (ST-Link virtual COM) + MAVLink  |

---

## What Changed From v2

### Problem: The v2 filter configuration was incorrect

In v2, the hardware Digital Low-Pass Filter (DLPF) on the MPU6050
was set to `DLPF_CFG = 1` → bandwidth = 184 Hz.

But the output rate of our system is 100 Hz (we read the FIFO every
10 ms). By the Nyquist theorem, any signal above 50 Hz aliases back
into the 0–50 Hz band after sampling, appearing as phantom low-frequency
noise. A 184 Hz hardware filter does not prevent this.

**The correct approach (what ArduPilot does):**
Set the hardware DLPF cutoff to just below the Nyquist limit of the
output rate. For a 100 Hz output rate, Nyquist = 50 Hz, so the hardware
filter must cut off below 50 Hz before the data enters the FIFO.

### Fix 1 — Hardware DLPF corrected (register CONFIG, 0x1A)

```
Old: DLPF_CFG = 1  →  BW = 184 Hz  ← aliasing risk
New: DLPF_CFG = 3  →  BW =  42 Hz  ← correct anti-alias for 100 Hz output
```

The gyro sample rate stays at 1 kHz (SMPLRT_DIV = 0x00 unchanged),
so the FIFO still fills at 1 kHz and we still average ~10 samples per
loop. Only the filter bandwidth changed.

### Fix 2 — Software IIR Low-Pass Filter added

After the FIFO drain and averaging step, a single-pole (first-order)
IIR filter is applied to all 6 axes as a second smoothing stage:

```
y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
```

Where `alpha = 0.80` (tunable via `IIR_ALPHA` define in
`inertial_sensor_mpu6050.c`).

The filter state is stored inside `MPU6050_Ctx_t` — it lives in the
backend context, invisible to the frontend and application code. On
the first call after calibration, the filter is seeded with the raw
value to prevent a startup transient spike.

### Complete data path (v3)

```
MPU6050 sensor @ 1 kHz
    ↓
Hardware DLPF — BW = 42 Hz   ← anti-aliasing (new)
    ↓
FIFO @ 1 kHz
    ↓
FIFO drain every 10 ms → average ~10 samples  ← box filter
    ↓
IIR software filter — alpha = 0.80            ← second stage (new)
    ↓
Bias subtraction (from IS_Calibrate)
    ↓
Complementary filter → roll_deg / pitch_deg
    ↓
PID → Motor mixing
```

---

## Lesson Learned: The Anti-Aliasing Role of the Hardware DLPF

The MPU6050's hardware DLPF is not just a noise-reduction feature.
Its primary role in a FIFO-based system is **anti-aliasing** — it
must block all frequencies above the Nyquist limit of the output
rate _before_ data enters the FIFO, not after.

ArduPilot sets the hardware DLPF to match the intended output rate:
hardware filter first, software filter second. This version now
follows the same principle.

| Parameter       | v2         | v3             | Why changed                          |
| --------------- | ---------- | -------------- | ------------------------------------ |
| DLPF_CFG        | 1 (184 Hz) | 3 (42 Hz)      | Correct anti-alias for 100 Hz output |
| Software filter | none       | IIR alpha=0.80 | Second smoothing stage               |
| Everything else | —          | unchanged      | Architecture intact                  |

---

## File Structure

Only `inertial_sensor_mpu6050.c` changed from v2. All other files
are identical.

```
Core/
├── Inc/
│   ├── inertial_sensor.h           Frontend public API (unchanged)
│   ├── inertial_sensor_backend.h   Abstract backend interface (unchanged)
│   ├── inertial_sensor_mpu6050.h   MPU6050 backend header (unchanged)
│   ├── pid.h                       (unchanged)
│   └── motor_mix.h                 (unchanged)
└── Src/
    ├── main.c                      (unchanged)
    ├── inertial_sensor.c           Frontend (unchanged)
    ├── inertial_sensor_mpu6050.c   ← CHANGED: DLPF + IIR added
    ├── pid.c                       (unchanged)
    └── motor_mix.c                 (unchanged)
```

---

## What Comes Next

**v3.1 — AHRS separation:**
Move the complementary filter out of `inertial_sensor.c` and into a
dedicated `ahrs.c` layer, mirroring ArduPilot's separation of
`AP_InertialSensor` (raw data) from `AP_AHRS` (attitude estimation).
This makes it trivial to replace the complementary filter with a
Mahony, Madgwick, or EKF algorithm later without touching the IMU driver.
