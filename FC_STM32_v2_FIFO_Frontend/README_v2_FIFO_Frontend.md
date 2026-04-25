# FC_STM32_v2_FIFO_Frontend

**A refactored flight controller that adopts ArduPilot's sensor
architecture: FIFO-based IMU sampling and a frontend/backend driver split.**

This project starts from the working v1 baseline and applies two
architectural upgrades inspired by studying ArduPilot's
`AP_InertialSensor` library. The flight behavior is identical to v1.
The internal structure is fundamentally different.

---

## Hardware

Same as v1:

| Component | Detail |
|---|---|
| MCU | STM32F411RE (NUCLEO-F411RE board) |
| IMU | MPU6050 (I2C address 0x68, fast mode 400kHz) |
| Motors | 4× brushless via ESC, PWM on TIM3 CH1–CH4 |
| Telemetry | USART2 → USB (ST-Link virtual COM) |

---

## Why This Project Exists

In v1, the IMU was polled once per loop at 100Hz. The MPU6050 is
capable of 1kHz output. This means **98.75% of sensor data was
discarded**, and the single sample read each loop contained all
the high-frequency vibration noise with no averaging.

Additionally, `mpu6050.c` mixed hardware knowledge (I2C addresses,
register values, scale factors) with application logic (calibration,
filtering). Swapping sensors would require rewriting everything.

This project fixes both problems.

---

## What Was Learned

### Lesson 1 — The MPU6050 FIFO Buffer

**The problem with polling:**
Every call to `MPU6050_ReadRaw()` in v1 read the sensor's output
registers directly. These registers hold only the *most recent* sample
and are overwritten every 1ms (at 1kHz). Polling at 100Hz means reading
one sample and ignoring the 9 that arrived in between.

**What the FIFO does:**
The MPU6050 has an internal 1024-byte FIFO buffer. When enabled, the
sensor pushes every new sample into it automatically. The MCU can then
read however many samples have accumulated, at any time, without missing
any data.

**How to enable it (register sequence):**
```
SMPLRT_DIV = 0x00     → sample rate = 1kHz / (1+0) = 1kHz
CONFIG     = 0x01     → DLPF_CFG=1, BW=184Hz, gyro_rate stays 1kHz
FIFO_EN    = 0x78     → enable accel XYZ + gyro XYZ into FIFO (bits 6,5,4,3)
USER_CTRL  = 0x44     → FIFO_EN=1 + FIFO_RESET=1 (start fresh)
```

**FIFO packet format:**
Each sample is exactly 12 bytes: 6 bytes accel (XYZ) + 6 bytes gyro (XYZ).
All values are big-endian signed 16-bit integers.

**Reading the FIFO:**
1. Read `FIFO_COUNT_H` + `FIFO_COUNT_L` (registers 0x72–0x73) → 2 bytes
2. Divide by 12 → number of complete samples available
3. Read that many 12-byte packets from register `FIFO_R_W` (0x74)
4. If `FIFO_COUNT >= 1008`, the buffer is near-full → reset it

**Stage A discovery (the race condition):**
When the MCU polls at 100Hz but the sensor writes at 125Hz
(`SMPLRT_DIV=0x09`), the FIFO fills up slowly and eventually overflows.
Observed output:
```
FIFO: 12 → 24 → 36 → ... → 996 → RESET → 12 → ...
```
Fix: drain ALL available samples every loop, not just one.

**Stage B result (1kHz sensor + averaging):**
With `SMPLRT_DIV=0x00` (1kHz) and a drain-all + average loop:
```
got: 11
got: 10
got: 11
got: 11
...
```
10–11 samples averaged per loop. Effective noise reduction: √10 ≈ 3.16×
improvement in signal quality, at zero hardware cost.

**Why 11 instead of 10:**
`HAL_UART_Transmit` for debug printing blocks the CPU slightly,
making the loop slightly longer than 10ms. The sensor writes one
extra sample in that extra time. This is called **scheduler jitter**
and is solved in ArduPilot by using DMA for I2C (CPU never blocks).

---

### Lesson 2 — Frontend / Backend Architecture

**The ArduPilot pattern:**
ArduPilot's `AP_InertialSensor` library is split into two layers:

- **Frontend** (`AP_InertialSensor`): the application-facing API.
  Only exposes `get_gyro()`, `get_accel()`, and attitude. Knows nothing
  about which physical sensor is attached.

- **Backend** (`AP_InertialSensor_Backend`): one implementation per
  sensor chip (MPU6050, ICM-42688, BMI160, etc.). Owns all hardware
  knowledge: I2C addresses, register maps, FIFO format, scale factors.

- **Probe pattern**: at boot, the frontend tries each known backend
  in sequence. The first one whose `probe()` returns success (i.e., the
  sensor answered on the bus) is registered and used. All others are
  ignored.

**How this was implemented in C (without C++ virtual functions):**
C++ uses vtables for polymorphism. In C, function pointers achieve
the same thing:

```c
// Abstract interface — any backend must fill these in
typedef struct {
    void *context;                                    // opaque device state
    HAL_StatusTypeDef (*init)(void *context);         // hardware setup
    HAL_StatusTypeDef (*read)(void *context,          // read + average
                              IMU_Sample_t *out,
                              uint8_t *n);
} ISBackend_t;
```

The frontend stores one `ISBackend_t`. It never knows the type of
`context` — it passes it back to the function pointers blindly.
Inside each backend's `.c` file, the context is cast to its true type:

```c
static HAL_StatusTypeDef mpu6050_read_avg(void *ctx, ...) {
    MPU6050_Ctx_t *c = (MPU6050_Ctx_t *)ctx;   // ← the "downcast"
    ...
}
```

This is the same concept as a C++ `this` pointer and virtual dispatch,
written manually.

**How to add a new sensor later:**
Write `inertial_sensor_icm42688.c` with its own `probe()`, `init()`,
and `read()`. Add one line to `IS_Init()` in `inertial_sensor.c`:
```c
if (ICM42688_Backend_Probe(&ins->backend, hi2c) == HAL_OK) goto found;
```
Nothing else changes. `main.c` does not know the sensor changed.

---

### Lesson 3 — Connection to ArduPilot Source Code

These are the ArduPilot files that directly correspond to this project:

| ArduPilot file | Equivalent here |
|---|---|
| `AP_InertialSensor.h/.cpp` | `inertial_sensor.h/.c` |
| `AP_InertialSensor_Backend.h` | `inertial_sensor_backend.h` |
| `AP_InertialSensor_Invensense.cpp` | `inertial_sensor_mpu6050.c` |
| `ADD_BACKEND(...)` macro in `.cpp` | probe chain in `IS_Init()` |
| `_publish_gyro()` accumulation | `mpu6050_read_avg()` sum loop |

The main difference in ArduPilot: the backend runs in a **separate
thread** (bus thread at high priority), and uses **DMA** for I2C
transfers. The frontend reads from a shared ring buffer. This project
uses polling in the same thread — the concept is identical, the
concurrency is absent (that comes with FreeRTOS).

---

## File Structure

```
Core/
├── Inc/
│   ├── inertial_sensor.h           Frontend public API
│   ├── inertial_sensor_backend.h   Abstract backend interface
│   ├── inertial_sensor_mpu6050.h   MPU6050 backend (probe + calibrate)
│   ├── pid.h                       Unchanged from v1
│   └── motor_mix.h                 Unchanged from v1
└── Src/
    ├── main.c                      Simplified — uses IS_* API only
    ├── inertial_sensor.c           Frontend (probe chain, comp filter)
    ├── inertial_sensor_mpu6050.c   Backend (FIFO, averaging, calibration)
    ├── pid.c                       Unchanged from v1
    └── motor_mix.c                 Unchanged from v1
```

`mpu6050.c` and `mpu6050.h` from v1 are **excluded from build**
(kept for reference).

---

## main.c — Before vs. After

**v1 (application knew everything):**
```c
MPU6050_RawData_t    raw_data;
MPU6050_ScaledData_t scaled_data;
MPU6050_Bias_t       imu_bias;

MPU6050_Init(&hi2c1);
MPU6050_Calibrate(&hi2c1, &imu_bias, 500);

while(1) {
    uint8_t n;
    MPU6050_ReadFromFIFO(&hi2c1, &raw_data, &n);
    MPU6050_ScaleData(&raw_data, &scaled_data);
    MPU6050_ApplyBias(&scaled_data, &imu_bias);
    MPU6050_ComplementaryFilter(&scaled_data, &attitude, DT, ALPHA);
    // ...
}
```

**v2 (application sees only attitude):**
```c
InertialSensor_t ins;

IS_Init(&ins, &hi2c1);
IS_Calibrate(&ins, 500);

while(1) {
    uint8_t new_data = 0;
    if (IS_Update(&ins, DT, &new_data) == HAL_OK && new_data) {
        float out_roll  = PID_Update(&pid_roll,  SETPOINT, ins.roll_deg, DT);
        float out_pitch = PID_Update(&pid_pitch, SETPOINT, ins.pitch_deg, DT);
        // ...
    }
}
```

`main.c` no longer contains any I2C addresses, register names,
scale factors, or FIFO logic. It only knows: initialize → calibrate → update.

---

## Key Numbers to Remember

| Parameter | v1 | v2 | Why changed |
|---|---|---|---|
| Sensor sample rate | ~125 Hz | 1000 Hz | `SMPLRT_DIV = 0x00` |
| DLPF cutoff | 44 Hz (`0x03`) | 184 Hz (`0x01`) | Wider BW, let software filter |
| Samples per loop | 1 | ~10–11 (averaged) | Drain-all FIFO strategy |
| I2C clock | 100 kHz | 400 kHz | Required for 1kHz throughput |
| Noise improvement | baseline | ~3× (√10) | Averaging effect |
| Sensor swap cost | rewrite all | add one `.c` file | Frontend/backend split |

---

## What Comes Next (Stage 2 and 3)

**Stage 2 — INT-pin driven sampling:**
The MPU6050 has an INT pin that pulses every new sample (every 1ms
at 1kHz). Connect it to an STM32 EXTI line. The ISR sets a flag.
The main loop reads only when the flag is set. This eliminates the
`FIFO_COUNT` polling and aligns the read precisely to the sensor's
output rate. Requires: Timers/EXTI from the STM32 Timers/PWM/DMA course.

**Stage 3 — RTOS-based separation:**
Move `IS_Update()` into a high-priority FreeRTOS task running at 1kHz.
Move the PID + motor mixing into a medium-priority task at 400Hz.
Share data via a mutex-protected buffer. This is the exact architecture
of ArduPilot running on ChibiOS. Requires: FreeRTOS course.
