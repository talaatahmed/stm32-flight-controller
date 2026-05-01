# FC_STM32_v4.1_MotorDesat

### STM32F411 Flight Controller — ArduPilot-style Motor Desaturation Mixer

---

## Overview

This is version 4.1 of a custom quadcopter flight controller firmware built on the **STM32F411RE (NUCLEO-F411RE)** using STM32 HAL libraries.

This version is a focused upgrade to the motor mixing layer only. The cascade controller, Mahony AHRS, and all other modules are carried forward unchanged from v4.0.

The key change is replacing the simple fixed-authority motor mixer with an **ArduPilot-style desaturating mixer** that mirrors `AP_MotorsMatrix::output_armed_stabilizing()`.

---

## The Problem with v4.0 Motor Mixer

The v4.0 mixer used a fixed ±200 tick authority and simple clamp:

```c
float r = roll  * 200.0f;   // fixed authority
out->m1 = clamp(base + r - p + y);  // silently loses correction on saturation
```

**Two fundamental problems:**

**1. Fixed authority** — The ±200 tick value was arbitrary and not related to the actual available PWM range or throttle level. At different throttle settings, the effective control authority changed unpredictably.

**2. Simple clamp on saturation** — When a motor hit PWM_MIN or PWM_MAX, the correction was silently discarded. This is the worst possible behavior: the controller demands a correction, the motor saturates, and the correction disappears exactly when it is needed most (during aggressive attitude recovery).

**Example of the failure:**

```
Drone tilts 30° → controller demands large roll correction
M1 wants 2200 µs → clamp to 2000 µs → correction lost
M4 wants 800 µs  → clamp to 1000 µs → correction lost
Result: drone continues tilting despite controller effort
```

---

## The Solution — Desaturation (AP_MotorsMatrix style)

Instead of clipping individual motors, the desaturating mixer shifts ALL motors equally to keep them within bounds, preserving the differential between motors (= attitude correction).

```
Old: motor = clamp(base + correction)
              ↑ sacrifices correction

New: if (any motor > PWM_MAX):
         shift ALL motors down by (max - PWM_MAX)
     if (any motor < PWM_MIN):
         shift ALL motors up by (PWM_MIN - min)
              ↑ preserves differential, trades total thrust
```

**Same example with desaturation:**

```
Drone tilts 30° → controller demands large roll correction
M1 raw = 2200 → all motors shift down by 200
M1 = 2000, M2 = 1700, M3 = 1100, M4 = 800+200 = 1000
Result: correction fully preserved, slight throttle reduction
```

---

## Implementation

The mixer now works entirely in **normalized float space [0.0, 1.0]** throughout, converting to PWM ticks only at the final step.

### Algorithm (4 steps)

**Step 1 — Mix in normalized space:**

```c
m[0] = throttle + r - p + y;   // M1 FL
m[1] = throttle - r - p - y;   // M2 FR
m[2] = throttle - r + p + y;   // M3 RR
m[3] = throttle + r + p - y;   // M4 RL
```

**Step 2 — Desaturate (shift all motors):**

```c
if (max > 1.0f): m[i] -= (max - 1.0f) for all i
if (min < 0.0f): m[i] += (-min)       for all i
```

**Step 3 — Safety clamp:**
Final clamp to [0.0, 1.0] handles extreme cases where both constraints cannot be satisfied simultaneously (e.g., full attitude correction at zero throttle). Attitude takes priority.

**Step 4 — Convert to PWM:**

```c
pwm = PWM_MIN + m[i] * (PWM_MAX - PWM_MIN)
```

### Authority Scaling

```c
#define ATTITUDE_AUTHORITY   0.5f
```

Roll/pitch/yaw inputs [-1, +1] are scaled by this factor before mixing. At 0.5, full deflection uses ±50% of the PWM range. Increase toward 1.0 for more aggressive authority, decrease toward 0.3 for smoother response.

---

## Files Changed

| File            | Change                                |
| --------------- | ------------------------------------- |
| `motor_mix.c`   | Complete rewrite — desaturating mixer |
| `motor_mix.h`   | No change — API unchanged             |
| All other files | No change                             |

**`main.c` requires zero modifications** — `Motor_Mix()` signature is identical to v4.0.

---

## Validated Behavior

From live test data with the cascade controller:

| Condition             | v4.0 behavior                                      | v4.1 behavior                                  |
| --------------------- | -------------------------------------------------- | ---------------------------------------------- |
| Level hover           | Correct                                            | Correct                                        |
| Roll 30° disturbance  | M1 clips to 1000, correction partially lost        | M1=1000, M2=1600 — differential preserved      |
| Roll 31° (aggressive) | M2 clips at 2000, M1 clips at 1000 — both saturate | All motors shifted — correction fully executed |
| Return to level       | Motors return to ~1300                             | Motors return to 1300 exactly                  |

The pattern `M1=1000, M2=1600, M3=1600, M4=1000` observed during 30° roll recovery confirms the desaturation is working — the correction is preserved at the cost of a slight throttle reduction across all motors.

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

| Motor | Mix formula                   |
| ----- | ----------------------------- |
| M1 FL | throttle + roll - pitch + yaw |
| M2 FR | throttle - roll - pitch - yaw |
| M3 RR | throttle - roll + pitch + yaw |
| M4 RL | throttle + roll + pitch - yaw |

---

## ArduPilot Reference

`libraries/AP_Motors/AP_MotorsMatrix.cpp`
`AP_MotorsMatrix::output_armed_stabilizing()`

---

## Version History Context

```
v4.0  CascadeControl  — cascade controller, yaw rate, throttle scaling
v4.1  MotorDesat      — desaturating motor mixer (this version)
v5.0  FailSafe        → next
```
