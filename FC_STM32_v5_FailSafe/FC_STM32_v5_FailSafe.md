# FC_STM32_v5_FailSafe

### STM32F411 Flight Controller — Hardware Safety Failsafe System

---

## Overview

This is version 5.0 of a custom quadcopter flight controller firmware built on the **STM32F411RE (NUCLEO-F411RE)** using STM32 HAL libraries.

This version adds a **hardware safety failsafe system** directly into the main control loop. All previous modules (Mahony AHRS, cascade controller, desaturating motor mixer, MAVLink telemetry) are carried forward unchanged from v4.1.

The failsafe system is designed for the critical transition from bench simulation to **real physical flight** — where an uncontrolled attitude can cause a crash, injury, or hardware damage within milliseconds.

---

## Motivation

In v4.0 and v4.1, the drone would continue attempting to control motors regardless of how extreme the attitude became. In a real flight scenario:

- A 60° tilt with spinning propellers → the drone is already crashing
- Continuing to run the PID controller in this state → makes it worse
- No software recovery is possible at extreme angles → immediate motor cutoff is the only safe action

This mirrors ArduPilot's philosophy in `AP_Arming` and the angle limit checks in `AC_AttitudeControl`.

---

## Failsafe Triggers

### 1. Tilt Failsafe (Roll / Pitch)

```c
#define TILT_FAILSAFE_DEG   45.0f
```

**Trigger condition:**

```c
fabsf(ahrs.roll_deg)  > TILT_FAILSAFE_DEG  ||
fabsf(ahrs.pitch_deg) > TILT_FAILSAFE_DEG
```

**Why 45°?**
At 45° tilt, the horizontal thrust component equals the vertical thrust component. The drone is no longer producing net upward force sufficient to maintain altitude — it is actively accelerating sideways. Recovery via software is unreliable at this point. 45° is the standard conservative threshold used in ArduPilot's angle limit checks.

---

### 2. Yaw Failsafe

```c
#define YAW_FAILSAFE_DEG    90.0f
```

**Trigger condition:**

```c
fabsf(ahrs.yaw_deg) > YAW_FAILSAFE_DEG
```

**Why 90°?**
The yaw controller holds rate = 0, not angle. If yaw drifts beyond 90°, it indicates sustained uncontrolled rotation — either a motor failure, mechanical imbalance, or ESC issue. Continuing flight in this state risks the drone spinning uncontrollably and leaving the safe operating area.

> **Note:** Without a magnetometer, yaw angle is Mahony-integrated gyro only and will drift slowly over time. The 90° threshold is chosen to be large enough to ignore normal drift but small enough to catch genuine uncontrolled rotation.

---

## Failsafe Response

When any condition triggers:

```c
Motor_Disarm(&htim3);     // all motors to PWM_MIN immediately
while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);   // LED blinks rapidly
    HAL_Delay(200);
}
```

**Hard stop with LED indication:**

- All 4 motors cut to PWM_MIN (1000 µs) instantly
- LD2 (PA5) blinks at 2.5 Hz indefinitely
- Requires manual power cycle to resume — no auto-recovery
- The `while(1)` prevents any further PID computation or motor commands

**Why no auto-recovery?**
At the moment of failsafe trigger, the drone is in an unsafe state with spinning propellers. Auto-recovery would require the drone to already be stable enough to recover — which contradicts the trigger condition. Manual intervention (power cycle, physical inspection) is always required before attempting another flight.

---

## Placement in Main Loop

The failsafe check runs **after AHRS update and before PID computation:**

```c
AHRS_Update(&ahrs, &ins.imu, DT);

/* ── Tilt + Yaw Failsafe ── */
if (fabsf(ahrs.roll_deg)  > TILT_FAILSAFE_DEG ||
    fabsf(ahrs.pitch_deg) > TILT_FAILSAFE_DEG ||
    fabsf(ahrs.yaw_deg)   > YAW_FAILSAFE_DEG)
{
    Motor_Disarm(&htim3);
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(200);
    }
}

/* PID control continues only if attitude is safe */
AttCtrl_Update(...);
Motor_Mix(...);
Motor_Write(...);
```

This placement ensures:

- Failsafe uses the freshest attitude estimate (just updated)
- No PID output is ever sent to motors during an unsafe attitude
- The check adds ~1 µs to loop time — negligible

---

## Defines Location

```c
/* USER CODE BEGIN PD */
#define TILT_FAILSAFE_DEG   45.0f
#define YAW_FAILSAFE_DEG    90.0f
/* USER CODE END PD */
```

---

## LED Blink Pattern

| State              | LD2 (PA5) behavior                          |
| ------------------ | ------------------------------------------- |
| Normal operation   | Solid ON (or slow HAL blink if implemented) |
| Failsafe triggered | Fast blink — 200ms ON / 200ms OFF (2.5 Hz)  |

The fast blink is immediately distinguishable from normal operation, allowing the operator to quickly identify that a failsafe — not a power failure — caused the motor cutoff.

---

## Pre-Flight Safety Checklist

Before any real flight attempt with this firmware:

1. **Verify axis orientation** — tilt drone right → `roll_deg` must go positive. Tilt forward → `pitch_deg` must go positive. If reversed, the controller corrects in the wrong direction and will flip immediately on takeoff.

2. **Props off first** — run motors without propellers, verify all 4 respond correctly and approximately equally to attitude commands.

3. **ESC calibration** — all ESCs must be calibrated to the same [1000, 2000] µs range before first flight. Uncalibrated ESCs produce unequal thrust at the same PWM command.

4. **Power cutoff ready** — have a physical way to cut power (switch or quick-disconnect) as a backup to the software failsafe.

5. **Throttle start low** — begin at throttle = 0.10 (1100 µs). Increment by 0.05 until hover point is found. Never jump directly to estimated hover throttle.

---

## Failsafe Thresholds — Tuning Guide

| Threshold           | Default | When to reduce                        | When to increase                |
| ------------------- | ------- | ------------------------------------- | ------------------------------- |
| `TILT_FAILSAFE_DEG` | 45°     | Indoor, confined space, first flights | Outdoor, aggressive maneuvers   |
| `YAW_FAILSAFE_DEG`  | 90°     | Magnetometer present (use 45°)        | No mag, slow yaw drift expected |

For first indoor flights, consider reducing `TILT_FAILSAFE_DEG` to **30°** for extra safety margin.

---

## Files Changed vs v4.1

| File            | Change                                                      |
| --------------- | ----------------------------------------------------------- |
| `main.c`        | Added failsafe check in control loop + `#define` thresholds |
| All other files | No change                                                   |

---

## ArduPilot Reference

`libraries/AP_Arming/AP_Arming.cpp` — pre-arm checks and angle limits
`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp` — angle limit enforcement

---

## Limitations

- Yaw failsafe is based on Mahony-integrated yaw only — will drift without magnetometer. The 90° threshold accounts for this.
- No position-based failsafe (horizontal drift) — requires optical flow or GPS sensor not yet integrated.
- No auto-recovery — manual power cycle required after every failsafe event.
- No failsafe logging — event is indicated by LED only, not recorded.

---

## Version History Context

```
v4.0  CascadeControl  — cascade controller, yaw rate, throttle scaling
v4.1  MotorDesat      — desaturating motor mixer
v5.0  FailSafe        — tilt + yaw failsafe (this version)
v5.x  (next)         — VL53L1X altitude hold + Mahony with magnetometer
```
