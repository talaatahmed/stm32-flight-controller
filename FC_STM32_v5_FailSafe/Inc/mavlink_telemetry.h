#ifndef INC_MAVLINK_TELEMETRY_H_
#define INC_MAVLINK_TELEMETRY_H_

#include "stm32f4xx_hal.h"

/* ─────────────────────────────────────────────────────────────────
 *  MAVLink Telemetry layer
 *
 *  Owns all MAVLink message construction and UART transmission.
 *  main.c never touches mavlink.h directly — it calls these two
 *  functions only.
 *
 *  Equivalent to ArduPilot's GCS_MAVLink module: the application
 *  layer calls high-level functions (send attitude, send heartbeat),
 *  and this layer handles serialization and transport.
 * ─────────────────────────────────────────────────────────────── */

/* MAVLink system identity — change if running multiple vehicles */
#define MAV_SYS_ID   1
#define MAV_COMP_ID  MAV_COMP_ID_AUTOPILOT1

/* ─── Init — must be called once before any send function ──── */
void MAVLink_Init(UART_HandleTypeDef *huart);

/* ─── Send HEARTBEAT message (call at 1Hz) ──────────────────── */
void MAVLink_SendHeartbeat(void);

/* ─── Send ATTITUDE message (call at 100Hz) ─────────────────── */
void MAVLink_SendAttitude(float roll_deg,  float pitch_deg,
                           float gyro_x_dps, float gyro_y_dps);

#endif /* INC_MAVLINK_TELEMETRY_H_ */
