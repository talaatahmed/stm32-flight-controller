/* ─────────────────────────────────────────────────────────────────
 *  MAVLink Telemetry layer — implementation
 *
 *  This file owns all MAVLink knowledge:
 *    - message packing (heartbeat, attitude)
 *    - unit conversion (degrees → radians for MAVLink)
 *    - UART transmission
 *
 *  main.c knows nothing about MAVLink message formats or IDs.
 *  It only calls MAVLink_SendHeartbeat() and MAVLink_SendAttitude().
 *
 *  To add a new message (e.g. BATTERY_STATUS, GPS_RAW_INT):
 *    1. Add a new function here.
 *    2. Add its declaration to mavlink_telemetry.h.
 *    3. Call it from main.c at the appropriate rate.
 *    Nothing else changes.
 * ─────────────────────────────────────────────────────────────── */

#include "mavlink_telemetry.h"
#include "mavlink.h"
#include "common/mavlink_msg_attitude.h"
#include "common/common.h"

/* Degrees to radians — MAVLink attitude message uses radians */
#define DEG_TO_RAD  0.01745329f

/* ─── Private: UART handle stored at init ───────────────────── */
static UART_HandleTypeDef *s_huart = NULL;

/* ────────────────────────────────────────────────────────────────
 *  MAVLink_Init — store the UART handle for all future transmits.
 *  Call once in main() before the loop.
 * ────────────────────────────────────────────────────────────── */
void MAVLink_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
}

/* ────────────────────────────────────────────────────────────────
 *  MAVLink_SendHeartbeat — tell the GCS we are alive.
 *
 *  Call at 1Hz. The GCS (Mission Planner / QGroundControl) will
 *  show the vehicle as disconnected if heartbeat stops for > 3s.
 * ────────────────────────────────────────────────────────────── */
void MAVLink_SendHeartbeat(void)
{
    if (!s_huart) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        MAV_SYS_ID, MAV_COMP_ID, &msg,
        MAV_TYPE_QUADROTOR,       /* vehicle type */
        MAV_AUTOPILOT_GENERIC,    /* autopilot type */
        MAV_MODE_MANUAL_ARMED,    /* base mode */
        0,                        /* custom mode */
        MAV_STATE_ACTIVE);        /* system status */

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(s_huart, buf, len, 50);
}

/* ────────────────────────────────────────────────────────────────
 *  MAVLink_SendAttitude — stream roll/pitch/gyro to the GCS.
 *
 *  Call at 100Hz (every loop iteration when new IMU data arrives).
 *
 *  Inputs are in degrees and degrees/sec (our internal units).
 *  MAVLink ATTITUDE message requires radians and radians/sec,
 *  so we convert here — keeping the rest of the system in degrees.
 * ────────────────────────────────────────────────────────────── */
void MAVLink_SendAttitude(float roll_deg,  float pitch_deg,
                           float gyro_x_dps, float gyro_y_dps)
{
    if (!s_huart) return;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /* Convert to radians for MAVLink */
    float roll_rad    = roll_deg    * DEG_TO_RAD;
    float pitch_rad   = pitch_deg   * DEG_TO_RAD;
    float gyro_x_rad  = gyro_x_dps  * DEG_TO_RAD;
    float gyro_y_rad  = gyro_y_dps  * DEG_TO_RAD;

    mavlink_msg_attitude_pack(
        MAV_SYS_ID, MAV_COMP_ID, &msg,
        HAL_GetTick(),  /* timestamp in ms */
        roll_rad,
        pitch_rad,
        0.0f,           /* yaw — not yet estimated (needs magnetometer) */
        gyro_x_rad,
        gyro_y_rad,
        0.0f);          /* yaw rate */

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(s_huart, buf, len, 50);
}
