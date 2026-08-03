/**
 * @file    app_gimbal_comm.c
 * @brief   云台CAN通信协议实现
 * @note    0x120 收到后自动转发到 0x111 给底盘
 *          CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include "bsp_can.h"
#include "project_cfg.h"

#include <string.h>

// ─── 私有变量 ────────────────────────────────────

static app_gimbal_radar_speed_cmd_t s_radar_speed;
static app_gimbal_speed_cmd_t       s_speed_no_shoot;
static app_gimbal_angle_cmd_t       s_angle_no_shoot;
static app_gimbal_speed_cmd_t       s_speed_shoot;
static app_gimbal_angle_cmd_t       s_angle_shoot;
static app_gimbal_control_cmd_t     s_control;

// ─── CAN RX 回调 (中断上下文) ──────────────────────

#if CURRENT_BOARD == BOARD_GIMBAL

static void on_radar_speed(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_radar_speed, data, 8);

    /* 协议规定: 云台收到 0x120 后立即转发到 0x111 给底盘 */
    app_chassis_comm_send_speed_cmd(s_radar_speed.vx,
                                    s_radar_speed.vy,
                                    s_radar_speed.vz,
                                    s_radar_speed.power_pct);
}

static void on_speed_no_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_speed_no_shoot, data, 8);
}

static void on_angle_no_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_angle_no_shoot, data, 8);
}

static void on_speed_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_speed_shoot, data, 8);
}

static void on_angle_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_angle_shoot, data, 8);
}

static void on_control_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    /* 协议规定 DLC=4 */
    if (len < 4) return;
    s_control.shoot_switch = data[0];
    s_control.retreat      = data[1];
}

#endif /* CURRENT_BOARD == BOARD_GIMBAL */

// ─── 公有接口 ─────────────────────────────────────

void app_gimbal_comm_init(void)
{
    memset(&s_radar_speed,   0, sizeof(s_radar_speed));
    memset(&s_speed_no_shoot, 0, sizeof(s_speed_no_shoot));
    memset(&s_angle_no_shoot, 0, sizeof(s_angle_no_shoot));
    memset(&s_speed_shoot,    0, sizeof(s_speed_shoot));
    memset(&s_angle_shoot,    0, sizeof(s_angle_shoot));
    memset(&s_control,        0, sizeof(s_control));

#if CURRENT_BOARD == BOARD_GIMBAL
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_RADAR_SPEED,
                                 on_radar_speed);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT,
                                 on_speed_no_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT,
                                 on_angle_no_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_SPEED_SHOOT,
                                 on_speed_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_SHOOT,
                                 on_angle_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_CONTROL_CMD,
                                 on_control_cmd);
#endif
}

const app_gimbal_radar_speed_cmd_t *app_gimbal_comm_get_radar_speed(void)
{
    return &s_radar_speed;
}

const app_gimbal_speed_cmd_t *app_gimbal_comm_get_speed_no_shoot(void)
{
    return &s_speed_no_shoot;
}

const app_gimbal_angle_cmd_t *app_gimbal_comm_get_angle_no_shoot(void)
{
    return &s_angle_no_shoot;
}

const app_gimbal_speed_cmd_t *app_gimbal_comm_get_speed_shoot(void)
{
    return &s_speed_shoot;
}

const app_gimbal_angle_cmd_t *app_gimbal_comm_get_angle_shoot(void)
{
    return &s_angle_shoot;
}

const app_gimbal_control_cmd_t *app_gimbal_comm_get_control(void)
{
    return &s_control;
}

// ─── 发送接口 ─────────────────────────────────────

void app_gimbal_comm_send_speed_feedback(float yaw_speed, float pitch_speed)
{
    uint8_t data[8];
    memcpy(data,      &yaw_speed,   sizeof(float));
    memcpy(data + 4,  &pitch_speed, sizeof(float));
    bsp_can_send(&hcan1, APP_GIMBAL_CAN_ID_SPEED_FEEDBACK, data);
}

void app_gimbal_comm_send_angle_feedback(float yaw_angle, float pitch_angle)
{
    uint8_t data[8];
    memcpy(data,      &yaw_angle,   sizeof(float));
    memcpy(data + 4,  &pitch_angle, sizeof(float));
    bsp_can_send(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK, data);
}

void app_gimbal_comm_send_angle_feedback_v2(uint16_t yaw, uint16_t pitch,
                                            uint16_t roll, uint16_t interval)
{
    uint8_t data[8];
    memcpy(data,      &yaw,      sizeof(uint16_t));
    memcpy(data + 2,  &pitch,    sizeof(uint16_t));
    memcpy(data + 4,  &roll,     sizeof(uint16_t));
    memcpy(data + 6,  &interval, sizeof(uint16_t));
    bsp_can_send(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2, data);
}

void app_gimbal_comm_send_shoot_feedback(const uint8_t data[8])
{
    bsp_can_send(&hcan1, APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK, (uint8_t *)data);
}

void app_gimbal_comm_send_imu_quaternion(int16_t q0, int16_t q1,
                                         int16_t q2, int16_t q3)
{
    uint8_t data[8];
    memcpy(data,      &q0, sizeof(int16_t));
    memcpy(data + 2,  &q1, sizeof(int16_t));
    memcpy(data + 4,  &q2, sizeof(int16_t));
    memcpy(data + 6,  &q3, sizeof(int16_t));
    bsp_can_send(&hcan1, APP_GIMBAL_CAN_ID_IMU_QUATERNION, data);
}