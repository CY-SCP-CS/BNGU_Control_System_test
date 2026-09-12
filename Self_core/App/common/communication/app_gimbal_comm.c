/**
 * @file    app_gimbal_comm.c
 * @brief   云台CAN通信协议实现
 * @note    0x120 收到后自动转发到 0x111 给底盘
 *          CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include "bsp_can.h"

#include <string.h>

#include <math.h>

// ─── 私有变量 ────────────────────────────────────

static app_gimbal_radar_cmd_t s_radar_cmd;
static app_gimbal_speed_cmd_t       s_speed_no_shoot;
static app_gimbal_angle_cmd_t       s_angle_no_shoot;
static app_gimbal_speed_cmd_t       s_speed_shoot;
static app_gimbal_angle_cmd_t       s_angle_shoot;
static app_gimbal_shoot_cmd_t     s_shoot_cmd;
static app_gimbal_angle_cmd_t s_latest_angle;
static uint32_t s_latest_angle_tick;
static uint8_t s_latest_angle_is_valid;
static volatile uint8_t s_radar_is_pending;

static void gimbal_forward_chassis_speed_cmd(const app_gimbal_radar_cmd_t *cmd);

// ─── CAN RX 回调 (中断上下文) ──────────────────────

static void on_radar_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_radar_cmd, data, 8);

    s_radar_is_pending = 1;
}

static void on_speed_no_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_speed_no_shoot, data, 8);
}

static void on_angle_no_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    app_gimbal_angle_cmd_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    if (!isfinite(cmd.yaw_abs) || !isfinite(cmd.pitch_abs)) {
        return;
    }
    s_angle_no_shoot = cmd;
    s_latest_angle = cmd;
    s_latest_angle_tick = HAL_GetTick();
    s_latest_angle_is_valid = 1;
}

static void on_speed_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_speed_shoot, data, 8);
}

static void on_angle_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    app_gimbal_angle_cmd_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    if (!isfinite(cmd.yaw_abs) || !isfinite(cmd.pitch_abs)) {
        return;
    }
    s_angle_shoot = cmd;
    s_latest_angle = cmd;
    s_latest_angle_tick = HAL_GetTick();
    s_latest_angle_is_valid = 1;
}

static void on_shoot_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    /* 协议规定 DLC=4 */
    if (len < 4) {
        return;
    }
    s_shoot_cmd.shoot_switch = data[0];
    s_shoot_cmd.retreat      = data[1];
}

// ─── 公有接口 ─────────────────────────────────────

void app_gimbal_comm_init(void)
{
    s_latest_angle_is_valid = 0;
    s_latest_angle_tick = 0;
    s_radar_is_pending = 0;
    memset(&s_radar_cmd,   0, sizeof(s_radar_cmd));
    memset(&s_speed_no_shoot, 0, sizeof(s_speed_no_shoot));
    memset(&s_angle_no_shoot, 0, sizeof(s_angle_no_shoot));
    memset(&s_speed_shoot,    0, sizeof(s_speed_shoot));
    memset(&s_angle_shoot,    0, sizeof(s_angle_shoot));
    memset(&s_shoot_cmd,        0, sizeof(s_shoot_cmd));

    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_RADAR_SPEED,
                                 on_radar_cmd);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT,
                                 on_speed_no_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT,
                                 on_angle_no_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_SPEED_SHOOT,
                                 on_speed_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_SHOOT,
                                 on_angle_shoot);
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_SHOOT_CMD,
                                 on_shoot_cmd);
}

const app_gimbal_radar_cmd_t* app_gimbal_comm_get_radar_cmd(void)
{
    return &s_radar_cmd;
}

uint8_t app_gimbal_comm_read_angle_cmd(app_gimbal_angle_cmd_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) {
        return 0;
    }
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_valid = s_latest_angle_is_valid
                       && (uint32_t)(HAL_GetTick() - s_latest_angle_tick) <= timeout_ms;
    if (is_valid) {
        *cmd = s_latest_angle;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}

void app_gimbal_comm_process(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_pending = s_radar_is_pending;
    app_gimbal_radar_cmd_t cmd = s_radar_cmd;
    s_radar_is_pending = 0;
    __set_PRIMASK(irq_state);
    if (is_pending) {
        gimbal_forward_chassis_speed_cmd(&cmd);
    }
}

static void gimbal_forward_chassis_speed_cmd(const app_gimbal_radar_cmd_t *cmd)
{
    uint8_t data[8];

    if (!cmd) {
        return;
    }
    memcpy(data, cmd, sizeof(data));
    bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
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

const app_gimbal_shoot_cmd_t *app_gimbal_comm_get_control(void)
{
    return &s_shoot_cmd;
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
