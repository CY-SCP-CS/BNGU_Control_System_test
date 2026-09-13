/**
 * @file    app_gimbal_comm.c
 * @brief   云台CAN通信协议实现
 * @note    0x120 收到后自动转发到 0x111 给底盘
 *          CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include "bsp_can.h"
#include "drv_dbus.h"
#include "lib_math.h"

#include <string.h>

#include <math.h>

// ─── 私有接收状态 ────────────────────────────────

static app_gimbal_comm_rx_t s_rx;
static volatile uint8_t s_radar_is_pending;
static volatile uint8_t s_updated_mask;

static void gimbal_forward_chassis_speed_cmd(const app_gimbal_radar_cmd_t *cmd);

// ─── CAN RX 回调 (中断上下文) ──────────────────────

static void on_radar_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.radar_speed, data, sizeof(s_rx.radar_speed));
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_RADAR_SPEED;
    s_radar_is_pending = 1;
}

static void on_speed_no_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.speed_no_shoot, data, sizeof(s_rx.speed_no_shoot));
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT;
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
    s_rx.angle_no_shoot = cmd;
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT;
}

static void on_speed_shoot(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.speed_shoot, data, sizeof(s_rx.speed_shoot));
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT;
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
    s_rx.angle_shoot = cmd;
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
}

static void on_shoot_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    /* 协议规定 DLC=4 */
    if (len < 4) {
        return;
    }
    s_rx.shoot.shoot_switch = data[0];
    s_rx.shoot.retreat      = data[1];
    s_updated_mask |= APP_GIMBAL_COMM_UPDATE_SHOOT;
}

// ─── 公有接口 ─────────────────────────────────────

void app_gimbal_comm_init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));
    s_radar_is_pending = 0U;
    s_updated_mask = 0U;

    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_RADAR_SPEED,
                                 on_radar_cmd);
    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT,
                                 on_speed_no_shoot);
    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT,
                                 on_angle_no_shoot);
    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_SPEED_SHOOT,
                                 on_speed_shoot);
    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_SHOOT,
                                 on_angle_shoot);
    bsp_can_rx_reg(&hcan1, APP_GIMBAL_CAN_ID_SHOOT_CMD,
                                 on_shoot_cmd);
}

uint8_t app_gimbal_comm_read_rx(app_gimbal_comm_rx_t *rx,
                                uint8_t *updated_mask)
{
    uint32_t irq_state;

    if (!rx || !updated_mask) {
        return 0U;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    *rx = s_rx;
    *updated_mask = s_updated_mask;
    s_updated_mask = 0U;
    __set_PRIMASK(irq_state);
    return 1U;
}

uint8_t app_gimbal_comm_dbus_rx(app_gimbal_dbus_input_t *input)
{
    const drv_dbus_data_t *dbus;

    if (!input) {
        return 0U;
    }
    dbus = drv_dbus_port_get_data();
    if (!dbus) {
        return 0U;
    }

    input->launcher_mode = dbus->rc.s1;
    input->chassis_vx_norm = lib_clamp(((float)dbus->rc.ch[0] - DRV_DBUS_CHANNEL_CENTER)
                                       / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);
    input->chassis_vy_norm = lib_clamp(((float)dbus->rc.ch[1] - DRV_DBUS_CHANNEL_CENTER)
                                       / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);
    input->yaw_rate_norm = lib_clamp(((float)dbus->rc.ch[2] - DRV_DBUS_CHANNEL_CENTER)
                                     / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);
    input->pitch_rate_norm = lib_clamp(((float)dbus->rc.ch[3] - DRV_DBUS_CHANNEL_CENTER)
                                       / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);
    input->chassis_omega_norm = lib_clamp(((float)dbus->rc.rolling_wheel
                                           - DRV_DBUS_CHANNEL_CENTER)
                                          / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);

    if (dbus->rc.s2 == DRV_DBUS_SWITCH_UP) {
        input->source = APP_GIMBAL_INPUT_DBUS;
    } else if (dbus->rc.s2 == DRV_DBUS_SWITCH_MIDDLE) {
        input->source = APP_GIMBAL_INPUT_CAN;
    } else {
        input->source = APP_GIMBAL_INPUT_ESTOP;
    }
    return 1U;
}

void app_gimbal_comm_process(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_pending = s_radar_is_pending;
    app_gimbal_radar_cmd_t cmd = s_rx.radar_speed;
    s_radar_is_pending = 0;
    __set_PRIMASK(irq_state);
    app_gimbal_dbus_input_t input;
    if (is_pending && app_gimbal_comm_dbus_rx(&input)
        && input.source == APP_GIMBAL_INPUT_CAN) {
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
    bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
}

// ─── 发送接口 ─────────────────────────────────────

void app_gimbal_comm_gyro_tx(float yaw_tar_speed, float pitch_tar_speed)
{
    uint8_t data[8];
    memcpy(data,      &yaw_tar_speed,   sizeof(float));
    memcpy(data + 4,  &pitch_tar_speed, sizeof(float));
    bsp_can_tx(&hcan1, APP_GIMBAL_CAN_ID_SPEED_FEEDBACK, data);
}

void app_gimbal_comm_angle_tx(float yaw_angle, float pitch_angle)
{
    uint8_t data[8];
    memcpy(data,      &yaw_angle,   sizeof(float));
    memcpy(data + 4,  &pitch_angle, sizeof(float));
    bsp_can_tx(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK, data);
}

void app_gimbal_comm_angle_tx_v2(uint16_t yaw, uint16_t pitch,
                                            uint16_t roll, uint16_t interval)
{
    uint8_t data[8];
    memcpy(data,      &yaw,      sizeof(uint16_t));
    memcpy(data + 2,  &pitch,    sizeof(uint16_t));
    memcpy(data + 4,  &roll,     sizeof(uint16_t));
    memcpy(data + 6,  &interval, sizeof(uint16_t));
    bsp_can_tx(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2, data);
}

void app_gimbal_comm_shoot_tx(const uint8_t data[8])
{
    bsp_can_tx(&hcan1, APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK, (uint8_t *)data);
}

void app_gimbal_comm_quat_tx(int16_t q0, int16_t q1,
                                         int16_t q2, int16_t q3)
{
    uint8_t data[8];
    memcpy(data,      &q0, sizeof(int16_t));
    memcpy(data + 2,  &q1, sizeof(int16_t));
    memcpy(data + 4,  &q2, sizeof(int16_t));
    memcpy(data + 6,  &q3, sizeof(int16_t));
    bsp_can_tx(&hcan1, APP_GIMBAL_CAN_ID_IMU_QUATERNION, data);
}
