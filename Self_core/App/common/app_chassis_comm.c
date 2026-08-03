/**
 * @file    app_chassis_comm.c
 * @brief   底盘CAN通信协议实现
 * @note    CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_chassis_comm.h"

#include "bsp_can.h"
#include "project_cfg.h"

#include <string.h>

// ─── 私有变量 ────────────────────────────────────

static app_chassis_speed_cmd_t     s_speed_cmd;
static app_chassis_ackermann_cmd_t s_ackermann_cmd;
static app_chassis_follow_cmd_t    s_follow_cmd;

// ─── CAN RX 回调 (中断上下文) ──────────────────────

#if CURRENT_BOARD == BOARD_CHASSIS

static void on_speed_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_speed_cmd, data, 8);
}

static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_ackermann_cmd, data, 8);
}

static void on_follow_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_follow_cmd, data, 8);
}

#endif /* CURRENT_BOARD == BOARD_CHASSIS */

// ─── 公有接口 ─────────────────────────────────────

void app_chassis_comm_init(void)
{
    memset(&s_speed_cmd,     0, sizeof(s_speed_cmd));
    memset(&s_ackermann_cmd, 0, sizeof(s_ackermann_cmd));
    memset(&s_follow_cmd,    0, sizeof(s_follow_cmd));

#if CURRENT_BOARD == BOARD_CHASSIS
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD,
                                 on_speed_cmd);
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_ACKERMANN_CMD,
                                 on_ackermann_cmd);
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_FOLLOW_CMD,
                                 on_follow_cmd);
#endif
}

const app_chassis_speed_cmd_t *app_chassis_comm_get_speed_cmd(void)
{
    return &s_speed_cmd;
}

const app_chassis_ackermann_cmd_t *app_chassis_comm_get_ackermann_cmd(void)
{
    return &s_ackermann_cmd;
}

const app_chassis_follow_cmd_t *app_chassis_comm_get_follow_cmd(void)
{
    return &s_follow_cmd;
}

void app_chassis_comm_send_power_feedback(int16_t power_x100)
{
    uint8_t data[8];
    memset(data, 0, sizeof(data));
    memcpy(data, &power_x100, sizeof(int16_t));
    bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_POWER_FEEDBACK, data);
}

void app_chassis_comm_send_speed_cmd(int16_t vx, int16_t vy, int16_t vz,
                                     int16_t power_pct)
{
    app_chassis_speed_cmd_t cmd;
    cmd.vx        = vx;
    cmd.vy        = vy;
    cmd.vz        = vz;
    cmd.power_pct = power_pct;

    uint8_t data[8];
    memcpy(data, &cmd, 8);
    bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
}