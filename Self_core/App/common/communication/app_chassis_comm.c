/**
 * @file    app_chassis_comm.c
 * @brief   底盘CAN通信协议实现
 * @note    CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_chassis_comm.h"

#include "bsp_can.h"

#include <string.h>

static app_chassis_speed_cmd_t     s_speed_cmd;
static app_chassis_ackermann_cmd_t s_ackermann_cmd;
static app_chassis_follow_cmd_t    s_follow_cmd;
static uint32_t                    s_speed_cmd_tick;//最后收到运动指令的时间戳, 0=从未收到
static uint8_t s_speed_cmd_is_valid;

static void on_speed_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_speed_cmd, data, sizeof(s_speed_cmd));
    s_speed_cmd_tick = HAL_GetTick();
    s_speed_cmd_is_valid = 1;
}

static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_ackermann_cmd, data, 8);
}

static void on_follow_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_follow_cmd, data, 8);
}

void app_chassis_comm_init(void)
{
    s_speed_cmd_tick = 0;
    s_speed_cmd_is_valid = 0;
    memset(&s_speed_cmd,     0, sizeof(s_speed_cmd));
    memset(&s_ackermann_cmd, 0, sizeof(s_ackermann_cmd));
    memset(&s_follow_cmd,    0, sizeof(s_follow_cmd));

    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD,
                                 on_speed_cmd);
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_ACKERMANN_CMD,
                                 on_ackermann_cmd);
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_FOLLOW_CMD,
                                 on_follow_cmd);
}

const app_chassis_speed_cmd_t *app_chassis_comm_get_speed_cmd(void)
{
    return &s_speed_cmd;
}

uint8_t app_chassis_comm_read_speed_cmd(app_chassis_speed_cmd_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) {
        return 0;
    }
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_valid = s_speed_cmd_is_valid
                       && (uint32_t)(HAL_GetTick() - s_speed_cmd_tick) <= timeout_ms;
    if (is_valid) {
        *cmd = s_speed_cmd;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}

uint32_t app_chassis_comm_get_speed_cmd_tick(void)
{
    return s_speed_cmd_tick;
}

const app_chassis_ackermann_cmd_t *app_chassis_comm_get_ackermann_cmd(void)
{
    return &s_ackermann_cmd;
}

const app_chassis_follow_cmd_t *app_chassis_comm_get_follow_cmd(void)
{
    return &s_follow_cmd;
}

uint8_t app_chassis_comm_send_power_feedback(int16_t power_x100)
{
    uint8_t data[8] = {0};
    uint16_t raw_power = (uint16_t)power_x100;

    data[0] = (uint8_t)(raw_power & 0xFFU);
    data[1] = (uint8_t)(raw_power >> 8);
    return (bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_POWER_FEEDBACK, data)
            == BSP_CAN_TX_OK) ? 0U : 1U;
}

uint8_t app_chassis_comm_send_omega_feedback(float omega_z)
{
    uint8_t data[8];
    memset(data, 0, sizeof(data));
    memcpy(data, &omega_z, sizeof(float));
    return (bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK, data)
            == BSP_CAN_TX_OK) ? 0 : 1;
}
