/**
 * @file    app_chassis_comm.c
 * @brief   底盘CAN通信协议实现
 * @note    CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_chassis_comm.h"

#include "bsp_can.h"

#include <string.h>

typedef struct {
    app_chassis_comm_rx_t data;      // 所有协议解包后的接收数据
    uint32_t speed_rx_tick;
    uint32_t ackermann_rx_tick;
    uint32_t follow_rx_tick;
    uint8_t speed_received;          
    uint8_t ackermann_received;     
    uint8_t follow_received;         
} app_chassis_comm_state_t;

static app_chassis_comm_state_t s_rx;

static void on_speed_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.speed, data, sizeof(s_rx.data.speed));
    s_rx.speed_rx_tick = HAL_GetTick();
    s_rx.speed_received = 1U;
}

static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.ackermann, data, sizeof(s_rx.data.ackermann));
    s_rx.ackermann_rx_tick = HAL_GetTick();
    s_rx.ackermann_received = 1U;
}

static void on_follow_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.follow, data, sizeof(s_rx.data.follow));
    s_rx.follow_rx_tick = HAL_GetTick();
    s_rx.follow_received = 1U;
}

void app_chassis_comm_init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));

    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD,
                                 on_speed_cmd);
    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_ACKERMANN_CMD,
                                 on_ackermann_cmd);
    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_FOLLOW_CMD,
                                 on_follow_cmd);
}

uint8_t app_chassis_comm_read_rx(app_chassis_comm_rx_t *rx)
{
    uint32_t irq_state;

    if (!rx) {
        return 0U;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    *rx = s_rx.data;
    __set_PRIMASK(irq_state);
    return 1U;
}

uint8_t app_chassis_comm_read_speed_cmd(app_chassis_speed_cmd_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) {
        return 0U;
    }
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_valid = s_rx.speed_received
                       && (uint32_t)(HAL_GetTick() - s_rx.speed_rx_tick) <= timeout_ms;
    if (is_valid) {
        *cmd = s_rx.data.speed;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}

uint8_t app_chassis_comm_read_ackermann_cmd(app_chassis_ackermann_cmd_t *cmd,
                                            uint32_t timeout_ms)
{
    uint32_t irq_state;
    uint8_t is_valid;

    if (!cmd) {
        return 0U;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    is_valid = s_rx.ackermann_received
               && (uint32_t)(HAL_GetTick() - s_rx.ackermann_rx_tick) <= timeout_ms;
    if (is_valid) {
        *cmd = s_rx.data.ackermann;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}

uint8_t app_chassis_comm_read_follow_cmd(app_chassis_follow_cmd_t *cmd,
                                         uint32_t timeout_ms)
{
    uint32_t irq_state;
    uint8_t is_valid;

    if (!cmd) {
        return 0U;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    is_valid = s_rx.follow_received
               && (uint32_t)(HAL_GetTick() - s_rx.follow_rx_tick) <= timeout_ms;
    if (is_valid) {
        *cmd = s_rx.data.follow;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}

uint8_t app_chassis_comm_power_tx(int16_t power_x100)
{
    uint8_t data[8] = {0};
    uint16_t raw_power = (uint16_t)power_x100;

    data[0] = (uint8_t)(raw_power & 0xFFU);
    data[1] = (uint8_t)(raw_power >> 8);
    return bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_POWER_FEEDBACK, data);
}

uint8_t app_chassis_comm_omega_tx(float omega_z)
{
    uint8_t data[8];
    memset(data, 0, sizeof(data));
    memcpy(data, &omega_z, sizeof(float));
    return bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK, data);
}
