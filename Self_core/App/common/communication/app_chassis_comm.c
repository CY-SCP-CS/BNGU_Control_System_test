/**
 * @file    app_chassis_comm.c
 * @brief   底盘CAN通信协议实现
 * @note    CAN RX 回调在中断上下文执行, 只做数据拷贝不调用复杂逻辑
 */
#include "app_chassis_comm.h"

#include "bsp_can.h"

#include <string.h>

typedef struct {
    app_chassis_comm_rx_t data;      // 所有协议解包后的最新接收数据
} app_chassis_comm_state_t;

static app_chassis_comm_state_t s_rx;

static void on_speed_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.speed, data, sizeof(s_rx.data.speed));
}

static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.ackermann, data, sizeof(s_rx.data.ackermann));
}

static void on_follow_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    memcpy(&s_rx.data.follow, data, sizeof(s_rx.data.follow));
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
