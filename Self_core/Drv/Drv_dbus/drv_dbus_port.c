/**
 * @file    drv_dbus_port.c
 * @brief   DBUS DMA 接收、有效帧校验与失联保护
 */
#include "drv_dbus.h"

#include "bsp_cfg.h"

#include <string.h>

// ─── 私有变量 ─────────────────────────
static UART_HandleTypeDef *s_port_huart;
static uint8_t s_port_buffer[DRV_DBUS_BUFFER_SIZE];
static drv_dbus_data_t s_port_data;
static drv_dbus_data_t s_port_snapshot;
static volatile uint32_t s_port_last_rx_tick;
static volatile uint8_t s_port_is_valid;

// ─── 公有接口 ─────────────────────────
void drv_dbus_port_init(void)
{
    s_port_huart = &huart3;
    s_port_is_valid = 0;
    s_port_last_rx_tick = 0;
    memset(&s_port_data, 0, sizeof(s_port_data));
    __HAL_UART_CLEAR_IDLEFLAG(s_port_huart);
    __HAL_UART_ENABLE_IT(s_port_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(s_port_huart, s_port_buffer, sizeof(s_port_buffer));
}

void drv_dbus_port_irq_handler(void)
{
    if (!s_port_huart || __HAL_UART_GET_FLAG(s_port_huart, UART_FLAG_IDLE) == RESET) {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(s_port_huart);
    HAL_UART_DMAStop(s_port_huart);
    if (__HAL_DMA_GET_COUNTER(s_port_huart->hdmarx) == 0) {
        drv_dbus_data_t decoded;
        drv_dbus_decode(s_port_buffer, &decoded);
        uint8_t is_valid = decoded.rc.s1 >= DRV_DBUS_SWITCH_UP
                           && decoded.rc.s1 <= DRV_DBUS_SWITCH_MIDDLE
                           && decoded.rc.s2 >= DRV_DBUS_SWITCH_UP
                           && decoded.rc.s2 <= DRV_DBUS_SWITCH_MIDDLE;
        for (int i = 0; i < 4; i++) {
            if (decoded.rc.ch[i] < DRV_DBUS_CHANNEL_CENTER - DRV_DBUS_CHANNEL_RANGE
                || decoded.rc.ch[i] > DRV_DBUS_CHANNEL_CENTER + DRV_DBUS_CHANNEL_RANGE) {
                is_valid = 0;
            }
        }
        if (is_valid) {
            s_port_data = decoded;
            s_port_last_rx_tick = HAL_GetTick();
        }
        s_port_is_valid = is_valid;
    }
    HAL_UART_Receive_DMA(s_port_huart, s_port_buffer, sizeof(s_port_buffer));
}

const drv_dbus_data_t *drv_dbus_port_get_data(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_online = s_port_is_valid
                        && (uint32_t)(HAL_GetTick() - s_port_last_rx_tick) <= DRV_DBUS_TIMEOUT_MS;
    if (is_online) {
        s_port_snapshot = s_port_data;
    }
    __set_PRIMASK(irq_state);
    return is_online ? &s_port_snapshot : NULL;
}
