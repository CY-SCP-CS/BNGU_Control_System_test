#include "drv_dbus.h"
#include "bsp_cfg.h"
#include "bsp_uart.h"
#include <string.h>

static UART_HandleTypeDef *s_port_huart;
static uint8_t s_port_buffer[DRV_DBUS_BUFFER_SIZE];
static drv_dbus_data_t s_port_data;
static drv_dbus_data_t s_port_snapshot;
static volatile uint8_t s_port_is_valid;

static void drv_dbus_port_uart_idle_handler(UART_HandleTypeDef *huart)
{
    drv_dbus_data_t decoded;
    uint8_t is_valid;
    uint16_t remain;
    int i;

    if (huart != s_port_huart) {
        return;
    }
    bsp_uart_clear_idle(huart);
    (void)bsp_uart_stop_dma(huart);
    remain = bsp_uart_get_rx_dma_remain(huart);
    if (remain == 0U) {
        drv_dbus_decode(s_port_buffer, &decoded);
        is_valid = decoded.rc.s1 >= DRV_DBUS_SWITCH_UP
                && decoded.rc.s1 <= DRV_DBUS_SWITCH_MIDDLE
                && decoded.rc.s2 >= DRV_DBUS_SWITCH_UP
                && decoded.rc.s2 <= DRV_DBUS_SWITCH_MIDDLE;
        for (i = 0; i < 4; i++) {
            if (decoded.rc.ch[i] < DRV_DBUS_CHANNEL_CENTER - DRV_DBUS_CHANNEL_RANGE
                || decoded.rc.ch[i] > DRV_DBUS_CHANNEL_CENTER + DRV_DBUS_CHANNEL_RANGE) {
                is_valid = 0U;
            }
        }
        if (is_valid) {
            s_port_data = decoded;
        }
        s_port_is_valid = is_valid;
    }
    (void)bsp_uart_rx_dma(huart, s_port_buffer, sizeof(s_port_buffer));
}

void drv_dbus_port_init(void)
{
    s_port_huart = &huart3;
    s_port_is_valid = 0U;
    memset(&s_port_data, 0, sizeof(s_port_data));
    bsp_uart_clear_idle(s_port_huart);
    bsp_uart_enable_idle_it(s_port_huart);
    bsp_uart_reg_rx_idle_callback(s_port_huart, drv_dbus_port_uart_idle_handler);
    (void)bsp_uart_rx_dma(s_port_huart, s_port_buffer, sizeof(s_port_buffer));
}

const drv_dbus_data_t *drv_dbus_port_get_data(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (s_port_is_valid) {
        s_port_snapshot = s_port_data;
    }
    __set_PRIMASK(irq_state);
    return s_port_is_valid ? &s_port_snapshot : NULL;
}