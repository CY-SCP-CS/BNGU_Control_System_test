/**
 * @file    drv_dbus_port.c
 * @brief   BSP 适配 — drv_dbus 的硬件端口挂接
 */
#include "drv_dbus.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────

static UART_HandleTypeDef *s_port_huart;
static volatile uint8_t             s_port_buffer[DRV_DBUS_BUFFER_SIZE];
static volatile drv_dbus_data_t     s_port_data;

void drv_dbus_port_init(void)
{
    s_port_huart = &huart3;

    /* 默认通道值设为中点 (1024) */
    s_port_data.rc.ch[0] = 1024;
    s_port_data.rc.ch[1] = 1024;
    s_port_data.rc.ch[2] = 1024;
    s_port_data.rc.ch[3] = 1024;
    s_port_data.rc.rolling_wheel = 1024;

    /* 使能 IDLE 中断 */
    __HAL_UART_ENABLE_IT(s_port_huart, UART_IT_IDLE);

    /* 启动 DMA 接收 */
    HAL_UART_Receive_DMA(s_port_huart, s_port_buffer, DRV_DBUS_BUFFER_SIZE);
}

void drv_dbus_port_irq_handler(void)
{
    if (__HAL_UART_GET_FLAG(s_port_huart, UART_FLAG_IDLE) == RESET)
        return;

    __HAL_UART_CLEAR_IDLEFLAG(s_port_huart);
    HAL_UART_DMAStop(s_port_huart);

    /* DMA 计数器为 0 → 一帧完整的 18 字节已接收完毕 */
    if (__HAL_DMA_GET_COUNTER(s_port_huart->hdmarx) == 0)
        drv_dbus_decode((const uint8_t *)s_port_buffer, &s_port_data);

    HAL_UART_Receive_DMA(s_port_huart, s_port_buffer, DRV_DBUS_BUFFER_SIZE);
}

const drv_dbus_data_t *drv_dbus_port_get_data(void)
{
    return &s_port_data;
}
