/**
 * @file    app_dbus.c
 * @brief   DBUS 上层胶水 — UART DMA + IDLE 接收, 挂接 DRV 解码
 */
#include "app_dbus.h"

// ─── 私有变量 ────────────────────────────────────

static UART_HandleTypeDef  *s_huart;
static DMA_HandleTypeDef   *s_hdma;
static uint8_t              s_buffer[DRV_DBUS_BUFFER_SIZE];
static drv_dbus_data_t      s_data;

// ─── 接口实现 ─────────────────────────────────────

void app_dbus_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    s_huart = huart;
    s_hdma  = hdma;

    /* 默认通道值设为中点 (1024) */
    s_data.rc.ch[0] = 1024;
    s_data.rc.ch[1] = 1024;
    s_data.rc.ch[2] = 1024;
    s_data.rc.ch[3] = 1024;
    s_data.rc.rolling_wheel = 1024;

    /* 使能 IDLE 中断 */
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    /* 启动 DMA 接收 */
    HAL_UART_Receive_DMA(s_huart, s_buffer, DRV_DBUS_BUFFER_SIZE);
}

void app_dbus_irq_handler(void)
{
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) == RESET)
        return;

    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
    HAL_UART_DMAStop(s_huart);

    /* DMA 计数器为 0 → 一帧完整的 18 字节已接收完毕 */
    if (__HAL_DMA_GET_COUNTER(s_hdma) == 0)
        drv_dbus_decode(s_buffer, &s_data);

    HAL_UART_Receive_DMA(s_huart, s_buffer, DRV_DBUS_BUFFER_SIZE);
}

const drv_dbus_data_t *app_dbus_get_data(void)
{
    return &s_data;
}
