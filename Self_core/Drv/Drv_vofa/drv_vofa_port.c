/**
 * @file    drv_vofa_port.c
 * @brief   BSP 适配 — drv_vofa 的硬件端口挂接
 */
#include "drv_vofa.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_uart.h"

void drv_vofa_port_init(uint8_t ch_count)
{
    drv_vofa_init(ch_count);
}

void drv_vofa_port_send(void *huart, uint8_t *data, uint16_t len)
{
    bsp_uart_tx((UART_HandleTypeDef *)huart, data, len);
}
