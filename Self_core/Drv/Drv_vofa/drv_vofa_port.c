/**
 * @file    drv_vofa_port.c
 * @brief   BSP 适配 — drv_vofa 的硬件端口挂接
 */
#include "drv_vofa.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_uart.h"

static void drv_vofa_port_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        drv_vofa_tx_cplt();
    }
}

void drv_vofa_port_init(uint8_t ch_count)
{
    drv_vofa_init(ch_count);
    bsp_uart_reg_tx_cplt_callback(&huart1, drv_vofa_port_tx_cplt);
}

void drv_vofa_port_tx(void *huart, uint8_t *data, uint16_t len)
{
    bsp_uart_tx((UART_HandleTypeDef *)huart, data, len);
}
