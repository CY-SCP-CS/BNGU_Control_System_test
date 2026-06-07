/**
 * @file    bsp_uart.c
 * @brief   UART 驱动实现: 非阻塞发送 / 回调分发
 */
#include "bsp_uart.h"

// ─── 回调表 ──────────────────────────────────────

typedef struct {
    UART_HandleTypeDef      *huart;
    bsp_uart_rx_callback_t   callback;
} bsp_uart_rx_cb_entry_t;

typedef struct {
    UART_HandleTypeDef     *huart;
    bsp_uart_tx_callback_t  callback;
} bsp_uart_tx_cb_entry_t;

static bsp_uart_rx_cb_entry_t s_rx_callbacks[BSP_UART_RX_CALLBACK_MAX];
static uint8_t                 s_rx_cb_count;
static bsp_uart_tx_cb_entry_t s_tx_callbacks[BSP_UART_TX_CALLBACK_MAX];
static uint8_t                 s_tx_cb_count;

// ─── 接口实现 ─────────────────────────────────────

HAL_StatusTypeDef bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit_IT(huart, data, len);
}

void bsp_uart_register_rx_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_rx_callback_t callback)
{
    uint8_t i;

    if (s_rx_cb_count >= BSP_UART_RX_CALLBACK_MAX) return;

    for (i = 0; i < s_rx_cb_count; i++) {
        if (s_rx_callbacks[i].huart == huart) {
            s_rx_callbacks[i].callback = callback;
            return;
        }
    }

    s_rx_callbacks[s_rx_cb_count].huart    = huart;
    s_rx_callbacks[s_rx_cb_count].callback = callback;
    s_rx_cb_count++;
}

void bsp_uart_rx_irq_handler(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0; i < s_rx_cb_count; i++) {
        if (s_rx_callbacks[i].huart == huart) {
            s_rx_callbacks[i].callback(huart->pRxBuffPtr, huart->RxXferCount);
            return;
        }
    }
}

void bsp_uart_register_tx_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_tx_callback_t callback)
{
    uint8_t i;

    if (s_tx_cb_count >= BSP_UART_TX_CALLBACK_MAX) return;

    for (i = 0; i < s_tx_cb_count; i++) {
        if (s_tx_callbacks[i].huart == huart) {
            s_tx_callbacks[i].callback = callback;
            return;
        }
    }

    s_tx_callbacks[s_tx_cb_count].huart    = huart;
    s_tx_callbacks[s_tx_cb_count].callback = callback;
    s_tx_cb_count++;
}

void bsp_uart_tx_irq_handler(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0; i < s_tx_cb_count; i++) {
        if (s_tx_callbacks[i].huart == huart) {
            s_tx_callbacks[i].callback();
            return;
        }
    }
}