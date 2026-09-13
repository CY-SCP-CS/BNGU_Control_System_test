/**
 * @file    bsp_uart.c
 * @brief   UART 驱动实现: 非阻塞发送
 */
#include "bsp_uart.h"

typedef struct {
    UART_HandleTypeDef *huart;
    bsp_uart_tx_cplt_callback_t callback;
} bsp_uart_tx_callback_entry_t;

static bsp_uart_tx_callback_entry_t s_tx_callbacks[BSP_UART_TX_CALLBACK_MAX];
static uint8_t s_tx_callback_count;

HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit_IT(huart, data, len);
}

void bsp_uart_reg_tx_cplt_callback(
    UART_HandleTypeDef *huart,
    bsp_uart_tx_cplt_callback_t callback)
{
    uint8_t i;

    if (!huart || !callback) {
        return;
    }

    for (i = 0U; i < s_tx_callback_count; i++) {
        if (s_tx_callbacks[i].huart == huart) {
            s_tx_callbacks[i].callback = callback;
            return;
        }
    }

    if (s_tx_callback_count >= BSP_UART_TX_CALLBACK_MAX) {
        return;
    }

    s_tx_callbacks[s_tx_callback_count].huart = huart;
    s_tx_callbacks[s_tx_callback_count].callback = callback;
    s_tx_callback_count++;
}

HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    if (!huart || !data || len == 0U) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(huart, data, len);
}

HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart)
{
    if (!huart) {
        return HAL_ERROR;
    }
    return HAL_UART_DMAStop(huart);
}

uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx) {
        return 0U;
    }
    return (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
}

uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart)
{
    return huart && __HAL_UART_GET_FLAG((UART_HandleTypeDef *)huart, UART_FLAG_IDLE) != RESET;
}

void bsp_uart_clear_idle(UART_HandleTypeDef *huart)
{
    if (huart) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < s_tx_callback_count; i++) {
        if (s_tx_callbacks[i].huart == huart && s_tx_callbacks[i].callback) {
            s_tx_callbacks[i].callback(huart);
            return;
        }
    }
}
