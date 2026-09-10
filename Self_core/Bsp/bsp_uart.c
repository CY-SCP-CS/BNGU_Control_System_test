/**
 * @file    bsp_uart.c
 * @brief   UART 驱动实现: 非阻塞发送
 */
#include "bsp_uart.h"

HAL_StatusTypeDef bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit_IT(huart, data, len);
}

HAL_StatusTypeDef bsp_uart_receive_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
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

uint16_t bsp_uart_get_rx_dma_remaining(const UART_HandleTypeDef *huart)
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
