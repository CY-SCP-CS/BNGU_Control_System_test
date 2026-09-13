/**
 * @file    drv_referee_port.c
 * @brief   裁判系统 USART6 DMA + IDLE 硬件适配
 */
#include "drv_referee.h"
#include "bsp_uart.h"
#include "usart.h"
#include <string.h>

static uint8_t s_dma_buf[REFEREE_RX_BUF_SIZE];
static uint8_t s_temp_buf[REFEREE_RX_BUF_SIZE];
static volatile uint8_t s_dma_is_pending;
static uint8_t s_dma_is_started;

void drv_referee_port_init(void)
{
    drv_referee_init();
    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_temp_buf, 0, sizeof(s_temp_buf));
    s_dma_is_pending = 0U;
    s_dma_is_started = 0U;

    bsp_uart_clear_idle(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf,
                                             sizeof(s_dma_buf)) == HAL_OK;
}

void drv_referee_port_uart_idle_handler(void)
{
    if (bsp_uart_is_idle(&huart6)) {
        bsp_uart_clear_idle(&huart6);
        s_dma_is_pending = 1U;
    }
}

void drv_referee_port_process(void)
{
    uint32_t irq_state = __get_PRIMASK();
    uint8_t is_pending;

    __disable_irq();
    is_pending = s_dma_is_pending;
    s_dma_is_pending = 0U;
    __set_PRIMASK(irq_state);

    if (!is_pending && s_dma_is_started
        && bsp_uart_get_rx_dma_remain(&huart6) != 0U) {
        return;
    }

    if (!s_dma_is_started) {
        s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf,
                                                 sizeof(s_dma_buf)) == HAL_OK;
        return;
    }

    uint16_t receive_len = (uint16_t)(sizeof(s_dma_buf)
                                       - bsp_uart_get_rx_dma_remain(&huart6));
    if (bsp_uart_stop_dma(&huart6) != HAL_OK) {
        s_dma_is_started = 0U;
        return;
    }
    memcpy(s_temp_buf, s_dma_buf, receive_len);
    s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf,
                                             sizeof(s_dma_buf)) == HAL_OK;
    drv_referee_process(s_temp_buf, receive_len);
}
