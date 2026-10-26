#include "drv_referee.h"
#include "bsp_uart.h"
#include "usart.h"
#include <string.h>

static uint8_t s_dma_buf[REFEREE_RX_BUF_SIZE];
static uint8_t s_temp_buf[REFEREE_RX_BUF_SIZE];
static volatile uint8_t s_dma_is_pending;
static uint8_t s_dma_is_started;
static uint8_t s_tx_buf[REFEREE_TX_BUF_SIZE];
static uint8_t s_tx_seq;
static volatile uint8_t s_tx_busy;

static void drv_referee_port_uart_idle_handler(UART_HandleTypeDef *huart)
{
    if (huart == &huart6 && bsp_uart_is_idle(huart)) {
        bsp_uart_clear_idle(huart);
        s_dma_is_pending = 1U;
    }
}

static void drv_referee_port_uart_tx_cplt_handler(
    UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {
        s_tx_busy = 0U;
    }
}

void drv_referee_port_init(void)
{
    drv_referee_init();
    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_temp_buf, 0, sizeof(s_temp_buf));
    s_dma_is_pending = 0U;
    s_dma_is_started = 0U;
    s_tx_seq = 0U;
    s_tx_busy = 0U;
    bsp_uart_clear_idle(&huart6);
    bsp_uart_enable_idle_it(&huart6);
    bsp_uart_reg_tx_cplt_callback(&huart6, drv_referee_port_uart_tx_cplt_handler);
    bsp_uart_reg_rx_idle_callback(&huart6, drv_referee_port_uart_idle_handler);
    s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf, sizeof(s_dma_buf)) == HAL_OK;
}

void drv_referee_port_process(void)
{
    uint32_t irq_state = __get_PRIMASK();
    uint8_t is_pending;
    uint16_t receive_len;

    __disable_irq();
    is_pending = s_dma_is_pending;
    s_dma_is_pending = 0U;
    __set_PRIMASK(irq_state);
    if (!is_pending && s_dma_is_started && bsp_uart_get_rx_dma_remain(&huart6) != 0U) {
        return;
    }
    if (!s_dma_is_started) {
        s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf, sizeof(s_dma_buf)) == HAL_OK;
        return;
    }
    receive_len = (uint16_t)(sizeof(s_dma_buf) - bsp_uart_get_rx_dma_remain(&huart6));
    if (bsp_uart_stop_dma(&huart6) != HAL_OK) {
        s_dma_is_started = 0U;
        return;
    }
    memcpy(s_temp_buf, s_dma_buf, receive_len);
    s_dma_is_started = bsp_uart_rx_dma(&huart6, s_dma_buf, sizeof(s_dma_buf)) == HAL_OK;
    drv_referee_process(s_temp_buf, receive_len);
}

int drv_referee_port_send(uint16_t cmd_id,const void *payload,uint16_t payload_len)
{
    if (s_tx_busy) {return 1;}
   uint16_t frame_len = drv_referee_build_frame(s_tx_buf,sizeof(s_tx_buf),cmd_id,
                                                             payload,payload_len,s_tx_seq);
   if (frame_len==0U)return -1;
    s_tx_busy = 1U;
    HAL_StatusTypeDef status =bsp_uart_tx(&huart6, s_tx_buf, frame_len);
    if (status != HAL_OK) {
        s_tx_busy = 0U;
        return (status == HAL_BUSY) ? 1 : -1;
    }
    s_tx_seq++;
    return 0;
}

int drv_referee_port_send_frame(const void *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 9U ||
        frame_len > sizeof(s_tx_buf)) {
        return -1;
        }

    if (s_tx_busy) {return 1;}

    memcpy(s_tx_buf, frame, frame_len);
    s_tx_busy = 1U;

    HAL_StatusTypeDef status = bsp_uart_tx(&huart6, s_tx_buf, frame_len);

    if (status != HAL_OK) {
        s_tx_busy = 0U;
        return (status == HAL_BUSY) ? 1 : -1;
    }

    return 0;
}
