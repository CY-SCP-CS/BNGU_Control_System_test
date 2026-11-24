/**
 * @file    drv_vofa_port.c
 * @brief   BSP 适配 — drv_vofa 的硬件端口挂接
 */
#include "drv_vofa.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_uart.h"

static uint8_t s_tx_frame[DRV_VOFA_FRAME_SIZE];
static volatile uint8_t s_tx_busy;

static void drv_vofa_port_tx_cplt(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        s_tx_busy = 0U;
    }
}

void drv_vofa_port_init(void)
{
    s_tx_busy = 0U;
    bsp_uart_reg_tx_cplt_callback(&huart1, drv_vofa_port_tx_cplt);
}

drv_vofa_status_t drv_vofa_port_send(const float data[DRV_VOFA_CHANNEL_COUNT])
{
    return drv_vofa_port_send_channels(data, DRV_VOFA_CHANNEL_COUNT);
}

drv_vofa_status_t drv_vofa_port_send_channels(const float *data, size_t channel_count)
{
    drv_vofa_status_t pack_status;
    uint32_t irq_state;

    irq_state = __get_PRIMASK();
    __disable_irq();
    if (s_tx_busy) {
        __set_PRIMASK(irq_state);
        return DRV_VOFA_ERR_BUSY;
    }
    /* 原子预留唯一的静态缓冲区，覆盖多执行上下文下的并发调用。 */
    s_tx_busy = 1U;
    __set_PRIMASK(irq_state);

    pack_status = drv_vofa_pack_channels(data, channel_count, s_tx_frame);
    if (pack_status != DRV_VOFA_OK) {
        s_tx_busy = 0U;
        return pack_status;
    }

    const uint16_t frame_size = (uint16_t)(channel_count * sizeof(*data) + DRV_VOFA_TAIL_SIZE);
    if (bsp_uart_tx(&huart1, s_tx_frame, frame_size) != HAL_OK) {
        s_tx_busy = 0U;
        return DRV_VOFA_ERR_UART;
    }

    return DRV_VOFA_OK;
}
