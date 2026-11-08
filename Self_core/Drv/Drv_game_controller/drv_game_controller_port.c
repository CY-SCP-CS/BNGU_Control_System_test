//
// Created by 33140 on 2026/11/7.
//

#include "Drv/Drv_game_controller/drv_game_controller.h"
#include "Bsp/bsp_uart.h"
#include <string.h>

#define GAME_CONTROLLER_DMA_BUFFER_SIZE 128U
#define GAME_CONTROLLER_FRAME_BUFFER_SIZE 64U


static UART_HandleTypeDef *s_port_huart;
static uint8_t s_dma_buffer[GAME_CONTROLLER_DMA_BUFFER_SIZE];
static uint8_t s_temp_buffer[GAME_CONTROLLER_DMA_BUFFER_SIZE];

static uint8_t s_frame_buffer[GAME_CONTROLLER_FRAME_BUFFER_SIZE];
static uint16_t s_frame_len;
static uint8_t s_discard_frame; // 超长或中断的帧：丢弃到下一个分隔符

static volatile uint8_t s_rx_pending = 0U; //是否有数据待处理
static uint8_t s_rx_started;  //是否开启接收
static drv_game_controller_buttons_t s_game_controller_data;
static uint8_t s_has_data;
static uint32_t s_last_rx_ms;

/** 当前绑定串口的空闲中断回调。 */
static void drv_game_controller_port_uart_idle_handler(
    UART_HandleTypeDef *huart)
{
    if (!s_port_huart || huart != s_port_huart) {
        return;
    }

    bsp_uart_clear_idle(huart);  // 清除硬件空闲标志
    s_rx_pending = 1U;          // 通知 process() 有数据待处理
}

static HAL_StatusTypeDef drv_game_controller_port_start_rx(void)
{
    bsp_uart_clear_idle(s_port_huart);
    HAL_StatusTypeDef status = bsp_uart_rx_dma(
        s_port_huart, s_dma_buffer, sizeof(s_dma_buffer));
    s_rx_started = status == HAL_OK;
    if (s_rx_started) {
        bsp_uart_enable_idle_it(s_port_huart);
    }
    return status;
}

uint8_t drv_game_controller_uart_init(UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx ||
        huart->RxState != HAL_UART_STATE_READY) {return 0U;}

    s_port_huart = huart;
    s_rx_pending = 0U;
    s_rx_started = 0U;
    s_frame_len = 0U;
    s_discard_frame = 0U;
    s_has_data = 0U;
    s_last_rx_ms = 0U;
    memset(&s_game_controller_data, 0, sizeof(s_game_controller_data));

    bsp_uart_reg_rx_idle_callback(huart, drv_game_controller_port_uart_idle_handler);
    return drv_game_controller_port_start_rx() == HAL_OK;
}

uint8_t drv_game_controller_uart_process(void)
{
    if (!s_port_huart) {return 2U;}

    /* 只消费已经发生的通知，重启后的新通知留给下一次调用。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_pending = s_rx_pending;
    s_rx_pending = 0U;
    __set_PRIMASK(irq_state);

    /* 即使没有新的空闲中断，也重试上次未启动成功的接收。 */
    if (!s_rx_started) {
        if (bsp_uart_stop_rx_dma(s_port_huart) != HAL_OK ||
            drv_game_controller_port_start_rx() != HAL_OK) {
            return 2U;
        }
        return 0U;
    }

    uint16_t remain = bsp_uart_get_rx_dma_remain(s_port_huart);
    if (remain > sizeof(s_dma_buffer) ||
        (remain != 0U && s_port_huart->RxState != HAL_UART_STATE_BUSY_RX)) {
        /* 接收异常结束，可能丢字节，不能与旧半帧继续拼接。 */
        s_rx_started = 0U;
        s_frame_len = 0U;
        s_discard_frame = 1U;
        return 2U;
    }
    if (!is_pending && remain != 0U) {return 0U;}

    /* 只停 RX，不影响 UART1 的 VOFA 发送。 */
    if (bsp_uart_stop_rx_dma(s_port_huart) != HAL_OK) {
        s_rx_started = 0U;
        s_frame_len = 0U;
        s_discard_frame = 1U;
        return 2U;
    }
    s_rx_started = 0U;

    remain = bsp_uart_get_rx_dma_remain(s_port_huart);
    if (remain > sizeof(s_dma_buffer)) {
        s_frame_len = 0U;
        s_discard_frame = 1U;
        (void)drv_game_controller_port_start_rx();
        return 2U;
    }
    uint16_t rx_len = (uint16_t)(sizeof(s_dma_buffer) - remain);
    memcpy(s_temp_buffer, s_dma_buffer, rx_len);

    HAL_StatusTypeDef status = drv_game_controller_port_start_rx();
    uint8_t has_new_frame = 0U;

    for (uint16_t i = 0U; i < rx_len; ++i) {
        uint8_t byte = s_temp_buffer[i];

        if (byte == '\0') {
            if (!s_discard_frame && s_frame_len > 0U) {
                s_frame_buffer[s_frame_len] = '\0';

                drv_game_controller_buttons_t decoded = {0};
                if (drv_game_controller_decode(s_frame_buffer, &decoded)) {
                    irq_state = __get_PRIMASK();
                    __disable_irq();
                    s_game_controller_data = decoded;
                    s_last_rx_ms = HAL_GetTick();
                    s_has_data = 1U;
                    __set_PRIMASK(irq_state);
                    has_new_frame = 1U;
                }
            }

            s_frame_len = 0U;
            s_discard_frame = 0U;
        }
        else if (!s_discard_frame) {
            if (s_frame_len < sizeof(s_frame_buffer) - 1U) {s_frame_buffer[s_frame_len++] = byte;}
            else {
                // 超长帧：丢弃剩余内容，直到遇到下一个 '\0'
                s_frame_len = 0U;
                s_discard_frame = 1U;
            }
        }
    }

    if (status != HAL_OK) {
        /* 重启失败期间可能丢字节，下一次从分隔符重新同步。 */
        s_frame_len = 0U;
        s_discard_frame = 1U;
        return 2U;
    }
    return has_new_frame;
}

uint8_t drv_game_controller_uart_read(drv_game_controller_buttons_t *out,
                                     uint32_t timeout_ms)
{
    if (!out) {return 0U;}

    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t is_valid = s_has_data && s_rx_started &&
        (uint32_t)(HAL_GetTick() - s_last_rx_ms) <= timeout_ms;
    if (is_valid) {
        *out = s_game_controller_data;
    }
    __set_PRIMASK(irq_state);
    return is_valid;
}
