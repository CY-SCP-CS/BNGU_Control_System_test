/**
 * @file    bsp_uart.h
 * @brief   UART 驱动：DMA 接收、非阻塞发送与中断回调分发
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_cfg.h"

#define BSP_UART_TX_CALLBACK_MAX       3U
#define BSP_UART_RX_IDLE_CALLBACK_MAX  3U

/** @brief UART 发送完成回调类型。 */
typedef void (*bsp_uart_tx_cplt_callback_t)(UART_HandleTypeDef *huart);

/** @brief UART 空闲中断回调类型。 */
typedef void (*bsp_uart_rx_idle_callback_t)(UART_HandleTypeDef *huart);

/**
 * @brief  以中断方式发送数据，不阻塞调用者。
 * @param  huart UART 句柄。
 * @param  data  发送缓冲区。
 * @param  len   发送字节数。
 * @return HAL 库执行状态。
 */
HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  注册指定 UART 的发送完成回调。
 * @param  huart    UART 句柄。
 * @param  callback 发送完成后调用的函数。
 */
void bsp_uart_reg_tx_cplt_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_tx_cplt_callback_t callback);

/**
 * @brief  启动 DMA 接收。
 * @param  huart UART 句柄。
 * @param  data  接收缓冲区。
 * @param  len   接收缓冲区长度，单位字节。
 * @return HAL 库执行状态。
 */
HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  停止 UART 的 DMA 接收。
 * @param  huart UART 句柄。
 * @return HAL 库执行状态。
 */
HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart);

/**
 * @brief  获取 DMA 尚未写入的接收缓冲区字节数。
 * @param  huart UART 句柄。
 * @return DMA 剩余传输字节数；句柄或 DMA 无效时返回 0。
 */
uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart);

/**
 * @brief  判断 UART 是否产生空闲标志。
 * @param  huart UART 句柄。
 * @return 1 表示空闲标志有效，0 表示未产生或句柄无效。
 */
uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart);

/**
 * @brief 清除 UART 空闲中断标志。
 * @param huart UART 句柄。
 */
void bsp_uart_clear_idle(UART_HandleTypeDef *huart);

/**
 * @brief 使能 UART 的空闲中断。
 * @param huart UART 句柄。
 */
void bsp_uart_enable_idle_it(UART_HandleTypeDef *huart);

/**
 * @brief 处理 USARTx_IRQHandler。
 * @note  该函数先分发空闲中断；其余中断继续交由 HAL_UART_IRQHandler 处理。
 * @param  huart UART 句柄。
 */
void bsp_uart_irq_handler(UART_HandleTypeDef *huart);

/**
 * @brief  注册指定 UART 的空闲中断回调。
 * @param  huart    UART 句柄。
 * @param  callback 空闲中断发生后调用的函数。
 */
void bsp_uart_reg_rx_idle_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_rx_idle_callback_t callback);

#endif