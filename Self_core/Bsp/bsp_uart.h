/**
 * @file    bsp_uart.h
 * @brief   UART 驱动: 非阻塞发送 / 回调订阅
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_cfg.h"

#define BSP_UART_TX_CALLBACK_MAX  3U

/** @brief UART 发送完成回调类型。 */
typedef void (*bsp_uart_tx_cplt_callback_t)(UART_HandleTypeDef *huart);

/**
 * @brief  非阻塞发送 (中断方式)
 * @param  huart UART 句柄
 * @param  data  发送缓冲区
 * @param  len   发送长度
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bsp_uart_tx(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  注册指定 UART 的发送完成回调
 * @param  huart UART 句柄
 * @param  callback 回调函数指针
 */
void bsp_uart_reg_tx_cplt_callback(
    UART_HandleTypeDef *huart,
    bsp_uart_tx_cplt_callback_t callback);
/**
 * @brief  非阻塞接收 (DMA方式)
 * @param  huart UART 句柄
 * @param  data  接收缓冲区
 * @param  len   接收长度
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bsp_uart_rx_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
/**
 * @brief  停止 DMA 接收
 * @param  huart UART 句柄
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bsp_uart_stop_dma(UART_HandleTypeDef *huart);
/**
 * @brief  获取 DMA 接收剩余字节数
 * @param  huart UART 句柄
 * @return uint16_t 剩余字节数
 */
uint16_t bsp_uart_get_rx_dma_remain(const UART_HandleTypeDef *huart);
/**
 * @brief  检查 UART 是否空闲
 * @param  huart UART 句柄
 * @return uint8_t
 */
uint8_t bsp_uart_is_idle(const UART_HandleTypeDef *huart);
/**
 * @brief  清除 UART 空闲状态
 * @param  huart UART 句柄
 */
void bsp_uart_clear_idle(UART_HandleTypeDef *huart);

#endif
