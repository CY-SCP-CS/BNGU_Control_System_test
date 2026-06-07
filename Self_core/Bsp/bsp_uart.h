/**
 * @file    bsp_uart.h
 * @brief   UART 驱动: 非阻塞发送 / 回调订阅
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "lib_typedef.h"
#include "bsp_cfg.h"

// ─── 回调注册 ────────────────────────────────────

#define BSP_UART_RX_CALLBACK_MAX  8
#define BSP_UART_TX_CALLBACK_MAX  4

typedef void (*bsp_uart_rx_callback_t)(uint8_t *data, uint16_t len);
typedef void (*bsp_uart_tx_callback_t)(void);

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  非阻塞发送 (中断方式)
 * @param  huart UART 句柄
 * @param  data  发送缓冲区 (需保持有效直到传输完成)
 * @param  len   发送长度
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  注册 UART 接收回调
 * @param  huart    UART 句柄
 * @param  callback 回调函数
 * @note   用法示例:
 *         static void on_dbus_rx(uint8_t *data, uint16_t len) {
 *             drv_dbus_solve(data);
 *         }
 *         bsp_uart_register_rx_callback(&huart1, on_dbus_rx);
 */
void bsp_uart_register_rx_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_rx_callback_t callback);

/**
 * @brief  注册 UART 发送完成回调
 * @param  huart    UART 句柄
 * @param  callback 回调函数 (发送完成后被调用)
 * @note   用法示例:
 *         static void on_tx_done(void) {
 *             s_busy = 0;  // 释放发送忙标志, 允许发下一帧
 *         }
 *         bsp_uart_register_tx_callback(&huart1, on_tx_done);
 */
void bsp_uart_register_tx_callback(UART_HandleTypeDef *huart,
                                   bsp_uart_tx_callback_t callback);

/**
 * @brief  UART 接收中断入口 (在 HAL 回调中调用)
 * @param  huart UART 句柄
 */
void bsp_uart_rx_irq_handler(UART_HandleTypeDef *huart);

/**
 * @brief  UART 发送中断入口 (在 HAL 回调中调用)
 * @param  huart UART 句柄
 */
void bsp_uart_tx_irq_handler(UART_HandleTypeDef *huart);

#endif