/**
 * @file    app_dbus.h
 * @brief   DBUS 上层胶水 — UART DMA + IDLE 接收, 挂接 DRV 解码
 */
#ifndef APP_DBUS_H
#define APP_DBUS_H

#include "drv_dbus.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  初始化 DBUS (UART DMA + IDLE 中断)
 * @param  huart   UART 句柄 (DBUS 接收口)
 * @param  hdma    UART RX DMA 句柄
 * @note   通常在 main 初始化阶段调用一次
 */
void app_dbus_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/**
 * @brief  UART IDLE 中断入口 (在 USARTx_IRQHandler 中调用)
 * @note   用户需要在 stm32f4xx_it.c 的 USARTx_IRQHandler 中调用此函数
 */
void app_dbus_irq_handler(void);

/**
 * @brief  获取最新解码的 DBUS 数据
 * @return const drv_dbus_data_t*  只读指针, 无需释放
 */
const drv_dbus_data_t *app_dbus_get_data(void);

#endif
