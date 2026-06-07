/**
 * @file    app_vofa.h
 * @brief   VOFA+ 上层胶水 — 串联 BSP UART 与 DRV VOFA
 */
#ifndef APP_VOFA_H
#define APP_VOFA_H

#include "lib_typedef.h"
#include "bsp_uart.h"

/**
 * @brief  初始化 VOFA 发送通道
 * @param  huart     UART 句柄 (用哪个串口发)
 * @param  ch_count  通道数
 */
void app_vofa_init(UART_HandleTypeDef *huart, uint8_t ch_count);

/**
 * @brief  发送一帧数据 (非阻塞)
 * @param  fdata  待发送的浮点数组
 */
void app_vofa_send(float *fdata);

#endif
