/**
 * @file    bsp_cfg.h
 * @brief   外设句柄映射（extern 声明 CubeMX 生成的句柄）
 */
#ifndef BSP_CFG_H
#define BSP_CFG_H

#include "stm32f4xx_hal.h"

// ─── CAN 句柄 ────────────────────────────────────

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// ─── UART 句柄 ───────────────────────────────────

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

// ─── SPI 句柄 ────────────────────────────────────

extern SPI_HandleTypeDef hspi1;

// ─── 定时器句柄 ──────────────────────────────────

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim14;

#endif
