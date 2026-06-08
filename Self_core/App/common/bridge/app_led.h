/**
 * @file    app_led.h
 * @brief   LED 上层胶水 — 挂接 BSP GPIO 与 DRV LED
 */
#ifndef APP_LED_H
#define APP_LED_H

#include "lib_typedef.h"

/**
 * @brief  初始化 LED (PH10-R, PH11-G, PH12-B, 默认全灭)
 */
void app_led_init(void);

/**
 * @brief  同时设置 RGB
 * @param  r  红色 0/1
 * @param  g  绿色 0/1
 * @param  b  蓝色 0/1
 */
void app_led_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif
