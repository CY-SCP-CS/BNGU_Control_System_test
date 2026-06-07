/**
 * @file    drv_led.h
 * @brief   RGB LED 驱动 (纯逻辑层)
 * @note    通过函数指针解耦 GPIO 操作, 不依赖 BSP / HAL
 */
#ifndef DRV_LED_H
#define DRV_LED_H

#include "lib_typedef.h"

// ─── LED 颜色枚举 ───────────────────────────────

typedef enum {
    DRV_LED_R = 0,      /* 红色 */
    DRV_LED_G = 1,      /* 绿色 */
    DRV_LED_B = 2       /* 蓝色 */
} drv_led_color_t;

/**
 * @brief  GPIO 写函数指针类型
 * @param  state  0=低电平(灭), 1=高电平(亮)
 */
typedef void (*drv_led_set_fn_t)(uint8_t state);

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  初始化 LED (默认全灭)
 * @param  set_r  红色引脚写函数
 * @param  set_g  绿色引脚写函数
 * @param  set_b  蓝色引脚写函数
 */
void drv_led_init(drv_led_set_fn_t set_r,
                  drv_led_set_fn_t set_g,
                  drv_led_set_fn_t set_b);

/**
 * @brief  设置单个 LED 亮灭
 * @param  color  颜色 (DRV_LED_R/G/B)
 * @param  state  0=灭, 1=亮
 */
void drv_led_set(drv_led_color_t color, uint8_t state);

/**
 * @brief  翻转单个 LED
 * @param  color  颜色 (DRV_LED_R/G/B)
 */
void drv_led_toggle(drv_led_color_t color);

/**
 * @brief  同时设置 RGB
 * @param  r  红色 0/1
 * @param  g  绿色 0/1
 * @param  b  蓝色 0/1
 */
void drv_led_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif
