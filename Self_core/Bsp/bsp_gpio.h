/**
 * @file    bsp_gpio.h
 * @brief   GPIO 控制抽象
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "lib_typedef.h"
#include "bsp_cfg.h"

/**
 * @brief  写 GPIO 引脚电平
 * @param  port   GPIO 端口 (如 GPIOH)
 * @param  pin    GPIO 引脚 (如 GPIO_PIN_10)
 * @param  state  0=低电平, 1=高电平
 */
void bsp_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state);

/**
 * @brief  翻转 GPIO 引脚电平
 * @param  port   GPIO 端口
 * @param  pin    GPIO 引脚
 */
void bsp_gpio_toggle_pin(GPIO_TypeDef *port, uint16_t pin);

#endif
