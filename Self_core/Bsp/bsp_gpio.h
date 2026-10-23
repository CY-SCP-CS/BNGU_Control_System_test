/**
 * @file    bsp_gpio.h
 * @brief   GPIO 控制抽象
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

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

typedef void (*bsp_gpio_exti_callback_t)(void *context);
/**
 * @brief  注册 GPIO 外部中断回调
 * @param  pin      GPIO 引脚
 * @param  callback 中断回调函数
 * @param  context  回调函数上下文
 * @return 0=成功, 1=失败
 */
uint8_t bsp_gpio_exti_register(uint16_t pin, bsp_gpio_exti_callback_t callback, void *context);

#endif
