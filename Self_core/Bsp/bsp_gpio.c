/**
 * @file    bsp_gpio.c
 * @brief   GPIO 控制实现
 */
#include "bsp_gpio.h"

void bsp_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    HAL_GPIO_WritePin(port, pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_gpio_toggle_pin(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}
