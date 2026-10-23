/**
 * @file    bsp_gpio.c
 * @brief   GPIO 控制实现
 */
#include "bsp_gpio.h"
#include <stddef.h>

static struct {
    bsp_gpio_exti_callback_t callback;
    void *context;
} s_exti[16];

uint8_t bsp_gpio_exti_register(uint16_t pin, bsp_gpio_exti_callback_t callback, void *context)
{
    if (pin == 0U || (pin & (pin - 1U)) != 0U) return 1U;
    unsigned index = 0U;
    while ((pin >> index) != 1U) ++index;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (callback != NULL && s_exti[index].callback != NULL &&
        (s_exti[index].callback != callback || s_exti[index].context != context)) {
        __set_PRIMASK(mask);
        return 1U;
    }
    s_exti[index].context = context;
    s_exti[index].callback = callback;
    __set_PRIMASK(mask);
    return 0U;
}

/* Single HAL callback owner; dispatch is independent of any particular sensor. */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    for (unsigned index = 0U; index < 16U; ++index) {
        if ((pin & (1U << index)) != 0U && s_exti[index].callback != NULL)
            s_exti[index].callback(s_exti[index].context);
    }
}

void bsp_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    HAL_GPIO_WritePin(port, pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_gpio_toggle_pin(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}
