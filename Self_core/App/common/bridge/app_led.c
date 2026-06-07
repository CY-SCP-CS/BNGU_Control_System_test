/**
 * @file    app_led.c
 * @brief   LED 上层胶水 — 挂接 BSP GPIO 与 DRV LED
 */
#include "app_led.h"

#include "bsp_gpio.h"
#include "drv_led.h"

// ─── 私有函数 (函数指针适配器) ────────────────────

static void led_r_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_10, state);
}

static void led_g_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_11, state);
}

static void led_b_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_12, state);
}

// ─── 接口实现 ─────────────────────────────────────

void app_led_init(void)
{
    drv_led_init(led_r_set, led_g_set, led_b_set);
}
