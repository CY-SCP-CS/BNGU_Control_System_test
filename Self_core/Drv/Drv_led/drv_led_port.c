/**
 * @file    drv_led_port.c
 * @brief   BSP 适配 — drv_led 的硬件端口挂接
 */
#include "drv_led.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_cfg.h"
#include "bsp_gpio.h"

static void port_led_r_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_10, state);
}

static void port_led_g_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_11, state);
}

static void port_led_b_set(uint8_t state)
{
    bsp_gpio_write_pin(GPIOH, GPIO_PIN_12, state);
}

void drv_led_port_init(void)
{
    drv_led_init(port_led_r_set, port_led_g_set, port_led_b_set);
}
