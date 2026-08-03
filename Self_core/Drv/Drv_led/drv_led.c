/**
 * @file    drv_led.c
 * @brief   RGB LED 驱动实现 (纯逻辑层)
 */
#include "drv_led.h"

// ─── 私有变量 ────────────────────────────────────

static drv_led_set_fn_t s_set[3];   /* 各颜色写函数 */
static uint8_t          s_state[3]; /* 0=灭, 1=亮   */

// ─── 接口实现 ─────────────────────────────────────

void drv_led_init(drv_led_set_fn_t set_r,
                  drv_led_set_fn_t set_g,
                  drv_led_set_fn_t set_b)
{
    s_set[DRV_LED_R] = set_r;
    s_set[DRV_LED_G] = set_g;
    s_set[DRV_LED_B] = set_b;

    s_state[DRV_LED_R] = 0;
    s_state[DRV_LED_G] = 0;
    s_state[DRV_LED_B] = 0;

    /* 初始化全灭 */
    if (s_set[DRV_LED_R]) s_set[DRV_LED_R](0);
    if (s_set[DRV_LED_G]) s_set[DRV_LED_G](0);
    if (s_set[DRV_LED_B]) s_set[DRV_LED_B](0);
}

void drv_led_set(drv_led_color_t color, uint8_t state)
{
    if (color > DRV_LED_B) return;
    if (!s_set[color])     return;

    s_state[color] = state ? 1 : 0;
    s_set[color](s_state[color]);
}

void drv_led_toggle(drv_led_color_t color)
{
    if (color > DRV_LED_B) return;
    drv_led_set(color, !s_state[color]);
}

void drv_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t val[] = {r, g, b};
    uint8_t i;

    for (i = 0; i <= DRV_LED_B; i++) {
        drv_led_set((drv_led_color_t)i, val[i]);
    }
}