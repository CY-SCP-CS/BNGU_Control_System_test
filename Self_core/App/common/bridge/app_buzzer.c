/**
 * @file    app_buzzer.c
 * @brief   蜂鸣器上层胶水 — 挂接 BSP TIM4_PWM 与 DRV 蜂鸣器
 */
#include "app_buzzer.h"

#include "bsp_tim.h"
#include "drv_buzzer.h"

// ─── 私有变量 ────────────────────────────────────

/* TIM4_CH3 = PD14, PWM 驱动有源蜂鸣器 */
static TIM_HandleTypeDef *s_htim = &htim4;

// ─── 私有函数 ────────────────────────────────────

static void buzzer_set(uint8_t state)
{
    if (state)
        bsp_tim_pwm_start(s_htim, TIM_CHANNEL_3);
    else
        bsp_tim_pwm_stop(s_htim, TIM_CHANNEL_3);
}

static void buzzer_set_freq(uint16_t freq_hz)
{
    bsp_tim_pwm_set_freq(s_htim, TIM_CHANNEL_3, freq_hz);
}

static void buzzer_set_duty(uint16_t duty)
{
    bsp_tim_pwm_set_compare(s_htim, TIM_CHANNEL_3, duty);
}

// ─── 接口实现 ─────────────────────────────────────

void app_buzzer_init(void)
{
    drv_buzzer_init(buzzer_set, buzzer_set_freq, buzzer_set_duty);
}

void app_buzzer_on(void)
{
    drv_buzzer_on();
}

void app_buzzer_off(void)
{
    drv_buzzer_off();
}
