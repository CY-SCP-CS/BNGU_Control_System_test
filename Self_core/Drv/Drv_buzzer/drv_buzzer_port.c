/**
 * @file    drv_buzzer_port.c
 * @brief   BSP 适配 — drv_buzzer 的硬件端口挂接
 */
#include "drv_buzzer.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_cfg.h"
#include "bsp_tim.h"

/* TIM4_CH3 = PD14, PWM 驱动有源蜂鸣器 */
static TIM_HandleTypeDef *s_port_htim = &htim4;

static void port_buzzer_set(uint8_t state)
{
    if (state)
        bsp_tim_pwm_start(s_port_htim, TIM_CHANNEL_3);
    else
        bsp_tim_pwm_stop(s_port_htim, TIM_CHANNEL_3);
}

static void port_buzzer_set_freq(uint16_t freq_hz)
{
    bsp_tim_pwm_set_freq(s_port_htim, TIM_CHANNEL_3, freq_hz);
}

static void port_buzzer_set_duty(uint16_t duty)
{
    bsp_tim_pwm_set_compare(s_port_htim, TIM_CHANNEL_3, duty);
}

void drv_buzzer_port_init(void)
{
    drv_buzzer_init(port_buzzer_set, port_buzzer_set_freq, port_buzzer_set_duty);
}
