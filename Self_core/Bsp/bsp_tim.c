/**
 * @file    bsp_tim.c
 * @brief   定时器驱动实现
 */
#include "bsp_tim.h"

// ─── 私有函数 ────────────────────────────────────

/**
 * @brief  获取定时器总线时钟 (考虑 APB 预分频)
 */
static uint32_t get_timer_clock(TIM_HandleTypeDef *htim)
{
    uint32_t clk;

    /* APB2: TIM1, TIM8~11 */
    switch ((uint32_t)htim->Instance) {
        case (uint32_t)TIM1:
        case (uint32_t)TIM8:
        case (uint32_t)TIM9:
        case (uint32_t)TIM10:
        case (uint32_t)TIM11:
            clk = HAL_RCC_GetPCLK2Freq();
            if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) != 0)
                clk *= 2;
            break;

        /* APB1: 其余所有 (TIM2~7, TIM12~14) */
        default:
            clk = HAL_RCC_GetPCLK1Freq();
            if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) != 0)
                clk *= 2;
            break;
    }

    return clk;
}

// ─── PWM ─────────────────────────────────────────

void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_TIM_PWM_Start(htim, channel);
}

void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_TIM_PWM_Stop(htim, channel);
}

void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}

void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz)
{
    uint32_t timer_clk = get_timer_clock(htim);
    uint32_t psc, arr;

    /* 计算 PSC 使 ARR ≤ 65535 (16位定时器), 占空比 50% */
    psc = (timer_clk / freq_hz / 65536) + 1;
    arr = (timer_clk / (psc + 1) / freq_hz) - 1;

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);
}
