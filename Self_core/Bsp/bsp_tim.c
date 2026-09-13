/**
 * @file    bsp_tim.c
 * @brief   定时器驱动实现
 */
#include "bsp_tim.h"

static TIM_HandleTypeDef *s_period_htim;
static bsp_tim_period_callback_t s_period_callback;

/**
 * @brief  获取定时器总线时钟 (考虑 APB 预分频)
 * @param  htim 定时器句柄
 * @return 定时器总线时钟 (Hz)
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
    uint32_t old_arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t old_ccr = __HAL_TIM_GET_COMPARE(htim, channel);
    uint32_t new_ccr;

    psc = (timer_clk / freq_hz / 65536) + 1;
    arr = (timer_clk / (psc + 1) / freq_hz) - 1;

    if (old_arr > 0 && old_ccr > 0) {
        new_ccr = (uint32_t)((uint64_t)old_ccr * arr / old_arr);
    } else {
        new_ccr = 0;
    }

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, new_ccr);
}


void bsp_tim_it_start(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start_IT(htim);
}

void bsp_tim_it_stop(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
}

void bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                                      bsp_tim_period_callback_t callback)
{
    s_period_htim = htim;
    s_period_callback = callback;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == s_period_htim && s_period_callback) {
        s_period_callback();
    }
}
