/**
 * @file    bsp_tim.c
 * @brief   定时器 BSP 实现
 */
#include "bsp_tim.h"

/* 当前注册的基础定时器更新中断回调。 */
static TIM_HandleTypeDef *s_period_htim;
static bsp_tim_period_callback_t s_period_callback;

/**
 * @brief  获取定时器实际输入时钟。
 * @note   当 APB 预分频不为 1 时，STM32 的定时器时钟为 APB 时钟的两倍。
 */
static uint32_t get_timer_clock(TIM_HandleTypeDef *htim)
{
    uint32_t clk;

    /* APB2：TIM1、TIM8~TIM11。 */
    switch ((uint32_t)htim->Instance) {
    case (uint32_t)TIM1:
    case (uint32_t)TIM8:
    case (uint32_t)TIM9:
    case (uint32_t)TIM10:
    case (uint32_t)TIM11:
        clk = HAL_RCC_GetPCLK2Freq();
        if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) != 0U) {
            clk *= 2U;
        }
        break;

    /* APB1：其余通用和基础定时器。 */
    default:
        clk = HAL_RCC_GetPCLK1Freq();
        if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) != 0U) {
            clk *= 2U;
        }
        break;
    }

    return clk;
}

void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Start(htim, channel);
}

void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)HAL_TIM_PWM_Stop(htim, channel);
}

void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel,
                             uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}

void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel,
                          uint32_t freq_hz)
{
    uint32_t timer_clk;
    uint32_t psc;
    uint32_t arr;
    uint32_t old_arr;
    uint32_t old_ccr;
    uint32_t new_ccr;

    if (!htim || freq_hz == 0U) {
        return;
    }

    timer_clk = get_timer_clock(htim);
    old_arr = __HAL_TIM_GET_AUTORELOAD(htim);
    old_ccr = __HAL_TIM_GET_COMPARE(htim, channel);

    /* 选择预分频，使 16 位 ARR 不超过 65535。 */
    psc = (timer_clk / freq_hz / 65536U) + 1U;
    arr = (timer_clk / (psc + 1U) / freq_hz) - 1U;

    /* 改频后按比例保留原有占空比。 */
    if (old_arr > 0U && old_ccr > 0U) {
        new_ccr = (uint32_t)((uint64_t)old_ccr * arr / old_arr);
    } else {
        new_ccr = 0U;
    }

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, new_ccr);
}

void bsp_tim_it_start(TIM_HandleTypeDef *htim)
{
    (void)HAL_TIM_Base_Start_IT(htim);
}

void bsp_tim_it_stop(TIM_HandleTypeDef *htim)
{
    (void)HAL_TIM_Base_Stop_IT(htim);
}

void bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                          bsp_tim_period_callback_t callback)
{
    s_period_htim = htim;
    s_period_callback = callback;
}

/**
 * @brief HAL 定时器更新回调的 BSP 分发入口。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == s_period_htim && s_period_callback) {
        s_period_callback();
    }
}

uint32_t bsp_tim_get_tick_ms(void)
{
    return HAL_GetTick();
}

void bsp_tim_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}