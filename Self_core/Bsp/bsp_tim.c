﻿/**
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
    uint32_t old_arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t old_ccr = __HAL_TIM_GET_COMPARE(htim, channel);
    uint32_t new_ccr;

    psc = (timer_clk / freq_hz / 65536) + 1;
    arr = (timer_clk / (psc + 1) / freq_hz) - 1;

    if (old_arr > 0 && old_ccr > 0) {
        new_ccr = (uint32_t)((uint64_t)old_ccr * arr / old_arr);
    } else {
        new_ccr = arr / 2;
    }

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, new_ccr);
}

// ─── 定时中断 ─────────────────────────────────────

void bsp_tim_it_start(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start_IT(htim);
}

void bsp_tim_it_stop(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
}

// ──── 定时器周期中断回调注册 ────────────────────────

typedef struct {
    TIM_HandleTypeDef          *htim;
    bsp_tim_period_callback_t   callback;
} bsp_tim_callback_entry_t;

static bsp_tim_callback_entry_t s_tim_callbacks[BSP_TIM_CALLBACK_MAX];
static uint8_t                  s_tim_callback_count;

HAL_StatusTypeDef bsp_tim_register_period_callback(
    TIM_HandleTypeDef *htim,
    bsp_tim_period_callback_t callback)
{
    uint8_t i;

    for (i = 0; i < s_tim_callback_count; i++) {
        if (s_tim_callbacks[i].htim == htim) {
            s_tim_callbacks[i].callback = callback;
            return HAL_OK;
        }
    }

    if (s_tim_callback_count >= BSP_TIM_CALLBACK_MAX)
        return HAL_ERROR;

    s_tim_callbacks[s_tim_callback_count].htim     = htim;
    s_tim_callbacks[s_tim_callback_count].callback = callback;
    s_tim_callback_count++;
    return HAL_OK;
}

void bsp_tim_period_irq_handler(TIM_HandleTypeDef *htim)
{
    uint8_t i;

    for (i = 0; i < s_tim_callback_count; i++) {
        if (s_tim_callbacks[i].htim == htim && s_tim_callbacks[i].callback) {
            s_tim_callbacks[i].callback(htim);
            return;
        }
    }
}