/**
 * @file    bsp_tim.h
 * @brief   定时器 PWM / 编码器 / 定时中断
 */
#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "bsp_cfg.h"
//没有实现编码器，目前感觉没必要

/** @brief 定时器周期中断回调类型。 */
typedef void (*bsp_tim_period_callback_t)(void);

/**
 * @brief  启动 PWM 输出
 * @param  htim    定时器句柄
 * @param  channel 通道 (TIM_CHANNEL_1 ~ TIM_CHANNEL_4)
 */
void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  停止 PWM 输出
 * @param  htim    定时器句柄
 * @param  channel 通道
 */
void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  设置 PWM 频率
 * @param  htim    定时器句柄
 * @param  channel 通道
 * @param  freq_hz 目标频率 (Hz)
 * @note   自动计算 PSC + ARR, 保持占空比不变
 */
void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz);

/**
 * @brief  设置 PWM 比较值 (占空比)
 * @param  htim    定时器句柄
 * @param  channel 通道
 * @param  compare 比较值 (0 ~ ARR)
 */
void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare);

/**
 * @brief  启动定时器基础定时中断
 * @param  htim  定时器句柄
 */
void bsp_tim_it_start(TIM_HandleTypeDef *htim);

/**
 * @brief  停止定时器基础定时中断
 * @param  htim  定时器句柄
 */
void bsp_tim_it_stop(TIM_HandleTypeDef *htim);

/**
 * @brief  注册周期中断回调
 * @param  htim      定时器句柄
 * @param  callback  周期到达时执行的函数
 */
void bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                                      bsp_tim_period_callback_t callback);


#endif
