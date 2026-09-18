/**
 * @file    bsp_tim.h
 * @brief   定时器 BSP：PWM、基础定时器中断与系统节拍接口
 */
#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "bsp_cfg.h"

/** @brief 定时器更新中断回调类型。 */
typedef void (*bsp_tim_period_callback_t)(void);

/**
 * @brief  启动 PWM 输出。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道，如 TIM_CHANNEL_1。
 */
void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  停止 PWM 输出。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 */
void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  设置 PWM 频率，并保持原有占空比。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 * @param  freq_hz  目标频率，单位 Hz。
 */
void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel,
                          uint32_t freq_hz);

/**
 * @brief  设置 PWM 比较值。
 * @param  htim     定时器句柄。
 * @param  channel  PWM 通道。
 * @param  compare  比较值，范围为 0 到 ARR。
 */
void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel,
                             uint32_t compare);

/**
 * @brief  启动基础定时器更新中断。
 * @param  htim  定时器句柄。
 */
void bsp_tim_it_start(TIM_HandleTypeDef *htim);

/**
 * @brief  停止基础定时器更新中断。
 * @param  htim  定时器句柄。
 */
void bsp_tim_it_stop(TIM_HandleTypeDef *htim);

/**
 * @brief  注册一个定时器更新中断回调。
 * @param  htim      要绑定的定时器句柄。
 * @param  callback  更新中断发生时调用的函数。
 * @note   当前 BSP 只保存一个定时器回调。
 */
void bsp_tim_reg_callback(TIM_HandleTypeDef *htim,
                          bsp_tim_period_callback_t callback);

/**
 * @brief  获取 HAL 系统节拍。
 * @return 自系统启动以来的毫秒数。
 */
uint32_t bsp_tim_get_tick_ms(void);

/**
 * @brief  阻塞延时。
 * @param  ms  延时时间，单位 ms。
 */
void bsp_tim_delay_ms(uint32_t ms);

#endif