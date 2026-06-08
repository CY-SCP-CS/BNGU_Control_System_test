/**
 * @file    drv_buzzer.h
 * @brief   蜂鸣器驱动 (纯逻辑层)
 * @note    通过函数指针解耦 PWM 操作, 不依赖 BSP / HAL
 *          支持开关控制与音调设置
 */
#ifndef DRV_BUZZER_H
#define DRV_BUZZER_H

#include "lib_typedef.h"

// ─── 函数指针类型 ────────────────────────────────

/**
 * @brief  蜂鸣器开关控制函数指针
 * @param  state  0=关闭, 1=开启
 */
typedef void (*drv_buzzer_set_fn_t)(uint8_t state);

/**
 * @brief  蜂鸣器音调设置函数指针
 * @param  freq_hz  频率 (Hz), 典型值 1000~5000
 */
typedef void (*drv_buzzer_freq_fn_t)(uint16_t freq_hz);

/**
 * @brief  蜂鸣器占空比设置函数指针
 * @param  duty  占空比比较值 (0 ~ ARR)
 */
typedef void (*drv_buzzer_duty_fn_t)(uint16_t duty);

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  初始化蜂鸣器 (默认关闭)
 * @param  set       开关控制函数 (state: 0=关, 1=开)
 * @param  set_freq  音调设置函数 (freq_hz: 频率)
 * @param  set_duty  占空比设置函数 (duty: 比较值), 可传 NULL
 */
void drv_buzzer_init(drv_buzzer_set_fn_t set, drv_buzzer_freq_fn_t set_freq,
                     drv_buzzer_duty_fn_t set_duty);

/**
 * @brief  打开蜂鸣器
 */
void drv_buzzer_on(void);

/**
 * @brief  关闭蜂鸣器
 */
void drv_buzzer_off(void);

/**
 * @brief  翻转蜂鸣器状态
 */
void drv_buzzer_toggle(void);

/**
 * @brief  设置蜂鸣器音调
 * @param  freq_hz  频率 (Hz)
 */
void drv_buzzer_set_freq(uint16_t freq_hz);

/**
 * @brief  设置蜂鸣器占空比
 * @param  duty  占空比比较值 (0 ~ ARR), 0=静音
 */
void drv_buzzer_set_duty(uint16_t duty);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  初始化蜂鸣器 (BSP 适配版)
 * @note   内部调用 drv_buzzer_init, 挂接 TIM4_PWM (PD14)
 */
void drv_buzzer_port_init(void);

#endif
