/**
 * @file    app_buzzer.h
 * @brief   蜂鸣器上层胶水 — 挂接 BSP TIM4_PWM 与 DRV 蜂鸣器
 */
#ifndef APP_BUZZER_H
#define APP_BUZZER_H

/**
 * @brief  初始化蜂鸣器 (默认关闭)
 */
void app_buzzer_init(void);

#endif
