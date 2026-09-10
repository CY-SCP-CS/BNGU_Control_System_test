/**
 * @file    app_control.h
 * @brief   统一控制调度 — 1kHz 控制循环入口
 * @note    所有车组共用此文件, 通过 CURRENT_BOARD / CURRENT_ROBOT 分支
 */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/** @brief 主循环调用，执行通信解析等后台任务。 */
void app_control_process(void);
/** @brief TIM14 中断每 1ms 调用一次，执行固定周期控制。 */
void app_control_1khz(void);

#endif
