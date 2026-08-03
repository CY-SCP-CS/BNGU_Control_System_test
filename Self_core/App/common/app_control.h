/**
 * @file    app_control.h
 * @brief   统一控制调度 — 1kHz 控制循环入口
 * @note    所有车组共用此文件, 通过 CURRENT_BOARD / CURRENT_ROBOT 分支
 */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H

void app_control_1khz(void);

#endif
