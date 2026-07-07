/**
 * @file    app_init.h
 * @brief   系统初始化入口 — 统一初始化所有硬件驱动和应用模块
 * @note    作为 main.c 的唯一 App 层入口, 替代传统手动初始化流程
 */
#ifndef APP_INIT_H
#define APP_INIT_H

void app_init(void);

#endif