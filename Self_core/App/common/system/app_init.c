/**
 * @file    app_init.c
 * @brief   系统初始化 — 按板型/车组分支, 统一初始化所有硬件驱动和应用模块
 * @note    app_init 负责系统级硬件与应用初始化，控制运算由 TIM14 固定周期中断调度。
 *          初始化顺序: BSP CAN 启动 -> DRV 端口初始化 -> APP 模块初始化,
 *          并按 CURRENT_BOARD / CURRENT_ROBOT 宏进行条件分支。
 */
#include "app_init.h"

#include "project_cfg.h"

// BSP (仅 app_init 作为系统级入口允许直接包含)
#include "bsp_cfg.h"
#include "bsp_can.h"

// DRV
#include "drv_buzzer.h"
#include "drv_dbus.h"
#include "drv_imu.h"
#include "drv_led.h"
#include "drv_melody.h"
#include "drv_referee.h"
#include "drv_vofa.h"

// APP
#include "app_chassis_comm.h"
#include "app_diagnostic.h"
#include "app_gimbal_comm.h"

#if CURRENT_ROBOT == ROBOT_SENTRY
#include "app_sentry_chassis.h"
#include "app_sentry_gimbal.h"
#endif

// ─── 私有宏 ──────────────────────────────────────

#define APP_INIT_VOFA_CH_COUNT   10

// ─── 私有变量 ────────────────────────────────────

#if CURRENT_BOARD == BOARD_GIMBAL
static drv_imu_t s_imu;
#endif

// ─── 私有函数声明 ────────────────────────────────

// ─── 公有接口 ─────────────────────────────────────

void app_init(void)
{
    // ── 1. BSP 层 CAN 启动 (必须最先, DRV 层 CAN 通信依赖此项) ──

    bsp_can_start(&hcan1, 0, 14);
    bsp_can_start(&hcan2, 14, 14);

    // ── 2. 基础外设驱动 (无条件, 所有板都需要) ──

    drv_led_port_init();
    drv_buzzer_port_init();
    //drv_melody_init();
    drv_dbus_port_init();
#if CURRENT_BOARD == BOARD_GIMBAL
    drv_imu_port_init(&s_imu);
#endif
    drv_vofa_port_init(APP_INIT_VOFA_CH_COUNT);

    // ── 3. 通用 APP 模块 ──

    //app_diagnostic_init();
    app_chassis_comm_init();
    app_gimbal_comm_init();

    // ── 3.1 底盘专用: 裁判系统 (USART6 直连) ──
#if CURRENT_BOARD == BOARD_CHASSIS
    drv_referee_port_init();
#endif

    // ── 4. 按车组分支 ──

#if CURRENT_ROBOT == ROBOT_HERO
    // 英雄特殊初始化 (待实现)
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    // 步兵特殊初始化 (待实现)
#elif CURRENT_ROBOT == ROBOT_SENTRY
    #if CURRENT_BOARD == BOARD_CHASSIS
        app_chassis_init();
    #else
        app_gimbal_init(&s_imu);
    #endif
#endif
}

// ─── 私有函数定义 ─────────────────────────────────
