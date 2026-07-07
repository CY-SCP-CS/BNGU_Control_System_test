/**
 * @file    app_init.c
 * @brief   系统初始化 — 按板型/车组分支, 统一初始化所有硬件驱动和应用模块
 * @note    app_init 作为系统级初始化编排器, 是唯一允许直接包含 BSP 的 App 模块。
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
#include "drv_vofa.h"

// APP
#include "app_diagnostic.h"

// ─── 私有宏 ──────────────────────────────────────

#define APP_INIT_VOFA_CH_COUNT   10

// ─── 私有变量 ────────────────────────────────────

static drv_imu_t s_imu;

// ─── 私有函数声明 ────────────────────────────────

static void app_init_vofa_tx_cb(void);

// ─── 公有接口 ─────────────────────────────────────

void app_init(void)
{
    // ── 1. BSP 层 CAN 启动 (必须最先, DRV 层 CAN 通信依赖此项) ──

    bsp_can_start(&hcan1, 0, 14);
    bsp_can_start(&hcan2, 14, 0);

    // ── 2. 基础外设驱动 (无条件, 所有板都需要) ──

    drv_led_port_init();
    drv_buzzer_port_init();
    drv_melody_init();
    drv_dbus_port_init();
    drv_imu_port_init(&s_imu);
    drv_imu_start(&s_imu);
    drv_vofa_port_init(&huart1, app_init_vofa_tx_cb, APP_INIT_VOFA_CH_COUNT);

    // ── 3. 通用 APP 模块 (无条件) ──

    app_diagnostic_init();

    // ── 4. 按车组分支 ──

#if CURRENT_ROBOT == ROBOT_HERO
    // 英雄特殊初始化 (待实现)
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    // 步兵特殊初始化 (待实现)
#elif CURRENT_ROBOT == ROBOT_SENTRY
    // 哨兵特殊初始化 (待实现)
#endif
}

// ─── 私有函数定义 ─────────────────────────────────

/**
 * @brief  VOFA UART 发送完成回调
 * @note   由 UART TX 完成中断调用, 释放 VOFA 发送忙标志
 */
static void app_init_vofa_tx_cb(void)
{
    drv_vofa_tx_complete();
}