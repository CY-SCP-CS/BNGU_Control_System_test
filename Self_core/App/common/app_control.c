/**
 * @file    app_control.c
 * @brief   统一控制调度 — 1kHz 控制循环, 按板型/车组分支
 * @note    由 SysTick 或 TIM 中断周期性调用 (1kHz)。
 *          调度顺序: 通用更新 -> 按板分支 -> 按车组分支
 */
#include "app_control.h"

#include "project_cfg.h"

// APP (仅依赖已实现的模块)
#include "app_diagnostic.h"

// ─── 公有接口 ─────────────────────────────────────

void app_control_1khz(void)
{
    // ── 1. 通用模块更新 (底盘/云台共用) ──

    app_diagnostic_update(NULL);

    // TODO: app_monitor_update();
    // TODO: app_referee_update();

    // ── 2. 按板型分支 ──

#if CURRENT_BOARD == BOARD_CHASSIS
    // TODO: app_chassis_control();
#else
    // TODO: app_gimbal_control();
    // TODO: app_shoot_control();
#endif

    // ── 3. 按车组分支 ──

#if CURRENT_ROBOT == ROBOT_HERO
    // TODO: 英雄特殊逻辑
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    // TODO: 步兵特殊逻辑
#elif CURRENT_ROBOT == ROBOT_SENTRY
    // TODO: 哨兵特殊逻辑 (自动瞄准等)
#endif
}