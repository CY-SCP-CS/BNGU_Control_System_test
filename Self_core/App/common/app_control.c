/**
 * @file    app_control.c
 * @brief   统一控制调度 — 1kHz 控制循环, 按板型/车组分支
 * @note    调度顺序: 1kHz AHRS → 1kHz/200Hz 控制 → 板间通信
 */
#include "app_control.h"

#include "project_cfg.h"

// APP
#include "app_diagnostic.h"

/* ── Sentry ── */
#if CURRENT_ROBOT == ROBOT_SENTRY
#include "app_sentry_common.h"
#include "app_sentry_chassis.h"
#include "app_sentry_gimbal.h"
#endif

// ─── 分频计数 ─────────────────────────────────────

static uint16_t s_ctrl_divider;   /* 1kHz→200Hz 分频 (uint16_t 避免快速回绕) */

// ─── 公有接口 ─────────────────────────────────────

void app_control_1khz(void)
{
    /* ── 1. 通用: 诊断更新 ── */
    app_diagnostic_update(NULL);

    /* ── 2. 板型分支 ── */

#if CURRENT_BOARD == BOARD_CHASSIS

    #if CURRENT_ROBOT == ROBOT_SENTRY
        app_sentry_chassis_control();             /**< 底盘 1kHz */
    #else
        // TODO: app_chassis_control();
    #endif

#else /* BOARD_GIMBAL */

    #if CURRENT_ROBOT == ROBOT_SENTRY
        /* ── 哨兵云台: AHRS@1kHz + 控制@200Hz ── */
        app_sentry_gimbal_ahrs_update(0.001f);    /**< IMU Mahony 1kHz */
        if ((s_ctrl_divider % 5) == 0) {          /**< Yaw/Pitch/Launch 200Hz */
            app_sentry_gimbal_control();
        }
    #else
        // TODO: app_gimbal_control();
        // TODO: app_shoot_control();
    #endif

#endif

    s_ctrl_divider++;

    /* ── 3. 车组分支 (预留) ── */
#if CURRENT_ROBOT == ROBOT_HERO
    // TODO
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    // TODO
#elif CURRENT_ROBOT == ROBOT_SENTRY
    /* 哨兵特殊逻辑已在上面处理 */
#endif
}
