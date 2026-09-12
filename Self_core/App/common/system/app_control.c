/**
 * @file    app_control.c
 * @brief   1kHz 定时控制与主循环后台任务调度
 * @note    控制由 TIM14 固定周期调用；通信解析等非实时任务在主循环执行。
 */
#include "app_control.h"
#include "drv_referee.h"
#include "app_gimbal_comm.h"
#include "project_cfg.h"

#include "app_sentry_chassis.h"
#include "app_sentry_gimbal.h"
// ─── 私有变量 ─────────────────────────
#if CURRENT_BOARD == BOARD_GIMBAL && CURRENT_ROBOT == ROBOT_SENTRY
static uint8_t s_control_divider;
#endif

#if CURRENT_BOARD == BOARD_CHASSIS
static void app_control_chassis_robot_1khz(void)
{
#if CURRENT_ROBOT == ROBOT_HERO
    /* 预留：App/hero/chassis/app_hero_chassis_ctrl() */
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    /* 预留：App/infantry/chassis/app_infantry_chassis_ctrl() */
#elif CURRENT_ROBOT == ROBOT_SENTRY
    app_sentry_chassis_ctrl();
#endif
}

#elif CURRENT_BOARD == BOARD_GIMBAL
static void app_control_gimbal_robot_1khz(void)
{
#if CURRENT_ROBOT == ROBOT_HERO
    /* 预留：App/hero/gimbal/app_hero_gimbal_ctrl() */
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    /* 预留：App/infantry/gimbal/app_infantry_gimbal_ctrl() */
#elif CURRENT_ROBOT == ROBOT_SENTRY
    app_gimbal_ahrs_update(0.001f);
    app_sentry_gimbal_ctrl_1khz();
    if (++s_control_divider >= 5U) {
        s_control_divider = 0;
        app_sentry_launcher_ctrl_200hz();
    }
#endif
}
#endif

// ─── 公有接口 ─────────────────────────
void app_control_process(void)
{
#if CURRENT_BOARD == BOARD_CHASSIS
    drv_referee_port_process();
#elif CURRENT_BOARD == BOARD_GIMBAL
    app_gimbal_comm_process();
#endif
}

void app_control_1khz(void)
{
#if CURRENT_BOARD == BOARD_CHASSIS
    app_control_chassis_robot_1khz();
#elif CURRENT_BOARD == BOARD_GIMBAL
    app_control_gimbal_robot_1khz();
#endif
}
