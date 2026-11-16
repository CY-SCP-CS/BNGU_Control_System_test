/**
 * @file    app_control.c
 * @brief   1kHz 定时控制与主循环后台任务调度
 * @note    控制由 TIM14 固定周期调用；裁判解析在主循环执行，CAN1 转发在 200Hz 执行。
 */
#include "app_control.h"
#include "drv_referee.h"
#include "drv_buzzer.h"
#include "drv_mag_port.h"
#include "app_gimbal_comm.h"
#include "project_cfg.h"

#include "app_sentry_chassis.h"
#include "app_sentry_gimbal.h"
#include "App/Navigation/navigation_ctrl.h"
// ─── 私有变量 ─────────────────────────
#if CURRENT_BOARD == BOARD_GIMBAL && CURRENT_ROBOT == ROBOT_SENTRY
static uint8_t s_control_divider;
#endif

#if CURRENT_BOARD ==  BOARD_CHASSIS && CURRENT_ROBOT == ROBOT_NAVIGATION
static uint8_t s_control_divider;
#endif

#if CURRENT_ROBOT == ROBOT_SENTRY
static uint8_t s_init_complete;
static uint8_t s_ready_announced;
static uint16_t s_ready_beep_ms;

/** 就绪时启动一次提示音，按 1 kHz 调度计时 500 ms。 */
static void app_control_ready_beep(void)
{
    if (!s_init_complete) {
        return;
    }
    if (!s_ready_announced) {
#if CURRENT_BOARD == BOARD_GIMBAL
        if (!app_sentry_gimbal_is_ready()) {
            return;
        }
#endif
        drv_buzzer_set_freq(2000U);
        drv_buzzer_on();
        s_ready_beep_ms = 500U;
        s_ready_announced = 1U;
    } else if (s_ready_beep_ms > 0U) {
        if (--s_ready_beep_ms == 0U) {
            drv_buzzer_off();
        }
    }
}
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
#elif CURRENT_ROBOT == ROBOT_NAVIGATION
    /* 预留：导航车底盘控制调度。 */
    navigation_ctrl_1khz(s_control_divider);
    if (++s_control_divider==250) s_control_divider = 0;
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
        app_gimbal_comm_process();
        app_sentry_gimbal_chassis_tx_200hz();
        app_sentry_gimbal_status_tx_200hz();
        app_sentry_launcher_ctrl_200hz();
    }
#if SENTRY_CAN_TEST_ENABLE
    app_gimbal_comm_test_tx(100U);
#endif
#elif CURRENT_ROBOT == ROBOT_NAVIGATION
    /* 预留：导航车云台控制调度。 */
#endif
    drv_mag_port_process();
}
#endif

// ─── 公有接口 ─────────────────────────
void app_control_init_complete(void)
{
#if CURRENT_ROBOT == ROBOT_SENTRY
    s_ready_announced = 0U;
    s_ready_beep_ms = 0U;
    s_init_complete = 1U;
#endif
}

void app_control_process(void)
{
#if CURRENT_BOARD == BOARD_CHASSIS
    drv_referee_port_process();
#endif
}

void app_control_1khz(void)
{
#if CURRENT_BOARD == BOARD_CHASSIS
    app_control_chassis_robot_1khz();
#elif CURRENT_BOARD == BOARD_GIMBAL
    app_control_gimbal_robot_1khz();
#endif
#if CURRENT_ROBOT == ROBOT_SENTRY
    app_control_ready_beep();
#endif
}
