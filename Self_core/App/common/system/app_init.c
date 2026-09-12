/**
 * @file    app_init.c
 * @brief   系统初始化
 */
#include "app_init.h"

#include "project_cfg.h"

#include "bsp_cfg.h"
#include "bsp_can.h"

#include "drv_buzzer.h"
#include "drv_dbus.h"
#include "drv_imu.h"
#include "drv_led.h"
#include "drv_referee.h"
#include "drv_vofa.h"

#include "app_chassis_comm.h"
#include "app_gimbal_comm.h"
#include "app_sentry_chassis.h"
#include "app_sentry_gimbal.h"

#define APP_INIT_VOFA_CH_COUNT  8U


#if CURRENT_BOARD == BOARD_CHASSIS
static void app_init_chassis_robot(void)
{
    app_chassis_comm_init();
    drv_referee_port_init();

#if CURRENT_ROBOT == ROBOT_HERO
    /* 预留：App/hero/chassis/app_hero_chassis_init() */
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    /* 预留：App/infantry/chassis/app_infantry_chassis_init() */
#elif CURRENT_ROBOT == ROBOT_SENTRY
    app_sentry_chassis_init();
#endif
}
#elif CURRENT_BOARD == BOARD_GIMBAL
static void app_init_gimbal_robot(void)
{
    static drv_imu_t s_imu;

    drv_imu_port_init(&s_imu);
    app_gimbal_comm_init();

#if CURRENT_ROBOT == ROBOT_HERO
    /* 预留：App/hero/gimbal/app_hero_gimbal_init(&s_imu) */
#elif CURRENT_ROBOT == ROBOT_INFANTRY
    /* 预留：App/infantry/gimbal/app_infantry_gimbal_init(&s_imu) */
#elif CURRENT_ROBOT == ROBOT_SENTRY
    app_sentry_gimbal_init(&s_imu);
#endif
}
#endif

void app_init(void)
{
    /* Bsp */
    bsp_can_start(&hcan1, 0U, 14U);
    bsp_can_start(&hcan2, 14U, 14U);

    /* Drv */
    drv_led_port_init();
    drv_buzzer_port_init();
    drv_dbus_port_init();
    drv_vofa_port_init(APP_INIT_VOFA_CH_COUNT);

    /* 板型与车组的选择只在 system 层进行。 */
#if CURRENT_BOARD == BOARD_CHASSIS
    app_init_chassis_robot();
#elif CURRENT_BOARD == BOARD_GIMBAL
    app_init_gimbal_robot();
#endif /* CURRENT_BOARD */
}
