/**
 * @file    app_sentry_chassis.h
 * @brief   Sentry 哨兵底盘 — 双舵轮 (swervedrive) 独立转向+驱动
 * @note    Ported from Steering_wheel_Chasssis_test
 *          每轮: GM6020 转向角度PID + M3508 驱动速度闭环
 *          底盘级: 速度PID → 目标力/力矩 → 轮级电流前馈分配
 *          CAN2 = 板内电机, CAN1 = 板间数据
 */
#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "app_sentry_common.h"

/* ════════════════════════════════════════════════════
 * 舵轮状态
 * ════════════════════════════════════════════════════ */

typedef struct {
    float angle;//舵轮角度，单位rad
    float speed;//驱动电机轮速，单位mm/s
    int8_t rev;//反转标志 +-1
} app_sentry_swerve_wheel_t;//轮组数据结构体

typedef struct {
    app_sentry_swerve_wheel_t wheel[2];   /**< [0]=左轮, [1]=右轮          */
    app_sentry_chassis_speed_t speed;     /**< 车体速度 (mm/s, rad/s)      */
    float omega_z;                        /**< 估算角速度 (rad/s)          */
    float power_w;                        /**< 功率计滤波后的实测功率 (W) */
    float power_limit;                  /**< 裁判系统功率上限 (W)         */
    float power_target;                 /**< 功率控制目标功率 (W) */
    float power_scale;                    /**< 最终电流缩放系数             */
    float battery_v;              /**< 功率计电池电压 (V)           */
    float battery_curr;              /**< 功率计电池电流 (A)           */
    uint8_t robot_level;                  /**< 裁判系统机器人等级           */
    uint8_t is_chassis_output_enabled;     /**< 1=裁判系统允许底盘供电       */
} app_sentry_chassis_state_t;//车体状态结构体

/* ════════════════════════════════════════════════════
 * 接口
 * ════════════════════════════════════════════════════ */

/** @brief 初始化双舵轮控制状态并注册电机反馈。 */
void app_sentry_chassis_init(void);

/**
 * @brief  底盘控制主函数 (1kHz)
 */
void app_sentry_chassis_ctrl(void);

/**
 * @brief  获取底盘状态 (供板间通信)
 */
const app_sentry_chassis_state_t *app_chassis_get_state(void);

#endif /* APP_SENTRY_CHASSIS_H */
