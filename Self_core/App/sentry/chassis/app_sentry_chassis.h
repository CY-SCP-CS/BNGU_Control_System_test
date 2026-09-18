/**
 * @file    app_sentry_chassis.h
 * @brief   哨兵双舵轮底盘控制接口
 * @note    每轮由 GM6020 负责转向、M3508 负责驱动；CAN2 连接电机，CAN1 交换板间数据。
 */
#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "app_sentry_common.h"

/** 单个舵轮的转向和驱动状态。 */
typedef struct {
    float steer_angle; // 转向角度，单位 rad。
    float drive_speed; // 驱动线速度，单位 mm/s。
    int8_t drive_rev;  // 驱动反转标志，取值 +1 或 -1。
} app_sentry_swerve_wheel_t;

/** 对外发布的底盘运行状态。 */
typedef struct {
    app_sentry_swerve_wheel_t wheel_cur[2]; // 当前左右舵轮状态。
    app_sentry_chassis_speed_t cur_speed;   // 当前估算车体速度。
    float cur_power;                         // 当前实测功率，W。
    float tar_power;                         // 功率控制目标，W。
    float power_scale;                       // 最终电流缩放系数，范围 0~1。
    float battery_voltage;                   // 电池电压，V。
    float battery_current;                   // 电池电流，A。
} app_sentry_chassis_state_t;

/** @brief 初始化双舵轮控制状态并注册 CAN 电机反馈。 */
void app_sentry_chassis_init(void);

/** @brief 执行一次底盘控制循环；调度频率为 1 kHz。 */
void app_sentry_chassis_ctrl(void);

/**
 * @brief  获取最近一次发布的底盘状态。
 * @return 只读状态指针。
 */
const app_sentry_chassis_state_t *app_chassis_get_state(void);

#endif