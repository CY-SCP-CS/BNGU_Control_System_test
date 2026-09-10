/**
 * @file    app_sentry_chassis.h
 * @brief   Sentry 哨兵底盘 — 双舵轮 (swervedrive) 独立转向+驱动
 * @note    Ported from Steering_wheel_Chasssis_test
 *          每轮: M3508 转向角度PID + M3508 驱动速度FF-PID
 *          底盘级: 3×增量PID → 极坐标力/力矩 → 每轮分配
 *          yaw: CAN1 0x124 云台BMI088
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
    float angle;     /**< 转向角 (deg)               */
    float speed;     /**< 驱动速度 (RPM)              */
    int8_t rev;      /**< 反转标志 (-1/1, 转向>90°优化) */
} app_sentry_swerve_wheel_t;

typedef struct {
    app_sentry_swerve_wheel_t wheel[2];   /**< [0]=左轮, [1]=右轮          */
    app_sentry_chassis_speed_t speed;     /**< 车体速度 (mm/s, rad/s)      */
    float yaw_deg;                        /**< 云台绝对 yaw 遥测 (CAN1 0x124) */
    float omega_z;                        /**< 估算角速度 (rad/s)          */
    float power_w;                        /**< 功率计实测功率，离线时为 0 (W) */
    float power_limit_w;                  /**< 裁判系统功率上限 (W)         */
    float power_target_w;                 /**< 指令比例处理后的目标功率 (W) */
    float power_scale;                    /**< 最终电流缩放系数             */
    float battery_voltage_v;              /**< 功率计电池电压 (V)           */
    float battery_current_a;              /**< 功率计电池电流 (A)           */
    uint8_t robot_level;                  /**< 裁判系统机器人等级           */
    uint8_t is_power_measured;             /**< 1=power_w 来自功率计          */
    uint8_t is_referee_valid;              /**< 1=裁判机器人状态未超时       */
    uint8_t is_chassis_output_enabled;     /**< 1=裁判系统允许底盘供电       */
} app_sentry_chassis_state_t;

/* ════════════════════════════════════════════════════
 * 接口
 * ════════════════════════════════════════════════════ */

/** @brief 初始化双舵轮控制状态并注册电机反馈。 */
void app_chassis_init(void);

/**
 * @brief  底盘控制主函数 (1kHz)
 * @note   控制流水线:
 *         1. 读 CAN1 0x111 小电脑指令 或 DBUS
 *         2. 在车体坐标系解算；0x124 云台 yaw 仅作遥测
 *         3. 逆运动学 → 每轮角度+RPM
 *         4. 正运动学 → 估算车体速度
 *         5. 底盘PID → 极坐标力/力矩
 *         6. 力分配 → 每轮驱动前馈
 *         7. 驱动FF-PID + 转向角度PID → 电流
 *         8. 功率限制 → CAN2发送
 */
void app_chassis_ctrl(void);

/**
 * @brief  CAN2 电机反馈回调
 */
void app_chassis_on_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

/**
 * @brief  获取底盘状态 (供板间通信)
 */
const app_sentry_chassis_state_t *app_chassis_get_state(void);

#endif /* APP_SENTRY_CHASSIS_H */
