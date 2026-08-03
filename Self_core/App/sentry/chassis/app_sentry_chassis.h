/**
 * @file    app_sentry_chassis.h
 * @brief   Sentry 底盘控制 — 麦轮运动学 + 级联PID + GM6020 yaw跟随
 * @note    Ported from Chassis_RD1/libs/CONTROL.h + DATA.h + PID.h
 *          控制频率 1kHz (由 app_control_1khz 调度)
 *          适配 BNGU 框架 lib_pid / drv_motor / drv_dbus / drv_imu
 */
#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "app_sentry_common.h"

/* ════════════════════════════════════════════════════
 * 数据结构
 * ════════════════════════════════════════════════════ */

/** 底盘速度 (世界坐标系) */
typedef struct {
    float v_x;    /**< x方向速度 (mm/s, 前为正)       */
    float v_y;    /**< y方向速度 (mm/s, 左为正)       */
    float v_w;    /**< 角速度 (rad/s, 逆时针为正)     */
} app_sentry_chassis_speed_t;

/** 力/力矩极坐标表示 */
typedef struct {
    float force;       /**< 合成力幅值                  */
    float angle;       /**< 力方向角 (rad)              */
    float torque;      /**< 绕z轴力矩                   */
} app_sentry_chassis_force_t;

/* ════════════════════════════════════════════════════
 * 接口
 * ════════════════════════════════════════════════════ */

void app_sentry_chassis_init(void);

/**
 * @brief  底盘控制主函数 (每 1ms 调用)
 * @note   从 Chassis_RD1 移植, 控制流水线:
 *         1. DBUS→目标速度 (世界坐标系)
 *         2. 选通反馈速度 (M3508编码器 + GM6020/IMU yaw)
 *         3. 级联PID: 速度环→力/力矩→轮电流
 *         4. CAN发送电流帧
 */
void app_sentry_chassis_control(void);

/**
 * @brief  解析电机反馈数据 (CAN回调中调用)
 * @param  std_id  CAN标准ID
 * @param  data   8字节数据
 * @param  len    数据长度
 */
void app_sentry_chassis_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

/**
 * @brief  获取底盘当前速度估算
 */
const app_sentry_chassis_speed_t *app_sentry_chassis_get_speed(void);

/**
 * @brief  获取底盘目标速度
 */
const app_sentry_chassis_speed_t *app_sentry_chassis_get_target(void);

#endif /* APP_SENTRY_CHASSIS_H */
