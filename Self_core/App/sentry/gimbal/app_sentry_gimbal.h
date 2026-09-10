/**
 * @file    app_sentry_gimbal.h
 * @brief   Sentry 云台控制 — 双yaw VMC + pitch重力补偿 + 发射控制
 * @note    Ported from gimbal_2yaw/app/src/gimbal_control.c
 *          CAN2 = 板内电机控制, CAN1 = 板间数据交换
 */
#ifndef APP_SENTRY_GIMBAL_H
#define APP_SENTRY_GIMBAL_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "drv_imu.h"
#include "app_sentry_common.h"

/* ════════════════════════════════════════════════════
 * 数据结构
 * ════════════════════════════════════════════════════ */

/** RC指令 */
typedef struct {
    uint8_t mode;
    float   yaw_speed, pitch_speed;
    float   yaw_angle, pitch_angle;
    float   yaw_inc, pitch_inc;
    uint8_t fire;
} app_sentry_gimbal_cmd_t;

/** VMC 配置 */
typedef struct {
    float k_virt, b_virt;
    float k_ff;
    float soft_limit_k, small_limit;
    float max_out_s, max_out_l;
    float inertia_small, inertia_big;
    float max_accel, max_curr_step;
    float k_tracking, b_tracking, max_vel;
} app_sentry_vmc_config_t;

/* ════════════════════════════════════════════════════
 * 接口
 * ════════════════════════════════════════════════════ */

/**
 * @brief  初始化云台控制
 * @param  imu   BMI088 IMU 实例指针 (来自 app_init.c)
 */
void app_gimbal_init(drv_imu_t *imu);

/**
 * @brief  BMI088 AHRS 更新 (每 1ms/1kHz 调用)
 * @note   必须与 gimbal_control() 在同一 1kHz 上下文中调用
 *         gimbal_control() 在 200Hz 执行 yaw/pitch/launch
 *         AHRS 在 1kHz 执行
 */
void app_gimbal_ahrs_update(float dt);

/**
 * @brief  云台控制主函数 (200Hz)
 */
void app_gimbal_ctrl(void);

/**
 * @brief  解析电机反馈 (CAN2 回调)
 */
void app_gimbal_on_motor_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len);

/**
 * @brief  获取当前 IMU 融合角度
 */
void app_gimbal_get_angles(float *yaw, float *pitch);

#endif /* APP_SENTRY_GIMBAL_H */
