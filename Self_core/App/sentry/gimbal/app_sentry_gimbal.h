/**
 * @file    app_sentry_gimbal.h
 * @brief   哨兵云台控制：双 yaw VMC、pitch 重力补偿和发射机构控制
 * @note    CAN2 连接本板电机，CAN1 交换板间指令与状态。
 */
#ifndef APP_SENTRY_GIMBAL_H
#define APP_SENTRY_GIMBAL_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "drv_imu.h"
#include "app_sentry_common.h"

/** 云台控制命令。 */
typedef struct {
    uint8_t mode;                    // app_sentry_gimbal_mode_t。
    float yaw_tar_speed;             // yaw 目标角速度，rad/s。
    float pitch_tar_speed;           // pitch 目标角速度，rad/s。
    float yaw_tar_angle;             // yaw 目标角度，rad。
    float pitch_tar_angle;           // pitch 目标角度，rad。
    float yaw_tar_angle_delta;       // yaw 目标角度增量，rad。
    float pitch_tar_angle_delta;     // pitch 目标角度增量，rad。
    uint8_t fire;                    // app_sentry_fire_mode_t。
} app_sentry_gimbal_cmd_t;

/**
 * @brief  初始化云台控制状态并注册电机 CAN 回调。
 * @param  imu IMU 驱动实例。
 */
void app_sentry_gimbal_init(drv_imu_t *imu);

/**
 * @brief  更新 IMU 的 AHRS。
 * @param  dt 本次更新周期，单位 s。
 */
void app_gimbal_ahrs_update(float dt);

/** @brief 执行云台 yaw、pitch 控制；调度频率为 1 kHz。 */
void app_sentry_gimbal_ctrl_1khz(void);

/** @brief 执行摩擦轮和拨弹轮控制；调度频率为 200 Hz。 */
void app_sentry_launcher_ctrl_200hz(void);

#endif