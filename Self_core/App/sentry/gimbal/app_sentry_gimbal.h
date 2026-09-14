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


typedef struct {
    uint8_t mode;//云台控制模式
    float   yaw_tar_speed, pitch_tar_speed;//yaw/pitch 目标角速度，单位 rad/s
    float   yaw_tar_angle, pitch_tar_angle;//yaw/pitch 目标角度，单位 rad
    float   yaw_tar_angle_delta, pitch_tar_angle_delta;//yaw/pitch 目标角度增量，单位 rad
    uint8_t fire;//发射机构控制模式
} app_sentry_gimbal_cmd_t;//云台控制命令结构体



/**
 * @brief  初始化云台控制
 * @param  imu   BMI088 IMU 实例指针 (来自 app_init.c)
 */
void app_sentry_gimbal_init(drv_imu_t *imu);

/**
 * @brief  BMI088 AHRS 更新（每 1 ms / 1 kHz 调用）
 * @param  dt    上次调用间隔时间，单位 s，目前使用 0.001 s
 */
void app_gimbal_ahrs_update(float dt);

/**
 * @brief  云台姿态控制（大 yaw、小 yaw、pitch；每 1 ms / 1 kHz 调用）
 */
void app_sentry_gimbal_ctrl_1khz(void);

/** @brief 发射机构控制（摩擦轮、拨弹轮；每 5 ms / 200 Hz 调用） */
void app_sentry_launcher_ctrl_200hz(void);

#endif /* APP_SENTRY_GIMBAL_H */
