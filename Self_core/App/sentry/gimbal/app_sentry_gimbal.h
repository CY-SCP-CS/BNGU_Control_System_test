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
    float   yaw_tar_speed, pitch_tar_speed;//云台速度控制模式下的目标角速度
    float   yaw_tar_angle, pitch_tar_angle;//云台绝对角度控制模式下的目标角度
    float   yaw_tar_det, pitch_tar_det;//云台增量角度控制模式下的目标角度增量
    uint8_t fire;//发射机构控制模式
} app_sentry_gimbal_cmd_t;//云台控制命令结构体

typedef struct {
    float k_virt, b_virt;//虚拟弹簧/阻尼
    float k_ff;//底盘前馈
    float soft_limit_k, small_limit;//回正补偿与最大小yaw限位
    float max_out_s, max_out_l;//大/小yaw最大电流输出
    float inertia_small, inertia_big;//
    float max_accel, max_curr_step;
    float k_tracking, b_tracking, max_vel;
} app_sentry_vmc_config_t;//双yaw VMC 参数结构体


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
