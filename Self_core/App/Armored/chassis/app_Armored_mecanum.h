//
// Created by 33140 on 2026/10/26.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H

#include "Drv/Drv_motor/drv_motor.h"
#include "Lib/Lib_pid/lib_pid.h"

typedef struct {
    drv_motor_data_t motor_data;
    lib_pid_t pid;
    uint16_t feedback_id;
    int16_t send_current;
    float tar_omega_speed;
}M3508_wheel_drive;

/**
 * @brief 将麦轮电机初始化并且注册麦轮电机can回调
 */
void mecnum_init(void);

/**
 * @brief 执行一次麦轮底盘控制：坐标转换、逆运动学解算、PID 计算及 CAN2 电流发送。
 * @param vx_world 世界坐标系 x 方向目标线速度，单位 m/s。
 * @param vy_world 世界坐标系 y 方向目标线速度，x方向逆时针转90°方向为正 ，单位 m/s。
 * @param wz 底盘绕竖直 z 轴的目标角速度，单位 rad/s，从上往下看逆时针为正；
 * @param yaw 底盘当前偏航角，单位度；世界坐标系 +x 方向为零，逆时针为正。
 * @param pitch 底盘当前俯仰角，单位度，用于坡度前馈补偿。
 * @return CAN 发送状态：0 表示已接受发送，1 表示提交失败；不代表电调已收到或输入有效。
 * @note 调用前须启动 CAN2 并调用 mecnum_init()；当前 PID 按 4 ms 控制周期计算。
 *       任一输入为 NaN 或无穷大时，重置四路 PID、清零目标转速和发送电流，
 *       并尝试发送零电流帧，返回该次发送状态。
 */
uint8_t mecanum_ctrl(float vx_world, float vy_world, float wz,float yaw, float pitch);

/**
 * @brief 清零四路目标转速和电流、重置 PID，并尝试发送零电流帧。
 * @return 0 表示发送已接受，1 表示未初始化或发送失败。
 */
uint8_t mecanum_stop(void);

#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_MECANUM_C_H
