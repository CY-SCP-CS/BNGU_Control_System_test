//
// Created by 33140 on 2026/11/3.
//

#ifndef BNGU_CONTROL_SYSTEM_CHASSIS_H
#define BNGU_CONTROL_SYSTEM_CHASSIS_H

#include "Drv/Drv_motor/drv_motor.h"
#include "Lib/Lib_pid/lib_pid.h"

typedef struct {
    drv_motor_data_t motor_data;
    lib_pid_t pid;
    uint16_t feedback_id;
    int16_t send_current;
    float tar_omega_speed;
} NAV_M3508_wheel_drive;

/** 初始化四轮 PID，并注册 CAN2 电机反馈回调；调用前须启动 CAN2。 */
void chassis_init(void);

/**
 * @brief 执行一次世界坐标系底盘控制：坐标转换、逆解算、PID 计算及 CAN2 电流发送。
 * @param vx_world 世界坐标系 x 方向目标线速度，单位 m/s。
 * @param vy_world 世界坐标系 y 方向目标线速度，+x 逆时针转 90° 为 +y，单位 m/s。
 * @param wz 底盘绕中心的目标角速度，俯视逆时针为正，单位 rad/s。
 * @param yaw 底盘偏航角，车头沿世界 +x 为零、逆时针为正，单位度。
 * @return 0 表示 CAN 已接受发送，1 表示未初始化或发送失败；不代表电调已收到。
 * @note 调用前须启动 CAN2 并调用 chassis_init()；每 5 ms 调用一次。
 *       任一输入为 NaN 或无穷大时调用 chassis_stop()，清零目标和电流并重置 PID。
 */
uint8_t chassis_ctrl_world(float vx_world, float vy_world, float wz, float yaw);

/**
 * @brief 执行一次车体坐标系底盘控制：逆解算、轮速限幅、PID 计算及 CAN2 电流发送。
 * @param vx_body 车体 x 方向目标线速度，向前为正，单位 m/s。
 * @param vy_body 车体 y 方向目标线速度，向左为正，单位 m/s。
 * @param wz 底盘绕中心的目标角速度，俯视逆时针为正，单位 rad/s。
 * @return 0 表示 CAN 已接受发送，1 表示未初始化或发送失败；不代表电调已收到。
 * @note 调用前须启动 CAN2 并调用 chassis_init()；每 5 ms 与世界坐标系接口二选一调用。
 *       任一输入为 NaN 或无穷大时调用 chassis_stop()，清零目标和电流并重置 PID。
 */
uint8_t chassis_ctrl_body(float vx_body, float vy_body, float wz);

/**
 * @brief 清零四轮目标转速和电流、重置 PID，并发送零电流帧。
 * @return 0 表示 CAN 已接受发送，1 表示未初始化或发送失败。
 */
uint8_t chassis_stop(void);

#endif // BNGU_CONTROL_SYSTEM_CHASSIS_H
