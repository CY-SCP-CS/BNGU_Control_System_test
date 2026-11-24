/**
 * @file    app_Armored_ctrl.h
 * @brief   重装底盘上层控制接口。
 */
#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_CTRL_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_CTRL_H

#include "drv_imu.h"

/**
 * @brief 上层控制初始化入口，待实现。
 * @param imu 已完成基础初始化的底盘 IMU 实例。
 */
void app_armored_ctrl_init(drv_imu_t *imu);

/** @brief 每 1 ms 更新 IMU 姿态，控制调度待实现。 */
void app_armored_ctrl_1khz(void);

/** 临时调参变量，可在 Watch 中修改；四轮/四关节分别共用增益。 */
typedef struct {
    float wheel_kp;             //kp=42
    float wheel_ki;             //ki=16
    float wheel_kd;
    float wheel_max_iout;       // 麦轮积分输出限幅，电流指令单位   500
    float joint_kp;            //先试一下 kp=60 或 100 kd=3 用平滑过度目标速度效果都非常好
    float joint_kd;
    float vx;                  // 车体前向速度，m/s
    float vy;                  // 车体左向速度，m/s
    float wz;                  // 逆时针角速度，rad/s
    float joint_pos[4];        // FL、RL、FR、RR 目标角度，rad
} armored_ctrl_temp_t;

extern volatile armored_ctrl_temp_t armored_ctrl_temp;

/** 临时调参初始化，系统层启动 CAN2 后调用。imu 为已初始化的底盘 IMU，可传 NULL。 */
void app_armored_ctrl_init_temp(drv_imu_t *imu);

/** 每 1 ms 更新 IMU；关节与麦轮各每 4 ms 控制一次，麦轮晚 2 ms，随后发送 VOFA。 */
void app_armored_ctrl_1khz_temp(void);

/** 通过 VOFA CH0 发送功率计最近一次测得的电池侧功率，单位 W。 */
void app_armored_ctrl_vofa_send_temp(void);

#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_CTRL_H
