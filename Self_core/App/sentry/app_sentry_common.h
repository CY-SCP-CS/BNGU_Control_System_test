/**
 * @file    app_sentry_common.h
 * @brief   哨兵共享配置：机构参数、CAN 标识和控制模式
 * @note    CAN1 用于底盘、云台和上位机之间的板间通信；CAN2 用于各板本地电机控制。
 */
#ifndef APP_SENTRY_COMMON_H
#define APP_SENTRY_COMMON_H

#include "lib_typedef.h"
#include "lib_math.h"

/* 底盘机械参数，长度单位 mm。 */
#define SENTRY_WHEEL_RADIUS     55.0f  // 舵轮半径。
#define SENTRY_WHEEL_HALF_TRACK 225.0f // 轮子在左右方向相对车体中心的距离。
#define SENTRY_WHEEL_HALF_BASE  225.0f // 轮子在前后方向相对车体中心的距离。

/* 底盘目标上限。 */
#define SENTRY_CHASSIS_VX_TAR_SPEED_MAX 3000.0f // x 方向最大速度，mm/s。
#define SENTRY_CHASSIS_VY_TAR_SPEED_MAX 3000.0f // y 方向最大速度，mm/s。
#define SENTRY_CHASSIS_VW_TAR_SPEED_MAX 10.0f   // 最大角速度，rad/s。
#define SENTRY_CHASSIS_TAR_POWER        100.0f  // 功率计闭环目标功率，W。
#define SENTRY_CHASSIS_ESTOP            (-32768) // 0x111 的 vx 急停保留值。

/* 舵轮编码器机械零位。 */
#define SENTRY_SWERVE_0_OFFSET 1859U // 左轮平行于车体 x 轴时的编码器值。
#define SENTRY_SWERVE_1_OFFSET 6790U // 右轮平行于车体 x 轴时的编码器值。

/** 车体坐标系速度，x/y 单位 mm/s，w 单位 rad/s。 */
typedef struct {
    float vx_speed; // x 方向速度，向前为正。
    float vy_speed; // y 方向速度，向左为正。
    float vw_speed; // 绕 z 轴角速度，逆时针为正。
} app_sentry_chassis_speed_t;

/* 云台电机编码器机械零位和正方向。 */
#define SENTRY_GIMBAL_L_YAW_ZERO 3463U // 大 yaw 机械零位编码器值。
#define SENTRY_GIMBAL_S_YAW_ZERO 7454U // 小 yaw 相对大 yaw 的机械零位编码器值。
#define SENTRY_GIMBAL_PITCH_ZERO 5487U // pitch 机械零位编码器值。
#define SENTRY_GIMBAL_L_YAW_DIR  1.0f  // 大 yaw 正方向相对底盘坐标的符号。
#define SENTRY_GIMBAL_S_YAW_DIR  1.0f  // 小 yaw 正方向相对大 yaw 的符号。

/* 云台机械与运动限制。 */
#define SENTRY_GIMBAL_S_YAW_MECH_LIMIT  (50.0f * LIB_PI / 180.0f) // 小 yaw 机械极限，rad。
#define SENTRY_GIMBAL_S_YAW_LIMIT       (SENTRY_GIMBAL_S_YAW_MECH_LIMIT - 5.0f * LIB_PI / 180.0f) // 小 yaw 软件工作边界，rad。
#define SENTRY_GIMBAL_S_YAW_SOFT_ZONE   (10.0f * LIB_PI / 180.0f) // 小 yaw 进入软件边界前的减速区，rad。
#define SENTRY_GIMBAL_S_YAW_BRAKE_GAIN  1500.0f                   // 小 yaw 边界速度制动，电流/(rad/s)。
#define SENTRY_GIMBAL_PITCH_MIN         (-35.0f * LIB_PI / 180.0f) // pitch 下限，rad。
#define SENTRY_GIMBAL_PITCH_MAX         (25.0f * LIB_PI / 180.0f)  // pitch 上限，rad。
#define SENTRY_GIMBAL_PITCH_SOFT_ZONE   (5.0f * LIB_PI / 180.0f)   // pitch 软限位减速区，rad。
#define SENTRY_GIMBAL_YAW_TAR_SPEED_MAX 5.0f // yaw 最大目标角速度，rad/s。
#define SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX 5.0f // pitch 最大目标角速度，rad/s。

/* 底盘 CAN2 电机反馈和电流帧。 */
#define SENTRY_CAN_CHASSIS_DRIVE_R  0x201U // 右驱动 M3508 反馈。
#define SENTRY_CAN_CHASSIS_DRIVE_L  0x202U // 左驱动 M3508 反馈。
#define SENTRY_CAN_CHASSIS_STEER_L  0x205U // 左转向 GM6020 反馈。1
#define SENTRY_CAN_CHASSIS_STEER_R  0x206U // 右转向 GM6020 反馈。2
#define SENTRY_CAN_CHASSIS_POWER    0x212U // 功率计反馈。
#define SENTRY_CAN_CHASSIS_TX_DRIVE 0x200U // [右驱动、左驱动] 电流帧。
#define SENTRY_CAN_CHASSIS_TX_STEER 0x1FEU // [左转向、右转向] 电流帧。

/* 云台 CAN2 电机反馈和电流帧。 */
#define SENTRY_CAN_GIMBAL_F1        0x201U // 左摩擦轮 M3508 反馈。
#define SENTRY_CAN_GIMBAL_D1        0x202U // 拨弹轮 M2006 反馈。
#define SENTRY_CAN_GIMBAL_F2        0x203U // 右摩擦轮 M3508 反馈。
#define SENTRY_CAN_GIMBAL_L_YAW     0x205U // 大 yaw GM6020 反馈。
#define SENTRY_CAN_GIMBAL_S_YAW     0x206U // 小 yaw GM6020 反馈。
#define SENTRY_CAN_GIMBAL_PITCH     0x207U // pitch GM6020 反馈。
#define SENTRY_CAN_GIMBAL_TX_YAW    0x1FEU // [大 yaw、小 yaw、pitch] 电流帧。
#define SENTRY_CAN_GIMBAL_TX_LAUNCH 0x200U // [左摩擦、拨弹、右摩擦] 电流帧。

#define SENTRY_DISC_TAR_SPEED     10.0f // 拨弹轮目标角速度，rad/s。
#define SENTRY_FRICTION_TAR_SPEED 10.0f // 摩擦轮目标角速度，rad/s。

/** 云台接收指令的控制方式。 */
typedef enum {
    APP_SENTRY_GIMBAL_MODE_INC_ANGLE = 1U, // 增量角度模式。
    APP_SENTRY_GIMBAL_MODE_SPEED     = 2U, // 速度模式。
    APP_SENTRY_GIMBAL_MODE_ABS_ANGLE = 3U  // 绝对角度模式。
} app_sentry_gimbal_mode_t;

/** 发射机构控制方式。 */
typedef enum {
    APP_SENTRY_FIRE_OFF     = 1U, // 摩擦轮和拨弹轮均停止。
    APP_SENTRY_FIRE_ON      = 2U, // 摩擦轮和拨弹轮正转。
    APP_SENTRY_FIRE_FIR     = 3U, // 摩擦轮转动，拨弹轮停止。
    APP_SENTRY_FIRE_REVERSE = 4U  // 摩擦轮转动，拨弹轮反转。
} app_sentry_fire_mode_t;

#endif