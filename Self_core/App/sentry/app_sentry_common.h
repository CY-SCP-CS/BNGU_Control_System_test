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

/* 速度斜坡：平移按二维速度增量模长限制，旋转独立限制，1 kHz 更新。
 * 普通减速、换向同样经过斜坡；急停立即清零。加速度须大于 0。
 * 以下为实车试调起点，减小更柔和、增大更跟手。
 */
#ifndef SENTRY_CHASSIS_RAMP_ENABLE
#define SENTRY_CHASSIS_RAMP_ENABLE 1
#endif
#define SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 3000.0f // 3 m/s^2。
#define SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 9.0f  // rad/s^2。

/* 舵轮编码器标定参考姿态：左轮 +45°、右轮 -45°。 */
#define SENTRY_SWERVE_0_OFFSET 5760U // 左轮处于 +45° 参考姿态时的编码器值。
#define SENTRY_SWERVE_1_OFFSET 4837U // 右轮处于 -45° 参考姿态时的编码器值。
#define SENTRY_SWERVE_L_CAL_ANGLE (LIB_PI * 0.25f)  // 左轮参考姿态相对车体 x 轴的角度。
#define SENTRY_SWERVE_R_CAL_ANGLE (-LIB_PI * 0.25f) // 右轮参考姿态相对车体 x 轴的角度。
#define SENTRY_STEER_L_DIRECTION  1.0f  // 左舵机逻辑正方向：编码器角度和输出电流共用。
#define SENTRY_STEER_R_DIRECTION 1.0f  // 右舵机逻辑正方向：编码器角度和输出电流共用。

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
#define SENTRY_GIMBAL_PITCH_MIN         (-35.0f * LIB_PI / 180.0f) // 对地 pitch 目标下限，rad。
#define SENTRY_GIMBAL_PITCH_MAX         (25.0f * LIB_PI / 180.0f)  // 对地 pitch 目标上限，rad。
#define SENTRY_GIMBAL_PITCH_JOINT_MIN   (-35.0f * LIB_PI / 180.0f) // 相对编码器机械零位的关节下限，rad。
#define SENTRY_GIMBAL_PITCH_JOINT_MAX   (25.0f * LIB_PI / 180.0f)  // 相对编码器机械零位的关节上限，rad。
#define SENTRY_GIMBAL_PITCH_SOFT_ZONE   (5.0f * LIB_PI / 180.0f)   // pitch 软限位减速区，rad。
#define SENTRY_GIMBAL_YAW_TAR_SPEED_MAX 5.0f // yaw 最大目标角速度，rad/s。
#define SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX 5.0f // pitch 最大目标角速度，rad/s。

/* 云台使能前等待有效反馈；运行中丢失反馈时停止输出并重新等待。 */
#define SENTRY_GIMBAL_FEEDBACK_TIMEOUT_MS 20U // IMU / 三个云台电机反馈的失效时间。
#define SENTRY_GIMBAL_STARTUP_WAIT_MS     20U // 反馈持续有效后的启动等待时间。

/* CAN 模式下的最低优先级自动巡扫，角度和角速度单位均为 rad、rad/s。 */
#define SENTRY_GIMBAL_SCAN_ENABLE       1U
#define SENTRY_GIMBAL_SCAN_YAW_SPEED    (20.0f * LIB_PI / 180.0f) // 持续左转。
#define SENTRY_GIMBAL_SCAN_PITCH_SPEED  (6.0f * LIB_PI / 180.0f)
#define SENTRY_GIMBAL_SCAN_PITCH_MIN    SENTRY_GIMBAL_PITCH_MIN
#define SENTRY_GIMBAL_SCAN_PITCH_MAX    SENTRY_GIMBAL_PITCH_MAX
#define SENTRY_GIMBAL_CAN_POSE_TIMEOUT_MS 200U

/* 底盘 CAN2 电机反馈和电流帧。 */
#define SENTRY_CAN_CHASSIS_DRIVE_R  0x201U // 右驱动 M3508 反馈。
#define SENTRY_CAN_CHASSIS_DRIVE_L  0x202U // 左驱动 M3508 反馈。
#define SENTRY_CAN_CHASSIS_STEER_L  0x205U // 左转向 GM6020 反馈。1
#define SENTRY_CAN_CHASSIS_STEER_R  0x206U // 右转向 GM6020 反馈。2
#define SENTRY_CAN_CHASSIS_POWER    0x212U // 功率计反馈。
#define SENTRY_CAN_CHASSIS_TX_DRIVE 0x200U // [右驱动、左驱动] 电流帧。
#define SENTRY_CAN_CHASSIS_TX_STEER 0x1FFU // [左转向、右转向] 电流帧。

/* 云台 CAN2 电机反馈和电流帧。 */
#define SENTRY_CAN_GIMBAL_F1        0x201U // 左摩擦轮 M3508 反馈。
#define SENTRY_CAN_GIMBAL_D1        0x202U // 拨弹轮 M2006 反馈。
#define SENTRY_CAN_GIMBAL_F2        0x203U // 右摩擦轮 M3508 反馈。
#define SENTRY_CAN_GIMBAL_L_YAW     0x205U // 大 yaw GM6020 反馈。
#define SENTRY_CAN_GIMBAL_S_YAW     0x206U // 小 yaw GM6020 反馈。
#define SENTRY_CAN_GIMBAL_PITCH     0x207U // pitch GM6020 反馈。
#define SENTRY_CAN_GIMBAL_TX_YAW    0x1FEU // [大 yaw、小 yaw、pitch] 电流帧。
#define SENTRY_CAN_GIMBAL_TX_LAUNCH 0x200U // [左摩擦、拨弹、右摩擦] 电流帧。

#define SENTRY_DISC_TAR_SPEED     400.0f // 拨弹轮目标角速度，rad/s。
#define SENTRY_FRICTION_TAR_SPEED 600.0f // 摩擦轮目标角速度，rad/s。

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
