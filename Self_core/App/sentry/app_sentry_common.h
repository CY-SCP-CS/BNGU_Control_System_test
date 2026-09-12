/**
 * @file    app_sentry_common.h
 * @brief   Sentry 哨兵共享配置 — 机构参数、CAN ID、模式定义
 * @note    底盘板和云台板共用参数
 *          CAN1 = 板间通信 (底盘↔云台↔小电脑)
 *          CAN2 = 板内电机控制 (各板独立, ID可复用)
 * @ref     Chassis_RD1 + gimbal_2yaw
 */
#ifndef APP_SENTRY_COMMON_H
#define APP_SENTRY_COMMON_H

#include "lib_typedef.h"
#include "lib_math.h"


//底盘参数
#define SENTRY_WHEEL_RADIUS_MM       75.0f//舵轮半径
#define SENTRY_WHEEL_HALF_TRACK_MM   250.0f//轮子在左右方向离中心 250 mm
#define SENTRY_WHEEL_HALF_BASE_MM    250.0f//轮子在前后方向离中心 250 mm
//超时
#define SENTRY_MOTOR_TIMEOUT_MS     100U//电机超时
#define SENTRY_IMU_TIMEOUT_MS         5U//IMU超时
//底盘电机参数
#define SENTRY_REDUCTION_RATIO       19.0f//3508减速比
//底盘电机标定
#define SENTRY_SWERVE_0_OFFSET       2735//左轮编码器平行
#define SENTRY_SWERVE_1_OFFSET       6819//右轮编码器平行//这两个还没标定
//运动速度上限
#define SENTRY_MAX_LINEAR_SPEED      3000.0f//最大线速度 mm/s
#define SENTRY_MAX_OMEGA             10.0f//最大角速度 rad/s

typedef struct {
    float v_x;//x方向速度，mm/s, 向前为正
    float v_y;//y方向速度，mm/s, 向左为正
    float v_w;//绕z轴角速度，rad/s, 逆时针为正
} app_sentry_chassis_speed_t;//底盘速度结构体



//云台电机标定
#define SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO   1000//小yaw电机编码器中心零点//待标定
#define SENTRY_GIMBAL_PITCH_ENCODER_ZERO    5509 // pitch电机编码器零点//待标定
//云台电限位
#define SENTRY_GIMBAL_SMALL_YAW_LIMIT_RAD   (50.0f * LIB_MATH_PI / 180.0f)//小yaw相对大yaw的最大偏移角度
#define SENTRY_GIMBAL_PITCH_MIN_RAD         (-20.0f * LIB_MATH_PI / 180.0f)//pitch最小角度
#define SENTRY_GIMBAL_PITCH_MAX_RAD         (40.0f * LIB_MATH_PI / 180.0f)//pitch最大角度



//底盘CAN ID RX
#define SENTRY_CAN_CHASSIS_DRIVE_R   0x201//右驱动 M3508 反馈
#define SENTRY_CAN_CHASSIS_DRIVE_L   0x202//左驱动 M3508 反馈
#define SENTRY_CAN_CHASSIS_STEER_L   0x205//左转向 GM6020 反馈
#define SENTRY_CAN_CHASSIS_STEER_R   0x206//右转向 GM6020 反馈
#define SENTRY_CAN_CHASSIS_POWER     0x212//功率计反馈
//底盘CAN ID TX
#define SENTRY_CAN_CHASSIS_TX_DRIVE  0x200//驱动电流帧 TX [R_H,R_L, L_H,L_L，0，0，0，0]
#define SENTRY_CAN_CHASSIS_TX_STEER  0x1FF //转向电流帧 TX [L_H,L_L, R_H,R_L，0，0，0，0]



//云台CAN ID RX
#define SENTRY_CAN_GIMBAL_LAUNCH_F1     0x201//左摩擦轮 M3508 反馈
#define SENTRY_CAN_GIMBAL_LAUNCH_D1     0x202//拨弹轮 M2006 反馈
#define SENTRY_CAN_GIMBAL_LAUNCH_F2     0x203//右摩擦轮 M3508 反馈
#define SENTRY_CAN_GIMBAL_YAW_LARGE     0x205//大yaw GM6020 反馈
#define SENTRY_CAN_GIMBAL_YAW_SMALL     0x206//小yaw GM6020 反馈
#define SENTRY_CAN_GIMBAL_PITCH         0x207//pitch GM6020 反馈
//云台CAN ID TX
#define SENTRY_CAN_GIMBAL_TX_YAW        0x1FF//云台控制，发送大yaw、小yaw、pitch电流 [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0]
#define SENTRY_CAN_GIMBAL_TX_LAUNCH     0x200//发射机构控制，发送左摩擦、拨弹轮、右摩擦电流 [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0]
//云台电机目标转速
#define SENTRY_DISC_TARGET_RAD_S        (8100.0f * 2.0f * LIB_MATH_PI / 60.0f)//拨弹轮目标转速 rad/s
#define SENTRY_FRICTION_TARGET_RAD_S    (3000.0f * 2.0f * LIB_MATH_PI / 60.0f)//摩擦轮目标转速 rad/s



typedef enum {
    APP_SENTRY_GIMBAL_MODE_INC_ANGLE = 1,//增量角度模式
    APP_SENTRY_GIMBAL_MODE_SPEED    = 2,//速度模式
    APP_SENTRY_GIMBAL_MODE_ABS_ANGLE = 3//绝对角度模式
} app_sentry_gimbal_mode_t;// 云台控制模式：增量角度、速度、绝对角度

typedef enum {
    APP_SENTRY_FIRE_OFF = 1,//停止
    APP_SENTRY_FIRE_ON  = 2,//持续射击
    APP_SENTRY_FIRE_FIR = 3//开摩擦轮（比赛常态）
} app_sentry_fire_mode_t;// 发射机构控制模式：停止、持续射击、开摩擦轮

#endif /* APP_SENTRY_COMMON_H */
