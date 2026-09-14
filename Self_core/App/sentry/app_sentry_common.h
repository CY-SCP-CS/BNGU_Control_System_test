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


//底盘机械参数
#define SENTRY_WHEEL_RADIUS       75.0f//舵轮半径
#define SENTRY_WHEEL_HALF_TRACK   250.0f//轮子在左右方向离中心 250 mm
#define SENTRY_WHEEL_HALF_BASE    250.0f//轮子在前后方向离中心 250 mm

//底盘运动上限
#define SENTRY_CHASSIS_VX_TAR_SPEED_MAX          3000.0f//底盘x方向最大速度 mm/s
#define SENTRY_CHASSIS_VY_TAR_SPEED_MAX          3000.0f//底盘y方向最大速度 mm/s
#define SENTRY_CHASSIS_VW_TAR_SPEED_MAX        10.0f//底盘最大角速度 rad/s
#define SENTRY_CHASSIS_TAR_POWER 100.0f//功率计闭环目标功率 W

//底盘电机标定
#define SENTRY_SWERVE_0_OFFSET       2735//左轮编码器平行
#define SENTRY_SWERVE_1_OFFSET       6819//右轮编码器平行//这两个还没标定

//底盘CAN1速度指令编码
#define SENTRY_CHASSIS_ESTOP       (-32768)//0x111 中标记底盘急停的 vx 保留值

typedef struct {
    float vx_speed;//x 方向速度，单位 mm/s，向前为正
    float vy_speed;//y 方向速度，单位 mm/s，向左为正
    float vw_speed;//绕 z 轴角速度，单位 rad/s，逆时针为正
} app_sentry_chassis_speed_t;//底盘速度结构体



//云台电机标定
#define SENTRY_GIMBAL_L_YAW_ZERO   2000U//大yaw机械零位对应的编码器值
#define SENTRY_GIMBAL_S_YAW_ZERO   1000//小yaw机械零位对应的编码器值
#define SENTRY_GIMBAL_L_YAW_DIR      1.0f//大yaw编码器角度正方向相对底盘的符号
#define SENTRY_GIMBAL_S_YAW_DIR      1.0f//小yaw编码器角度正方向相对大yaw的符号
#define SENTRY_GIMBAL_PITCH_ZERO    5509 // pitch电机编码器零点//待标定
//云台电限位
#define SENTRY_GIMBAL_S_YAW_LIMIT   (50.0f * LIB_PI / 180.0f)//小yaw相对大yaw的最大偏移角度
#define SENTRY_GIMBAL_PITCH_MIN         (-20.0f * LIB_PI / 180.0f)//pitch最小角度
#define SENTRY_GIMBAL_PITCH_MAX         (40.0f * LIB_PI / 180.0f)//pitch最大角度

//云台运动上限
#define SENTRY_GIMBAL_YAW_TAR_SPEED_MAX  5//云台 yaw 最大角速度 rad/s
#define SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX 5//云台 pitch 最大角速度 rad/s//待定

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
#define SENTRY_CAN_GIMBAL_F1     0x201//左摩擦轮 M3508 反馈
#define SENTRY_CAN_GIMBAL_D1     0x202//拨弹轮 M2006 反馈
#define SENTRY_CAN_GIMBAL_F2     0x203//右摩擦轮 M3508 反馈
#define SENTRY_CAN_GIMBAL_L_YAW     0x205//大yaw GM6020 反馈
#define SENTRY_CAN_GIMBAL_S_YAW     0x206//小yaw GM6020 反馈
#define SENTRY_CAN_GIMBAL_PITCH         0x207//pitch GM6020 反馈
//云台CAN ID TX
#define SENTRY_CAN_GIMBAL_TX_YAW        0x1FF//云台控制，发送大yaw、小yaw、pitch电流 [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0]
#define SENTRY_CAN_GIMBAL_TX_LAUNCH     0x200//发射机构控制，发送左摩擦、拨弹轮、右摩擦电流 [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0]
//云台电机目标角速度
#define SENTRY_DISC_TAR_SPEED        10//拨弹轮目标角速度，单位 rad/s
#define SENTRY_FRICTION_TAR_SPEED    10//摩擦轮目标角速度，单位 rad/s；待定



typedef enum {
    APP_SENTRY_GIMBAL_MODE_INC_ANGLE = 1,//增量角度模式
    APP_SENTRY_GIMBAL_MODE_SPEED    = 2,//速度模式
    APP_SENTRY_GIMBAL_MODE_ABS_ANGLE = 3//绝对角度模式
} app_sentry_gimbal_mode_t;// 云台控制模式：增量角度、速度、绝对角度

typedef enum {
    APP_SENTRY_FIRE_OFF = 1,//停止
    APP_SENTRY_FIRE_ON  = 2,//持续射击
    APP_SENTRY_FIRE_FIR = 3,//开摩擦轮（比赛常态）
    APP_SENTRY_FIRE_REVERSE = 4//摩擦轮开启，拨弹盘反转
} app_sentry_fire_mode_t;// 发射机构控制模式：停止、持续射击、开摩擦轮

#endif /* APP_SENTRY_COMMON_H */
