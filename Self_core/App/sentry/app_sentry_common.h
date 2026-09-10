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

/* ════════════════════════════════════════════════════
 * 舵轮机构参数 (from Steering_wheel_Chasssis_test)
 * ════════════════════════════════════════════════════ */

#define SENTRY_WHEEL_RADIUS_MM       75.0f
#define SENTRY_WHEEL_HALF_TRACK_MM   250.0f      /* 半轮距 L_HALF                    */
/* 沿用原代码的对角轮位置：左(+half_base,+half_track)，右(-half_base,-half_track)。
 * 若实际两轮同轴，将 HALF_BASE_MM 校准为 0。 */
#define SENTRY_WHEEL_HALF_BASE_MM    250.0f
#define SENTRY_MOTOR_TIMEOUT_MS     100U
#define SENTRY_IMU_TIMEOUT_MS         5U
#define SENTRY_REDUCTION_RATIO       19.0f       /* M3508 减速比 19:1                */

#define SENTRY_SWERVE_0_OFFSET       2735        /* 左轮转向零点编码器值              */
#define SENTRY_SWERVE_1_OFFSET       6819        /* 右轮转向零点编码器值              */

#define SENTRY_MAX_LINEAR_SPEED      3000.0f     /* mm/s                             */
#define SENTRY_MAX_OMEGA             10.0f       /* rad/s                            */
#define SENTRY_MAX_MOTOR_RPM         6000.0f     /* RPM                              */

/* mm/s ↔ RPM 转换 */
#define SENTRY_RPM_TO_MMPS(rpm)      ((float)(rpm) * 2.0f * LIB_MATH_PI \
                                      * SENTRY_WHEEL_RADIUS_MM / (60.0f * SENTRY_REDUCTION_RATIO))
#define SENTRY_MMPS_TO_RPM(mmps)     ((float)(mmps) * 60.0f * SENTRY_REDUCTION_RATIO \
                                      / (2.0f * LIB_MATH_PI * SENTRY_WHEEL_RADIUS_MM))

/* 编码器(0-8191) ↔ 角度(deg) */
#define SENTRY_ENC_TO_DEG(enc)       ((float)(enc) * 360.0f / 8192.0f)

/* 底盘速度 (body/world-frame 通用) */
typedef struct {
    float v_x;    /**< x方向 (mm/s, 前为正)         */
    float v_y;    /**< y方向 (mm/s, 左为正)         */
    float v_w;    /**< 角速度 (rad/s, 逆时针为正)   */
} app_sentry_chassis_speed_t;

/* ════════════════════════════════════════════════════
 * 云台机构参数
 * ════════════════════════════════════════════════════ */

/* -1 沿用首次反馈作为中心；量产/比赛前应填入实测的 0..8191 机械零点。 */
#define SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO  (-1)
/* 当前值来自原工程：仅重力前馈，不具备 Pitch 位置跟踪能力，须实机整定。 */
#define SENTRY_GIMBAL_PITCH_KP  0.0f
#define SENTRY_GIMBAL_PITCH_KI  0.0f
#define SENTRY_GIMBAL_PITCH_KD  0.0f

#define SENTRY_GIMBAL_SMALL_YAW_LIMIT_DEG   50.0f
#define SENTRY_GIMBAL_PITCH_MIN_DEG        -20.0f
#define SENTRY_GIMBAL_PITCH_MAX_DEG         40.0f
#define SENTRY_GIMBAL_PITCH_ENCODER_ZERO    5509     /* pitch水平零点 (8191 scale)           */

/* ════════════════════════════════════════════════════
 * CAN2 — 板内电机控制 (每板独立, ID可复用)
 * ════════════════════════════════════════════════════ */

/* ── 底盘电机 CAN2 ID (from Steering_wheel_Chasssis_test) ── */
#define SENTRY_CAN_CHASSIS_DRIVE_R   0x201    /**< 右驱动 M3508 反馈               */
#define SENTRY_CAN_CHASSIS_DRIVE_L   0x202    /**< 左驱动 M3508 反馈               */
#define SENTRY_CAN_CHASSIS_STEER_L   0x205    /**< 左转向 M3508 反馈               */
#define SENTRY_CAN_CHASSIS_STEER_R   0x206    /**< 右转向 M3508 反馈               */
#define SENTRY_CAN_CHASSIS_GIMBAL_G0 0x207    /**< 云台yaw G0 反馈 (朝向)          */
#define SENTRY_CAN_CHASSIS_POWER     0x212    /**< 功率计反馈                       */
#define SENTRY_CAN_CHASSIS_TX_DRIVE  0x200    /**< 驱动电流帧 TX [R_H,R_L, L_H,L_L]*/
#define SENTRY_CAN_CHASSIS_TX_STEER  0x1FF    /**< 转向+G0电流帧 TX [SL,SR,G0]     */

/* ── 云台电机 (CAN2 云台板) ── */
#define SENTRY_CAN_GIMBAL_LAUNCH_F1     0x201    /**< 左摩擦轮 M3508 反馈               */
#define SENTRY_CAN_GIMBAL_LAUNCH_D1     0x202    /**< 拨弹轮 M2006 反馈                 */
#define SENTRY_CAN_GIMBAL_LAUNCH_F2     0x203    /**< 右摩擦轮 M3508 反馈               */
#define SENTRY_CAN_GIMBAL_YAW_LARGE     0x205    /**< 大yaw GM6020 反馈                 */
#define SENTRY_CAN_GIMBAL_YAW_SMALL     0x206    /**< 小yaw GM6020 反馈                 */
#define SENTRY_CAN_GIMBAL_PITCH         0x207    /**< pitch GM6020 反馈                 */

#define SENTRY_CAN_GIMBAL_TX_YAW        0x1FF    /**< Yaw+Pitch 电流帧 TX               */
#define SENTRY_CAN_GIMBAL_TX_LAUNCH     0x200    /**< 发射控制帧 TX (M3508×2+M2006)     */

/* ════════════════════════════════════════════════════
 * CAN1 — 板间数据交换
 * ════════════════════════════════════════════════════ */

/* 云台→底盘: yaw角度反馈 (监听 app_gimbal_comm 0x124)
 *   chassis 通过 CAN1 接收 gimbal 的 BMI088 yaw 用于世界坐标正运动学 */

/* 底盘→云台: 底盘角速度 omega_z (CAN1 0x119, VMC前馈项, 已实现) */

/* ════════════════════════════════════════════════════
 * 发射控制参数
 * ════════════════════════════════════════════════════ */

#define SENTRY_DISC_TARGET_RPM          8100     /**< 拨弹轮目标转速 (RPM)               */
#define SENTRY_FRICTION_TARGET_RPM      3000     /**< 摩擦轮目标转速 (RPM)               */

/* ════════════════════════════════════════════════════
 * 云台模式枚举
 * ════════════════════════════════════════════════════ */

typedef enum {
    APP_SENTRY_GIMBAL_MODE_INC_ANGLE = 1,
    APP_SENTRY_GIMBAL_MODE_SPEED    = 2,
    APP_SENTRY_GIMBAL_MODE_ABS_ANGLE = 3
} app_sentry_gimbal_mode_t;

typedef enum {
    APP_SENTRY_FIRE_OFF = 1,
    APP_SENTRY_FIRE_ON  = 2,
    APP_SENTRY_FIRE_FIR = 3
} app_sentry_fire_mode_t;

#endif /* APP_SENTRY_COMMON_H */
