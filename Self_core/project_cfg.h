/**
 * @file    project_cfg.h
 * @brief   唯一配置入口 — 选择板型 + 车组
 */
#ifndef PROJECT_CFG_H
#define PROJECT_CFG_H

// ========== [必改] 选择主板 ==========
#define BOARD_CHASSIS       0
#define BOARD_GIMBAL        1

#ifndef CURRENT_BOARD
#define CURRENT_BOARD    BOARD_GIMBAL         // 可选: BOARD_CHASSIS / BOARD_GIMBAL
#endif

// ========== [必改] 选择车组 ==========
#define ROBOT_HERO          0
#define ROBOT_INFANTRY      1
#define ROBOT_SENTRY        2

#ifndef CURRENT_ROBOT
#define CURRENT_ROBOT       ROBOT_SENTRY       // 可选: ROBOT_HERO / ROBOT_INFANTRY / ROBOT_SENTRY
#endif

// ========== IMU 姿态算法 (枚举定义见 drv_imu.h) ==========
#ifndef IMU_AHRS_ALGORITHM
#define IMU_AHRS_ALGORITHM   DRV_IMU_ALGO_VQF // 可选: DRV_IMU_ALGO_EKF / DRV_IMU_ALGO_VQF / DRV_IMU_ALGO_MAHONY
#endif

/* IMU 固定安装角，单位度，ZYX：传感器系 -> 等效水平机体系。
 * 实机静止时的原始四元数表明传感器相对水平机体系约为绕 Y 轴 -90 度。
 * 先在四元数上消除安装旋转，再输出欧拉角；水平安装时三项设为 0。
 */
#ifndef IMU_MOUNT_ROLL_DEG
#define IMU_MOUNT_ROLL_DEG    0.0f
#endif
#ifndef IMU_MOUNT_PITCH_DEG
#define IMU_MOUNT_PITCH_DEG   -90.0f
#endif
#ifndef IMU_MOUNT_YAW_DEG
#define IMU_MOUNT_YAW_DEG     0.0f
#endif

#if CURRENT_BOARD != BOARD_CHASSIS && CURRENT_BOARD != BOARD_GIMBAL
#error "Unsupported CURRENT_BOARD"
#endif

#if CURRENT_ROBOT != ROBOT_HERO && CURRENT_ROBOT != ROBOT_INFANTRY && CURRENT_ROBOT != ROBOT_SENTRY
#error "Unsupported CURRENT_ROBOT"
#endif

#endif
