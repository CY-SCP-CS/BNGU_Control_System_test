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
#define CURRENT_BOARD   BOARD_CHASSIS        // 可选: BOARD_CHASSIS / BOARD_GIMBAL
#endif

// ========== [必改] 选择车组 ==========
#define ROBOT_HERO          0
#define ROBOT_INFANTRY      1
#define ROBOT_SENTRY        2
#define ROBOT_NAVIGATION    3

#ifndef CURRENT_ROBOT
#define CURRENT_ROBOT       ROBOT_HERO       // 可选: ROBOT_HERO / ROBOT_INFANTRY / ROBOT_SENTRY / ROBOT_NAVIGATION
#endif

// ========== IMU 姿态算法 (枚举定义见 drv_imu.h) ==========
#ifndef IMU_AHRS_ALGORITHM
#define IMU_AHRS_ALGORITHM   DRV_IMU_ALGO_VQF // 可选: DRV_IMU_ALGO_EKF / DRV_IMU_ALGO_VQF / DRV_IMU_ALGO_MAHONY
#endif

// ========== 哨兵云台 CAN1 通信测试 ==========
#ifndef SENTRY_CAN_TEST_ENABLE
#define SENTRY_CAN_TEST_ENABLE 0U // 0=关闭，1=每 100 ms 发送一次 0x300 测试帧。
#endif

#if CURRENT_BOARD != BOARD_CHASSIS && CURRENT_BOARD != BOARD_GIMBAL
#error "Unsupported CURRENT_BOARD"
#endif

#if CURRENT_ROBOT != ROBOT_HERO && CURRENT_ROBOT != ROBOT_INFANTRY && CURRENT_ROBOT != ROBOT_SENTRY && CURRENT_ROBOT != ROBOT_NAVIGATION
#error "Unsupported CURRENT_ROBOT"
#endif

#endif
