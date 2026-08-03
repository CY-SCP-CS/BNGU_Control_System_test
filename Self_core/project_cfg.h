/**
 * @file    project_cfg.h
 * @brief   唯一配置入口 — 选择板型 + 车组
 */
#ifndef PROJECT_CFG_H
#define PROJECT_CFG_H

// ========== [必改] 选择主板 ==========
#define BOARD_CHASSIS       0
#define BOARD_GIMBAL        1

#define CURRENT_BOARD       BOARD_CHASSIS      // 可选: BOARD_CHASSIS / BOARD_GIMBAL

// ========== [必改] 选择车组 ==========
#define ROBOT_HERO          0
#define ROBOT_INFANTRY      1
#define ROBOT_SENTRY        2

#define CURRENT_ROBOT       ROBOT_HERO         // 可选: ROBOT_HERO / ROBOT_INFANTRY / ROBOT_SENTRY

#endif
