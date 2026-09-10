/**
 * @file    app_gimbal_comm.h
 * @brief   云台CAN通信协议 — 解析/发送 0x120~0x129 指令, 0x122/124/126/130/233 反馈
 * @note    云台坐标系: 前=x, 左=y, 上=z; yaw左转为正, pitch下转为正
 *          所有数据小端序, STM32F4 原生小端无需转字节序
 */
#ifndef APP_GIMBAL_COMM_H
#define APP_GIMBAL_COMM_H

#include "lib_typedef.h"

// ─── CAN ID 定义 ──────────────────────────────────

#define APP_GIMBAL_CAN_ID_RADAR_SPEED      0x120
#define APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT   0x121
#define APP_GIMBAL_CAN_ID_SPEED_FEEDBACK   0x122
#define APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT   0x123
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK   0x124
#define APP_GIMBAL_CAN_ID_SPEED_SHOOT      0x125
#define APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK   0x126
#define APP_GIMBAL_CAN_ID_ANGLE_SHOOT      0x127
#define APP_GIMBAL_CAN_ID_CONTROL_CMD      0x129
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2 0x130
#define APP_GIMBAL_CAN_ID_IMU_QUATERNION   0x233

// ─── 0x120: 雷达速度指令 (格式同底盘0x111, 云台收到后转发) ──

typedef struct {
    int16_t vx;
    int16_t vy;
    int16_t vz;
    int16_t power_pct;
} app_gimbal_radar_speed_cmd_t;

// ─── 0x121 / 0x125: 速度指令 (增量式) ──────────────

typedef struct {
    float yaw_inc;        /* yaw轴角度增量 [-PI, PI]           */
    float pitch_inc;      /* pitch轴角度增量 [-PI, PI]         */
} app_gimbal_speed_cmd_t;

// ─── 0x123 / 0x127: 角度指令 (绝对式) ──────────────

typedef struct {
    float yaw_abs;        /* yaw轴IMU绝对角度 [-PI, PI]        */
    float pitch_abs;      /* pitch轴IMU绝对角度 [-0.4712, 0.2269] */
} app_gimbal_angle_cmd_t;

/* 0x127 特殊值: float全0xFFFF 表示检测到目标 */
#define APP_GIMBAL_ANGLE_SHOOT_TARGET_DETECTED  0xFFFF

// ─── 0x129: 控制指令 (DLC=4) ─────────────────────

typedef struct {
    uint8_t shoot_switch; /* 0xFF=持续开, 0x00=持续关         */
    uint8_t retreat;      /* 0xFF=持续退弹, 0x00=持续关       */
} app_gimbal_control_cmd_t;

// ─── 接口声明 ─────────────────────────────────────

/** @brief 初始化云台通信接收状态与回调。 */
void app_gimbal_comm_init(void);

/* 获取最新指令数据 */
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_radar_speed_cmd_t *app_gimbal_comm_get_radar_speed(void);
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_speed_cmd_t       *app_gimbal_comm_get_speed_no_shoot(void);
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_angle_cmd_t       *app_gimbal_comm_get_angle_no_shoot(void);
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_speed_cmd_t       *app_gimbal_comm_get_speed_shoot(void);
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_angle_cmd_t       *app_gimbal_comm_get_angle_shoot(void);
/** @brief 读取最新协议缓存，主循环使用。 */
const app_gimbal_control_cmd_t     *app_gimbal_comm_get_control(void);

/** @brief 读取最后收到的有效绝对角度帧；拒绝非有限数，超时返回 0。 */
uint8_t app_gimbal_comm_read_angle_cmd(app_gimbal_angle_cmd_t *cmd, uint32_t timeout_ms);

/** @brief 主循环处理雷达速度转发，避免在接收中断中竞争 CAN 发送邮箱。 */
void app_gimbal_comm_process(void);

/* 发送接口 */
/** @brief 发送对应 CAN 遥测帧。 */
void app_gimbal_comm_send_speed_feedback(float yaw_speed, float pitch_speed);
/** @brief 发送对应 CAN 遥测帧。 */
void app_gimbal_comm_send_angle_feedback(float yaw_angle, float pitch_angle);
/** @brief 发送对应 CAN 遥测帧。 */
void app_gimbal_comm_send_angle_feedback_v2(uint16_t yaw, uint16_t pitch,
                                            uint16_t roll, uint16_t interval);
/** @brief 发送对应 CAN 遥测帧。 */
void app_gimbal_comm_send_shoot_feedback(const uint8_t data[8]);
/** @brief 发送对应 CAN 遥测帧。 */
void app_gimbal_comm_send_imu_quaternion(int16_t q0, int16_t q1,
                                         int16_t q2, int16_t q3);

#endif
