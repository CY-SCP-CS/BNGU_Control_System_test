/**
 * @file    app_chassis_comm.h
 * @brief   底盘CAN通信协议 — 解析/发送 0x111~0x115 指令, 0x112 状态反馈
 * @note    底盘坐标系: 前=x, 左=y, 上=z; 逆时针旋转=vz正
 *          所有数据小端序, STM32F4 原生小端无需转字节序
 */
#ifndef APP_CHASSIS_COMM_H
#define APP_CHASSIS_COMM_H

#include "lib_typedef.h"

// ─── CAN ID 定义 ──────────────────────────────────

#define APP_CHASSIS_CAN_ID_SPEED_CMD       0x111
#define APP_CHASSIS_CAN_ID_POWER_FEEDBACK  0x112
#define APP_CHASSIS_CAN_ID_ACKERMANN_CMD   0x113
#define APP_CHASSIS_CAN_ID_FOLLOW_CMD      0x115
#define APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK  0x119   /* 底盘→云台 ωz 反馈 (VMC前馈) */

// ─── 0x111: 底盘速度指令 ──────────────────────────

typedef struct {
    int16_t vx;           /* x轴速度分量                          */
    int16_t vy;           /* y轴速度分量                          */
    int16_t vz;           /* z轴角速度分量, 逆时针为正             */
    int16_t power_pct;    /* 允许功率百分比 x100                   */
} app_chassis_speed_cmd_t;

// ─── 0x113: 阿克曼底盘指令 ──────────────────────────

typedef struct {
    float speed;          /* 驱动速度 [-100, 100], 向前为正        */
    float steer_angle;    /* 前轮转角 [-PI, PI], 向左为正          */
} app_chassis_ackermann_cmd_t;

// ─── 0x115: 跟随模式指令 ────────────────────────────

typedef struct {
    int16_t vx;
    int16_t vy;
    int16_t gimbal_angle; /* 云台角度 x1000                       */
    int16_t custom;
} app_chassis_follow_cmd_t;

// ─── 接口声明 ─────────────────────────────────────

void app_chassis_comm_init(void);

/* 获取最新指令数据 */
const app_chassis_speed_cmd_t     *app_chassis_comm_get_speed_cmd(void);
const app_chassis_ackermann_cmd_t *app_chassis_comm_get_ackermann_cmd(void);
const app_chassis_follow_cmd_t    *app_chassis_comm_get_follow_cmd(void);
uint32_t app_chassis_comm_get_speed_cmd_tick(void);   /* 最后收到0x111的tick, 0=从未收到 */

/* 发送接口: 返回 0=成功, 非0=发送失败 */
uint8_t app_chassis_comm_send_power_feedback(int16_t power_x100);
uint8_t app_chassis_comm_send_omega_feedback(float omega_z);   /* 0x119 ωz (VMC前馈) */
void app_chassis_comm_send_speed_cmd(int16_t vx, int16_t vy, int16_t vz, int16_t power_pct);

#endif