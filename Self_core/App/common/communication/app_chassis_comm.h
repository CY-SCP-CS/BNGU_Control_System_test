/**
 * @file    app_chassis_comm.h
 * @brief   底盘 CAN1 板间通信协议：接收控制指令，发送功率与角速度反馈
 * @note    底盘坐标系：前为 x、左为 y、上为 z；绕 z 轴逆时针为正。
 *          多字节数据按小端序编码；STM32F4 可直接复制整数和 float 字节。
 */
#ifndef APP_CHASSIS_COMM_H
#define APP_CHASSIS_COMM_H

#include "lib_typedef.h"

#define APP_CHASSIS_CAN_ID_SPEED_CMD       0x111U // 底盘速度指令。
#define APP_CHASSIS_CAN_ID_POWER_FEEDBACK  0x112U // 实际功率反馈。
#define APP_CHASSIS_CAN_ID_ACKERMANN_CMD   0x113U // 阿克曼底盘指令。
#define APP_CHASSIS_CAN_ID_FOLLOW_CMD      0x115U // 跟随模式指令。
#define APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK  0x119U // 底盘角速度反馈。
#define APP_CHASSIS_OMEGA_RAD_S_PER_LSB    0.001f // 0x111 中 vz 的量化单位，rad/s。

/** 0x111 底盘速度指令。 */
typedef struct {
    int16_t vx;        // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;        // y 方向速度，单位 mm/s，向左为正。
    int16_t vz;        // z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} app_chassis_speed_cmd_t;

/** 0x113 阿克曼底盘指令。 */
typedef struct {
    float speed;       // 车体线速度，单位 mm/s，向前为正。
    float steer_angle; // 转向角，单位 rad，向左为正。
} app_chassis_ackermann_cmd_t;

/** 0x115 跟随模式指令。 */
typedef struct {
    int16_t vx;           // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;           // y 方向速度，单位 mm/s，向左为正。
    int16_t gimbal_angle; // 云台角度 ÷ 0.001 rad，向左为正。
    int16_t custom;       // 协议预留字段。
} app_chassis_follow_cmd_t;

/** 底盘板接收并解包后的全部控制数据。 */
typedef struct {
    app_chassis_speed_cmd_t speed;         // 0x111。
    app_chassis_ackermann_cmd_t ackermann; // 0x113。
    app_chassis_follow_cmd_t follow;       // 0x115。
} app_chassis_comm_rx_t;

/**
 * @brief 初始化底盘 CAN 接收缓存并注册协议 ID 的回调。
 */
void app_chassis_comm_init(void);

/**
 * @brief  原子读取全部已解包的底盘接收数据。
 * @param  rx 输出快照。
 * @return 1 表示成功；rx 为空时返回 0。
 */
uint8_t app_chassis_comm_read_rx(app_chassis_comm_rx_t *rx);

/**
 * @brief  发送实际功率反馈至 0x112。
 * @param  power_x100 实际功率 × 100，以 int16_t 小端序写入 data[0..1]。
 * @return BSP CAN 发送状态：0 成功，非 0 失败。
 */
uint8_t app_chassis_comm_power_tx(int16_t power_x100);

/**
 * @brief  发送底盘 z 轴角速度反馈至 0x119。
 * @param  omega_z 绕 z 轴角速度，单位 rad/s，逆时针为正。
 * @return BSP CAN 发送状态：0 成功，非 0 失败。
 */
uint8_t app_chassis_comm_omega_tx(float omega_z);

#endif