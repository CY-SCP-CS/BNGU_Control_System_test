/**
 * @file    app_chassis_comm.h
 * @brief   底盘 CAN1 板间通信协议 — 解析/发送控制指令与状态反馈
 * @note    底盘坐标系: 前=x, 左=y, 上=z; 逆时针旋转=vz正
 *          所有数据小端序, STM32F4 原生小端无需转字节序
 */
#ifndef APP_CHASSIS_COMM_H
#define APP_CHASSIS_COMM_H

#include "lib_typedef.h"

#define APP_CHASSIS_CAN_ID_SPEED_CMD       0x111//底盘速度指令
#define APP_CHASSIS_CAN_ID_POWER_FEEDBACK  0x112//功率反馈
#define APP_CHASSIS_CAN_ID_ACKERMANN_CMD   0x113//阿克曼指令
#define APP_CHASSIS_CAN_ID_FOLLOW_CMD      0x115//跟随模式指令
#define APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK  0x119 //wz反馈
#define APP_CHASSIS_OMEGA_RAD_S_PER_LSB    0.001f//0x111 中 vz 的角速度编码单位 rad/s

typedef struct {
    int16_t vx;// x轴速度分量，单位mm/s
    int16_t vy;// y轴速度分量，单位mm/s
    int16_t vz;// z轴速度分量 * 1000，单位rad/s，逆时针旋转为正
    int16_t power_pct;// 功率百分比*100, 单位W，先保留，暂时不用
} app_chassis_speed_cmd_t;//0x111: 底盘速度指令

typedef struct {
    float speed;// 车体速度，单位mm/s，向前为正
    float steer_angle;// 车体转向角，单位rad，向左为正
} app_chassis_ackermann_cmd_t;//0x113: 阿克曼底盘指令

typedef struct {
    int16_t vx;// x轴速度分量，单位mm/s，向前为正
    int16_t vy;// y轴速度分量，单位mm/s，向前为正
    int16_t gimbal_angle;// 云台角度 * 1000，单位rad，向左为正，[-PI, PI]
    int16_t custom;
} app_chassis_follow_cmd_t;//0x115: 跟随模式指令

/** @brief 底盘板间通信的全部接收数据。 */
typedef struct {
    app_chassis_speed_cmd_t speed;       // 0x111
    app_chassis_ackermann_cmd_t ackermann; // 0x113
    app_chassis_follow_cmd_t follow;     // 0x115
} app_chassis_comm_rx_t;

/** 
 * @brief 初始化底盘通信接收状态与回调
*/
void app_chassis_comm_init(void);

/**
 * @brief  原子读取全部已解包的底盘接收数据。
 * @param  rx  输出快照
 * @return 0 失败（空指针），1 成功
 */
uint8_t app_chassis_comm_read_rx(app_chassis_comm_rx_t *rx);







/**
 * @brief 发送实际功率
 * @param power_x100 实际功率 ×100，按 int16_t 小端序放入 data[0..1]。
 * @return 0 成功，非 0 失败
 */
uint8_t app_chassis_comm_power_tx(int16_t power_x100);

/** 
 * @brief 发送 rad/s 角速度反馈
 * @param omega_z 车体坐标系下绕 z 轴角速度，单位 rad/s，逆时针旋转为正
 * @return 0 成功，非 0 失败
*/
uint8_t app_chassis_comm_omega_tx(float omega_z);   /* 0x119 ωz (VMC前馈) */


#endif
