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

/** 
 * @brief 初始化底盘通信接收状态与回调
*/
void app_chassis_comm_init(void);

/** 
 * @brief 获取原始缓存；控制逻辑应使用 read_speed_cmd 快照接口
 */
const app_chassis_speed_cmd_t* app_chassis_comm_get_speed_cmd(void);
/** 
 * @brief 获取最新阿克曼指令原始缓存
*/
const app_chassis_ackermann_cmd_t* app_chassis_comm_get_ackermann_cmd(void);
/** 
 * @brief 获取最新跟随指令原始缓存
*/
const app_chassis_follow_cmd_t* app_chassis_comm_get_follow_cmd(void);
/** 
 * @brief 获取最后收帧时间；有效性由 read_speed_cmd 判断
*/
uint32_t app_chassis_comm_get_speed_cmd_tick(void);

/** 
 * @brief 原子读取有效速度指令；超时或从未收到时返回 0。
*/
uint8_t app_chassis_comm_read_speed_cmd(app_chassis_speed_cmd_t *cmd, uint32_t timeout_ms);

/**
 * @brief 发送实际功率
 * @param power_x100 实际功率 ×100，按 int16_t 小端序放入 data[0..1]。
 * @return 0 成功，非 0 失败
 */
uint8_t app_chassis_comm_send_power_feedback(int16_t power_x100);

/** 
 * @brief 发送 rad/s 角速度反馈
 * @param omega_z 车体坐标系下绕 z 轴角速度，单位 rad/s，逆时针旋转为正
 * @return 0 成功，非 0 失败
*/
uint8_t app_chassis_comm_send_omega_feedback(float omega_z);   /* 0x119 ωz (VMC前馈) */


#endif
