/**
 * @file    app_gimbal_comm.h
 * @brief   云台CAN通信协议 — 解析/发送 0x120~0x129 指令, 0x122/124/126/130/233 反馈
 * @note    云台坐标系: 前=x, 左=y, 上=z; yaw左转为正, pitch下转为正
 *          所有数据小端序, STM32F4 原生小端无需转字节序
 */
#ifndef APP_GIMBAL_COMM_H
#define APP_GIMBAL_COMM_H

#include "lib_typedef.h"
//达妙IMU暂不适配，等待补充

#define APP_GIMBAL_CAN_ID_RADAR_SPEED      0x120//雷达指令，由云台转发给底盘，ID=0x111，不理解为什么
#define APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT   0x121//无射击速度指令
#define APP_GIMBAL_CAN_ID_SPEED_FEEDBACK   0x122//速度反馈
#define APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT   0x123//无射击角度指令
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK   0x124//角度反馈
#define APP_GIMBAL_CAN_ID_SPEED_SHOOT      0x125//射击速度指令
#define APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK   0x126//射击反馈，自定义，不知道是干什么的
#define APP_GIMBAL_CAN_ID_ANGLE_SHOOT      0x127//射击角度指令
#define APP_GIMBAL_CAN_ID_SHOOT_CMD      0x129//发射机构控制指令
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2 0x130//角度反馈V2（可能是英雄？）
#define APP_GIMBAL_CAN_ID_IMU_QUATERNION   0x233//IMU四元数
#define APP_GIMBAL_CAN_ID_DM_IMU_REQUEST 0x31//请求达妙IMU数据
#define APP_GIMBAL_CAN_ID_DM_IMU_RESPONSE 0x39//达妙IMU反馈

#define APP_GIMBAL_ANGLE_SHOOT_TARGET_DETECTED  0xFFFF


typedef struct {
    int16_t vx;// x轴速度分量，单位mm/s
    int16_t vy;// y轴速度分量，单位mm/s
    int16_t vz;// z轴速度分量 * 1000，单位rad/s，逆时针旋转为正
    int16_t power_pct;// 功率百分比*100, 单位W
} app_gimbal_radar_cmd_t;//0x120: 雷达指令，转发给底盘，ID=0x111


typedef struct {
    float yaw_inc;// yaw轴角度增量 [-PI, PI]，单位rad，左转为正
    float pitch_inc;// pitch轴角度增量 [-PI, PI]，单位rad，下转为正
} app_gimbal_speed_cmd_t;//0x121 / 0x125: 速度指令 (增量式)，云台自用

typedef struct {
    float yaw_abs; // yaw轴IMU绝对角度 [-PI, PI]，单位rad，左转为正
    float pitch_abs; //pitch轴IMU绝对角度  [-PI, PI]，单位rad，下转为正，记得处理限位截断
} app_gimbal_angle_cmd_t;//0x123 / 0x127: 角度指令 (绝对式)，云台自用


typedef struct {
    uint8_t shoot_switch;// 0xFF=持续射击, 0x00=停止射击
    uint8_t retreat;// 0xFF=发弹, 0x00=不发弹
} app_gimbal_shoot_cmd_t;//0x129: 发射机构控制指令 (DLC=4)
    

/**
 *  @brief 初始化云台通信接收状态与回调。
 */
void app_gimbal_comm_init(void);

/** 
 * @brief 读取雷达指令缓存
 */
const app_gimbal_radar_cmd_t* app_gimbal_comm_get_radar_cmd(void);
/** 
 * @brief 读取无射击速度指令缓存
 */
const app_gimbal_speed_cmd_t* app_gimbal_comm_get_speed_no_shoot(void);
/** 
 * @brief 读取无射击角度指令缓存
 */
const app_gimbal_angle_cmd_t* app_gimbal_comm_get_angle_no_shoot(void);
/** 
 * @brief 读取射击速度指令缓存
 */
const app_gimbal_speed_cmd_t* app_gimbal_comm_get_speed_shoot(void);
/** 
 * @brief 读取射击角度指令缓存
 */
const app_gimbal_angle_cmd_t* app_gimbal_comm_get_angle_shoot(void);
/** 
 * @brief 读取最新射击指令缓存
 */
const app_gimbal_shoot_cmd_t* app_gimbal_comm_get_control(void);

/**
 *  @brief 读取最后收到的有效绝对角度帧；拒绝非有限数，超时返回 0。
 */
uint8_t app_gimbal_comm_read_angle_cmd(app_gimbal_angle_cmd_t *cmd, uint32_t timeout_ms);

/**
 *  @brief 主循环处理雷达速度转发，避免在接收中断中竞争 CAN 发送邮箱。
 */
void app_gimbal_comm_process(void);

/* 发送接口 */
/** 
 * @brief 发送速度反馈
 * @param yaw_speed 云台坐标系下绕 yaw 轴增量角度，单位 rad，左转为正
 * @param pitch_speed 云台坐标系下绕 pitch 轴增量角度，单位 rad，下转为正
 */
void app_gimbal_comm_send_speed_feedback(float yaw_speed, float pitch_speed);
/** 
 * @brief 发送角度反馈
 * @param yaw_angle 云台坐标系下绕 yaw 轴绝对角度，单位 rad，左转为正
 * @param pitch_angle 云台坐标系下绕 pitch 轴绝对角度，单位 rad，下转为正
*/
void app_gimbal_comm_send_angle_feedback(float yaw_angle, float pitch_angle);
/** 
 * @brief 发送角度反馈V2
 * @note 叶师傅写的，应该是适配英雄云台，暂时保留
 */
void app_gimbal_comm_send_angle_feedback_v2(uint16_t yaw, uint16_t pitch,
                                            uint16_t roll, uint16_t interval);
/** 
 * @brief 发送发射机构反馈
 */
void app_gimbal_comm_send_shoot_feedback(const uint8_t data[8]);
/** 
 * @brief 发送IMU四元数（*30000）
 * @param q0 四元数 w 分量 * 30000
 * @param q1 四元数 x 分量 * 30000
 * @param q2 四元数 y 分量 * 30000
 * @param q3 四元数 z 分量 * 30000
 */
void app_gimbal_comm_send_imu_quaternion(int16_t q0, int16_t q1,
                                         int16_t q2, int16_t q3);

#endif
