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
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2 0x130//角度反馈V2（可能是英雄？），不用了，暂时保留
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


typedef struct {
    app_gimbal_radar_cmd_t radar_speed;       // 0x120
    app_gimbal_speed_cmd_t speed_no_shoot;    // 0x121
    app_gimbal_angle_cmd_t angle_no_shoot;    // 0x123
    app_gimbal_speed_cmd_t speed_shoot;       // 0x125
    app_gimbal_angle_cmd_t angle_shoot;       // 0x127
    app_gimbal_shoot_cmd_t shoot;             // 0x129
} app_gimbal_comm_rx_t;//云台板间通信的全部接收数据

typedef enum {
    APP_GIMBAL_INPUT_DBUS = 1U,       // S2 上挡：遥控器
    APP_GIMBAL_INPUT_CAN  = 3U,       // S2 中挡：上位机 CAN
    APP_GIMBAL_INPUT_ESTOP = 2U       // S2 下挡：急停
} app_gimbal_input_source_t;

typedef struct {
    app_gimbal_input_source_t source;
    float chassis_vx_norm;
    float chassis_vy_norm;
    float chassis_omega_norm;
    float yaw_rate_norm;
    float pitch_rate_norm;
    uint8_t launcher_mode;
} app_gimbal_dbus_input_t;

typedef enum {
    APP_GIMBAL_COMM_UPDATE_RADAR_SPEED      = (1U << 0),
    APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT   = (1U << 1),
    APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT   = (1U << 2),
    APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT      = (1U << 3),
    APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT      = (1U << 4),
    APP_GIMBAL_COMM_UPDATE_SHOOT            = (1U << 5)
} app_gimbal_comm_update_t;

/**
 *  @brief 初始化云台通信接收状态与回调。
 */
void app_gimbal_comm_init(void);

/**
 * @brief  原子读取全部已解包的云台接收数据。
 * @param  rx  输出快照
 * @return 0 失败（空指针），1 成功
 */
uint8_t app_gimbal_comm_read_rx(app_gimbal_comm_rx_t *rx,
                                uint8_t *updated_mask);

/**
 * @brief  读取 DBUS 输入并根据 S2 选择控制源。
 * @note   摇杆仅归一化，不在通信层乘具体机构的速度上限。
 */
uint8_t app_gimbal_comm_dbus_rx(app_gimbal_dbus_input_t *input);

/**
 *  @brief 主循环处理雷达速度转发，避免在接收中断中竞争 CAN 发送邮箱。
 */
void app_gimbal_comm_process(void);

/* 发送接口 */
/** 
 * @brief 发送速度反馈
 * @param yaw_tar_speed 云台坐标系下绕 yaw 轴角速度，单位 rad/s，左转为正
 * @param pitch_tar_speed 云台坐标系下绕 pitch 轴角速度，单位 rad/s，下转为正
 */
void app_gimbal_comm_gyro_tx(float yaw_tar_speed, float pitch_tar_speed);
/** 
 * @brief 发送角度反馈
 * @param yaw_angle 云台坐标系下绕 yaw 轴绝对角度，单位 rad，左转为正
 * @param pitch_angle 云台坐标系下绕 pitch 轴绝对角度，单位 rad，下转为正
*/
void app_gimbal_comm_angle_tx(float yaw_angle, float pitch_angle);
/** 
 * @brief 发送角度反馈V2
 * @note 叶师傅写的，应该是适配英雄云台，暂时保留
 */
void app_gimbal_comm_angle_tx_v2(uint16_t yaw, uint16_t pitch,
                                            uint16_t roll, uint16_t interval);
/** 
 * @brief 发送发射机构反馈
 */
void app_gimbal_comm_shoot_tx(const uint8_t data[8]);
/** 
 * @brief 发送IMU四元数（*30000）
 * @param q0 四元数 w 分量 * 30000
 * @param q1 四元数 x 分量 * 30000
 * @param q2 四元数 y 分量 * 30000
 * @param q3 四元数 z 分量 * 30000
 */
void app_gimbal_comm_quat_tx(int16_t q0, int16_t q1,
                                         int16_t q2, int16_t q3);

#endif
