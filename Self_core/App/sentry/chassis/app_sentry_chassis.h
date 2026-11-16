/**
 * @file    app_sentry_chassis.h
 * @brief   哨兵双舵轮底盘控制接口
 * @note    每轮由 GM6020 负责转向、M3508 负责驱动；CAN2 连接电机，CAN1 交换板间数据。
 */
#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "lib_filter.h"
#include "drv_motor.h"
#include "drv_power_measure.h"
#include "app_sentry_common.h"

/** 单个舵轮的转向和驱动状态。 */
typedef struct {
    float steer_angle; // 转向角度，单位 rad。
    float drive_speed; // 驱动线速度，单位 mm/s。
    uint8_t drive_reversed; // 驱动反转标志，0=正向，1=反向。
} app_sentry_swerve_wheel_t;

/** 对外发布的底盘运行状态。 */
typedef struct {
    app_sentry_swerve_wheel_t wheel_cur[2]; // 当前左右舵轮状态。
    app_sentry_chassis_speed_t cur_speed; // 当前估算车体速度。
    float cur_power; // 当前实测功率，W。
    float tar_power; // 功率控制目标，W。
    float power_scale; // 最终电流缩放系数，范围 0~1。
    float battery_voltage; // 电池电压，V。
    float battery_current; // 电池电流，A。
} app_sentry_chassis_state_t;

enum { MOTOR_COUNT = 4U };

/** 底盘滤波与通信超时参数。 */
typedef struct {
    float omega_cutoff_hz;       // 角速度反馈低通截止频率，Hz。
    float power_cutoff_hz;       // 功率低通截止频率，Hz。
    uint32_t command_timeout_ms; // 0x111 速度指令失效时间，ms。
} app_sentry_chassis_config_t;

/** 底盘电机在反馈表和电流帧中的索引。 */
enum { M_DRIVE_R = 0, M_DRIVE_L = 1, M_STEER_L = 2, M_STEER_R = 3 };

/** 电机反馈：编码器值及转速(RPM)。 */
typedef struct {
    uint16_t angle;
    int16_t speed;
} app_sentry_chassis_motor_feedback_t;

/** 电机反馈快照、CAN 接收缓冲、电流输出和驱动前馈。 */
typedef struct {
    app_sentry_chassis_motor_feedback_t feedback[MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    app_sentry_chassis_motor_feedback_t rx[MOTOR_COUNT]; // CAN 中断写入的反馈缓冲。
    int16_t current[MOTOR_COUNT]; // 本周期电流输出。
    int16_t drive_current_ff[2]; // 两个驱动轮的电流前馈。
} app_sentry_chassis_motor_state_t;

/** 底盘目标/当前状态和动力学中间量。 */
typedef struct {
    app_sentry_swerve_wheel_t wheel_cur[2]; // 当前舵轮状态。
    app_sentry_swerve_wheel_t wheel_tar[2]; // 目标舵轮状态。
    app_sentry_chassis_speed_t cmd_speed; // 限幅后的原始速度指令。
    app_sentry_chassis_speed_t tar_speed; // 斜坡后的车体目标速度，兼作斜坡历史状态。
    app_sentry_chassis_speed_t cur_speed; // 正运动学估算的车体速度。
    lib_lpf_t omega_tx_lpf; // CAN1 角速度反馈低通滤波器。
} app_sentry_chassis_motion_state_t;

/** 功率计反馈、滤波状态和电流缩放系数。 */
typedef struct {
    drv_power_data_t rx; // 功率计 CAN 接收缓冲。
    lib_lpf_t measure_lpf; // 实测功率低通滤波器。
    float measured_power; // 滤波后的实测功率，W。
    float tar_power; // 功率控制目标，W。
    float scale; // 电机电流缩放系数，范围 0~1。
} app_sentry_chassis_power_state_t;

/** 底盘各级 PID 状态。 */
typedef struct {
    lib_pid_t x; // 车体 x 方向速度环。
    lib_pid_t y; // 车体 y 方向速度环。
    lib_pid_t w; // 车体角速度环。
    lib_pid_t steer[2]; // 两个转向角度环。
    lib_pid_t drive[2]; // 两个驱动速度环。
    lib_pid_t power; // 功率缩放环。
} app_sentry_chassis_control_state_t;

/**
 * @brief 底盘全局调试快照。
 *
 * 保留外部链接便于调试器 Watch 在任意断点查看；不在头文件声明，
 * 仅保存运行状态的副本，不作为模块间访问接口，也不允许回写控制状态。
 */
typedef struct {
    app_sentry_chassis_motor_state_t motor;
    app_sentry_chassis_motion_state_t motion;
    app_sentry_chassis_power_state_t power;
    app_sentry_chassis_control_state_t pid;
    app_sentry_chassis_state_t state;
} app_sentry_chassis_debug_t;

/** @brief 初始化双舵轮控制状态并注册 CAN 电机反馈。 */
void app_sentry_chassis_init(void);

/** @brief 执行一次底盘控制循环；调度频率为 1 kHz。 */
void app_sentry_chassis_ctrl(void);

/**
 * @brief  获取最近一次发布的底盘状态。
 * @return 只读状态指针。
 */
const app_sentry_chassis_state_t *app_chassis_get_state(void);

#endif
