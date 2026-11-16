/**
 * @file    app_sentry_gimbal.h
 * @brief   哨兵云台控制：双 yaw 独立 PID、pitch 重力补偿和发射机构控制
 * @note    CAN2 连接本板电机，CAN1 交换板间指令与状态。
 */
#ifndef APP_SENTRY_GIMBAL_H
#define APP_SENTRY_GIMBAL_H

#include "lib_typedef.h"
#include "lib_pid.h"
#include "lib_filter.h"
#include "drv_motor.h"
#include "app_gimbal_comm.h"
#include "drv_imu.h"
#include "app_sentry_common.h"

/** 云台控制命令。 */
typedef struct {
    uint8_t mode; // app_sentry_gimbal_mode_t。
    float yaw_tar_speed; // yaw 目标角速度，rad/s。
    float pitch_tar_speed; // pitch 目标角速度，rad/s。
    float yaw_tar_angle; // yaw 目标角度，rad。
    float pitch_tar_angle; // pitch 目标角度，rad。
    float yaw_tar_angle_delta; // yaw 目标角度增量，rad。
    float pitch_tar_angle_delta; // pitch 目标角度增量，rad。
    uint8_t fire; // app_sentry_fire_mode_t。
} app_sentry_gimbal_cmd_t;

enum { GIMBAL_MOTOR_COUNT = 6U, GIMBAL_AXIS_COUNT = 3U };

/** 云台电机在反馈表和发送电流帧中的索引。 */
enum {
    MOTOR_YAW_L = 0, // 大 yaw GM6020，反馈 ID 0x205。
    MOTOR_YAW_S = 1, // 小 yaw GM6020，反馈 ID 0x206。
    MOTOR_PITCH = 2, // pitch GM6020，反馈 ID 0x207。
    MOTOR_FRIC_L = 3, // 左摩擦轮 M3508，反馈 ID 0x201。
    MOTOR_DISC = 4, // 拨弹轮 M2006，反馈 ID 0x202。
    MOTOR_FRIC_R = 5  // 右摩擦轮 M3508，反馈 ID 0x203。
};

/** 电机反馈：编码器值、转速(RPM)及反馈电流。 */
typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t current;
} app_sentry_gimbal_motor_feedback_t;

/** 电机反馈快照、CAN 接收缓冲和电流输出。 */
typedef struct {
    app_sentry_gimbal_motor_feedback_t feedback[GIMBAL_MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    app_sentry_gimbal_motor_feedback_t rx[GIMBAL_MOTOR_COUNT]; // CAN 中断写入的反馈缓冲。
    uint32_t rx_tick_ms[GIMBAL_AXIS_COUNT]; // 三个云台电机最近一次有效反馈时间。
    uint8_t rx_valid_mask; // 明确区分未收首帧与合法零值。
    int16_t current[GIMBAL_MOTOR_COUNT]; // 本周期电流输出。
    float pitch_cur_speed; // pitch 电机角速度，rad/s。
    lib_lpf_t pitch_speed_lpf; // pitch 转速低通滤波状态。
} app_sentry_gimbal_motor_state_t;

/** 控制与 CAN1 共用的云台姿态结果，单位 rad、rad/s。 */
typedef struct {
    app_sentry_euler_t euler; // rad。
    float yaw_rate; // rad/s。
    drv_imu_quat_t quat; // 最近一次 CAN1 云台姿态，w/x/y/z。
} app_sentry_gimbal_pose_t;

/** IMU 采样调度及有效性。 */
typedef struct {
    drv_imu_t *instance;
    lib_lpf_t yaw_rate_lpf; // yaw 控制角速度滤波，out 单位 rad/s。
    float ahrs_elapsed_s;
    uint8_t is_valid; // 0=尚无有效采样，1=已收到有效采样。
    uint32_t ahrs_tick_ms;
} app_sentry_gimbal_sampling_t;

/** pitch 加速度前馈：滤波、参数和输出电流。 */
typedef struct {
    lib_lpf_t bias_lpf; // 加速度慢基线滤波，out 单位 g。
    lib_lpf_t dynamic_lpf; // 去基线后加速度滤波，out 单位 g。
    float deadband; // 软死区，g。
    float gain; // 前馈增益，电流指令/g。
    float limit; // 前馈电流指令限幅。
    float current; // 本周期前馈电流指令。
} app_sentry_gimbal_pitch_ff_t;

/** 云台协议输入与控制器内部目标。 */
typedef struct {
    app_sentry_gimbal_cmd_t data; // 当前控制输入。
    float yaw_tar_angle; // yaw 目标角度，rad。
    float pitch_tar_angle; // pitch 目标角度，rad。
    struct {
        uint8_t fire_override; // 0x129 是否已接管发射状态。
        uint8_t pose_is_valid; // 是否已接收有效 CAN 姿态命令。
        uint32_t pose_tick_ms; // 最近一次有效姿态命令时间，ms。
    } can;
    struct {
        uint8_t is_active; // 自动巡扫是否启用。
        uint8_t pitch_increasing; // 0=角度减小，1=角度增大。
    } scan;
} app_sentry_gimbal_command_state_t;

/** CAN1 底盘角速度反馈及接收新鲜度。 */
typedef struct {
    float vw_cur_speed; // 底盘当前对地角速度，rad/s。
    uint32_t timeout_ms; // 反馈有效期，ms。
    uint32_t vw_rx_tick_ms; // 最近一次 0x119 接收时间。
    uint8_t vw_is_valid; // 已收到有效的 0x119 角速度。
} app_sentry_gimbal_chassis_feedback_t;

/** 云台转发给底盘的控制命令。 */
typedef struct {
    int16_t vx_tar_speed; // 底盘 x 方向目标速度，mm/s。
    int16_t vy_tar_speed; // 底盘 y 方向目标速度，mm/s。
    int16_t vw_tar_speed; // 底盘目标角速度的 0.001 rad/s 协议值。
    uint8_t emergency_stop; // 底盘急停状态。
    app_gimbal_input_source_t input_source;// 当前控制源。
} app_sentry_gimbal_chassis_state_t;

/** CAN1 发送诊断，不参与控制源或命令决策。 */
typedef struct {
    uint32_t tx_attempt_count; // 0x111 发送尝试次数。
    uint32_t tx_accept_count; // BSP 已接受次数（含进入软件缓冲）。
    uint32_t tx_fail_count; // BSP 拒绝次数。
    uint32_t tx_can_skip_count; // CAN 控制源下跳过遥控器发送的次数。
    uint8_t last_tx_result; // 最近一次 BSP 结果：0 接受、1 失败。
} app_sentry_gimbal_tx_stats_t;

/** 电机使能前的反馈连续有效等待及目标对齐状态。 */
typedef struct {
    uint8_t is_ready; // 目标已对齐，允许闭环输出。
    uint8_t is_settling; // 正在等待反馈持续有效。
    uint32_t ready_tick_ms; // 本次连续有效反馈起始时间，ms。
} app_sentry_gimbal_startup_t;

/** 云台各控制环 PID 状态。 */
typedef struct {
    lib_pid_t yaw_large; // 大 yaw 绝对角度环。
    lib_pid_t yaw_small; // 小 yaw 绝对角度环。
    lib_pid_t pitch; // pitch 角度环。
    lib_pid_t disc; // 拨弹轮速度环。
    lib_pid_t fric_l; // 左摩擦轮速度环。
    lib_pid_t fric_r; // 右摩擦轮速度环。
} app_sentry_gimbal_pid_state_t;

/**
 * @brief 云台全局调试快照。
 *
 * 保留外部链接便于调试器 Watch 在任意断点查看；不在头文件声明，
 * 仅保存运行状态的副本，不作为模块间访问接口，也不允许回写控制状态。
 */
typedef struct {
    app_sentry_gimbal_motor_state_t motor;
    app_sentry_gimbal_pose_t pose;
    app_sentry_gimbal_pitch_ff_t pitch_ff;
    app_sentry_gimbal_command_state_t command;
    app_sentry_gimbal_chassis_state_t chassis;
    app_sentry_gimbal_chassis_feedback_t chassis_feedback;
    app_sentry_gimbal_startup_t startup;
    app_sentry_gimbal_tx_stats_t chassis_tx;
    app_sentry_gimbal_pid_state_t pid;
} app_sentry_gimbal_debug_t;

/**
 * @brief  初始化云台控制状态并注册电机 CAN 回调。
 * @param  imu IMU 驱动实例。
 */
void app_sentry_gimbal_init(drv_imu_t *imu);

/**
 * @brief  更新 IMU 的 AHRS。
 * @param  dt 本次更新周期，单位 s。
 */
void app_gimbal_ahrs_update(float dt);

/** @brief 执行云台 yaw、pitch 控制；调度频率为 1 kHz。 */
void app_sentry_gimbal_ctrl_1khz(void);

/** @brief 每 200 Hz 无条件发送 CAN1 云台状态帧；姿态帧要求 IMU 实例有效。 */
void app_sentry_gimbal_status_tx_200hz(void);

/** @brief 每 200 Hz 优先发送遥控器底盘速度或急停命令；CAN 控制源由通信模块转发。 */
void app_sentry_gimbal_chassis_tx_200hz(void);

/** 将云台坐标系平面速度转换为底盘坐标系速度。 */
void app_sentry_gimbal_velocity_to_chassis(float vx_gimbal, float vy_gimbal,
                                           float *vx_chassis, float *vy_chassis);

/** @brief 执行摩擦轮和拨弹轮控制；调度频率为 200 Hz。 */
void app_sentry_launcher_ctrl_200hz(void);

#endif
