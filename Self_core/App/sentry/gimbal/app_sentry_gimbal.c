/**
 * @file    app_sentry_gimbal.c
 * @brief   哨兵云台控制实现：双 yaw 独立 PID、pitch 控制与发射机构控制
 * @note    AHRS、yaw 和 pitch 按 1 kHz 调度；发射机构按 200 Hz 调度。
 *          CAN2 发送本板电机电流，CAN1 交换板间命令和云台状态。
 */
#include "app_sentry_gimbal.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"
#include "bsp_tim.h"

#include "drv_imu.h"
#include "drv_motor.h"

#include "drv_vofa.h"
#include "bsp_cfg.h"
#include "project_cfg.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include <math.h>
#include <string.h>

#define GIMBAL_MOTOR_COUNT            6U
#define GIMBAL_AXIS_COUNT             3U
#define GIMBAL_CHASSIS_OMEGA_TIMEOUT_MS 20U

#define PITCH_ACC_BIAS_CUTOFF_HZ       0.3f
#define PITCH_ACC_DYNAMIC_CUTOFF_HZ    25.0f
#define PITCH_ACC_DEADBAND_G           0.05f
#define PITCH_ACC_FF_GAIN              3000.0f
#define PITCH_ACC_FF_LIMIT             1900.0f

/** 云台电机在反馈表和发送电流帧中的索引。 */
enum {
    MOTOR_YAW_L  = 0, // 大 yaw GM6020，反馈 ID 0x205。
    MOTOR_YAW_S  = 1, // 小 yaw GM6020，反馈 ID 0x206。
    MOTOR_PITCH  = 2, // pitch GM6020，反馈 ID 0x207。
    MOTOR_FRIC_L = 3, // 左摩擦轮 M3508，反馈 ID 0x201。
    MOTOR_DISC   = 4, // 拨弹轮 M2006，反馈 ID 0x202。
    MOTOR_FRIC_R = 5  // 右摩擦轮 M3508，反馈 ID 0x203。
};

/** 电机反馈快照、CAN 接收缓冲和电流输出。 */
typedef struct {
    drv_motor_data_t feedback[GIMBAL_MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    drv_motor_data_t rx[GIMBAL_MOTOR_COUNT];       // CAN 中断写入的反馈缓冲。
    uint32_t rx_tick_ms[GIMBAL_AXIS_COUNT];       // 三个云台电机最近一次有效反馈时间。
    uint8_t rx_valid_mask;                        // 明确区分未收首帧与合法零值。
    int16_t current[GIMBAL_MOTOR_COUNT];           // 本周期电流输出。
    float pitch_cur_speed; // Pitch angular speed from motor feedback.
    lib_lpf_t pitch_speed_lpf; // Pitch speed low-pass filter.
} app_sentry_gimbal_motor_state_t;

/** 融合 IMU 与编码器得到的云台控制姿态，控制和发送共用此快照。 */
typedef struct {
    drv_imu_t *instance;            // IMU 驱动实例。
    float yaw_cur_angle;            // 当前 yaw 角度，rad。
    float pitch_cur_angle;          // 当前 pitch 角度，rad。
    float roll_cur_angle;           // 云台 roll，rad；安装轴映射后为 -IMU pitch。
    float yaw_gyro;                 // 当前 yaw 陀螺角速度，rad/s。
    float ahrs_elapsed_s;           // 距上次完整采样的时间，避免 DMA 跳帧时少积分。
    float ahrs_last_dt_s;           // 最近一次实际融合的 dt，s。
    uint32_t ahrs_update_count;     // 仅在收到新采样并提交融合时递增。
    uint32_t ahrs_tick_ms;          // 最近一次消费新 IMU 采样并更新融合的时间。
    drv_imu_quat_t body_quat;       // 消除固定竖装角后的 IMU 四元数，机体系 -> 世界系。
    drv_imu_quat_t tx_quat;         // 最近一次 0x233 打包前的云台四元数，含关节补偿。
    drv_imu_real_t body_gyro;       // 同一机体系角速度，已扣除静态及当前算法的在线零偏。
    float pitch_joint_angle;        // pitch 关节相对机械零位的角度，rad。
    float pitch_joint_accel;        // pitch 关节角速度微分并滤波，rad/s^2。
    float pitch_prev_joint_speed;   // 上一周期 pitch 关节角速度。
    uint8_t pitch_derivative_valid; // pitch 角加速度微分历史已建立。
    lib_lpf_t pitch_accel_lpf;      // pitch 关节角加速度低通滤波器。
    float pitch_acc_bias;           // acc.y 慢变化基线，g。
    float pitch_acc_dynamic;        // 去除基线后的动态加速度，g。
    float pitch_acc_smoothed;       // 动态加速度 25 Hz 低通值，g。
    float pitch_acc_shaped;         // 平滑动态加速度经过软死区后的值，g。
    float pitch_acc_ff;             // 加速度前馈电流，原始电流值。
    float pitch_acc_deadband;       // 运行时软死区，g。
    float pitch_acc_ff_gain;        // 运行时加速度前馈系数，原始电流/g。
    float pitch_acc_ff_limit;       // 运行时前馈电流单独限幅。
    lib_lpf_t pitch_acc_bias_lpf;   // acc.y 慢基线估计。
    lib_lpf_t pitch_acc_dynamic_lpf;// 动态加速度平滑。
    lib_lpf_t yaw_gyro_lpf;         // yaw 角速度低通滤波器。
} app_sentry_gimbal_imu_state_t;

/** 云台协议输入与控制器内部目标。 */
typedef struct {
    app_sentry_gimbal_cmd_t data; // 当前控制输入。
    float yaw_tar_angle;          // yaw 目标角度，rad。
    float pitch_tar_angle;        // pitch 目标角度，rad。
} app_sentry_gimbal_command_state_t;

/** 云台转发给底盘的命令及从底盘收到的角速度。 */
typedef struct {
    float vw_cur_speed;                   // 底盘当前对地角速度，rad/s。
    float yaw_rel_angle;                  // 相机（小 yaw）相对底盘 x 轴的朝向，rad。
    uint32_t vw_rx_tick_ms;               // 最近一次 0x119 接收时间。
    uint8_t vw_is_valid;                  // 已收到有效的 0x119 角速度。
    int16_t vx_tar_speed;                 // 底盘 x 方向目标速度，mm/s。
    int16_t vy_tar_speed;                 // 底盘 y 方向目标速度，mm/s。
    int16_t vw_tar_speed;                 // 底盘目标角速度的 0.001 rad/s 协议值。
    uint8_t emergency_stop;               // 底盘急停状态。
    app_gimbal_input_source_t input_source;// 当前控制源。
    uint32_t tx_attempt_count;            // 0x111 发送尝试次数。
    uint32_t tx_accept_count;             // BSP 已接受次数（含进入软件缓冲）。
    uint32_t tx_fail_count;               // BSP 拒绝次数。
    uint32_t tx_can_skip_count;           // CAN 控制源下跳过遥控器发送的次数。
    uint8_t last_tx_result;               // 最近一次 BSP 结果：0 接受、1 失败。
} app_sentry_gimbal_chassis_state_t;

/** 需要跨控制周期保存的控制状态。 */
typedef struct {
    uint8_t is_ready; // 首次有效反馈完成目标对齐后置位。
    uint8_t feedback_is_settling;
    uint32_t feedback_ready_tick_ms; // 本次反馈连续有效的起始时间。
    uint8_t can_fire_override; // 本次 CAN 模式已由 0x129 接管发射状态。
    uint32_t can_pose_tick_ms; // 最近一次处理有效 CAN 姿态命令的时间。
    uint8_t can_pose_is_valid;
    uint8_t scan_is_active;
    int8_t scan_pitch_direction; // -1 抬头，+1 低头。
} app_sentry_gimbal_control_state_t;

/** 云台各控制环 PID 状态。 */
typedef struct {
    lib_pid_t yaw_large; // 大 yaw 绝对角度环。
    lib_pid_t yaw_small; // 小 yaw 绝对角度环。
    lib_pid_t pitch;   // pitch 角度环。
    lib_pid_t disc;    // 拨弹轮速度环。
    lib_pid_t fric_l;  // 左摩擦轮速度环。
    lib_pid_t fric_r;  // 右摩擦轮速度环。
} app_sentry_gimbal_pid_state_t;

/**
 * @brief 云台全局调试快照。
 *
 * 保留外部链接便于调试器 Watch 在任意断点查看；不在头文件声明，
 * 不作为模块间访问接口。
 */
typedef struct {
    app_sentry_gimbal_motor_state_t motor;
    app_sentry_gimbal_imu_state_t imu;
    app_sentry_gimbal_command_state_t command;
    app_sentry_gimbal_chassis_state_t chassis;
    app_sentry_gimbal_control_state_t control;
    app_sentry_gimbal_pid_state_t pid;
} app_sentry_gimbal_debug_t;

app_sentry_gimbal_debug_t app_sentry_gimbal_debug = {
    .chassis = {
        .input_source = APP_GIMBAL_INPUT_CAN,
    },
};

#define s_motor_state  app_sentry_gimbal_debug.motor
#define s_imu_state    app_sentry_gimbal_debug.imu
#define s_command_state app_sentry_gimbal_debug.command
#define s_chassis_state app_sentry_gimbal_debug.chassis
#define s_control_state app_sentry_gimbal_debug.control
#define s_pid_yaw_large app_sentry_gimbal_debug.pid.yaw_large
#define s_pid_yaw_small app_sentry_gimbal_debug.pid.yaw_small
#define s_pid_pitch    app_sentry_gimbal_debug.pid.pitch
#define s_pid_disc     app_sentry_gimbal_debug.pid.disc
#define s_pid_fric_l   app_sentry_gimbal_debug.pid.fric_l
#define s_pid_fric_r   app_sentry_gimbal_debug.pid.fric_r

static void gimbal_vofa_send(void);
static void pid_init_all(void);//所有PID实例初始化
static void gimbal_reset(void);//云台PID清零
static void launcher_reset(void);//发射机构PID清零

static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input);
static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx,
                                 uint8_t updated_mask);
static void gimbal_scan_update(void);

static float gimbal_coord_trans(void);
static void gimbal_send_chassis_cmd(void);

static void imu_fusion(void);

static void yaw_pid_control(void);
static void pitch_control(void);
static void launch_control(void);

static void gimbal_can2_tx(void);
static void launcher_can2_tx(void);
static void gimbal_can1_tx(void);

static void on_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
void app_sentry_gimbal_init(drv_imu_t *imu)
{
    s_imu_state.instance = imu;
    s_imu_state.ahrs_elapsed_s = 0.0f;
    s_imu_state.ahrs_last_dt_s = 0.0f;
    s_imu_state.ahrs_update_count = 0U;
    s_imu_state.ahrs_tick_ms = 0U;
    s_motor_state.rx_valid_mask = 0U;
    s_control_state.can_pose_is_valid = 0U;
    s_control_state.scan_is_active = 0U;
    if (imu) {
        /* 先消除板子竖装的固定旋转，再沿用等效水平 IMU 的云台轴映射。 */
        drv_imu_set_mounting(imu, IMU_MOUNT_ROLL_DEG, IMU_MOUNT_PITCH_DEG, IMU_MOUNT_YAW_DEG);
        /* 云台轴映射依赖此分解约定；不使用驱动的其他欧拉角模式。 */
        drv_imu_set_euler_mode(imu, DRV_IMU_EULER_XYZ);
        drv_imu_get_body_quat(imu, &s_imu_state.body_quat);
    }
    pid_init_all();
    gimbal_reset();

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_L_YAW,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_S_YAW,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_PITCH,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_F1,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_D1,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_F2,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK,
                                 on_chassis_omega_feedback);

}

void app_gimbal_ahrs_update(float dt)
{
    uint8_t has_new_snapshot;

    if (!s_imu_state.instance || !isfinite(dt) || dt <= 0.0f || dt > 0.1f) {
        return;
    }
    s_imu_state.ahrs_elapsed_s = fminf(s_imu_state.ahrs_elapsed_s + dt, 0.1f);
    has_new_snapshot = drv_imu_port_snapshot_update(s_imu_state.instance);
    drv_imu_port_async_start();
    if (has_new_snapshot) {
        drv_imu_data_convert(s_imu_state.instance);
        drv_imu_update(s_imu_state.instance, s_imu_state.ahrs_elapsed_s);
        s_imu_state.ahrs_last_dt_s = s_imu_state.ahrs_elapsed_s;
        s_imu_state.ahrs_update_count++;
        s_imu_state.ahrs_tick_ms = bsp_tim_get_tick_ms();
        s_imu_state.ahrs_elapsed_s = 0.0f;
    }
}

void app_sentry_gimbal_ctrl_1khz(void)
{
    app_gimbal_dbus_input_t input = {
        /* 未连接 DBUS 时默认使用 CAN，姿态命令空闲时允许自动巡扫。 */
        .source = APP_GIMBAL_INPUT_CAN,
    };
    uint8_t feedback_is_ready = 1U;
    uint32_t now;
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    now = bsp_tim_get_tick_ms();
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i) {
        if ((s_motor_state.rx_valid_mask & (1U << i)) == 0U
            || (uint32_t)(now - s_motor_state.rx_tick_ms[i])
                >= SENTRY_GIMBAL_FEEDBACK_TIMEOUT_MS) {
            feedback_is_ready = 0U;
            break;
        }
    }
    __set_PRIMASK(irq_state);

    (void)app_gimbal_comm_dbus_rx(&input);
    /* 首帧尚未到达的全零结构体也是有限数，不能据此使能闭环。 */
    if (!feedback_is_ready || !s_imu_state.instance
        || s_imu_state.ahrs_last_dt_s <= 0.0f
        || (uint32_t)(now - s_imu_state.ahrs_tick_ms)
            >= SENTRY_GIMBAL_FEEDBACK_TIMEOUT_MS) {
        gimbal_reset();
        gimbal_can2_tx();
        return;
    }

    s_motor_state.pitch_cur_speed = lib_lpf_update(
        &s_motor_state.pitch_speed_lpf,
        lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_PITCH].speed),
        0.001f);

    imu_fusion();

    if (!isfinite(s_imu_state.yaw_cur_angle) || !isfinite(s_imu_state.pitch_cur_angle)
        || !isfinite(s_imu_state.yaw_gyro) || !isfinite(s_motor_state.pitch_cur_speed)) {
        gimbal_reset();

        gimbal_can2_tx();
        return;
    }
    /* 新鲜采样不等于滤波历史已建立；VQF 启动平均期间只更新姿态，不使能电机。 */
    if (!drv_imu_filter_is_ready(s_imu_state.instance)) {
        gimbal_reset();
        gimbal_can2_tx();
        return;
    }
    if (input.source == APP_GIMBAL_INPUT_ESTOP) {
        gimbal_update_cmd(&input);
        gimbal_reset();
        gimbal_can2_tx();
        return;
    }
    if (!s_control_state.is_ready) {
        if (!s_control_state.feedback_is_settling) {
            s_control_state.feedback_is_settling = 1U;
            s_control_state.feedback_ready_tick_ms = now;
        }
        if ((uint32_t)(now - s_control_state.feedback_ready_tick_ms)
            < SENTRY_GIMBAL_STARTUP_WAIT_MS) {
            gimbal_can2_tx();
            return;
        }
        s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
        s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        s_control_state.is_ready = 1U;
        /* 与急停恢复相同：先对齐真实姿态，下个周期再接收命令并输出。 */
        gimbal_can2_tx();
        return;
    }
    gimbal_update_cmd(&input);

    yaw_pid_control();

    pitch_control();

    gimbal_can2_tx();
}

void app_sentry_launcher_ctrl_200hz(void)
{
    if (!s_control_state.is_ready || s_chassis_state.emergency_stop) {
        launcher_reset();

    } else {
        launch_control();
    }
    launcher_can2_tx();

    gimbal_vofa_send();
}

void app_sentry_gimbal_status_tx_200hz(void)
{
    /* 姿态反馈由固定周期调度，不依赖小电脑/CAN 控制源或发射机构状态。 */
    gimbal_can1_tx();
}

void app_sentry_gimbal_chassis_tx_200hz(void)
{
    gimbal_send_chassis_cmd();
}

static void on_motor_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    if (!data || len != 8U) {
        return;
    }
    int motor_index = -1;
    switch (std_id) {
    case SENTRY_CAN_GIMBAL_L_YAW:  motor_index = MOTOR_YAW_L;   break;
    case SENTRY_CAN_GIMBAL_S_YAW:  motor_index = MOTOR_YAW_S;   break;
    case SENTRY_CAN_GIMBAL_PITCH:  motor_index = MOTOR_PITCH;   break;
    case SENTRY_CAN_GIMBAL_F1:  motor_index = MOTOR_FRIC_L;  break;
    case SENTRY_CAN_GIMBAL_D1:  motor_index = MOTOR_DISC;    break;
    case SENTRY_CAN_GIMBAL_F2:  motor_index = MOTOR_FRIC_R;  break;
    default: return;
    }
    if (motor_index >= 0 && motor_index < GIMBAL_MOTOR_COUNT) {
        uint16_t angle = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
        if (angle >= 8192U) {
            return;
        }
        drv_motor_solve_dji(data, &s_motor_state.rx[motor_index]);
        if (motor_index < GIMBAL_AXIS_COUNT) {
            s_motor_state.rx_tick_ms[motor_index] = bsp_tim_get_tick_ms();
            s_motor_state.rx_valid_mask |= (uint8_t)(1U << motor_index);
        }
    }
}

static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    float omega_z;

    (void)std_id;
    if (!data || len < sizeof(omega_z)) {
        return;
    }
    memcpy(&omega_z, data, sizeof(omega_z));
    if (!isfinite(omega_z)
        || fabsf(omega_z) > SENTRY_CHASSIS_VW_TAR_SPEED_MAX * 1.5f) {
        return;
    }

    s_chassis_state.vw_cur_speed = omega_z;
    s_chassis_state.vw_rx_tick_ms = bsp_tim_get_tick_ms();
    s_chassis_state.vw_is_valid = 1U;
}

/* 私有函数实现。 */

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_yaw_large, 29000.0f, 2000.0f, 7300.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_yaw_small, 32000.0f, 2100.0f, 2100.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_pitch, 26000.0f, 1000.0f, 900.0f, 3800.0f, 0.0f,
                 5000.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    //p100 d0.3    p40 i250
    /* Disc feedforward: unity gain, with a +/-3000 current-command limit. */
    lib_pid_init(&s_pid_disc,85.0f, 150.0f, 0.0f, 1.0f, 0.0f,
                 3000.0f, 0.0f, DRV_MOTOR_M2006_CURRENT_MIN, DRV_MOTOR_M2006_CURRENT_MAX, 7000.0f);
    lib_pid_init(&s_pid_fric_l, 15.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_r, 15.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);

    lib_lpf_init(&s_imu_state.yaw_gyro_lpf, 25.0f);
    lib_lpf_init(&s_motor_state.pitch_speed_lpf, 25.0f);
    lib_lpf_init(&s_imu_state.pitch_accel_lpf, 15.0f);
    lib_lpf_init(&s_imu_state.pitch_acc_bias_lpf, PITCH_ACC_BIAS_CUTOFF_HZ);
    lib_lpf_init(&s_imu_state.pitch_acc_dynamic_lpf, PITCH_ACC_DYNAMIC_CUTOFF_HZ);
    s_imu_state.pitch_acc_deadband = PITCH_ACC_DEADBAND_G;
    s_imu_state.pitch_acc_ff_gain = PITCH_ACC_FF_GAIN;
    s_imu_state.pitch_acc_ff_limit = PITCH_ACC_FF_LIMIT;
}
static void launcher_reset(void)
{
    s_motor_state.current[MOTOR_FRIC_L] = 0;
    s_motor_state.current[MOTOR_FRIC_R] = 0;
    s_motor_state.current[MOTOR_DISC] = 0;
    lib_pid_reset(&s_pid_disc);
    lib_pid_reset(&s_pid_fric_l);
    lib_pid_reset(&s_pid_fric_r);
}

static void gimbal_reset(void)
{
    memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
    s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    s_command_state.data.yaw_tar_speed = 0.0f;
    s_command_state.data.pitch_tar_speed = 0.0f;
    s_chassis_state.vx_tar_speed = 0;
    s_chassis_state.vy_tar_speed = 0;
    s_chassis_state.vw_tar_speed = 0;
    s_chassis_state.input_source = APP_GIMBAL_INPUT_ESTOP;
    s_chassis_state.emergency_stop = 1U;
    s_control_state.is_ready = 0U;
    s_imu_state.pitch_derivative_valid = 0U;
    s_imu_state.pitch_accel_lpf.initialized = 0U;
    s_imu_state.pitch_acc_bias_lpf.initialized = 0U;
    s_imu_state.pitch_acc_dynamic_lpf.initialized = 0U;
    s_imu_state.pitch_acc_bias = 0.0f;
    s_imu_state.pitch_acc_dynamic = 0.0f;
    s_imu_state.pitch_acc_smoothed = 0.0f;
    s_imu_state.pitch_acc_shaped = 0.0f;
    s_imu_state.pitch_acc_ff = 0.0f;
    s_control_state.feedback_is_settling = 0U;
    s_control_state.can_fire_override = 0U;
    s_control_state.can_pose_is_valid = 0U;
    s_control_state.scan_is_active = 0U;
    lib_pid_reset(&s_pid_yaw_large);
    lib_pid_reset(&s_pid_yaw_small);
    lib_pid_reset(&s_pid_pitch);
    lib_lpf_init(&s_imu_state.yaw_gyro_lpf, 25.0f);
    lib_lpf_init(&s_motor_state.pitch_speed_lpf, 25.0f);
    launcher_reset();
}
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input)
{
    app_gimbal_comm_rx_t can_rx;
    uint8_t updated_mask = 0U;

    if (!input) {
        return;
    }

    /* 非 CAN 模式也清空更新标志，防止切换控制源时执行旧指令。 */
    (void)app_gimbal_comm_read_rx(&can_rx, &updated_mask);

    if (s_chassis_state.input_source != input->source) {
        /* 切换后等待新 CAN 帧；遥控模式则立即按当前 S1 挡位控制发射。 */
        updated_mask = 0U;
        s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
        s_control_state.can_fire_override = 0U;
        s_control_state.can_pose_is_valid = 0U;
        s_control_state.scan_is_active = 0U;
        s_command_state.data.yaw_tar_speed = 0.0f;
        s_command_state.data.pitch_tar_speed = 0.0f;
        s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
        s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        launcher_reset();
    }
    s_chassis_state.input_source = input->source;
    s_chassis_state.emergency_stop = (input->source == APP_GIMBAL_INPUT_ESTOP);
    if (s_chassis_state.emergency_stop) {
        s_chassis_state.vx_tar_speed = 0;
        s_chassis_state.vy_tar_speed = 0;
        s_chassis_state.vw_tar_speed = 0;
        return;
    }
    if (input->source == APP_GIMBAL_INPUT_CAN) {
        const uint8_t pose_mask = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
        s_command_state.data.yaw_tar_speed = 0.0f;
        s_command_state.data.pitch_tar_speed = 0.0f;
        if ((updated_mask & pose_mask) != 0U) {
            /* 增量命令从当前实际姿态接管，不能继续累加巡扫的跟踪误差。 */
            if (s_control_state.scan_is_active) {
                s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
                s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
                lib_pid_reset(&s_pid_yaw_large);
                lib_pid_reset(&s_pid_yaw_small);
                lib_pid_reset(&s_pid_pitch);
            }
            s_control_state.can_pose_tick_ms = bsp_tim_get_tick_ms();
            s_control_state.can_pose_is_valid = 1U;
            s_control_state.scan_is_active = 0U;
        }
        gimbal_apply_can_cmd(&can_rx, updated_mask);
        if (SENTRY_GIMBAL_SCAN_ENABLE
            && (!s_control_state.can_pose_is_valid
                || (uint32_t)(bsp_tim_get_tick_ms() - s_control_state.can_pose_tick_ms)
                    >= SENTRY_GIMBAL_CAN_POSE_TIMEOUT_MS)) {
            gimbal_scan_update();
        }
        return;
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.fire = input->launcher_mode;
    s_command_state.data.yaw_tar_speed = input->yaw_rate_norm * SENTRY_GIMBAL_YAW_TAR_SPEED_MAX;
    s_command_state.data.pitch_tar_speed = input->pitch_rate_norm * SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX;

    float vx_gimbal_tar_speed = input->chassis_vx_norm * SENTRY_CHASSIS_VX_TAR_SPEED_MAX;
    float vy_gimbal_tar_speed = input->chassis_vy_norm * SENTRY_CHASSIS_VY_TAR_SPEED_MAX;
    float camera_yaw_rel_angle = gimbal_coord_trans();
    float vx_chassis_tar_speed;
    float vy_chassis_tar_speed;
    app_sentry_gimbal_velocity_to_chassis(vx_gimbal_tar_speed, vy_gimbal_tar_speed,
                                          &vx_chassis_tar_speed, &vy_chassis_tar_speed);
    s_chassis_state.yaw_rel_angle = camera_yaw_rel_angle;
    s_chassis_state.vx_tar_speed = (int16_t)lroundf(vx_chassis_tar_speed);
    s_chassis_state.vy_tar_speed = (int16_t)lroundf(vy_chassis_tar_speed);
    s_chassis_state.vw_tar_speed = (int16_t)lroundf(input->chassis_omega_norm
        * SENTRY_CHASSIS_VW_TAR_SPEED_MAX / APP_CHASSIS_OMEGA_RAD_S_PER_LSB);

    s_command_state.yaw_tar_angle += s_command_state.data.yaw_tar_speed * 0.001f;
    s_command_state.pitch_tar_angle += s_command_state.data.pitch_tar_speed * 0.001f;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle);
}

static void gimbal_scan_update(void)
{
    if (!s_control_state.scan_is_active) {
        s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
        s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
        s_control_state.scan_pitch_direction = -1;
        s_control_state.scan_is_active = 1U;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        return; /* 切入时只对齐目标，下个控制周期再推进。 */
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.yaw_tar_speed = SENTRY_GIMBAL_SCAN_YAW_SPEED;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle
        + SENTRY_GIMBAL_SCAN_YAW_SPEED * 0.001f);

    float joint_angle = lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_PITCH].angle, SENTRY_GIMBAL_PITCH_ZERO);
    float platform_pitch = s_imu_state.pitch_cur_angle - joint_angle;
    float pitch_min = fmaxf(SENTRY_GIMBAL_SCAN_PITCH_MIN,
                            platform_pitch + SENTRY_GIMBAL_PITCH_JOINT_MIN);
    float pitch_max = fminf(SENTRY_GIMBAL_SCAN_PITCH_MAX,
                            platform_pitch + SENTRY_GIMBAL_PITCH_JOINT_MAX);
    if (pitch_min >= pitch_max) {
        /* 对地与关节范围没有交集时，交给 pitch_control 的现有限位处理。 */
        s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
        return;
    }

    float pitch_target = s_command_state.pitch_tar_angle
        + (float)s_control_state.scan_pitch_direction * SENTRY_GIMBAL_SCAN_PITCH_SPEED * 0.001f;
    if (pitch_target <= pitch_min) {
        pitch_target = pitch_min;
        s_control_state.scan_pitch_direction = 1;
    } else if (pitch_target >= pitch_max) {
        pitch_target = pitch_max;
        s_control_state.scan_pitch_direction = -1;
    }
    s_command_state.pitch_tar_angle = pitch_target;
    s_command_state.data.pitch_tar_speed =
        (float)s_control_state.scan_pitch_direction * SENTRY_GIMBAL_SCAN_PITCH_SPEED;
}

static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx,
                                 uint8_t updated_mask)
{
    if (!rx) {
        return;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(
            s_command_state.yaw_tar_angle + rx->speed_no_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_no_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(
            s_command_state.yaw_tar_angle + rx->speed_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_no_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_no_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_shoot.pitch_abs;
    }
    if (!s_control_state.can_fire_override
        && (updated_mask & (APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT
                            | APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT))) {
        s_command_state.data.fire = APP_SENTRY_FIRE_ON;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SHOOT) {
        /* 显式发射指令保持有效，直到再次收到 0x129 或退出当前 CAN 模式。 */
        s_control_state.can_fire_override = 1U;
        if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0x00U) s_command_state.data.fire = APP_SENTRY_FIRE_ON;
        else if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_FIR;
        else if (rx->shoot.shoot_switch == 0x00U && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_REVERSE;
        else s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    }
}

static float gimbal_coord_trans(void)
{
    float large_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_L_YAW_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_S_YAW_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(
        -SENTRY_GIMBAL_L_YAW_DIR * large_yaw_cur_angle
        - SENTRY_GIMBAL_S_YAW_DIR * small_yaw_cur_angle);
}

void app_sentry_gimbal_velocity_to_chassis(float vx_gimbal, float vy_gimbal,
                                           float *vx_chassis, float *vy_chassis)
{
    if (!vx_chassis || !vy_chassis) {
        return;
    }

    float yaw_rel = gimbal_coord_trans();
    float cos_yaw = cosf(yaw_rel);
    float sin_yaw = sinf(yaw_rel);
    /* 两个平移轴的实车方向都与命令相反，统一反转转换后的平面速度。 */
    *vx_chassis = -(cos_yaw * vx_gimbal + sin_yaw * vy_gimbal);
    *vy_chassis = -(sin_yaw * vx_gimbal - cos_yaw * vy_gimbal);
}

static void gimbal_send_chassis_cmd(void)
{
    uint8_t data[8] = {0};
    if (s_chassis_state.input_source == APP_GIMBAL_INPUT_CAN) {
        s_chassis_state.tx_can_skip_count++;
        return;
    }
    int16_t vx = s_chassis_state.emergency_stop ? SENTRY_CHASSIS_ESTOP
                                           : s_chassis_state.vx_tar_speed;

    memcpy(data, &vx, sizeof(vx));
    memcpy(data + 2, &s_chassis_state.vy_tar_speed, sizeof(s_chassis_state.vy_tar_speed));
    memcpy(data + 4, &s_chassis_state.vw_tar_speed, sizeof(s_chassis_state.vw_tar_speed));
    s_chassis_state.tx_attempt_count++;
    s_chassis_state.last_tx_result = bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
    if (s_chassis_state.last_tx_result == 0U) {
        s_chassis_state.tx_accept_count++;
    } else {
        s_chassis_state.tx_fail_count++;
    }
}

static void imu_fusion(void)
{
    if (!s_imu_state.instance) {
        return;
    }
    drv_imu_quat_to_euler(s_imu_state.instance);
    drv_imu_get_body_quat(s_imu_state.instance, &s_imu_state.body_quat);

    s_imu_state.yaw_cur_angle = lib_deg_to_rad(s_imu_state.instance->euler.yaw);

    float pitch_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_PITCH].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_PITCH_ZERO, LIB_ENC13_TO_RAD));
    s_imu_state.pitch_joint_angle = pitch_cur_angle;
    s_imu_state.pitch_cur_angle = lib_deg_to_rad(s_imu_state.instance->euler.roll) + pitch_cur_angle;
    /* 此处 euler 已消除固定竖装角：等效水平 IMU x 对应云台 y，y 对应云台 -x。
     * 默认 IMU 欧拉角分解映射到云台 ZYX 后，roll = -IMU pitch。
     * pitch 保留平台倾角 + 俯仰关节编码器的原有补偿模型。 */
    s_imu_state.roll_cur_angle = -lib_deg_to_rad(s_imu_state.instance->euler.pitch);

    drv_imu_get_body_gyro(s_imu_state.instance, &s_imu_state.body_gyro);
    s_imu_state.yaw_gyro = lib_lpf_update(&s_imu_state.yaw_gyro_lpf, s_imu_state.body_gyro.z, 0.001f);

    if (!s_imu_state.pitch_derivative_valid) {
        s_imu_state.pitch_prev_joint_speed = s_motor_state.pitch_cur_speed;
        s_imu_state.pitch_joint_accel = 0.0f;
        s_imu_state.pitch_derivative_valid = 1U;
    } else {
        const float accel_raw =
            (s_motor_state.pitch_cur_speed - s_imu_state.pitch_prev_joint_speed) / 0.001f;
        s_imu_state.pitch_joint_accel = lib_lpf_update(
            &s_imu_state.pitch_accel_lpf, accel_raw, 0.001f);
        s_imu_state.pitch_prev_joint_speed = s_motor_state.pitch_cur_speed;
    }

    const float acc_y_raw = s_imu_state.instance->acc.y;
    s_imu_state.pitch_acc_bias = lib_lpf_update(
        &s_imu_state.pitch_acc_bias_lpf, acc_y_raw, 0.001f);
    s_imu_state.pitch_acc_dynamic = acc_y_raw - s_imu_state.pitch_acc_bias;
    s_imu_state.pitch_acc_smoothed = lib_lpf_update(
        &s_imu_state.pitch_acc_dynamic_lpf, s_imu_state.pitch_acc_dynamic, 0.001f);

    const float acc_deadband = isfinite(s_imu_state.pitch_acc_deadband)
        ? lib_clamp(s_imu_state.pitch_acc_deadband, 0.0f, 1.0f)
        : PITCH_ACC_DEADBAND_G;
    if (fabsf(s_imu_state.pitch_acc_smoothed) <= acc_deadband) {
        s_imu_state.pitch_acc_shaped = 0.0f;
    } else {
        s_imu_state.pitch_acc_shaped = copysignf(
            fabsf(s_imu_state.pitch_acc_smoothed) - acc_deadband,
            s_imu_state.pitch_acc_smoothed);
    }

    const float acc_ff_gain = isfinite(s_imu_state.pitch_acc_ff_gain)
        ? lib_clamp(s_imu_state.pitch_acc_ff_gain, -10000.0f, 10000.0f)
        : 0.0f;
    const float acc_ff_limit = isfinite(s_imu_state.pitch_acc_ff_limit)
        ? lib_clamp(s_imu_state.pitch_acc_ff_limit, 0.0f, 7000.0f)
        : 0.0f;
    s_imu_state.pitch_acc_ff = lib_clamp(
        acc_ff_gain * s_imu_state.pitch_acc_shaped,
        -acc_ff_limit, acc_ff_limit);
}

static void yaw_pid_control(void)
{
    float yaw_cur_angle = s_imu_state.yaw_cur_angle;
    float small_yaw_cur_angle = SENTRY_GIMBAL_S_YAW_DIR * lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_YAW_S].angle, SENTRY_GIMBAL_S_YAW_ZERO);
    float small_yaw_tar_angle = yaw_cur_angle + lib_get_shortest_path(
        s_command_state.yaw_tar_angle, yaw_cur_angle);
    float large_yaw_tar_angle = yaw_cur_angle + lib_get_shortest_path(
        lib_rad_norm(s_command_state.yaw_tar_angle + small_yaw_cur_angle), yaw_cur_angle);
    float large_yaw_cur_speed = SENTRY_GIMBAL_L_YAW_DIR * lib_rpm_to_rad_s(
        (float)s_motor_state.feedback[MOTOR_YAW_L].speed);
    float small_yaw_cur_speed = SENTRY_GIMBAL_S_YAW_DIR * lib_rpm_to_rad_s(
        (float)s_motor_state.feedback[MOTOR_YAW_S].speed);
    float chassis_yaw_speed = 0.0f;
    float large_yaw_ground_speed;
    float small_yaw_ground_speed;
    float chassis_yaw_feedback;
    uint32_t chassis_omega_rx_tick_ms;
    uint32_t irq_state;
    uint8_t has_chassis_omega;
    uint8_t chassis_omega_is_valid;
    float small_yaw_current;
    float limit_weight = 0.0f;

    irq_state = __get_PRIMASK();
    __disable_irq();
    chassis_yaw_feedback = s_chassis_state.vw_cur_speed;
    chassis_omega_rx_tick_ms = s_chassis_state.vw_rx_tick_ms;
    chassis_omega_is_valid = s_chassis_state.vw_is_valid;
    __set_PRIMASK(irq_state);

    has_chassis_omega = chassis_omega_is_valid
        && (uint32_t)(bsp_tim_get_tick_ms() - chassis_omega_rx_tick_ms)
            <= GIMBAL_CHASSIS_OMEGA_TIMEOUT_MS;
    if (has_chassis_omega) {
        chassis_yaw_speed = chassis_yaw_feedback;
    }
    if (has_chassis_omega) {
        large_yaw_ground_speed = chassis_yaw_speed + large_yaw_cur_speed;
        small_yaw_ground_speed = large_yaw_ground_speed + small_yaw_cur_speed;
    } else {
        /* 没有可信底盘速度时，维持原来的相对速度阻尼。 */
        large_yaw_ground_speed = large_yaw_cur_speed;
        small_yaw_ground_speed = small_yaw_cur_speed;
    }

    small_yaw_current = lib_pid_pos_calc(
        &s_pid_yaw_small, small_yaw_tar_angle, yaw_cur_angle,
        0.0f, 0.0f, small_yaw_ground_speed, 0.001f);
    s_motor_state.current[MOTOR_YAW_L] = (int16_t)(
        SENTRY_GIMBAL_L_YAW_DIR * lib_pid_pos_calc(
            &s_pid_yaw_large, large_yaw_tar_angle, yaw_cur_angle,
            0.0f, 0.0f, large_yaw_ground_speed, 0.001f));

    if (small_yaw_cur_angle > SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE) {
        limit_weight = lib_remap_clamp(small_yaw_cur_angle,
            SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE,
            SENTRY_GIMBAL_S_YAW_LIMIT, 0.0f, 1.0f);
        if (small_yaw_current > 0.0f) {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed > 0.0f) {
            small_yaw_current -= SENTRY_GIMBAL_S_YAW_BRAKE_GAIN
                               * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    } else if (small_yaw_cur_angle < -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE) {
        limit_weight = lib_remap_clamp(small_yaw_cur_angle,
            -SENTRY_GIMBAL_S_YAW_LIMIT,
            -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE, 1.0f, 0.0f);
        if (small_yaw_current < 0.0f) {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed < 0.0f) {
            small_yaw_current -= SENTRY_GIMBAL_S_YAW_BRAKE_GAIN
                               * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    }

    small_yaw_current = lib_clamp(small_yaw_current,
        DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
    s_pid_yaw_small.out = small_yaw_current;
    s_motor_state.current[MOTOR_YAW_S] = (int16_t)(
        SENTRY_GIMBAL_S_YAW_DIR * small_yaw_current);
}
static void pitch_control(void)
{
    float pitch_cur_angle = s_imu_state.pitch_cur_angle;
    float pitch_joint_angle = lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_PITCH].angle, SENTRY_GIMBAL_PITCH_ZERO);
    float platform_pitch = pitch_cur_angle - pitch_joint_angle;
    float pitch_tar_angle;
    float pitch_current;

    s_command_state.pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
        SENTRY_GIMBAL_PITCH_MIN, SENTRY_GIMBAL_PITCH_MAX);
    /* 将关节可达范围换到对地角度，避免平台倾斜时目标越过机械边界。 */
    pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
        platform_pitch + SENTRY_GIMBAL_PITCH_JOINT_MIN,
        platform_pitch + SENTRY_GIMBAL_PITCH_JOINT_MAX);

    pitch_current = lib_pid_pos_calc(
        &s_pid_pitch, pitch_tar_angle, pitch_cur_angle,
        cosf(pitch_cur_angle), 0.0f, s_motor_state.pitch_cur_speed, 0.001f);

    pitch_current = lib_clamp(
        pitch_current + s_imu_state.pitch_acc_ff,
        DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);

    /* 机械软限位只看关节角；仅削减继续向外的电流，保留向内恢复能力。 */
    if (pitch_current > 0.0f
        && pitch_joint_angle > SENTRY_GIMBAL_PITCH_JOINT_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE) {
        pitch_current *= lib_remap_clamp(pitch_joint_angle,
            SENTRY_GIMBAL_PITCH_JOINT_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE,
            SENTRY_GIMBAL_PITCH_JOINT_MAX, 1.0f, 0.0f);
        s_pid_pitch.integral = 0.0f;
    } else if (pitch_current < 0.0f
               && pitch_joint_angle < SENTRY_GIMBAL_PITCH_JOINT_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE) {
        pitch_current *= lib_remap_clamp(pitch_joint_angle,
            SENTRY_GIMBAL_PITCH_JOINT_MIN,
            SENTRY_GIMBAL_PITCH_JOINT_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE, 0.0f, 1.0f);
        s_pid_pitch.integral = 0.0f;
    }

    s_pid_pitch.out = pitch_current;
    s_motor_state.current[MOTOR_PITCH] = (int16_t)pitch_current;
}

static void launch_control(void)
{
    switch (s_command_state.data.fire) {
    case APP_SENTRY_FIRE_OFF:
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);

        break;
    default:
        launcher_reset();

        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_ff_calc(
            &s_pid_disc, SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
            3000.0f, 0.0f, 0.005f);
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_REVERSE:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_ff_calc(
            &s_pid_disc, -SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
            -3000.0f, 0.0f, 0.005f);
        break;

    }
}

static void gimbal_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x1FE: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[MOTOR_YAW_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[MOTOR_YAW_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[MOTOR_YAW_S]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[MOTOR_YAW_S]);
    frame[4] = LIB_HI_BYTE(s_motor_state.current[MOTOR_PITCH]);
    frame[5] = LIB_LO_BYTE(s_motor_state.current[MOTOR_PITCH]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_YAW, frame);
}

static void launcher_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x200: [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[MOTOR_FRIC_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[MOTOR_FRIC_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[MOTOR_DISC]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[MOTOR_DISC]);
    frame[4] = LIB_HI_BYTE(s_motor_state.current[MOTOR_FRIC_R]);
    frame[5] = LIB_LO_BYTE(s_motor_state.current[MOTOR_FRIC_R]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_LAUNCH, frame);
}

static void gimbal_can1_tx(void)
{
    const float yaw = s_imu_state.yaw_cur_angle;
    const float pitch = s_imu_state.pitch_cur_angle;
    const float roll = s_imu_state.roll_cur_angle;
    drv_imu_quat_t control_quat;

    if (!s_imu_state.instance || !isfinite(yaw) || !isfinite(pitch) || !isfinite(roll)) {
        return;
    }

    /* 用控制反馈的同一组角度重建云台到世界的四元数。
     * ZYX：q = qz(yaw) * qy(pitch) * qx(roll)，标量在前。
     * 不直接发送传感器四元数：它不含 pitch 关节编码器补偿。 */
    const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    control_quat.q0 = cy * cp * cr + sy * sp * sr;
    control_quat.q1 = cy * cp * sr - sy * sp * cr;
    control_quat.q2 = cy * sp * cr + sy * cp * sr;
    control_quat.q3 = sy * cp * cr - cy * sp * sr;
    s_imu_state.tx_quat = control_quat;

    /* 0x122：发送 yaw/pitch 角速度，单位 rad/s。 */
    app_gimbal_comm_gyro_tx(
        s_imu_state.yaw_gyro,
        s_motor_state.pitch_cur_speed);

    /* 0x124：发送 yaw/pitch 当前角度，单位 rad。 */
    app_gimbal_comm_angle_tx(
        yaw,
        pitch);

    app_gimbal_comm_quat_tx(
        (int16_t)(control_quat.q0 * 30000.0f), (int16_t)(control_quat.q1 * 30000.0f),
        (int16_t)(control_quat.q2 * 30000.0f), (int16_t)(control_quat.q3 * 30000.0f));

    /* 0x139 与 0x124、PID 使用完全相同的 yaw/pitch，仅单位改为度。 */
    app_gimbal_comm_fused_yaw_pitch_tx(
        lib_rad_to_deg(yaw),
        lib_rad_to_deg(pitch));
}

static void gimbal_vofa_send(void)
{
    /* 9 通道 pitch 加速度滤波/死区测试页：
     * I0/I1/I2/I3 为原始 acc.y、慢基线、动态值、平滑动态值，g；
     * I4 为 0.05g 软死区后的动态值，g；
     * I5 为 pitch 控制角/deg，I6 为关节角速度/rad/s，
     * I7/I8 为命令/反馈电流。 */
    float data[DRV_VOFA_CHANNEL_COUNT] = {0};
    if (s_imu_state.instance) {
        data[0] = s_imu_state.instance->acc.y;
    }
    data[1] = s_imu_state.pitch_acc_bias;
    data[2] = s_imu_state.pitch_acc_dynamic;
    data[3] = s_imu_state.pitch_acc_smoothed;
    data[4] = s_imu_state.pitch_acc_shaped;
    data[5] = lib_rad_to_deg(s_imu_state.pitch_cur_angle);
    data[6] = s_motor_state.pitch_cur_speed;
    data[7] = (float)s_motor_state.current[MOTOR_PITCH];
    data[8] = (float)s_motor_state.feedback[MOTOR_PITCH].current;
    (void)drv_vofa_port_send(data);
}
