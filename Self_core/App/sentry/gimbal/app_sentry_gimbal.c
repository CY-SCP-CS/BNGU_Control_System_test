/**
 * @file    app_sentry_gimbal.c
 * @brief   哨兵云台控制实现：双 yaw VMC、pitch 控制与发射机构控制
 * @note    AHRS、yaw 和 pitch 按 1 kHz 调度；发射机构按 200 Hz 调度。
 *          CAN2 发送本板电机电流，CAN1 交换板间命令和云台状态。
 */
#include "app_sentry_gimbal.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"

#include "drv_imu.h"
#include "drv_motor.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include <math.h>
#include <string.h>

#define GIMBAL_MOTOR_COUNT            6U
#define GIMBAL_SMALL_YAW_TRACK_SHARE  0.8f // 公共 yaw 跟踪电流由小 yaw 承担的最大比例。

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
    int16_t current[GIMBAL_MOTOR_COUNT];           // 本周期电流输出。
} app_sentry_gimbal_motor_state_t;

/** IMU 角度、角速度及其低通滤波状态。 */
typedef struct {
    drv_imu_t *instance;            // IMU 驱动实例。
    float yaw_cur_angle;            // 当前 yaw 角度，rad。
    float pitch_cur_angle;          // 当前 pitch 角度，rad。
    float yaw_gyro;                 // 当前 yaw 陀螺角速度，rad/s。
    float pitch_gyro;               // 当前 pitch 陀螺角速度，rad/s。
    lib_lpf_t yaw_gyro_lpf;         // yaw 角速度低通滤波器。
    lib_lpf_t pitch_gyro_lpf;       // pitch 角速度低通滤波器。
} app_sentry_gimbal_imu_state_t;

/** 云台协议输入与控制器内部目标。 */
typedef struct {
    app_sentry_gimbal_cmd_t data; // 当前控制输入。
    float yaw_tar_angle;          // yaw 目标角度，rad。
    float pitch_tar_angle;        // pitch 目标角度，rad。
    float yaw_tar_speed;          // yaw 目标角速度，rad/s。
} app_sentry_gimbal_command_state_t;

/** 云台转发给底盘的命令及从底盘收到的角速度。 */
typedef struct {
    float vw_cur_speed;                   // 底盘当前角速度，rad/s。
    int16_t vx_tar_speed;                 // 底盘 x 方向目标速度，mm/s。
    int16_t vy_tar_speed;                 // 底盘 y 方向目标速度，mm/s。
    int16_t vw_tar_speed;                 // 底盘目标角速度的 0.001 rad/s 协议值。
    uint8_t emergency_stop;               // 底盘急停状态。
    app_gimbal_input_source_t input_source;// 当前控制源。
} app_sentry_gimbal_chassis_state_t;

/** 需要跨控制周期保存的控制状态。 */
typedef struct {
    uint8_t is_ready;             // 首次有效反馈完成目标对齐后置位。
    float yaw_tar_speed_last;     // 上一周期 yaw 目标角速度，rad/s。
} app_sentry_gimbal_control_state_t;

/** 云台各控制环 PID 状态。 */
typedef struct {
    lib_pid_t yaw_i;   // yaw 稳态误差积分环。
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

/** 双 yaw VMC 的配置参数。 */
typedef struct {
    float inertia;       // 等效惯量前馈，电流/(rad/s²)。
    float spring;        // 虚拟弹簧增益，电流/rad。
    float damper;        // 虚拟阻尼增益，电流/(rad/s)。
    float chassis_vw_ff; // 底盘角速度前馈增益，电流/(rad/s)。
    float integral;      // yaw 积分增益。
    float center_ratio;  // 小 yaw 回中强度相对弹簧项的比例。
    float small_weight_start_angle; // 小 yaw 开始降低跟踪权重的角度，rad。
    float small_weight_zero_angle;  // 小 yaw 跟踪权重降为零的角度，rad。
    float current_limit; // 单台 yaw 电机的电流上限。
    float max_accel;     // 目标角加速度上限，rad/s²。
} app_sentry_yaw_vmc_config_t;

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
#define s_pid_yaw_i    app_sentry_gimbal_debug.pid.yaw_i
#define s_pid_pitch    app_sentry_gimbal_debug.pid.pitch
#define s_pid_disc     app_sentry_gimbal_debug.pid.disc
#define s_pid_fric_l   app_sentry_gimbal_debug.pid.fric_l
#define s_pid_fric_r   app_sentry_gimbal_debug.pid.fric_r

/* VMC 参数保留在云台控制模块内，当前统一从零开始标定。 */
static const app_sentry_yaw_vmc_config_t s_yaw_vmc_cfg = {
    .inertia      = 0.0f,
    .spring       = 0.0f,
    .damper       = 0.0f,
    .chassis_vw_ff = 0.0f,
    .integral     = 0.0f,
    .center_ratio = 0.0f,
    .small_weight_start_angle = 0.8f * SENTRY_GIMBAL_S_YAW_LIMIT,
    .small_weight_zero_angle  = SENTRY_GIMBAL_S_YAW_LIMIT,
    .current_limit = DRV_MOTOR_GM6020_CURRENT_MAX,
    .max_accel    = 0.0f,
};

static void pid_init_all(void);
static void gimbal_reset(void);
static void launcher_reset(void);
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input);
static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx,
                                 uint8_t updated_mask);
static float gimbal_get_yaw_cur_angle(void);
static void gimbal_send_chassis_cmd(void);
static void imu_fusion(void);
static void yaw_vmc_control(void);
static float yaw_track_current_calc(void);
static void yaw_current_allocate(float yaw_track_current, float small_yaw_cur_angle);
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
    pid_init_all();

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

    if (!s_imu_state.instance) {
        return;
    }
    has_new_snapshot = drv_imu_port_snapshot_update(s_imu_state.instance);
    drv_imu_port_async_start();
    if (has_new_snapshot) {
        drv_imu_data_convert(s_imu_state.instance);
        drv_imu_mahony_update(s_imu_state.instance, dt);
    }
}

void app_sentry_gimbal_ctrl_1khz(void)
{
    app_gimbal_dbus_input_t input = {
        /* 未连接 DBUS 时默认使用 CAN；没有 CAN 指令时各目标保持初始零值。 */
        .source = APP_GIMBAL_INPUT_CAN,
    };
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    __set_PRIMASK(irq_state);

    imu_fusion();
    (void)app_gimbal_comm_dbus_rx(&input);

    if (!s_imu_state.instance || !isfinite(s_imu_state.yaw_cur_angle) || !isfinite(s_imu_state.pitch_cur_angle)
        || !isfinite(s_imu_state.yaw_gyro) || !isfinite(s_imu_state.pitch_gyro)) {
        gimbal_reset();

        gimbal_can2_tx();
        return;
    }
    if (!s_control_state.is_ready) {
        s_command_state.yaw_tar_angle = s_imu_state.yaw_cur_angle;
        s_command_state.pitch_tar_angle = s_imu_state.pitch_cur_angle;
        s_command_state.yaw_tar_speed = 0.0f;
        s_control_state.yaw_tar_speed_last = 0.0f;
        lib_pid_reset(&s_pid_yaw_i);
        s_control_state.is_ready = 1;
    }
    gimbal_update_cmd(&input);
    if (s_chassis_state.emergency_stop) {
        gimbal_reset();

        gimbal_can2_tx();
        return;
    }

    yaw_vmc_control();

    pitch_control();

    gimbal_can2_tx();
}

void app_sentry_launcher_ctrl_200hz(void)
{
    gimbal_send_chassis_cmd();
    gimbal_can1_tx();
    if (!s_control_state.is_ready || s_chassis_state.emergency_stop) {
        launcher_reset();

    } else {
        launch_control();
    }
    launcher_can2_tx();
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
        drv_motor_solve_dji(data, &s_motor_state.rx[motor_index]);
    }
}

/* 私有函数实现。 */

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_yaw_i, 0.0f, s_yaw_vmc_cfg.integral, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, -s_yaw_vmc_cfg.current_limit, s_yaw_vmc_cfg.current_limit, 1000.0f);
    lib_pid_init(&s_pid_pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_disc, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M2006_CURRENT_MIN, DRV_MOTOR_M2006_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_l, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);

    lib_lpf_init(&s_imu_state.yaw_gyro_lpf, 25.0f);
    lib_lpf_init(&s_imu_state.pitch_gyro_lpf, 25.0f);
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
    s_command_state.yaw_tar_speed = 0.0f;
    s_control_state.yaw_tar_speed_last = 0.0f;
    s_chassis_state.vx_tar_speed = 0;
    s_chassis_state.vy_tar_speed = 0;
    s_chassis_state.vw_tar_speed = 0;
    s_chassis_state.input_source = APP_GIMBAL_INPUT_ESTOP;
    s_chassis_state.emergency_stop = 1U;
    s_control_state.is_ready = 0U;
    lib_pid_reset(&s_pid_yaw_i);
    lib_pid_reset(&s_pid_pitch);
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

    s_chassis_state.input_source = input->source;
    s_chassis_state.emergency_stop = (input->source == APP_GIMBAL_INPUT_ESTOP);
    if (s_chassis_state.emergency_stop) {
        s_chassis_state.vx_tar_speed = 0;
        s_chassis_state.vy_tar_speed = 0;
        s_chassis_state.vw_tar_speed = 0;
        return;
    }
    if (input->source == APP_GIMBAL_INPUT_CAN) {
        s_command_state.data.yaw_tar_speed = 0.0f;
        s_command_state.yaw_tar_speed = 0.0f;
        gimbal_apply_can_cmd(&can_rx, updated_mask);
        return;
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.fire = input->launcher_mode;
    s_command_state.data.yaw_tar_speed = input->yaw_rate_norm * SENTRY_GIMBAL_YAW_TAR_SPEED_MAX;
    s_command_state.data.pitch_tar_speed = input->pitch_rate_norm * SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX;
    s_command_state.yaw_tar_speed = s_command_state.data.yaw_tar_speed;

    float vx_gimbal_tar_speed = input->chassis_vx_norm * SENTRY_CHASSIS_VX_TAR_SPEED_MAX;
    float vy_gimbal_tar_speed = input->chassis_vy_norm * SENTRY_CHASSIS_VY_TAR_SPEED_MAX;
    float gimbal_yaw_cur_angle = gimbal_get_yaw_cur_angle();
    float cos_yaw = cosf(gimbal_yaw_cur_angle);
    float sin_yaw = sinf(gimbal_yaw_cur_angle);
    s_chassis_state.vx_tar_speed = (int16_t)lroundf(cos_yaw * vx_gimbal_tar_speed - sin_yaw * vy_gimbal_tar_speed);
    s_chassis_state.vy_tar_speed = (int16_t)lroundf(sin_yaw * vx_gimbal_tar_speed + cos_yaw * vy_gimbal_tar_speed);
    s_chassis_state.vw_tar_speed = (int16_t)lroundf(input->chassis_omega_norm
        * SENTRY_CHASSIS_VW_TAR_SPEED_MAX / APP_CHASSIS_OMEGA_RAD_S_PER_LSB);

    s_command_state.yaw_tar_angle += s_command_state.data.yaw_tar_speed * 0.001f;
    s_command_state.pitch_tar_angle += s_command_state.data.pitch_tar_speed * 0.001f;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle);
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
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SHOOT) {
        if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0x00U) s_command_state.data.fire = APP_SENTRY_FIRE_ON;
        else if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_FIR;
        else if (rx->shoot.shoot_switch == 0x00U && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_REVERSE;
        else s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    }
}

static float gimbal_get_yaw_cur_angle(void)
{
    float large_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_L_YAW_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_S_YAW_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(
        SENTRY_GIMBAL_L_YAW_DIR * large_yaw_cur_angle
        + SENTRY_GIMBAL_S_YAW_DIR * small_yaw_cur_angle);
}

static void gimbal_send_chassis_cmd(void)
{
    uint8_t data[8] = {0};
    if (s_chassis_state.input_source == APP_GIMBAL_INPUT_CAN) {
        return;
    }
    int16_t vx = s_chassis_state.emergency_stop ? SENTRY_CHASSIS_ESTOP
                                           : s_chassis_state.vx_tar_speed;

    memcpy(data, &vx, sizeof(vx));
    memcpy(data + 2, &s_chassis_state.vy_tar_speed, sizeof(s_chassis_state.vy_tar_speed));
    memcpy(data + 4, &s_chassis_state.vw_tar_speed, sizeof(s_chassis_state.vw_tar_speed));
    (void)bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
}

static void imu_fusion(void)
{
    if (!s_imu_state.instance) {
        return;
    }
    drv_imu_quat_to_euler(s_imu_state.instance);

    s_imu_state.yaw_cur_angle = lib_deg_to_rad(s_imu_state.instance->euler.yaw);

    float pitch_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_PITCH].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_PITCH_ZERO, LIB_ENC13_TO_RAD));
    s_imu_state.pitch_cur_angle = lib_deg_to_rad(s_imu_state.instance->euler.roll) + pitch_cur_angle;

    s_imu_state.yaw_gyro = lib_lpf_update(&s_imu_state.yaw_gyro_lpf, s_imu_state.instance->gyro.z, 0.001f);
    s_imu_state.pitch_gyro = lib_lpf_update(&s_imu_state.pitch_gyro_lpf, s_imu_state.instance->gyro.x, 0.001f);
}

static float yaw_track_current_calc(void)
{
    float yaw_angle_error = lib_get_shortest_path(
        s_command_state.yaw_tar_angle, s_imu_state.yaw_cur_angle);
    float yaw_tar_accel = lib_clamp(
        (s_command_state.yaw_tar_speed - s_control_state.yaw_tar_speed_last) * 1000.0f,
        -s_yaw_vmc_cfg.max_accel, s_yaw_vmc_cfg.max_accel);
    s_control_state.yaw_tar_speed_last = s_command_state.yaw_tar_speed;

    float yaw_track_current = s_yaw_vmc_cfg.inertia * yaw_tar_accel
                            + s_yaw_vmc_cfg.spring * yaw_angle_error
                            + s_yaw_vmc_cfg.damper
                            * (s_command_state.yaw_tar_speed - s_imu_state.yaw_gyro)
                            + s_yaw_vmc_cfg.chassis_vw_ff * s_chassis_state.vw_cur_speed
                            + lib_pid_calc(&s_pid_yaw_i,
                                           s_command_state.yaw_tar_angle,
                                           s_imu_state.yaw_cur_angle, 0.001f);

    if ((yaw_track_current > s_yaw_vmc_cfg.current_limit && yaw_angle_error > 0.0f)
        || (yaw_track_current < -s_yaw_vmc_cfg.current_limit && yaw_angle_error < 0.0f)) {
        s_pid_yaw_i.integral -= yaw_angle_error * 0.001f;
        yaw_track_current -= s_pid_yaw_i.ki * yaw_angle_error * 0.001f;
    }
    return lib_clamp(yaw_track_current,
                     -s_yaw_vmc_cfg.current_limit,
                     s_yaw_vmc_cfg.current_limit);
}

static void yaw_current_allocate(float yaw_track_current, float small_yaw_cur_angle)
{
    float small_track_weight = GIMBAL_SMALL_YAW_TRACK_SHARE * lib_remap_clamp(
        fabsf(small_yaw_cur_angle),
        s_yaw_vmc_cfg.small_weight_start_angle,
        s_yaw_vmc_cfg.small_weight_zero_angle,
        1.0f, 0.0f);
    float yaw_center_current = s_yaw_vmc_cfg.center_ratio
                             * s_yaw_vmc_cfg.spring * small_yaw_cur_angle;
    float small_current_raw = small_track_weight * yaw_track_current
                            - yaw_center_current;
    float small_current = lib_clamp(small_current_raw,
                                    -s_yaw_vmc_cfg.current_limit,
                                    s_yaw_vmc_cfg.current_limit);

    if ((small_yaw_cur_angle >= s_yaw_vmc_cfg.small_weight_zero_angle && small_current > 0.0f)
        || (small_yaw_cur_angle <= -s_yaw_vmc_cfg.small_weight_zero_angle && small_current < 0.0f)) {
        small_current = 0.0f;
    }

    s_motor_state.current[MOTOR_YAW_S] = (int16_t)small_current;
    s_motor_state.current[MOTOR_YAW_L] = (int16_t)lib_clamp(
        (1.0f - small_track_weight) * yaw_track_current
        + yaw_center_current + small_current_raw - small_current,
        -s_yaw_vmc_cfg.current_limit, s_yaw_vmc_cfg.current_limit);
}

static void yaw_vmc_control(void)
{
    float small_yaw_cur_angle = SENTRY_GIMBAL_S_YAW_DIR * lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_YAW_S].angle, SENTRY_GIMBAL_S_YAW_ZERO);

    yaw_current_allocate(yaw_track_current_calc(), small_yaw_cur_angle);
}

static void pitch_control(void)
{
    s_command_state.pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
        SENTRY_GIMBAL_PITCH_MIN, SENTRY_GIMBAL_PITCH_MAX);

    float ff_gravity = cosf(s_imu_state.pitch_cur_angle);

    s_motor_state.current[MOTOR_PITCH] = (int16_t)lib_pid_pos_calc(
        &s_pid_pitch, s_command_state.pitch_tar_angle, s_imu_state.pitch_cur_angle,
        ff_gravity, 0, s_imu_state.pitch_gyro, 0.001f);

    if (s_imu_state.pitch_cur_angle >= SENTRY_GIMBAL_PITCH_MAX && s_motor_state.current[MOTOR_PITCH] > 0) {
        s_motor_state.current[MOTOR_PITCH] = 0;
    } else if (s_imu_state.pitch_cur_angle <= SENTRY_GIMBAL_PITCH_MIN && s_motor_state.current[MOTOR_PITCH] < 0) {
        s_motor_state.current[MOTOR_PITCH] = 0;
    }
}

static void launch_control(void)
{
    switch (s_command_state.data.fire) {
    case APP_SENTRY_FIRE_OFF:
    default:
        launcher_reset();

        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed), 0.005f);
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_REVERSE:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, -SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed), 0.005f);
        break;

    }
}

static void gimbal_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x1FF: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
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
    if (!s_imu_state.instance) {
        return;
    }
    /* 0x122：发送 yaw/pitch 角速度，单位 rad/s。 */
    app_gimbal_comm_gyro_tx(
        s_imu_state.yaw_gyro,
        s_imu_state.pitch_gyro);

    /* 0x124：发送 yaw/pitch 当前角度，单位 rad。 */
    app_gimbal_comm_angle_tx(
        s_imu_state.yaw_cur_angle,
        s_imu_state.pitch_cur_angle);

    app_gimbal_comm_quat_tx(
        (int16_t)(s_imu_state.instance->quat.q0 * 30000.0f), (int16_t)(s_imu_state.instance->quat.q1 * 30000.0f),
        (int16_t)(s_imu_state.instance->quat.q2 * 30000.0f), (int16_t)(s_imu_state.instance->quat.q3 * 30000.0f));
}

static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    float vw_cur_speed;
    memcpy(&vw_cur_speed, data, sizeof(vw_cur_speed));
    if (!isfinite(vw_cur_speed)) {
        return;
    }
    s_chassis_state.vw_cur_speed = vw_cur_speed;
}
