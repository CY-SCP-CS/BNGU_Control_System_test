/**
 * @file    app_sentry_gimbal.c
 * @brief   Sentry 云台控制实现 — 双yaw VMC + pitch重力补偿 + 发射控制
 * @note    Ported from gimbal_2yaw/app/src/gimbal_control.c
 *          CAN2 = 板内电机, CAN1 = 板间数据
 *
 * 控制频率:
 *   AHRS: 1kHz (drv_imu_mahony_update)
 *   Yaw/Pitch: 1kHz
 *   Launcher: 200Hz (app_control 五分频)
 *
 * VMC 双yaw:
 *   轨迹规划器 → 虚拟弹簧-阻尼 → sigmoid加权分配 →
 *   惯量补偿 → 软限位保护 → slew rate限制 → CAN2发送
 *
 * 发射:
 *   OFF: 全部停止
 *   ON:  摩擦轮与拨弹轮角速度（rad/s）闭环
 *   FIR: 摩擦轮维持, 拨弹轮停止
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

// ─── 私有宏 ─────────────────────────
#define GIMBAL_MOTOR_COUNT  6

enum {
    MOTOR_YAW_L  = 0,//大yaw GM6020  (0x205)
    MOTOR_YAW_S  = 1,//小yaw GM6020  (0x206)
    MOTOR_PITCH  = 2,//pitch GM6020  (0x207)
    MOTOR_FRIC_L = 3,//左摩擦 M3508  (0x201)
    MOTOR_DISC   = 4,//拨弹轮 M2006  (0x202)
    MOTOR_FRIC_R = 5 //右摩擦 M3508  (0x203)
};//电机索引

typedef struct {
    drv_motor_data_t feedback[GIMBAL_MOTOR_COUNT]; // 控制循环使用的稳定反馈快照
    drv_motor_data_t rx[GIMBAL_MOTOR_COUNT];       // CAN 中断写入的反馈缓冲
    uint32_t rx_tick_ms[GIMBAL_MOTOR_COUNT];       // 每路电机最近接收时刻
    uint8_t rx_valid[GIMBAL_MOTOR_COUNT];          // 每路电机是否收到过反馈
    int16_t current[GIMBAL_MOTOR_COUNT];           // 本周期输出电流
    int16_t last_current[GIMBAL_MOTOR_COUNT];      // 上周期输出电流
} app_sentry_gimbal_motor_state_t;

typedef struct {
    drv_imu_t *instance;       // IMU 驱动实例
    float yaw_rad;             // 当前 yaw 角度
    float pitch_rad;           // 当前 pitch 角度
    float yaw_tar_speed;      // 当前 yaw 角速度
    float pitch_tar_speed;    // 当前 pitch 角速度
    lib_lpf_t yaw_rate_lpf;    // yaw 角速度低通滤波器
    lib_lpf_t pitch_rate_lpf;  // pitch 角速度低通滤波器
} app_sentry_gimbal_imu_state_t;

typedef struct {
    app_sentry_gimbal_cmd_t data;  // 当前控制输入
    float target_yaw_rad;          // yaw 目标角度
    float target_pitch_rad;        // pitch 目标角度
    float target_yaw_rate_rad_s;   // VMC 轨迹规划后的 yaw 目标角速度
} app_sentry_gimbal_command_state_t;

typedef struct {
    float omega_z_rad_s;                    // 底盘反馈角速度
    uint32_t omega_rx_tick_ms;               // 底盘角速度反馈时刻
    uint8_t omega_received;                  // 是否收到过底盘角速度
    int16_t vx_cmd_mm_s;                     // 待发送底盘 x 速度
    int16_t vy_cmd_mm_s;                     // 待发送底盘 y 速度
    int16_t omega_cmd_lsb;                   // 待发送底盘角速度协议值
    uint8_t emergency_stop;                  // 底盘急停状态
    app_gimbal_input_source_t input_source;  // 当前控制源
} app_sentry_gimbal_chassis_state_t;

typedef struct {
    uint8_t is_ready;              // 首次有效反馈完成对齐后置位
    float last_gyro_yaw_rad_s;     // 上一周期 yaw 角速度
} app_sentry_gimbal_control_state_t;

static app_sentry_gimbal_motor_state_t s_motor_state;
static app_sentry_gimbal_imu_state_t s_imu_state;
static app_sentry_gimbal_command_state_t s_command_state;
static app_sentry_gimbal_chassis_state_t s_chassis_state = {
    .input_source = APP_GIMBAL_INPUT_ESTOP,
};
static app_sentry_gimbal_control_state_t s_control_state;
static const app_sentry_vmc_config_t s_vmc_cfg = {
    .k_virt        = 0.0f,
    .b_virt        = 0.0f,
    .k_ff          = 0.0f,
    .soft_limit_k  = 0.0f,
    .small_limit   = 0.0f,
    .max_out_s     = DRV_MOTOR_GM6020_CURRENT_MAX,
    .max_out_l     = DRV_MOTOR_GM6020_CURRENT_MAX,
    .inertia_small = 0.0f,
    .inertia_big   = 0.0f,
    .max_accel     = 0.0f,
    .max_curr_step = 0.0f,
    .k_tracking    = 0.0f,
    .b_tracking    = 0.0f,
    .max_vel       = 0.0f,
};

static lib_pid_t s_pid_pitch;
static lib_pid_t s_pid_disc;
static lib_pid_t s_pid_fric_l;
static lib_pid_t s_pid_fric_r;


static void pid_init_all(void);

static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input);
static void gimbal_apply_can_cmd(void);
static float gimbal_get_yaw_rel_rad(void);
static void gimbal_send_chassis_cmd(void);

static void imu_fusion(void);

static void yaw_vmc_control(void);
static void pitch_control(void);
static void launch_control(void);

static void gimbal_send_yaw_can2(void);
static void launcher_send_can2(void);
static void gimbal_send_can1(void);

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
    uint32_t now;
    uint8_t is_motor_online = 1;
    app_gimbal_dbus_input_t input;

    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++) {
        if (!s_motor_state.rx_valid[i] || (uint32_t)(now - s_motor_state.rx_tick_ms[i]) > SENTRY_MOTOR_TIMEOUT) {
            is_motor_online = 0;
        }
    }
    __set_PRIMASK(irq_state);

    imu_fusion();

    if (!app_gimbal_comm_dbus_rx(&input) || !s_imu_state.instance || !is_motor_online || !drv_imu_port_is_online(SENTRY_IMU_TIMEOUT)
        || !isfinite(s_imu_state.yaw_rad) || !isfinite(s_imu_state.pitch_rad)
        || !isfinite(s_imu_state.yaw_tar_speed) || !isfinite(s_imu_state.pitch_tar_speed)) {
        memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
        memset(s_motor_state.last_current, 0, sizeof(s_motor_state.last_current));
        s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
        s_command_state.target_yaw_rate_rad_s = 0.0f;
        s_chassis_state.emergency_stop = 1U;
        s_control_state.is_ready = 0;
        lib_pid_reset(&s_pid_pitch);
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        gimbal_send_chassis_cmd();
        gimbal_send_yaw_can2();
        return;
    }
    if (!s_control_state.is_ready) {
        s_command_state.target_yaw_rad = s_imu_state.yaw_rad;
        s_command_state.target_pitch_rad = s_imu_state.pitch_rad;
        s_control_state.last_gyro_yaw_rad_s = s_imu_state.yaw_tar_speed;
        s_control_state.is_ready = 1;
    }

    memcpy(s_motor_state.last_current, s_motor_state.current, sizeof(s_motor_state.current));
    gimbal_update_cmd(&input);
    if (s_chassis_state.emergency_stop) {
        memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
        memset(s_motor_state.last_current, 0, sizeof(s_motor_state.last_current));
        s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
        s_command_state.target_yaw_rate_rad_s = 0.0f;
        lib_pid_reset(&s_pid_pitch);
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        gimbal_send_chassis_cmd();
        gimbal_send_yaw_can2();
        return;
    }

    yaw_vmc_control();

    pitch_control();

    gimbal_send_yaw_can2();

    gimbal_send_can1();
}

void app_sentry_launcher_ctrl_200hz(void)
{
    gimbal_send_chassis_cmd();
    if (!s_control_state.is_ready || s_chassis_state.emergency_stop) {
        s_motor_state.current[MOTOR_FRIC_L] = 0;
        s_motor_state.current[MOTOR_FRIC_R] = 0;
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
    } else {
        launch_control();
    }
    launcher_send_can2();
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
    case SENTRY_CAN_GIMBAL_PITCH:      motor_index = MOTOR_PITCH;   break;
    case SENTRY_CAN_GIMBAL_F1:  motor_index = MOTOR_FRIC_L;  break;
    case SENTRY_CAN_GIMBAL_D1:  motor_index = MOTOR_DISC;    break;
    case SENTRY_CAN_GIMBAL_F2:  motor_index = MOTOR_FRIC_R;  break;
    default: return;
    }
    if (motor_index >= 0 && motor_index < GIMBAL_MOTOR_COUNT) {
        drv_motor_solve_dji(data, &s_motor_state.rx[motor_index]);
        s_motor_state.rx_tick_ms[motor_index] = HAL_GetTick();
        s_motor_state.rx_valid[motor_index] = 1;
    }
}

// ─── 私有函数实现 ─────────────────────────

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_disc, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M2006_CURRENT_MIN, DRV_MOTOR_M2006_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_l, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);

    lib_lpf_init(&s_imu_state.yaw_rate_lpf,   0.15f);
    lib_lpf_init(&s_imu_state.pitch_rate_lpf, 0.15f);
}

static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input)
{
    if (!input) {
        return;
    }
    s_chassis_state.input_source = input->source;
    s_chassis_state.emergency_stop = (input->source == APP_GIMBAL_INPUT_ESTOP);
    if (s_chassis_state.emergency_stop) {
        s_chassis_state.vx_cmd_mm_s = 0;
        s_chassis_state.vy_cmd_mm_s = 0;
        s_chassis_state.omega_cmd_lsb = 0;
        return;
    }
    if (input->source == APP_GIMBAL_INPUT_CAN) {
        gimbal_apply_can_cmd();
        return;
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.fire = input->launcher_mode;
    s_command_state.data.yaw_tar_speed = input->yaw_rate_norm * SENTRY_GIMBAL_YAW_SPEED_MAX;
    s_command_state.data.pitch_tar_speed = input->pitch_rate_norm * SENTRY_GIMBAL_PITCH_SPEED_MAX;

    float vx_gimbal_mm_s = input->chassis_vx_norm * SENTRY_CHASSIS_MAX_VX;
    float vy_gimbal_mm_s = input->chassis_vy_norm * SENTRY_CHASSIS_MAX_VY;
    float gimbal_yaw_rel_rad = gimbal_get_yaw_rel_rad();
    float cos_yaw = cosf(gimbal_yaw_rel_rad);
    float sin_yaw = sinf(gimbal_yaw_rel_rad);
    s_chassis_state.vx_cmd_mm_s = (int16_t)lroundf(cos_yaw * vx_gimbal_mm_s - sin_yaw * vy_gimbal_mm_s);
    s_chassis_state.vy_cmd_mm_s = (int16_t)lroundf(sin_yaw * vx_gimbal_mm_s + cos_yaw * vy_gimbal_mm_s);
    s_chassis_state.omega_cmd_lsb = (int16_t)lroundf(input->chassis_omega_norm
        * SENTRY_CHASSIS_MAX_VW / APP_CHASSIS_OMEGA_RAD_S_PER_LSB);

    s_command_state.target_yaw_rad += s_command_state.data.yaw_tar_speed * 0.001f;
    s_command_state.target_pitch_rad += s_command_state.data.pitch_tar_speed * 0.001f;
    s_command_state.target_yaw_rad = lib_rad_norm(s_command_state.target_yaw_rad);
}

static void gimbal_apply_can_cmd(void)
{
    app_gimbal_comm_rx_t rx;
    uint8_t updated_mask;

    if (!app_gimbal_comm_read_rx(&rx, &updated_mask)) return;
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT) {
        s_command_state.target_yaw_rad = lib_rad_norm(s_command_state.target_yaw_rad + rx.speed_no_shoot.yaw_inc);
        s_command_state.target_pitch_rad += rx.speed_no_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT) {
        s_command_state.target_yaw_rad = lib_rad_norm(s_command_state.target_yaw_rad + rx.speed_shoot.yaw_inc);
        s_command_state.target_pitch_rad += rx.speed_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT) {
        s_command_state.target_yaw_rad = lib_rad_norm(rx.angle_no_shoot.yaw_abs);
        s_command_state.target_pitch_rad = rx.angle_no_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT) {
        s_command_state.target_yaw_rad = lib_rad_norm(rx.angle_shoot.yaw_abs);
        s_command_state.target_pitch_rad = rx.angle_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SHOOT) {
        if (rx.shoot.shoot_switch == 0xFFU && rx.shoot.retreat == 0x00U) s_command_state.data.fire = APP_SENTRY_FIRE_ON;
        else if (rx.shoot.shoot_switch == 0xFFU && rx.shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_FIR;
        else if (rx.shoot.shoot_switch == 0x00U && rx.shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_REVERSE;
        else s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    }
}

static float gimbal_get_yaw_rel_rad(void)
{
    float large_yaw_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_L_YAW_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_S_YAW_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(
        SENTRY_GIMBAL_L_YAW_DIR * large_yaw_rel_rad
        + SENTRY_GIMBAL_S_YAW_DIR * small_yaw_rel_rad);
}

static void gimbal_send_chassis_cmd(void)
{
    uint8_t data[8] = {0};
    if (s_chassis_state.input_source == APP_GIMBAL_INPUT_CAN) {
        return;
    }
    int16_t vx = s_chassis_state.emergency_stop ? SENTRY_CHASSIS_ESTOP
                                           : s_chassis_state.vx_cmd_mm_s;

    memcpy(data, &vx, sizeof(vx));
    memcpy(data + 2, &s_chassis_state.vy_cmd_mm_s, sizeof(s_chassis_state.vy_cmd_mm_s));
    memcpy(data + 4, &s_chassis_state.omega_cmd_lsb, sizeof(s_chassis_state.omega_cmd_lsb));
    (void)bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
}

static void imu_fusion(void)
{
    if (!s_imu_state.instance) {
        return;
    }
    drv_imu_quat_to_euler(s_imu_state.instance);

    s_imu_state.yaw_rad = lib_deg_to_rad(s_imu_state.instance->euler.yaw);

    float pitch_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_PITCH].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_PITCH_ZERO, LIB_ENC13_TO_RAD));
    s_imu_state.pitch_rad = lib_deg_to_rad(s_imu_state.instance->euler.roll) + pitch_rel_rad;

    s_imu_state.yaw_tar_speed = lib_lpf_update(&s_imu_state.yaw_rate_lpf, s_imu_state.instance->gyro.z);
    s_imu_state.pitch_tar_speed = lib_lpf_update(&s_imu_state.pitch_rate_lpf, s_imu_state.instance->gyro.x);
}

static void yaw_vmc_control(void)
{
    float yaw_err_rad = lib_get_shortest_path(s_command_state.target_yaw_rad, s_imu_state.yaw_rad);

    float target_accel = s_vmc_cfg.k_tracking * yaw_err_rad
                       - s_vmc_cfg.b_tracking * s_command_state.target_yaw_rate_rad_s;
    target_accel = lib_clamp(target_accel,
                                  -s_vmc_cfg.max_accel, s_vmc_cfg.max_accel);
    s_command_state.target_yaw_rate_rad_s += target_accel * 0.001f;
    s_command_state.target_yaw_rate_rad_s = lib_clamp(s_command_state.target_yaw_rate_rad_s,
                                             -s_vmc_cfg.max_vel, s_vmc_cfg.max_vel);


    float omega_err = s_command_state.target_yaw_rate_rad_s - s_imu_state.yaw_tar_speed;
    float tau_vm = s_vmc_cfg.k_virt * yaw_err_rad
                 + s_vmc_cfg.b_virt * omega_err;


    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    float chassis_omega = s_chassis_state.omega_z_rad_s;
    uint8_t is_omega_fresh = s_chassis_state.omega_received
                         && (uint32_t)(HAL_GetTick() - s_chassis_state.omega_rx_tick_ms) <= 200U;
    __set_PRIMASK(irq_state);
    if (is_omega_fresh) {
        tau_vm += s_vmc_cfg.k_ff * chassis_omega;
    }


    int32_t diff = (int32_t)s_motor_state.feedback[MOTOR_YAW_S].angle
                 - (int32_t)SENTRY_GIMBAL_S_YAW_ZERO;
    if (diff > 4096) {
        diff -= 8192;
    }
    else if (diff < -4095) {
        diff += 8192;
    }
    float small_rel = lib_enc_conv((float)diff, LIB_ENC13_TO_RAD);
    float sigmoid_in = (fabsf(small_rel) - lib_deg_to_rad(18.0f))
                     * (0.4f * 180.0f / LIB_PI);
    float ratio  = lib_fast_sigmoid(sigmoid_in);
    float w_big   = 0.4f + 0.5f * ratio;
    float w_small = 1.0f - w_big;


    float ang_accel = (s_imu_state.yaw_tar_speed - s_control_state.last_gyro_yaw_rad_s) / 0.001f;
    s_control_state.last_gyro_yaw_rad_s = s_imu_state.yaw_tar_speed;
    float inertia_comp_s = ang_accel * s_vmc_cfg.inertia_small;
    float inertia_comp_b = ang_accel * s_vmc_cfg.inertia_big;


    float out_small = tau_vm * w_small
                    + target_accel * s_vmc_cfg.inertia_small
                    + inertia_comp_s;
    float out_big   = tau_vm * w_big
                    + target_accel * s_vmc_cfg.inertia_big
                    + inertia_comp_b;

    out_big += small_rel * s_vmc_cfg.soft_limit_k;


    if (small_rel > s_vmc_cfg.small_limit) {
        out_small = lib_clamp(out_small, -s_vmc_cfg.max_out_s, 0);
    } else if (small_rel < -s_vmc_cfg.small_limit) {
        out_small = lib_clamp(out_small, 0, s_vmc_cfg.max_out_s);
    } else {
        out_small = lib_clamp(out_small,
                                   -s_vmc_cfg.max_out_s, s_vmc_cfg.max_out_s);
    }
    out_big = lib_clamp(out_big,
                             -s_vmc_cfg.max_out_l, s_vmc_cfg.max_out_l);


    float step_s = out_small - s_motor_state.last_current[MOTOR_YAW_S];
    step_s = lib_clamp(step_s, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_state.current[MOTOR_YAW_S] = s_motor_state.last_current[MOTOR_YAW_S]
                                 + (int16_t)step_s;

    float step_b = out_big - s_motor_state.last_current[MOTOR_YAW_L];
    step_b = lib_clamp(step_b, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_state.current[MOTOR_YAW_L] = s_motor_state.last_current[MOTOR_YAW_L]
                                 + (int16_t)step_b;

    if (small_rel >= s_vmc_cfg.small_limit && s_motor_state.current[MOTOR_YAW_S] > 0) {
        s_motor_state.current[MOTOR_YAW_S] = 0;
    } else if (small_rel <= -s_vmc_cfg.small_limit && s_motor_state.current[MOTOR_YAW_S] < 0) {
        s_motor_state.current[MOTOR_YAW_S] = 0;
    }
}

static void pitch_control(void)
{
    s_command_state.target_pitch_rad = lib_clamp(s_command_state.target_pitch_rad,
        SENTRY_GIMBAL_PITCH_MIN, SENTRY_GIMBAL_PITCH_MAX);

    float ff_gravity = cosf(s_imu_state.pitch_rad);

    s_motor_state.current[MOTOR_PITCH] = (int16_t)lib_pid_pos_calc(
        &s_pid_pitch, s_command_state.target_pitch_rad, s_imu_state.pitch_rad,
        ff_gravity, 0, s_imu_state.pitch_tar_speed);

    if (s_imu_state.pitch_rad >= SENTRY_GIMBAL_PITCH_MAX && s_motor_state.current[MOTOR_PITCH] > 0) {
        s_motor_state.current[MOTOR_PITCH] = 0;
    } else if (s_imu_state.pitch_rad <= SENTRY_GIMBAL_PITCH_MIN && s_motor_state.current[MOTOR_PITCH] < 0) {
        s_motor_state.current[MOTOR_PITCH] = 0;
    }
}

static void launch_control(void)
{
    switch (s_command_state.data.fire) {
    case APP_SENTRY_FIRE_OFF:
    default:
        s_motor_state.current[MOTOR_FRIC_L] = 0;
        s_motor_state.current[MOTOR_FRIC_R] = 0;
        s_motor_state.current[MOTOR_DISC]   = 0;
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed));
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed));
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, SENTRY_DISC_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed));
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed));
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed));
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_REVERSE:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed));
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed));
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, -SENTRY_DISC_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed));
        break;

    }
}

static void gimbal_send_yaw_can2(void)
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

static void launcher_send_can2(void)
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

static void gimbal_send_can1(void)
{
    if (!s_imu_state.instance) {
        return;
    }
    /* 0x122: 速度反馈 yaw/pitch (rad/s) — 上位机监控用 */
    app_gimbal_comm_gyro_tx(
        s_imu_state.yaw_tar_speed,
        s_imu_state.pitch_tar_speed);

    /* 0x124: 角度反馈 yaw/pitch (rad) — 上位机+底盘用 */
    app_gimbal_comm_angle_tx(
        s_imu_state.yaw_rad,
        s_imu_state.pitch_rad);

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
    float omega_z;
    memcpy(&omega_z, data, sizeof(omega_z));
    if (!isfinite(omega_z)) {
        return;
    }
    s_chassis_state.omega_z_rad_s = omega_z;
    s_chassis_state.omega_rx_tick_ms = HAL_GetTick();
    s_chassis_state.omega_received = 1;
}
