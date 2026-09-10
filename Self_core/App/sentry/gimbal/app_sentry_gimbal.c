/**
 * @file    app_sentry_gimbal.c
 * @brief   Sentry 云台控制实现 — 双yaw VMC + pitch重力补偿 + 发射控制
 * @note    Ported from gimbal_2yaw/app/src/gimbal_control.c
 *          CAN2 = 板内电机, CAN1 = 板间数据
 *
 * 控制频率:
 *   AHRS: 1kHz (drv_imu_mahony_update)
 *   Yaw/Pitch/Launch: 200Hz (app_control 分频)
 *
 * VMC 双yaw:
 *   轨迹规划器 → 虚拟弹簧-阻尼 → sigmoid加权分配 →
 *   惯量补偿 → 软限位保护 → slew rate限制 → CAN2发送
 *
 * 发射:
 *   OFF: 全部停止
 *   ON:  摩擦轮 RPM 闭环 + 拨弹轮高速
 *   FIR: 摩擦轮维持, 拨弹轮停止
 */
#include "app_sentry_gimbal.h"
#include "project_cfg.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"

#include "drv_imu.h"
#include "drv_dbus.h"
#include "drv_motor.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"    /* 0x119 角速度反馈 */

#include <math.h>
#include <string.h>

#if CURRENT_ROBOT == ROBOT_SENTRY && CURRENT_BOARD == BOARD_GIMBAL

// ─── 私有宏 ─────────────────────────
#define GIMBAL_MOTOR_COUNT  6

enum {
    MOTOR_YAW_L  = 0,   /* 大yaw GM6020  (0x205)  */
    MOTOR_YAW_S  = 1,   /* 小yaw GM6020  (0x206)  */
    MOTOR_PITCH  = 2,   /* pitch GM6020  (0x207)  */
    MOTOR_FRIC_L = 3,   /* 左摩擦 M3508  (0x201)  */
    MOTOR_DISC   = 4,   /* 拨弹轮 M2006  (0x202)  */
    MOTOR_FRIC_R = 5    /* 右摩擦 M3508  (0x203)  */
};

static drv_motor_data_t s_motor[GIMBAL_MOTOR_COUNT];
static drv_motor_data_t s_motor_rx[GIMBAL_MOTOR_COUNT];
static uint32_t s_motor_rx_tick[GIMBAL_MOTOR_COUNT];
static uint8_t s_motor_rx_is_valid[GIMBAL_MOTOR_COUNT];
static uint8_t s_control_is_ready;
static uint8_t s_feedback_phase;
static float s_last_gyro_yaw;
static uint32_t s_chassis_omega_tick;
static uint8_t s_chassis_omega_is_valid;
static int16_t          s_motor_current[GIMBAL_MOTOR_COUNT];
static int16_t          s_motor_last_current[GIMBAL_MOTOR_COUNT];
static uint32_t         s_can_tx_error_count;   /**< CAN 发送失败计数 */

/* 小yaw 编码器中心 (机械零位), 首次收到反馈时捕获 */
static uint16_t s_small_yaw_center_enc;
static uint8_t  s_small_yaw_center_valid;

static drv_imu_t *s_imu;
static float s_yaw_angle_deg;
static float s_pitch_angle_deg;
static float s_gyro_yaw_dps;       /* yaw角速度 deg/s (LPF后)   */
static float s_gyro_pitch_dps;     /* pitch角速度 deg/s (LPF后) */

/* 两个独立LPF */
static lib_filter_lpf_t s_gyro_yaw_lpf;
static lib_filter_lpf_t s_gyro_pitch_lpf;

static app_sentry_gimbal_cmd_t s_cmd;
static float s_target_yaw_deg;
static float s_target_pitch_deg;
static float s_target_yaw_vel;      /* 轨迹规划器输出的目标角速度 deg/s */
static float s_chassis_omega_z;     /**< 底盘角速度 (来自CAN1 0x119, VMC前馈) */

static const app_sentry_vmc_config_t s_vmc_cfg = {
    .k_virt        = 500.0f,
    .b_virt        = 500.0f,
    .k_ff          = 1.2f,
    .soft_limit_k  = 150.0f,
    .small_limit   = SENTRY_GIMBAL_SMALL_YAW_LIMIT_DEG,
    .max_out_s     = 16000.0f,
    .max_out_l     = 16000.0f,
    .inertia_small = 200.0f,
    .inertia_big   = 800.0f,
    .max_accel     = 5000.0f,
    .max_curr_step = 300.0f,
    .k_tracking    = 25.0f,
    .b_tracking    = 10.0f,
    .max_vel       = 800.0f,
};

static lib_pid_t s_pid_pitch;      /* Kp=0 Ki=0 Kd=0 Kff_g=5000 */
static lib_pid_t s_pid_disc;       /* Kp=2.0 Ki=0.5 Kd=0.5        */
static lib_pid_t s_pid_fric_l;     /* Kp=3.0 Ki=0.5 Kd=1.0        */
static lib_pid_t s_pid_fric_r;     /* Kp=3.0 Ki=0.5 Kd=1.0        */

// ─── 私有函数声明 ─────────────────────────

static void pid_init_all(void);
static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd);
static void imu_fusion(void);
static void yaw_vmc_control(void);
static void pitch_control(void);
static void launch_control(void);
static void motor_send_can2(void);
static void gimbal_send_can1(void);
static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

// ─── 公有接口实现 ─────────────────────────

void app_gimbal_init(drv_imu_t *imu)
{
    s_imu = imu;
    memset(s_motor_rx, 0, sizeof(s_motor_rx));
    memset(s_motor_rx_tick, 0, sizeof(s_motor_rx_tick));
    memset(s_motor_rx_is_valid, 0, sizeof(s_motor_rx_is_valid));
    s_control_is_ready = 0;
    s_feedback_phase = 0;
    s_last_gyro_yaw = 0.0f;
    s_chassis_omega_z = 0.0f;
    s_chassis_omega_tick = 0;
    s_chassis_omega_is_valid = 0;
    memset(s_motor,              0, sizeof(s_motor));
    memset(s_motor_current,      0, sizeof(s_motor_current));
    memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
    memset(&s_cmd,               0, sizeof(s_cmd));
    s_can_tx_error_count = 0;

    s_target_yaw_deg   = 0;
    s_target_pitch_deg = 0;
    s_target_yaw_vel   = 0;
    s_yaw_angle_deg    = 0;
    s_pitch_angle_deg  = 0;
    s_gyro_yaw_dps     = 0;
    s_gyro_pitch_dps   = 0;
#if SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO >= 0
    s_small_yaw_center_enc = SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO;
    s_small_yaw_center_valid = 1;
#else
    s_small_yaw_center_enc = 0;
    s_small_yaw_center_valid = 0;
#endif

    pid_init_all();

    /* ── 注册 CAN2 电机反馈回调 (板内) ── */
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_YAW_LARGE,
                                 app_gimbal_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_YAW_SMALL,
                                 app_gimbal_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_PITCH,
                                 app_gimbal_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F1,
                                 app_gimbal_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_D1,
                                 app_gimbal_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F2,
                                 app_gimbal_on_motor_feedback);

    /* ── 注册 CAN1 板间: 底盘 ωz (0x119, VMC前馈用) ── */
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK,
                                 on_chassis_omega_feedback);
}

void app_gimbal_ahrs_update(float dt)
{
    uint8_t has_new_snapshot;

    if (!s_imu) {
        return;
    }
    has_new_snapshot = drv_imu_port_snapshot_update(s_imu);
    drv_imu_port_async_start();
    if (has_new_snapshot) {
        drv_imu_data_convert(s_imu);
        drv_imu_mahony_update(s_imu, dt);
    }
}

void app_gimbal_ctrl(void)
{
    uint32_t now;
    uint8_t is_motor_online = 1;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    now = drv_motor_port_get_tick();
    memcpy(s_motor, s_motor_rx, sizeof(s_motor));
    for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++) {
        if (!s_motor_rx_is_valid[i] || (uint32_t)(now - s_motor_rx_tick[i]) > SENTRY_MOTOR_TIMEOUT_MS) {
            is_motor_online = 0;
        }
    }
    __set_PRIMASK(irq_state);
    imu_fusion();
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    if (!dbus || !s_imu || !is_motor_online || !drv_imu_port_is_online(SENTRY_IMU_TIMEOUT_MS)
        || !isfinite(s_yaw_angle_deg) || !isfinite(s_pitch_angle_deg)
        || !isfinite(s_gyro_yaw_dps) || !isfinite(s_gyro_pitch_dps)) {
        memset(s_motor_current, 0, sizeof(s_motor_current));
        memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
        s_cmd.fire = APP_SENTRY_FIRE_OFF;
        s_target_yaw_vel = 0.0f;
        s_control_is_ready = 0;
        lib_pid_reset(&s_pid_pitch);
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        motor_send_can2();
        return;
    }
    if (!s_control_is_ready) {
        /* 上电与恢复连接时从当前姿态起步，不追赶失联前的目标。 */
        s_target_yaw_deg = s_yaw_angle_deg;
        s_target_pitch_deg = s_pitch_angle_deg;
        s_last_gyro_yaw = s_gyro_yaw_dps;
        s_control_is_ready = 1;
    }
    memcpy(s_motor_last_current, s_motor_current, sizeof(s_motor_current));
    dbus_to_cmd(&s_cmd);

    /* ── 3. Yaw VMC ── */
    yaw_vmc_control();

    /* ── 4. Pitch ── */
    pitch_control();

    /* ── 5. 发射 ── */
    launch_control();

    /* ── 6. CAN2 电机发送 ── */
    motor_send_can2();

    /* ── 7. CAN1 角度反馈 (云台→底盘+上位机) ── */
    gimbal_send_can1();
}

void app_gimbal_on_motor_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    if (!data || len != 8U) {
        return;
    }
    int motor_index = -1;
    switch (std_id) {
    case SENTRY_CAN_GIMBAL_YAW_LARGE:  motor_index = MOTOR_YAW_L;   break;
    case SENTRY_CAN_GIMBAL_YAW_SMALL:  motor_index = MOTOR_YAW_S;   break;
    case SENTRY_CAN_GIMBAL_PITCH:      motor_index = MOTOR_PITCH;   break;
    case SENTRY_CAN_GIMBAL_LAUNCH_F1:  motor_index = MOTOR_FRIC_L;  break;
    case SENTRY_CAN_GIMBAL_LAUNCH_D1:  motor_index = MOTOR_DISC;    break;
    case SENTRY_CAN_GIMBAL_LAUNCH_F2:  motor_index = MOTOR_FRIC_R;  break;
    default: return;
    }
    if (motor_index >= 0 && motor_index < GIMBAL_MOTOR_COUNT) {
        drv_motor_solve_dji_data(data, &s_motor_rx[motor_index]);
        s_motor_rx_tick[motor_index] = drv_motor_port_get_tick();
        s_motor_rx_is_valid[motor_index] = 1;
        /* 首次收到小yaw反馈 → 捕获机械中心 (VMC软限位用) */
        if (motor_index == MOTOR_YAW_S && !s_small_yaw_center_valid) {
            s_small_yaw_center_enc   = s_motor_rx[MOTOR_YAW_S].angle;
            s_small_yaw_center_valid = 1;
        }
    }
}

void app_gimbal_get_angles(float *yaw, float *pitch)
{
    if (yaw) {
        *yaw   = s_yaw_angle_deg;
    }
    if (pitch) {
        *pitch = s_pitch_angle_deg;
    }
}

// ─── 私有函数实现 ─────────────────────────

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_pitch, SENTRY_GIMBAL_PITCH_KP, SENTRY_GIMBAL_PITCH_KI,
                 SENTRY_GIMBAL_PITCH_KD, 5000.0f, 0, 16000, 0,
                 -16000, 16000, 1000);
    lib_pid_init(&s_pid_disc,   2.0f, 0.5f, 0.5f, 0, 0, 0, 0,
                 -10000, 10000, 600);
    lib_pid_init(&s_pid_fric_l, 3.0f, 0.5f, 1.0f, 0, 0, 0, 0,
                 -16000, 16000, 300);
    lib_pid_init(&s_pid_fric_r, 3.0f, 0.5f, 1.0f, 0, 0, 0, 0,
                 -16000, 16000, 300);

    lib_filter_lpf_init(&s_gyro_yaw_lpf,   0.15f);
    lib_filter_lpf_init(&s_gyro_pitch_lpf, 0.15f);
}

static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd)
{
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    if (!dbus) {
        return;
    }
    cmd->mode = (uint8_t)dbus->rc.s1;
    cmd->fire = (uint8_t)dbus->rc.s2;

    cmd->yaw_speed   = ((float)dbus->rc.ch[2] - DRV_DBUS_CHANNEL_CENTER) * 1200.0f / 660.0f;
    cmd->pitch_speed = ((float)dbus->rc.ch[3] - DRV_DBUS_CHANNEL_CENTER) * 4.3f  / 660.0f;

    cmd->yaw_inc   = ((float)dbus->rc.ch[0] - DRV_DBUS_CHANNEL_CENTER) * 2.0f  / 660.0f;
    cmd->pitch_inc = ((float)dbus->rc.ch[1] - DRV_DBUS_CHANNEL_CENTER) * 0.5f  / 660.0f;

    cmd->yaw_angle   = 0;
    cmd->pitch_angle = 0;

    app_gimbal_angle_cmd_t vision;
    if (app_gimbal_comm_read_angle_cmd(&vision, 100U)) {
        cmd->yaw_angle = lib_math_rad2deg(lib_math_rad_normalize(vision.yaw_abs));
        cmd->pitch_angle = lib_math_rad2deg(lib_math_rad_normalize(vision.pitch_abs));
    } else {
        cmd->yaw_angle = s_target_yaw_deg;
        cmd->pitch_angle = s_target_pitch_deg;
    }

    switch (cmd->mode) {
    case APP_SENTRY_GIMBAL_MODE_SPEED:
        s_target_yaw_deg   += cmd->yaw_speed   * 0.005f;
        s_target_pitch_deg += cmd->pitch_speed * 0.005f;
        break;
    case APP_SENTRY_GIMBAL_MODE_ABS_ANGLE:
        s_target_yaw_deg   = cmd->yaw_angle;
        s_target_pitch_deg = cmd->pitch_angle;
        break;
    case APP_SENTRY_GIMBAL_MODE_INC_ANGLE:
    default:
        s_target_yaw_deg   += cmd->yaw_inc;
        s_target_pitch_deg += cmd->pitch_inc;
        break;
    }
    s_target_yaw_deg = lib_math_rad2deg(lib_math_rad_normalize(lib_math_deg2rad(s_target_yaw_deg)));
}

static void imu_fusion(void)
{
    if (!s_imu) {
        return;
    }
    drv_imu_quat_to_euler(s_imu);

    /* yaw: 直接 BMI088 */
    s_yaw_angle_deg = s_imu->euler.yaw;

    /* pitch: BMI088 roll + pitch电机相对零点 */
    float pitch_rel = lib_math_rad2deg(lib_math_get_shortest_path(
        lib_math_deg2rad(SENTRY_ENC_TO_DEG(s_motor[MOTOR_PITCH].angle)),
        lib_math_deg2rad(SENTRY_ENC_TO_DEG(SENTRY_GIMBAL_PITCH_ENCODER_ZERO))));
    s_pitch_angle_deg = s_imu->euler.roll + pitch_rel;

    /* 陀螺角速度 LPF (两个独立滤波器) */
    s_gyro_yaw_dps   = lib_filter_lpf_update(&s_gyro_yaw_lpf,
                         s_imu->gyro.z * (180.0f / (float)LIB_MATH_PI));
    s_gyro_pitch_dps = lib_filter_lpf_update(&s_gyro_pitch_lpf,
                         s_imu->gyro.x * (180.0f / (float)LIB_MATH_PI));
}

static void yaw_vmc_control(void)
{
    /* ── 1. 轨迹规划器 ── */
    float yaw_err_rad = lib_math_get_shortest_path(
        lib_math_deg2rad(s_target_yaw_deg),
        lib_math_deg2rad(s_yaw_angle_deg));
    float yaw_err_deg = yaw_err_rad * (180.0f / (float)LIB_MATH_PI);

    float target_accel = s_vmc_cfg.k_tracking * yaw_err_deg
                       - s_vmc_cfg.b_tracking * s_target_yaw_vel;
    target_accel = lib_math_clamp(target_accel,
                                  -s_vmc_cfg.max_accel, s_vmc_cfg.max_accel);
    s_target_yaw_vel += target_accel * 0.005f;
    s_target_yaw_vel = lib_math_clamp(s_target_yaw_vel,
                                      -s_vmc_cfg.max_vel, s_vmc_cfg.max_vel);

    /* ── 2. 虚拟弹簧-阻尼 ── */
    float omega_err = s_target_yaw_vel - s_gyro_yaw_dps;
    float tau_vm = s_vmc_cfg.k_virt * yaw_err_deg
                 + s_vmc_cfg.b_virt * omega_err;

    /* 0x119 是 rad/s，VMC 的其余速度量均为 deg/s。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    float chassis_omega = s_chassis_omega_z;
    uint8_t is_omega_fresh = s_chassis_omega_is_valid
                             && (uint32_t)(drv_motor_port_get_tick() - s_chassis_omega_tick) <= 200U;
    __set_PRIMASK(irq_state);
    if (is_omega_fresh) {
        tau_vm += s_vmc_cfg.k_ff * lib_math_rad2deg(chassis_omega);
    }

    /* ── 3. Sigmoid 加权 ──
     *    小yaw相对中心角度 (带符号, ±180°), 中心在首次反馈时捕获
     *    (修复: 原代码直接用无符号 0-360° 编码器, 软限位判断全错) */
    float small_rel = 0;
    if (s_small_yaw_center_valid) {
        int32_t diff = (int32_t)s_motor[MOTOR_YAW_S].angle
                     - (int32_t)s_small_yaw_center_enc;
        if (diff > 4096) {
            diff -= 8192;
        }
        else if (diff < -4095) {
            diff += 8192;
        }
        small_rel = (float)diff * 360.0f / 8192.0f;
    }
    float sigmoid_in = (fabsf(small_rel) - 18.0f) * 0.4f;
    float ratio  = lib_math_fast_sigmoid(sigmoid_in);
    float w_big   = 0.4f + 0.5f * ratio;
    float w_small = 1.0f - w_big;

    /* ── 4. 惯量补偿 (用上一周期角速度差分近似加速度) ── */
    float ang_accel = (s_gyro_yaw_dps - s_last_gyro_yaw) / 0.005f;
    s_last_gyro_yaw = s_gyro_yaw_dps;
    float inertia_comp_s = ang_accel * s_vmc_cfg.inertia_small;
    float inertia_comp_b = ang_accel * s_vmc_cfg.inertia_big;

    /* ── 5. 电流计算 ── */
    /* 电流计算 (chassis_w 已合并到 tau_vm) */
    float out_small = tau_vm * w_small
                    + target_accel * s_vmc_cfg.inertia_small
                    + inertia_comp_s;
    float out_big   = tau_vm * w_big
                    + target_accel * s_vmc_cfg.inertia_big
                    + inertia_comp_b;

    out_big += small_rel * s_vmc_cfg.soft_limit_k;

    /* ── 6. 限位保护 ── */
    if (small_rel > s_vmc_cfg.small_limit) {
        out_small = lib_math_clamp(out_small, -s_vmc_cfg.max_out_s, 0);
    } else if (small_rel < -s_vmc_cfg.small_limit) {
        out_small = lib_math_clamp(out_small, 0, s_vmc_cfg.max_out_s);
    } else {
        out_small = lib_math_clamp(out_small,
                                   -s_vmc_cfg.max_out_s, s_vmc_cfg.max_out_s);
    }
    out_big = lib_math_clamp(out_big,
                             -s_vmc_cfg.max_out_l, s_vmc_cfg.max_out_l);

    /* ── 7. Slew rate 限制 ── */
    float step_s = out_small - s_motor_last_current[MOTOR_YAW_S];
    step_s = lib_math_clamp(step_s, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_current[MOTOR_YAW_S] = s_motor_last_current[MOTOR_YAW_S]
                                 + (int16_t)step_s;

    float step_b = out_big - s_motor_last_current[MOTOR_YAW_L];
    step_b = lib_math_clamp(step_b, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_current[MOTOR_YAW_L] = s_motor_last_current[MOTOR_YAW_L]
                                 + (int16_t)step_b;
    /* 最终限位优先于电流变化率；否则上一拍外推电流还会继续越界。 */
    if (small_rel >= s_vmc_cfg.small_limit && s_motor_current[MOTOR_YAW_S] > 0) {
        s_motor_current[MOTOR_YAW_S] = 0;
    } else if (small_rel <= -s_vmc_cfg.small_limit && s_motor_current[MOTOR_YAW_S] < 0) {
        s_motor_current[MOTOR_YAW_S] = 0;
    }
}

static void pitch_control(void)
{
    s_target_pitch_deg = lib_math_clamp(s_target_pitch_deg,
        SENTRY_GIMBAL_PITCH_MIN_DEG, SENTRY_GIMBAL_PITCH_MAX_DEG);

    float pitch_rad = lib_math_deg2rad(s_pitch_angle_deg);
    float ff_gravity = cosf(pitch_rad);

    s_motor_current[MOTOR_PITCH] = (int16_t)lib_pid_pos_calc(
        &s_pid_pitch, s_target_pitch_deg, s_pitch_angle_deg,
        ff_gravity, 0, s_gyro_pitch_dps);
    if (s_pitch_angle_deg >= SENTRY_GIMBAL_PITCH_MAX_DEG && s_motor_current[MOTOR_PITCH] > 0) {
        s_motor_current[MOTOR_PITCH] = 0;
    } else if (s_pitch_angle_deg <= SENTRY_GIMBAL_PITCH_MIN_DEG && s_motor_current[MOTOR_PITCH] < 0) {
        s_motor_current[MOTOR_PITCH] = 0;
    }
}

static void launch_control(void)
{
    switch (s_cmd.fire) {
    case APP_SENTRY_FIRE_OFF:
    default:
        s_motor_current[MOTOR_FRIC_L] = 0;
        s_motor_current[MOTOR_FRIC_R] = 0;
        s_motor_current[MOTOR_DISC]   = 0;
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RPM,
            s_motor[MOTOR_FRIC_L].speed);
        s_motor_current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RPM,
            s_motor[MOTOR_FRIC_R].speed);
        s_motor_current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, SENTRY_DISC_TARGET_RPM,
            s_motor[MOTOR_DISC].speed);
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RPM,
            s_motor[MOTOR_FRIC_L].speed);
        s_motor_current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RPM,
            s_motor[MOTOR_FRIC_R].speed);
        s_motor_current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    }
}

static void motor_send_can2(void)
{
    uint8_t frame[8];
    bsp_can_tx_status_t status;

    /* 0x1FF: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[3] = LIB_LO_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[4] = LIB_HI_BYTE(s_motor_current[MOTOR_PITCH]);
    frame[5] = LIB_LO_BYTE(s_motor_current[MOTOR_PITCH]);
    status = bsp_can_send(&hcan2, SENTRY_CAN_GIMBAL_TX_YAW, frame);
    if (status != BSP_CAN_TX_OK) {
        s_can_tx_error_count++;
    }
    /* 0x200: [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[MOTOR_DISC]);
    frame[3] = LIB_LO_BYTE(s_motor_current[MOTOR_DISC]);
    frame[4] = LIB_HI_BYTE(s_motor_current[MOTOR_FRIC_R]);
    frame[5] = LIB_LO_BYTE(s_motor_current[MOTOR_FRIC_R]);
    status = bsp_can_send(&hcan2, SENTRY_CAN_GIMBAL_TX_LAUNCH, frame);
    if (status != BSP_CAN_TX_OK) {
        s_can_tx_error_count++;
    }
}

static void gimbal_send_can1(void)
{
    if (!s_imu) {
        return;
    }
    /* 0x122: 速度反馈 yaw/pitch (rad/s) — 上位机监控用 */
    float yaw_dps  = s_gyro_yaw_dps;
    float pitch_dps = s_gyro_pitch_dps;
    app_gimbal_comm_send_speed_feedback(
        lib_math_deg2rad(yaw_dps),
        lib_math_deg2rad(pitch_dps));

    /* 0x124: 角度反馈 yaw/pitch (rad) — 上位机+底盘用 */
    app_gimbal_comm_send_angle_feedback(
        lib_math_deg2rad(s_yaw_angle_deg),
        lib_math_deg2rad(s_pitch_angle_deg));

    /* 基础反馈占两个邮箱，附加反馈交替发送，避免第四帧固定丢失。 */
    if (s_feedback_phase == 0U) {
        float yaw_deg = fmodf(s_yaw_angle_deg + 360.0f, 360.0f);
        float pitch_deg = fmodf(s_pitch_angle_deg + 360.0f, 360.0f);
        float roll_deg = fmodf(s_imu->euler.roll + 360.0f, 360.0f);
        app_gimbal_comm_send_angle_feedback_v2(
            (uint16_t)(yaw_deg * 65536.0f / 360.0f),
            (uint16_t)(pitch_deg * 65536.0f / 360.0f),
            (uint16_t)(roll_deg * 65536.0f / 360.0f), 10U);
    } else {
        app_gimbal_comm_send_imu_quaternion(
            (int16_t)(s_imu->quat.q0 * 30000.0f), (int16_t)(s_imu->quat.q1 * 30000.0f),
            (int16_t)(s_imu->quat.q2 * 30000.0f), (int16_t)(s_imu->quat.q3 * 30000.0f));
    }
    s_feedback_phase ^= 1U;
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
    s_chassis_omega_z = omega_z;
    s_chassis_omega_tick = drv_motor_port_get_tick();
    s_chassis_omega_is_valid = 1;
}

#endif
