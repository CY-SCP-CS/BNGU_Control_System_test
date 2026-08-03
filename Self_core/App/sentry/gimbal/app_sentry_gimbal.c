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

#include "app_sentry_common.h"
#include "drv_dbus.h"
#include "drv_imu.h"
#include "drv_motor.h"
#include "lib_filter.h"
#include "lib_math.h"
#include "bsp_can.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"    /* 0x112 CAN ID */

#include <math.h>
#include <string.h>

/* ── 字节打包 ─────────────────────────────────── */
#define SENTRY_HI_BYTE(x)  ((uint8_t)((x) >> 8))
#define SENTRY_LO_BYTE(x)  ((uint8_t)(x))

/* ════════════════════════════════════════════════════
 * 电机 (CAN2)
 * ════════════════════════════════════════════════════ */

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
static int16_t          s_motor_current[GIMBAL_MOTOR_COUNT];
static int16_t          s_motor_last_current[GIMBAL_MOTOR_COUNT];

/* ════════════════════════════════════════════════════
 * IMU
 * ════════════════════════════════════════════════════ */

static drv_imu_t *s_imu;
static float s_yaw_angle_deg;
static float s_pitch_angle_deg;
static float s_gyro_yaw_dps;       /* yaw角速度 deg/s (LPF后)   */
static float s_gyro_pitch_dps;     /* pitch角速度 deg/s (LPF后) */

/* 两个独立LPF */
static lib_filter_lpf_t s_gyro_yaw_lpf;
static lib_filter_lpf_t s_gyro_pitch_lpf;

/* ════════════════════════════════════════════════════
 * 控制命令和目标
 * ════════════════════════════════════════════════════ */

static app_sentry_gimbal_cmd_t s_cmd;
static float s_target_yaw_deg;
static float s_target_pitch_deg;
static float s_target_yaw_vel;      /* 轨迹规划器输出的目标角速度 deg/s */
static float s_chassis_omega_z;     /**< 底盘角速度 (来自CAN1 0x112, VMC前馈) */

/* ════════════════════════════════════════════════════
 * VMC 配置 (from gimbal_2yaw)
 * ════════════════════════════════════════════════════ */

static const app_sentry_vmc_config_t s_vmc_cfg = {
    .k_virt        = 500.0f,
    .b_virt        = 500.0f,
    .k_ff          = 1.2f,
    .soft_limit_k  = 150.0f,
    .small_limit   = 50.0f,
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

/* ════════════════════════════════════════════════════
 * PID
 * ════════════════════════════════════════════════════ */

static lib_pid_t s_pid_pitch;      /* Kp=0 Ki=0 Kd=0 Kff_g=5000 */
static lib_pid_t s_pid_disc;       /* Kp=2.0 Ki=0.5 Kd=0.5        */
static lib_pid_t s_pid_fric_l;     /* Kp=3.0 Ki=0.5 Kd=1.0        */
static lib_pid_t s_pid_fric_r;     /* Kp=3.0 Ki=0.5 Kd=1.0        */

/* ════════════════════════════════════════════════════
 * 私有函数
 * ════════════════════════════════════════════════════ */

static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd);
static void imu_fusion(void);
static void yaw_vmc_control(void);
static void pitch_control(void);
static void launch_control(void);
static void motor_send_can2(void);
static void gimbal_send_can1(void);

/* ════════════════════════════════════════════════════
 * PID 初始化
 * ════════════════════════════════════════════════════ */

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_pitch,  0, 0, 0, 5000.0f, 0, 16000, 0,
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

/* ════════════════════════════════════════════════════
 * DBUS → 控制指令
 * ════════════════════════════════════════════════════ */

static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd)
{
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    if (!dbus) return;

    cmd->mode = (uint8_t)dbus->rc.s1;
    cmd->fire = (uint8_t)dbus->rc.s2;

    cmd->yaw_speed   = (float)dbus->rc.ch[2] * 1200.0f / 660.0f;
    cmd->pitch_speed = (float)dbus->rc.ch[3] * 4.3f  / 660.0f;

    cmd->yaw_inc   = (float)dbus->rc.ch[0] * 2.0f  / 660.0f;
    cmd->pitch_inc = (float)dbus->rc.ch[1] * 0.5f  / 660.0f;

    cmd->yaw_angle   = 0;
    cmd->pitch_angle = 0;

    /* ABS_ANGLE 模式: 从 CAN1 视觉指令获取目标角度 */
    {
        const app_gimbal_angle_cmd_t *vis =
            app_gimbal_comm_get_angle_no_shoot();   /* 0x123 优先 */
        if (vis->yaw_abs != 0 || vis->pitch_abs != 0) {
            cmd->yaw_angle   = lib_math_rad2deg(vis->yaw_abs);
            cmd->pitch_angle = lib_math_rad2deg(vis->pitch_abs);
        } else {
            /* 0x127 射击模式角度 (0xFFFF标记=检测到目标) */
            const app_gimbal_angle_cmd_t *vis_s =
                app_gimbal_comm_get_angle_shoot();
            if (vis_s->yaw_abs != 0 || vis_s->pitch_abs != 0) {
                cmd->yaw_angle   = lib_math_rad2deg(vis_s->yaw_abs);
                cmd->pitch_angle = lib_math_rad2deg(vis_s->pitch_abs);
            }
        }
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
}

/* ════════════════════════════════════════════════════
 * IMU 融合
 * ════════════════════════════════════════════════════ */

static void imu_fusion(void)
{
    if (!s_imu) return;

    drv_imu_quat_to_euler(s_imu);

    /* yaw: 直接 BMI088 */
    s_yaw_angle_deg = s_imu->euler.yaw;

    /* pitch: BMI088 roll + pitch电机相对零点 */
    float pitch_rel = (float)((int16_t)s_motor[MOTOR_PITCH].angle
                              - SENTRY_GIMBAL_PITCH_ENCODER_ZERO)
                      * 360.0f / 8192.0f;
    s_pitch_angle_deg = s_imu->euler.roll + pitch_rel;

    /* 陀螺角速度 LPF (两个独立滤波器) */
    s_gyro_yaw_dps   = lib_filter_lpf_update(&s_gyro_yaw_lpf,
                         s_imu->gyro.z * (180.0f / (float)LIB_MATH_PI));
    s_gyro_pitch_dps = lib_filter_lpf_update(&s_gyro_pitch_lpf,
                         s_imu->gyro.x * (180.0f / (float)LIB_MATH_PI));
}

/* ════════════════════════════════════════════════════
 * 双Yaw VMC控制
 * ════════════════════════════════════════════════════ */

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

    /* 底盘前馈 (来自 CAN1 0x112 omega_z) */
    tau_vm += s_vmc_cfg.k_ff * s_chassis_omega_z;

    /* ── 3. Sigmoid 加权 ── */
    float small_rel = SENTRY_ENC_TO_DEG(s_motor[MOTOR_YAW_S].angle);
    float sigmoid_in = (fabsf(small_rel) - 18.0f) * 0.4f;
    float ratio  = lib_math_fast_sigmoid(sigmoid_in);
    float w_big   = 0.4f + 0.5f * ratio;
    float w_small = 1.0f - w_big;

    /* ── 4. 惯量补偿 (用上一周期角速度差分近似加速度) ── */
    static float s_last_gyro_yaw;
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
}

/* ════════════════════════════════════════════════════
 * Pitch 重力补偿
 * ════════════════════════════════════════════════════ */

static void pitch_control(void)
{
    s_target_pitch_deg = lib_math_clamp(s_target_pitch_deg,
        SENTRY_GIMBAL_PITCH_MIN_DEG, SENTRY_GIMBAL_PITCH_MAX_DEG);

    float pitch_rad = lib_math_deg2rad(s_pitch_angle_deg);
    float ff_gravity = cosf(pitch_rad);

    s_motor_current[MOTOR_PITCH] = (int16_t)lib_pid_pos_calc(
        &s_pid_pitch, s_target_pitch_deg, s_pitch_angle_deg,
        ff_gravity, 0, s_gyro_pitch_dps);
}

/* ════════════════════════════════════════════════════
 * 发射控制
 * ════════════════════════════════════════════════════ */

static void launch_control(void)
{
    switch (s_cmd.fire) {
    case APP_SENTRY_FIRE_OFF:
        s_motor_current[MOTOR_FRIC_L] = 0;
        s_motor_current[MOTOR_FRIC_R] = 0;
        s_motor_current[MOTOR_DISC]   = 0;
        memset(&s_pid_fric_l.integral, 0, sizeof(float) * 3);
        memset(&s_pid_fric_r.integral, 0, sizeof(float) * 3);
        memset(&s_pid_disc.integral,   0, sizeof(float) * 3);
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
        break;

    default:
        break;
    }
}

/* ════════════════════════════════════════════════════
 * CAN2 发送 (板内电机)
 * ════════════════════════════════════════════════════ */

static void motor_send_can2(void)
{
    uint8_t frame[8];

    /* 0x1FF: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = SENTRY_HI_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[1] = SENTRY_LO_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[2] = SENTRY_HI_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[3] = SENTRY_LO_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[4] = SENTRY_HI_BYTE(s_motor_current[MOTOR_PITCH]);
    frame[5] = SENTRY_LO_BYTE(s_motor_current[MOTOR_PITCH]);
    bsp_can_send(&hcan2, SENTRY_CAN_GIMBAL_TX_YAW, frame);

    /* 0x200: [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = SENTRY_HI_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[1] = SENTRY_LO_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[2] = SENTRY_HI_BYTE(s_motor_current[MOTOR_DISC]);
    frame[3] = SENTRY_LO_BYTE(s_motor_current[MOTOR_DISC]);
    frame[4] = SENTRY_HI_BYTE(s_motor_current[MOTOR_FRIC_R]);
    frame[5] = SENTRY_LO_BYTE(s_motor_current[MOTOR_FRIC_R]);
    bsp_can_send(&hcan2, SENTRY_CAN_GIMBAL_TX_LAUNCH, frame);
}

/* ════════════════════════════════════════════════════
 * CAN1 发送 (板间: 云台→底盘/上位机)
 * ════════════════════════════════════════════════════ */

static void gimbal_send_can1(void)
{
    if (!s_imu) return;

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

    /* 0x130: 角度反馈 v2 (deg, uint16 65536/rev) — 上位机 */
    uint16_t yaw_u16   = (uint16_t)(s_yaw_angle_deg   * 65536.0f / 360.0f);
    uint16_t pitch_u16 = (uint16_t)(s_pitch_angle_deg * 65536.0f / 360.0f);
    uint16_t roll_u16  = (uint16_t)(s_imu->euler.roll * 65536.0f / 360.0f);
    app_gimbal_comm_send_angle_feedback_v2(yaw_u16, pitch_u16, roll_u16, 0);

    /* 0x233: IMU四元数 (int16 ×4, 除30000后用) — 上位机视觉 */
    float q0 = s_imu->quat.q0, q1 = s_imu->quat.q1;
    float q2 = s_imu->quat.q2, q3 = s_imu->quat.q3;
    app_gimbal_comm_send_imu_quaternion(
        (int16_t)(q0 * 30000), (int16_t)(q1 * 30000),
        (int16_t)(q2 * 30000), (int16_t)(q3 * 30000));
}

/* ════════════════════════════════════════════════════
 * CAN1 接收: 底盘 ωz (来自 0x112, VMC前馈用)
 *   0x112: [power×100(int16), _, _, omega_z(float)]
 * ════════════════════════════════════════════════════ */

static void on_chassis_power_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    memcpy(&s_chassis_omega_z, data + 4, sizeof(float));
}

/* ════════════════════════════════════════════════════
 * 公有接口
 * ════════════════════════════════════════════════════ */

void app_sentry_gimbal_init(drv_imu_t *imu)
{
    s_imu = imu;
    memset(s_motor,              0, sizeof(s_motor));
    memset(s_motor_current,      0, sizeof(s_motor_current));
    memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
    memset(&s_cmd,               0, sizeof(s_cmd));

    s_target_yaw_deg   = 0;
    s_target_pitch_deg = 0;
    s_target_yaw_vel   = 0;
    s_yaw_angle_deg    = 0;
    s_pitch_angle_deg  = 0;
    s_gyro_yaw_dps     = 0;
    s_gyro_pitch_dps   = 0;

    pid_init_all();

    /* ── 注册 CAN2 电机反馈回调 (板内) ── */
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_YAW_LARGE,
                                 app_sentry_gimbal_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_YAW_SMALL,
                                 app_sentry_gimbal_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_PITCH,
                                 app_sentry_gimbal_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F1,
                                 app_sentry_gimbal_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_D1,
                                 app_sentry_gimbal_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F2,
                                 app_sentry_gimbal_motor_feedback);

    /* ── 注册 CAN1 板间: 底盘 ωz (0x112, VMC前馈用) ── */
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_POWER_FEEDBACK,
                                 on_chassis_power_feedback);
}

void app_sentry_gimbal_ahrs_update(float dt)
{
    if (!s_imu) return;

    /* 读原始数据 + 转换 */
    drv_imu_read_acc_raw(s_imu);
    drv_imu_read_gyro_raw(s_imu);
    drv_imu_data_convert(s_imu);

    /* Mahony AHRS (必须有, 否则四元数不更新) */
    drv_imu_mahony_update(s_imu, dt);
}

void app_sentry_gimbal_control(void)
{
    /* 保存上周期电流 (slew rate) */
    memcpy(s_motor_last_current, s_motor_current, sizeof(s_motor_current));

    /* ── 1. DBUS → 指令 ── */
    dbus_to_cmd(&s_cmd);

    /* ── 2. IMU 融合 (四元数已在 1kHz AHRS 中更新) ── */
    imu_fusion();

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

void app_sentry_gimbal_motor_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    (void)len;
    int idx = -1;
    switch (std_id) {
    case SENTRY_CAN_GIMBAL_YAW_LARGE:  idx = MOTOR_YAW_L;   break;
    case SENTRY_CAN_GIMBAL_YAW_SMALL:  idx = MOTOR_YAW_S;   break;
    case SENTRY_CAN_GIMBAL_PITCH:      idx = MOTOR_PITCH;   break;
    case SENTRY_CAN_GIMBAL_LAUNCH_F1:  idx = MOTOR_FRIC_L;  break;
    case SENTRY_CAN_GIMBAL_LAUNCH_D1:  idx = MOTOR_DISC;    break;
    case SENTRY_CAN_GIMBAL_LAUNCH_F2:  idx = MOTOR_FRIC_R;  break;
    default: return;
    }
    if (idx >= 0 && idx < GIMBAL_MOTOR_COUNT) {
        drv_motor_solve_dji_data(data, &s_motor[idx]);
    }
}

void app_sentry_gimbal_get_angles(float *yaw, float *pitch)
{
    if (yaw)   *yaw   = s_yaw_angle_deg;
    if (pitch) *pitch = s_pitch_angle_deg;
}
