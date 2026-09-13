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
#include "drv_dbus.h"
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

static drv_motor_data_t s_motor[GIMBAL_MOTOR_COUNT];//电机反馈数据
static drv_motor_data_t s_motor_rx[GIMBAL_MOTOR_COUNT];//电机反馈数据缓冲
static uint32_t s_motor_rx_tick[GIMBAL_MOTOR_COUNT];//电机反馈数据接收时间戳 (ms)
static uint8_t s_motor_rx_is_valid[GIMBAL_MOTOR_COUNT];//电机反馈数据有效标志
static uint8_t s_control_is_ready;// 云台控制就绪标志
static float s_last_gyro_yaw_rad_s;// 上次陀螺角速度 (rad/s)
static uint32_t s_chassis_omega_tick;// 底盘角速度反馈时间戳 (ms)
static uint8_t s_chassis_omega_is_valid;// 底盘角速度反馈有效标志
static int16_t          s_motor_current[GIMBAL_MOTOR_COUNT];//电机电流
static int16_t          s_motor_last_current[GIMBAL_MOTOR_COUNT];//上一帧的电机电流
static uint32_t         s_can_tx_error_count;// CAN发送错误计数, 用于调试


static drv_imu_t *s_imu;//接收外部的IMU实例
static float s_yaw_angle_rad;//当前yaw角度
static float s_pitch_angle_rad;//当前pitch角度，融合后的
static float s_gyro_yaw_rad_s;//当前IMUyaw角速度
static float s_gyro_pitch_rad_s;//当前IMUpitch角速度

static lib_lpf_t s_gyro_yaw_lpf;//当前IMUyaw角速度低通滤波
static lib_lpf_t s_gyro_pitch_lpf;//当前IMUpitch角速度低通滤波

static app_sentry_gimbal_cmd_t s_cmd;
static float s_target_yaw_rad;
static float s_target_pitch_rad;
static float s_target_yaw_rate_rad_s;

static float s_chassis_omega_z_rad_s;
static int16_t s_chassis_vx_cmd_mm_s;
static int16_t s_chassis_vy_cmd_mm_s;
static int16_t s_chassis_omega_cmd_lsb;//底盘转速 * 1000

static uint8_t s_chassis_emergency_stop;//底盘急停标志位

static const app_sentry_vmc_config_t s_vmc_cfg = {
    .k_virt        = 0.0f,
    .b_virt        = 0.0f,
    .k_ff          = 0.0f,
    .soft_limit_k  = 0.0f,
    .small_limit   = 0.0f,
    .max_out_s     = SENTRY_GM6020_CURRENT_MAX,
    .max_out_l     = SENTRY_GM6020_CURRENT_MAX,
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

static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd);
static float dbus_channel_normalize(uint16_t raw);
static float gimbal_get_yaw_rel_rad(void);
static void gimbal_send_chassis_dbus_cmd(void);

static void imu_fusion(void);

static void yaw_vmc_control(void);
static void pitch_control(void);
static void launch_control(void);

static void gimbal_send_yaw_can2(void);
static void launcher_send_can2(void);
static void gimbal_send_can1(void);

static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

void app_sentry_gimbal_init(drv_imu_t *imu)
{
    s_imu = imu;
    memset(s_motor_rx, 0, sizeof(s_motor_rx));
    memset(s_motor_rx_tick, 0, sizeof(s_motor_rx_tick));
    memset(s_motor_rx_is_valid, 0, sizeof(s_motor_rx_is_valid));
    s_control_is_ready = 0;
    s_last_gyro_yaw_rad_s = 0.0f;
    s_chassis_omega_z_rad_s = 0.0f;
    s_chassis_omega_tick = 0;
    s_chassis_omega_is_valid = 0;
    memset(s_motor,              0, sizeof(s_motor));
    memset(s_motor_current,      0, sizeof(s_motor_current));
    memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
    memset(&s_cmd,               0, sizeof(s_cmd));
    s_can_tx_error_count = 0;

    s_target_yaw_rad = 0.0f;
    s_target_pitch_rad = 0.0f;
    s_target_yaw_rate_rad_s = 0.0f;
    s_chassis_vx_cmd_mm_s = 0;
    s_chassis_vy_cmd_mm_s = 0;
    s_chassis_omega_cmd_lsb = 0;
    s_chassis_emergency_stop = 0U;
    s_yaw_angle_rad = 0.0f;
    s_pitch_angle_rad = 0.0f;
    s_gyro_yaw_rad_s = 0.0f;
    s_gyro_pitch_rad_s = 0.0f;
    pid_init_all();

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_YAW_LARGE,
                                 app_gimbal_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_YAW_SMALL,
                                 app_gimbal_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_PITCH,
                                 app_gimbal_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F1,
                                 app_gimbal_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_D1,
                                 app_gimbal_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_LAUNCH_F2,
                                 app_gimbal_on_motor_feedback);

    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK,
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

void app_sentry_gimbal_ctrl_1khz(void)
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
        || !isfinite(s_yaw_angle_rad) || !isfinite(s_pitch_angle_rad)
        || !isfinite(s_gyro_yaw_rad_s) || !isfinite(s_gyro_pitch_rad_s)) {
        memset(s_motor_current, 0, sizeof(s_motor_current));
        memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
        s_cmd.fire = APP_SENTRY_FIRE_OFF;
        s_target_yaw_rate_rad_s = 0.0f;
        s_chassis_emergency_stop = 1U;
        s_control_is_ready = 0;
        lib_pid_reset(&s_pid_pitch);
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        gimbal_send_chassis_dbus_cmd();
        gimbal_send_yaw_can2();
        return;
    }
    if (!s_control_is_ready) {
        s_target_yaw_rad = s_yaw_angle_rad;
        s_target_pitch_rad = s_pitch_angle_rad;
        s_last_gyro_yaw_rad_s = s_gyro_yaw_rad_s;
        s_control_is_ready = 1;
    }

    memcpy(s_motor_last_current, s_motor_current, sizeof(s_motor_current));
    dbus_to_cmd(&s_cmd);
    if (s_chassis_emergency_stop) {
        memset(s_motor_current, 0, sizeof(s_motor_current));
        memset(s_motor_last_current, 0, sizeof(s_motor_last_current));
        s_cmd.fire = APP_SENTRY_FIRE_OFF;
        s_target_yaw_rate_rad_s = 0.0f;
        lib_pid_reset(&s_pid_pitch);
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
        gimbal_send_chassis_dbus_cmd();
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
    gimbal_send_chassis_dbus_cmd();
    if (!s_control_is_ready || s_chassis_emergency_stop) {
        s_motor_current[MOTOR_FRIC_L] = 0;
        s_motor_current[MOTOR_FRIC_R] = 0;
        s_motor_current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        lib_pid_reset(&s_pid_fric_l);
        lib_pid_reset(&s_pid_fric_r);
    } else {
        launch_control();
    }
    launcher_send_can2();
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
    }
}

// ─── 私有函数实现 ─────────────────────────

static void pid_init_all(void)
{
    lib_pid_init(&s_pid_pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, SENTRY_GM6020_CURRENT_MIN, SENTRY_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_disc, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, SENTRY_M2006_CURRENT_MIN, SENTRY_M2006_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_l, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_r, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX, 1000.0f);

    lib_lpf_init(&s_gyro_yaw_lpf,   0.15f);
    lib_lpf_init(&s_gyro_pitch_lpf, 0.15f);
}

static void dbus_to_cmd(app_sentry_gimbal_cmd_t *cmd)
{
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    if (!dbus) {
        return;
    }

    cmd->mode = APP_SENTRY_GIMBAL_MODE_SPEED;//使用遥控时默认为速度模式，等效增量角度

    cmd->fire = dbus->rc.s1;

    cmd->yaw_rate_rad_s = dbus_channel_normalize(dbus->rc.ch[2])
                         * SENTRY_GIMBAL_MAX_YAW_RATE_RAD_S;
    cmd->pitch_rate_rad_s = dbus_channel_normalize(dbus->rc.ch[3])
                           * SENTRY_GIMBAL_MAX_PITCH_RATE_RAD_S;

    cmd->yaw_delta_rad = 0.0f;
    cmd->pitch_delta_rad = 0.0f;
    cmd->yaw_target_rad = s_target_yaw_rad;
    cmd->pitch_target_rad = s_target_pitch_rad;

    s_chassis_emergency_stop = (dbus->rc.s2 == DRV_DBUS_SWITCH_DOWN);
    if (s_chassis_emergency_stop) {
        s_chassis_vx_cmd_mm_s = 0;
        s_chassis_vy_cmd_mm_s = 0;
        s_chassis_omega_cmd_lsb = 0;
    } else {
        float vx_gimbal_mm_s = dbus_channel_normalize(dbus->rc.ch[0])
                             * SENTRY_CHASSIS_MAX_VX_MM_S;
        float vy_gimbal_mm_s = dbus_channel_normalize(dbus->rc.ch[1])
                             * SENTRY_CHASSIS_MAX_VY_MM_S;
        float gimbal_yaw_rel_rad = gimbal_get_yaw_rel_rad();
        float cos_yaw = cosf(gimbal_yaw_rel_rad);
        float sin_yaw = sinf(gimbal_yaw_rel_rad);

        /* 遥控器速度以云台坐标系表示，发送前转换到底盘坐标系。 */
        s_chassis_vx_cmd_mm_s = (int16_t)lroundf(
            cos_yaw * vx_gimbal_mm_s - sin_yaw * vy_gimbal_mm_s);
        s_chassis_vy_cmd_mm_s = (int16_t)lroundf(
            sin_yaw * vx_gimbal_mm_s + cos_yaw * vy_gimbal_mm_s);
        s_chassis_omega_cmd_lsb = (int16_t)lroundf(
            dbus_channel_normalize(dbus->rc.rolling_wheel)
            * SENTRY_CHASSIS_MAX_OMEGA_RAD_S
            / SENTRY_CHASSIS_OMEGA_RAD_S_PER_LSB);
    }

    s_target_yaw_rad += cmd->yaw_rate_rad_s * 0.001f;
    s_target_pitch_rad += cmd->pitch_rate_rad_s * 0.001f;
    s_target_yaw_rad = lib_rad_norm(s_target_yaw_rad);
}

static float dbus_channel_normalize(uint16_t raw)
{
    return lib_clamp(((float)raw - DRV_DBUS_CHANNEL_CENTER)
                          / DRV_DBUS_CHANNEL_RANGE, -1.0f, 1.0f);
}

static float gimbal_get_yaw_rel_rad(void)
{
    float large_yaw_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_LARGE_YAW_ENCODER_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(
        SENTRY_GIMBAL_LARGE_YAW_DIRECTION * large_yaw_rel_rad
        + SENTRY_GIMBAL_SMALL_YAW_DIRECTION * small_yaw_rel_rad);
}

static void gimbal_send_chassis_dbus_cmd(void)
{
    uint8_t data[8] = {0};
    int16_t vx = s_chassis_emergency_stop ? SENTRY_CHASSIS_ESTOP_VX_RAW
                                           : s_chassis_vx_cmd_mm_s;

    memcpy(data, &vx, sizeof(vx));
    memcpy(data + 2, &s_chassis_vy_cmd_mm_s, sizeof(s_chassis_vy_cmd_mm_s));
    memcpy(data + 4, &s_chassis_omega_cmd_lsb, sizeof(s_chassis_omega_cmd_lsb));
    (void)bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
}

static void imu_fusion(void)
{
    if (!s_imu) {
        return;
    }
    drv_imu_quat_to_euler(s_imu);

    s_yaw_angle_rad = lib_deg_to_rad(s_imu->euler.yaw);

    float pitch_rel_rad = lib_get_shortest_path(
        lib_enc_conv((float)s_motor[MOTOR_PITCH].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_PITCH_ENCODER_ZERO, LIB_ENC13_TO_RAD));
    s_pitch_angle_rad = lib_deg_to_rad(s_imu->euler.roll) + pitch_rel_rad;

    s_gyro_yaw_rad_s = lib_lpf_update(&s_gyro_yaw_lpf, s_imu->gyro.z);
    s_gyro_pitch_rad_s = lib_lpf_update(&s_gyro_pitch_lpf, s_imu->gyro.x);
}

static void yaw_vmc_control(void)
{
    float yaw_err_rad = lib_get_shortest_path(s_target_yaw_rad, s_yaw_angle_rad);

    float target_accel = s_vmc_cfg.k_tracking * yaw_err_rad
                       - s_vmc_cfg.b_tracking * s_target_yaw_rate_rad_s;
    target_accel = lib_clamp(target_accel,
                                  -s_vmc_cfg.max_accel, s_vmc_cfg.max_accel);
    s_target_yaw_rate_rad_s += target_accel * 0.001f;
    s_target_yaw_rate_rad_s = lib_clamp(s_target_yaw_rate_rad_s,
                                             -s_vmc_cfg.max_vel, s_vmc_cfg.max_vel);


    float omega_err = s_target_yaw_rate_rad_s - s_gyro_yaw_rad_s;
    float tau_vm = s_vmc_cfg.k_virt * yaw_err_rad
                 + s_vmc_cfg.b_virt * omega_err;


    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    float chassis_omega = s_chassis_omega_z_rad_s;
    uint8_t is_omega_fresh = s_chassis_omega_is_valid
                             && (uint32_t)(drv_motor_port_get_tick() - s_chassis_omega_tick) <= 200U;
    __set_PRIMASK(irq_state);
    if (is_omega_fresh) {
        tau_vm += s_vmc_cfg.k_ff * chassis_omega;
    }


    int32_t diff = (int32_t)s_motor[MOTOR_YAW_S].angle
                 - (int32_t)SENTRY_GIMBAL_SMALL_YAW_ENCODER_ZERO;
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


    float ang_accel = (s_gyro_yaw_rad_s - s_last_gyro_yaw_rad_s) / 0.001f;
    s_last_gyro_yaw_rad_s = s_gyro_yaw_rad_s;
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


    float step_s = out_small - s_motor_last_current[MOTOR_YAW_S];
    step_s = lib_clamp(step_s, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_current[MOTOR_YAW_S] = s_motor_last_current[MOTOR_YAW_S]
                                 + (int16_t)step_s;

    float step_b = out_big - s_motor_last_current[MOTOR_YAW_L];
    step_b = lib_clamp(step_b, -s_vmc_cfg.max_curr_step,
                            s_vmc_cfg.max_curr_step);
    s_motor_current[MOTOR_YAW_L] = s_motor_last_current[MOTOR_YAW_L]
                                 + (int16_t)step_b;

    if (small_rel >= s_vmc_cfg.small_limit && s_motor_current[MOTOR_YAW_S] > 0) {
        s_motor_current[MOTOR_YAW_S] = 0;
    } else if (small_rel <= -s_vmc_cfg.small_limit && s_motor_current[MOTOR_YAW_S] < 0) {
        s_motor_current[MOTOR_YAW_S] = 0;
    }
}

static void pitch_control(void)
{
    s_target_pitch_rad = lib_clamp(s_target_pitch_rad,
        SENTRY_GIMBAL_PITCH_MIN_RAD, SENTRY_GIMBAL_PITCH_MAX_RAD);

    float ff_gravity = cosf(s_pitch_angle_rad);

    s_motor_current[MOTOR_PITCH] = (int16_t)lib_pid_pos_calc(
        &s_pid_pitch, s_target_pitch_rad, s_pitch_angle_rad,
        ff_gravity, 0, s_gyro_pitch_rad_s);

    if (s_pitch_angle_rad >= SENTRY_GIMBAL_PITCH_MAX_RAD && s_motor_current[MOTOR_PITCH] > 0) {
        s_motor_current[MOTOR_PITCH] = 0;
    } else if (s_pitch_angle_rad <= SENTRY_GIMBAL_PITCH_MIN_RAD && s_motor_current[MOTOR_PITCH] < 0) {
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
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor[MOTOR_FRIC_L].speed));
        s_motor_current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor[MOTOR_FRIC_R].speed));
        s_motor_current[MOTOR_DISC] = (int16_t)lib_pid_calc(
            &s_pid_disc, SENTRY_DISC_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor[MOTOR_DISC].speed));
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor[MOTOR_FRIC_L].speed));
        s_motor_current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, SENTRY_FRICTION_TARGET_RAD_S,
            lib_rpm_to_rad_s((float)s_motor[MOTOR_FRIC_R].speed));
        s_motor_current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    }
}

static void gimbal_send_yaw_can2(void)
{
    uint8_t frame[8];

    /* 0x1FF: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[MOTOR_YAW_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[3] = LIB_LO_BYTE(s_motor_current[MOTOR_YAW_S]);
    frame[4] = LIB_HI_BYTE(s_motor_current[MOTOR_PITCH]);
    frame[5] = LIB_LO_BYTE(s_motor_current[MOTOR_PITCH]);
    if (bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_YAW, frame)) {
        s_can_tx_error_count++;
    }
}

static void launcher_send_can2(void)
{
    uint8_t frame[8];

    /* 0x200: [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[MOTOR_FRIC_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[MOTOR_DISC]);
    frame[3] = LIB_LO_BYTE(s_motor_current[MOTOR_DISC]);
    frame[4] = LIB_HI_BYTE(s_motor_current[MOTOR_FRIC_R]);
    frame[5] = LIB_LO_BYTE(s_motor_current[MOTOR_FRIC_R]);
    if (bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_LAUNCH, frame)) {
        s_can_tx_error_count++;
    }
}

static void gimbal_send_can1(void)
{
    if (!s_imu) {
        return;
    }
    /* 0x122: 速度反馈 yaw/pitch (rad/s) — 上位机监控用 */
    app_gimbal_comm_send_speed_feedback(
        s_gyro_yaw_rad_s,
        s_gyro_pitch_rad_s);

    /* 0x124: 角度反馈 yaw/pitch (rad) — 上位机+底盘用 */
    app_gimbal_comm_send_angle_feedback(
        s_yaw_angle_rad,
        s_pitch_angle_rad);

    app_gimbal_comm_send_imu_quaternion(
        (int16_t)(s_imu->quat.q0 * 30000.0f), (int16_t)(s_imu->quat.q1 * 30000.0f),
        (int16_t)(s_imu->quat.q2 * 30000.0f), (int16_t)(s_imu->quat.q3 * 30000.0f));
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
    s_chassis_omega_z_rad_s = omega_z;
    s_chassis_omega_tick = drv_motor_port_get_tick();
    s_chassis_omega_is_valid = 1;
}
