/**
 * @file    app_sentry_chassis.c
 * @brief   Sentry 哨兵底盘 — 双舵轮 swervedrive 控制
 * @note    Ported from Steering_wheel_Chasssis_test
 *
 * 舵轮 (swervedrive): 2轮独立转向+驱动, 底盘级力/力矩PID + 轮级FF-PID
 *
 * 电机 (CAN2):
 *   M1=右驱动 M3508 (0x201), M0=左驱动 M3508 (0x202)
 *   S0=左转向 M3508 (0x205), S1=右转向 M3508 (0x206)
 *
 * 控制流水线 (1kHz):
 *   CAN1 0x111 或 DBUS → body-frame目标速度
 *   → 在车体坐标系完成解算（云台绝对 yaw 仅作遥测）
 *   → 逆运动学 (每轮角度+速度, >90°反转优化)
 *   → 正运动学 → body-frame估算速度 + ωz
 *   → 底盘PID → 极坐标力/力矩
 *   → 力分配 → 每轮驱动前馈
 *   → 驱动FF-PID + 转向角度PID
 *   → 裁判功率上限 + 功率计 PI 反馈 → CAN2发送
 */
#include "app_sentry_chassis.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"

#include "drv_dbus.h"
#include "drv_motor.h"
#include "drv_power_measure.h"

#include "app_chassis_comm.h"
#include "app_gimbal_comm.h"
#include "drv_referee.h"

#include <math.h>
#include <string.h>

// ─── 私有宏 ─────────────────────────
#define MOTOR_COUNT                        4
#define SENTRY_POWER_DEFAULT_W             100.0f
#define SENTRY_POWER_LIMIT_RESERVE_W       5.0f
#define SENTRY_POWER_ERROR_DEADBAND_W      1.0f
#define SENTRY_POWER_SCALE_RECOVERY        0.005f
#define SENTRY_POWER_METER_TIMEOUT_MS      50U
#define SENTRY_POWER_METER_FALLBACK_SCALE  0.3f
#define SENTRY_REFEREE_TIMEOUT_MS          500U
#define SENTRY_CAN_CMD_TIMEOUT_MS          200U
#define SENTRY_YAW_TIMEOUT_MS              200U

enum { M_DRIVE_R=0, M_DRIVE_L=1, M_STEER_L=2, M_STEER_R=3 };

static drv_motor_data_t s_motor[MOTOR_COUNT];
static int16_t          s_motor_current[MOTOR_COUNT];     /* 电流输出      */
static int16_t          s_motor_speed_rpm[MOTOR_COUNT];   /* DJI 反馈边界：RPM */
static drv_motor_data_t s_motor_rx[MOTOR_COUNT];
static uint32_t s_motor_rx_tick[MOTOR_COUNT];
static uint8_t s_motor_rx_is_valid[MOTOR_COUNT];
static uint8_t s_feedback_divider;
static const uint8_t s_drive_index[2] = { M_DRIVE_L, M_DRIVE_R };
static const float s_drive_direction[2] = { -1.0f, 1.0f };
static int16_t          s_motor_ff[2];                    /* 驱动前馈扭矩   */

static app_sentry_swerve_wheel_t s_swerve_cur[2];  /**< 当前状态 [0]=左 [1]=右  */
static app_sentry_swerve_wheel_t s_swerve_tar[2];  /**< 目标状态                 */
static app_sentry_chassis_speed_t s_body_tar;      /**< body-frame 目标速度      */
static app_sentry_chassis_speed_t s_body_cur;      /**< body-frame 估算速度      */
static app_sentry_chassis_state_t s_chassis_state; /**< 全局状态                 */

/* yaw 来源: CAN1 0x124 云台 BMI088 角度 */
static float s_yaw_from_can_rad;      /**< 云台 BMI088 yaw (rad)           */
static uint8_t s_yaw_from_can_valid;  /**< 0x124 数据是否已到达             */

static float s_force, s_force_angle, s_torque;     /**< PID输出的力/力矩         */

static drv_power_data_t s_power_meter_rx;
static uint32_t s_power_meter_rx_tick;
static uint8_t s_power_meter_rx_is_valid;
static uint32_t s_power_filter_tick;
static uint32_t s_power_feedback_tick;
static lib_filter_lpf_t s_power_measure_lpf;

static float s_power_scale = 1.0f;      /**< 实测功率 PI 得到的电流缩放系数 */
static float s_power_measured_w;        /**< 功率计滤波后的实测功率          */
static float s_power_current_w;         /**< 对外发布的实测功率              */
static float s_power_limit_w = SENTRY_POWER_DEFAULT_W;
static float s_power_target_w = SENTRY_POWER_DEFAULT_W;
static float s_battery_voltage_v;
static float s_battery_current_a;
static uint8_t s_power_meter_is_online;
static uint8_t s_referee_is_online;
static uint8_t s_robot_level;
static uint8_t s_is_chassis_output_enabled = 1U;

static lib_pid_t s_pid_x;        /**< vx PID */
static lib_pid_t s_pid_y;        /**< vy PID  同上                                 */
static lib_pid_t s_pid_w;        /**< vw PID  同上                                 */
static lib_pid_t s_pid_steer[2]; /**< 舵轮 PID */
static lib_pid_t s_pid_drive[2]; /**< 驱动轮 PID*/
static lib_pid_t s_pid_power;    /**< 功率控制 PID */

static uint8_t  s_use_can_cmd;            /**< 当前使用CAN指令 (1=CAN, 0=DBUS) */
static uint32_t s_can_tx_error_count;     /**< CAN 发送失败计数         */

static uint32_t s_last_yaw_tick;      /**< 最后一次收到0x124的tick */

// ─── 私有函数声明 ─────────────────────────

static void pid_init_all(void);
static float calc_logical_angle(uint16_t raw_enc, uint16_t offset);
static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd);
static void forward_kinematics(void);
static void chassis_pid(void);
static void force_distribute(void);
static void wheel_control(void);
static void power_input_update(uint32_t now, float command_ratio);
static uint8_t power_measure_read_snapshot(drv_power_data_t *power,
                                           uint32_t *sample_tick,
                                           uint32_t now);
static void power_state_update(void);
static void power_limit(void);
static void chassis_send_can2(void);
static void chassis_send_can1(void);
static void on_gimbal_angle_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
static void on_power_measure_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

// ─── 公有接口实现 ─────────────────────────

void app_sentry_chassis_init(void)
{
    memset(s_motor,           0, sizeof(s_motor));
    memset(s_motor_current,   0, sizeof(s_motor_current));
    memset(s_motor_speed_rpm, 0, sizeof(s_motor_speed_rpm));
    memset(s_motor_rx, 0, sizeof(s_motor_rx));
    memset(s_motor_rx_tick, 0, sizeof(s_motor_rx_tick));
    memset(s_motor_rx_is_valid, 0, sizeof(s_motor_rx_is_valid));
    s_feedback_divider = 0;
    memset(s_motor_ff,        0, sizeof(s_motor_ff));
    memset(s_swerve_cur,      0, sizeof(s_swerve_cur));
    memset(s_swerve_tar,      0, sizeof(s_swerve_tar));
    memset(&s_body_tar,       0, sizeof(s_body_tar));
    memset(&s_body_cur,       0, sizeof(s_body_cur));
    memset(&s_chassis_state,  0, sizeof(s_chassis_state));
    memset(&s_power_meter_rx, 0, sizeof(s_power_meter_rx));
    s_power_meter_rx_tick = 0U;
    s_power_meter_rx_is_valid = 0U;
    s_power_filter_tick = 0xFFFFFFFFU;
    s_power_feedback_tick = 0xFFFFFFFFU;
    lib_filter_lpf_init(&s_power_measure_lpf, 0.2f);
    s_power_scale = SENTRY_POWER_METER_FALLBACK_SCALE;
    s_power_measured_w = 0.0f;
    s_power_current_w = 0.0f;
    s_power_limit_w = SENTRY_POWER_DEFAULT_W;
    s_power_target_w = SENTRY_POWER_DEFAULT_W;
    s_battery_voltage_v = 0.0f;
    s_battery_current_a = 0.0f;
    s_power_meter_is_online = 0U;
    s_referee_is_online = 0U;
    s_robot_level = 0U;
    s_is_chassis_output_enabled = 1U;
    s_force = s_force_angle = s_torque = 0;
    s_yaw_from_can_rad   = 0.0f;
    s_yaw_from_can_valid = 0;
    s_last_yaw_tick      = 0;
    s_can_tx_error_count = 0;
    s_use_can_cmd        = 0;

    pid_init_all();

    /* 注册 CAN2 电机反馈 (板内) */
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R,
                                 app_chassis_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L,
                                 app_chassis_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_STEER_L,
                                 app_chassis_on_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_STEER_R,
                                 app_chassis_on_motor_feedback);
    drv_power_port_can_init(&hcan2, on_power_measure_feedback);

    /* CAN1 板间: 0x111 速度指令由 app_chassis_comm 统一接收,
     *   哨兵在控制循环中通过 app_chassis_comm_read_speed_cmd() 读取快照 */

    /* 注册 CAN1 板间: 云台 yaw 角度 0x124 */
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK,
                                 on_gimbal_angle_feedback);
}
void app_sentry_chassis_ctrl(void)
{
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    uint32_t now;
    uint8_t is_motor_online = 1;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    now = drv_motor_port_get_tick();
    memcpy(s_motor, s_motor_rx, sizeof(s_motor));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_speed_rpm[i] = s_motor[i].speed;
        if (!s_motor_rx_is_valid[i] || (uint32_t)(now - s_motor_rx_tick[i]) > SENTRY_MOTOR_TIMEOUT_MS) {
            is_motor_online = 0;
        }
    }
    __set_PRIMASK(irq_state);

    app_chassis_speed_cmd_t can_cmd;
    s_use_can_cmd = app_chassis_comm_read_speed_cmd(&can_cmd, SENTRY_CAN_CMD_TIMEOUT_MS);
    if (dbus && dbus->rc.s1 == DRV_DBUS_SWITCH_UP) {
        s_use_can_cmd = 0;
    }

    float command_power_ratio = 1.0f;
    if (s_use_can_cmd) {
        command_power_ratio = lib_math_clamp(can_cmd.power_ratio,
                                             0.0f, 1.0f);
    }
    power_input_update(now, command_power_ratio);

    if (!is_motor_online || (dbus && dbus->rc.s1 == DRV_DBUS_SWITCH_DOWN)
        || (!s_use_can_cmd && !dbus)
        || (s_referee_is_online && !s_is_chassis_output_enabled)) {
        /* 急停和失联必须旁路全部 PID、功率缩放及转向保持。 */
        memset(s_motor_current, 0, sizeof(s_motor_current));
        memset(&s_body_tar, 0, sizeof(s_body_tar));
        memset(&s_body_cur, 0, sizeof(s_body_cur));
        memset(&s_chassis_state, 0, sizeof(s_chassis_state));
        if (!s_power_meter_is_online) {
            s_power_current_w = 0.0f;
        }
        s_power_scale = 0.0f;
        pid_init_all();
        power_state_update();
        chassis_send_can2();
        if (++s_feedback_divider >= 10U) {
            s_feedback_divider = 0;
            chassis_send_can1();
        }
        return;
    }
    if (s_use_can_cmd) {
        s_body_tar.v_x = can_cmd.vx_mm_s;
        s_body_tar.v_y = can_cmd.vy_mm_s;
        s_body_tar.v_w = can_cmd.omega_z_rad_s;
    } else {
        /* 解码器输出原始 0..2047 通道值，先去除 1024 中点。 */
        s_body_tar.v_x = ((float)dbus->rc.ch[3] - DRV_DBUS_CHANNEL_CENTER)
                         * SENTRY_MAX_LINEAR_SPEED / DRV_DBUS_CHANNEL_RANGE;
        s_body_tar.v_y = ((float)dbus->rc.ch[2] - DRV_DBUS_CHANNEL_CENTER)
                         * SENTRY_MAX_LINEAR_SPEED / DRV_DBUS_CHANNEL_RANGE;
        s_body_tar.v_w = ((float)dbus->rc.ch[1] - DRV_DBUS_CHANNEL_CENTER)
                         * SENTRY_MAX_OMEGA / DRV_DBUS_CHANNEL_RANGE;
    }
    s_body_tar.v_x = lib_math_clamp(s_body_tar.v_x, -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    s_body_tar.v_y = lib_math_clamp(s_body_tar.v_y, -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    s_body_tar.v_w = lib_math_clamp(s_body_tar.v_w, -SENTRY_MAX_OMEGA, SENTRY_MAX_OMEGA);

    /* ── 2. 当前舵轮状态 (编码器→角度) ── */
    s_swerve_cur[0].angle = calc_logical_angle(s_motor[M_STEER_L].angle,
                                               SENTRY_SWERVE_0_OFFSET);
    s_swerve_cur[1].angle = calc_logical_angle(s_motor[M_STEER_R].angle,
                                               SENTRY_SWERVE_1_OFFSET);
    s_swerve_cur[0].speed = -lib_math_motor_rpm_to_linear_mm_s(
        (float)s_motor_speed_rpm[M_DRIVE_L], SENTRY_WHEEL_RADIUS_MM,
        SENTRY_REDUCTION_RATIO);
    s_swerve_cur[1].speed = lib_math_motor_rpm_to_linear_mm_s(
        (float)s_motor_speed_rpm[M_DRIVE_R], SENTRY_WHEEL_RADIUS_MM,
        SENTRY_REDUCTION_RATIO);
    s_swerve_cur[0].rev = s_swerve_cur[1].rev = 1;   /* 正运动学用 */

    /* ── 3. 云台绝对 yaw 仅作遥测，不作为底盘姿态参与解算 ── */
    irq_state = __get_PRIMASK();
    __disable_irq();
    if (s_yaw_from_can_valid
        && (now - s_last_yaw_tick) <= SENTRY_YAW_TIMEOUT_MS) {
        s_chassis_state.yaw_rad = s_yaw_from_can_rad;
    } else {
        s_chassis_state.yaw_rad = 0.0f;
    }

    __set_PRIMASK(irq_state);

    /* ── 4. 逆运动学: body目标 → 每轮角度(rad)+速度(mm/s) ── */
    inverse_kinematics(&s_body_tar);
    /* 同比缩放所有自由度，保持转弯曲率且不超过电机转速上限。 */
    /* ── 5. 正运动学: 每轮状态 → body估算速度 + ωz ── */
    forward_kinematics();
    s_chassis_state.omega_z = s_body_cur.v_w;

    /* ── 6. 底盘PID: 速度误差 → 力/力矩 ── */
    chassis_pid();

    /* ── 7. 力分配 → 驱动前馈 ── */
    force_distribute();

    /* ── 8. 轮级控制: 驱动FF-PID + 转向角度PID ── */
    wheel_control();

    /* ── 10. 功率限制 ── */
    power_limit();

    /* ── 11. CAN2 发送 ── */
    chassis_send_can2();

    /* ── 12. CAN1 发送 (功率+ωz给云台VMC前馈) ── */
    if (++s_feedback_divider >= 10U) {
        s_feedback_divider = 0;
        chassis_send_can1();
    }

    /* ── 13. 更新全局状态 ── */
    memcpy(s_chassis_state.wheel, s_swerve_cur, sizeof(s_swerve_cur));
    s_chassis_state.speed = s_body_cur;
    power_state_update();
}

void app_chassis_on_motor_feedback(uint32_t std_id, uint8_t *data,
                                       uint8_t len)
{
    if (!data || len != 8U) {
        return;
    }
    int motor_index = -1;
    switch (std_id) {
    case SENTRY_CAN_CHASSIS_DRIVE_R:   motor_index = M_DRIVE_R;   break;
    case SENTRY_CAN_CHASSIS_DRIVE_L:   motor_index = M_DRIVE_L;   break;
    case SENTRY_CAN_CHASSIS_STEER_L:   motor_index = M_STEER_L;   break;
    case SENTRY_CAN_CHASSIS_STEER_R:   motor_index = M_STEER_R;   break;
    default: return;
    }
    if (motor_index >= 0 && motor_index < MOTOR_COUNT) {
        drv_motor_solve_dji_data(data, &s_motor_rx[motor_index]);
        s_motor_rx_tick[motor_index] = drv_motor_port_get_tick();
        s_motor_rx_is_valid[motor_index] = 1;
    }
}

const app_sentry_chassis_state_t *app_chassis_get_state(void)
{
    return &s_chassis_state;
}

// ─── 私有函数实现 ─────────────────────────

static void pid_init_all(void)
{
    /* 底盘速度PID */
    lib_pid_init(&s_pid_x, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);
    lib_pid_init(&s_pid_y, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);
    lib_pid_init(&s_pid_w, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);

    /* 转向角度PID: Kp=500, Ki=0.5, out±16384, i=2000 */
    int i;
    for (i = 0; i < 2; i++) {
        lib_pid_init(&s_pid_steer[i], 500.0f * 180.0f / LIB_MATH_PI,
                     0.5f * 180.0f / LIB_MATH_PI, 0.0f,
                     0, 0, 0, 0, -16384, 16384,
                     2000.0f * LIB_MATH_PI / 180.0f);
    }

    /* 驱动转速FF-PID: Kp=1.0, Kff_v=0.8, out±12000, i=2000 */
    for (i = 0; i < 2; i++) {
        float mm_s_per_rpm = lib_math_motor_rpm_to_linear_mm_s(
            1.0f, SENTRY_WHEEL_RADIUS_MM, SENTRY_REDUCTION_RATIO);
        lib_pid_init(&s_pid_drive[i], 1.0f / mm_s_per_rpm,
                     0.0f, 0.0f, 0.8f / mm_s_per_rpm, 0,
                     10000, 0, -12000, 12000,
                     lib_math_motor_rpm_to_linear_mm_s(
                         2000.0f, SENTRY_WHEEL_RADIUS_MM,
                         SENTRY_REDUCTION_RATIO));
    }

    /* 误差单位为 W，输出是附加在 1.0 上的负缩放量。 */
    lib_pid_init(&s_pid_power, 0.05f, 0.001f, 0.0f, 0, 0,
                 0, 0, -1.0f, 0.0f, 1000.0f);
}

static float calc_logical_angle(uint16_t raw_enc, uint16_t offset)
{
    int32_t diff = (int32_t)raw_enc - (int32_t)offset;
    if (diff > 4096) {
        diff -= 8192;
    }
    else if (diff < -4095) {
        diff += 8192;
    }
    return lib_math_enc_convert((float)diff, LIB_MATH_ENC13_TO_RAD);
}

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd)
{
    float vx = lib_math_clamp(body_spd->v_x,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vy = lib_math_clamp(body_spd->v_y,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vw = lib_math_clamp(body_spd->v_w,
                              -SENTRY_MAX_OMEGA, SENTRY_MAX_OMEGA);

    /* 轮位置沿用原解算的对角布置；所有量均在车体坐标系。 */
    for (int i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float ix = vx - sign * vw * SENTRY_WHEEL_HALF_TRACK_MM;
        float iy = vy + sign * vw * SENTRY_WHEEL_HALF_BASE_MM;

        float raw_speed = sqrtf(ix * ix + iy * iy);
        if (raw_speed < 0.001f) {
            s_swerve_tar[i].angle = s_swerve_cur[i].angle;
            s_swerve_tar[i].speed = 0.0f;
            s_swerve_tar[i].rev = 1;
            continue;
        }
        float raw_angle = atan2f(iy, ix);

        /* 超过 PI/2 时反转驱动方向，减小转向行程。 */
        float diff = lib_math_get_shortest_path(raw_angle, s_swerve_cur[i].angle);

        if (fabsf(diff) > LIB_MATH_PI * 0.5f) {
            s_swerve_tar[i].rev  = -1;
            s_swerve_tar[i].angle = lib_math_rad_normalize(
                raw_angle + (diff > 0.0f ? -LIB_MATH_PI : LIB_MATH_PI));
        } else {
            s_swerve_tar[i].rev  = 1;
            s_swerve_tar[i].angle = raw_angle;
        }
        s_swerve_tar[i].speed = raw_speed * (float)s_swerve_tar[i].rev;
    }
}

static void forward_kinematics(void)
{
    float velocity_x[2], velocity_y[2];
    for (int i = 0; i < 2; i++) {
        float speed = lib_math_motor_rpm_to_linear_mm_s(
            (float)s_motor_speed_rpm[s_drive_index[i]], SENTRY_WHEEL_RADIUS_MM,
            SENTRY_REDUCTION_RATIO) * s_drive_direction[i];
        float angle_rad = s_swerve_cur[i].angle;
        velocity_x[i] = speed * cosf(angle_rad);
        velocity_y[i] = speed * sinf(angle_rad);
    }
    float half_track = SENTRY_WHEEL_HALF_TRACK_MM;
    float half_base = SENTRY_WHEEL_HALF_BASE_MM;
    s_body_cur.v_x = (velocity_x[0] + velocity_x[1]) * 0.5f;
    s_body_cur.v_y = (velocity_y[0] + velocity_y[1]) * 0.5f;
    s_body_cur.v_w = (half_track * (velocity_x[1] - velocity_x[0])
                     + half_base * (velocity_y[0] - velocity_y[1]))
                    / (2.0f * (half_track * half_track + half_base * half_base));
}

static void chassis_pid(void)
{
    float fx = lib_pid_calc(&s_pid_x, s_body_tar.v_x, s_body_cur.v_x);
    float fy = lib_pid_calc(&s_pid_y, s_body_tar.v_y, s_body_cur.v_y);
    s_force       = sqrtf(fx * fx + fy * fy);
    s_force_angle = atan2f(fy, fx);
    s_torque      = lib_pid_calc(&s_pid_w, s_body_tar.v_w, s_body_cur.v_w);
}

static void force_distribute(void)
{
    float fx = s_force * cosf(s_force_angle);
    float fy = s_force * sinf(s_force_angle);
    float half_track = SENTRY_WHEEL_HALF_TRACK_MM;
    float half_base = SENTRY_WHEEL_HALF_BASE_MM;
    float lever_squared = half_track * half_track + half_base * half_base;

    int i;
    for (i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float fix = fx * 0.5f - sign * s_torque * half_track / (2.0f * lever_squared);
        float fiy = fy * 0.5f + sign * s_torque * half_base / (2.0f * lever_squared);
        float angle_rad = s_swerve_cur[i].angle;

        /* 投影到车轮前进方向 */
        s_motor_ff[i] = (int16_t)lib_math_clamp(
            (fix * cosf(angle_rad) + fiy * sinf(angle_rad))
            * SENTRY_WHEEL_RADIUS_MM / SENTRY_REDUCTION_RATIO,
            -10000, 10000);
    }
}

static void wheel_control(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        uint8_t drive_index = s_drive_index[i];
        float target_speed = s_swerve_tar[i].speed * s_drive_direction[i];
        float measured_speed = lib_math_motor_rpm_to_linear_mm_s(
            (float)s_motor_speed_rpm[drive_index], SENTRY_WHEEL_RADIUS_MM,
            SENTRY_REDUCTION_RATIO);
        s_motor_current[drive_index] = (int16_t)lib_pid_ff_calc(
            &s_pid_drive[i], target_speed, measured_speed,
            (float)s_motor_ff[i] * s_drive_direction[i], 0);

        /* 转向角度 PID */
        float angle_err = lib_math_get_shortest_path(
            s_swerve_tar[i].angle, s_swerve_cur[i].angle);

        s_motor_current[M_STEER_L + i] = (int16_t)lib_pid_calc(
            &s_pid_steer[i], angle_err, 0);
    }
}

static void power_input_update(uint32_t now, float command_ratio)
{
    drv_referee_chassis_power_t referee_power;
    drv_power_data_t measured_power;
    uint32_t sample_tick;
    uint8_t was_power_meter_online = s_power_meter_is_online;

    s_referee_is_online = drv_referee_read_chassis_power(
        &referee_power, SENTRY_REFEREE_TIMEOUT_MS);
    if (s_referee_is_online) {
        s_robot_level = referee_power.robot_level;
        s_is_chassis_output_enabled = referee_power.is_chassis_output_enabled;
        s_power_limit_w = (float)referee_power.power_limit_w;
    } else {
        s_referee_is_online = 0U;
        s_robot_level = 0U;
        s_is_chassis_output_enabled = 1U;
        s_power_limit_w = SENTRY_POWER_DEFAULT_W;
    }
    s_power_target_w = s_power_limit_w * command_ratio;

    s_power_meter_is_online = power_measure_read_snapshot(
        &measured_power, &sample_tick, now);
    if (s_power_meter_is_online) {
        if (!was_power_meter_online) {
            s_power_measure_lpf.out = measured_power.power;
            s_power_filter_tick = sample_tick;
        } else if (sample_tick != s_power_filter_tick) {
            s_power_measure_lpf.out = lib_filter_lpf_update(
                &s_power_measure_lpf, measured_power.power);
            s_power_filter_tick = sample_tick;
        }
        s_power_measured_w = fmaxf(s_power_measure_lpf.out, 0.0f);
        s_power_current_w = s_power_measured_w;
        s_battery_voltage_v = measured_power.bat_v / 100.0f;
        s_battery_current_a = measured_power.bat_i / 100.0f;
    } else {
        s_power_current_w = 0.0f;
        s_battery_voltage_v = 0.0f;
        s_battery_current_a = 0.0f;
    }
}

static uint8_t power_measure_read_snapshot(drv_power_data_t *power,
                                           uint32_t *sample_tick,
                                           uint32_t now)
{
    uint32_t irq_state;
    uint8_t is_valid;

    if (!power || !sample_tick) {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    is_valid = s_power_meter_rx_is_valid
               && (uint32_t)(now - s_power_meter_rx_tick)
                  <= SENTRY_POWER_METER_TIMEOUT_MS;
    if (is_valid) {
        *power = s_power_meter_rx;
        *sample_tick = s_power_meter_rx_tick;
    }
    __set_PRIMASK(irq_state);

    return is_valid;
}

static void power_state_update(void)
{
    s_chassis_state.power_w = s_power_current_w;
    s_chassis_state.power_limit_w = s_power_limit_w;
    s_chassis_state.power_target_w = s_power_target_w;
    s_chassis_state.power_scale = s_power_scale;
    s_chassis_state.battery_voltage_v = s_battery_voltage_v;
    s_chassis_state.battery_current_a = s_battery_current_a;
    s_chassis_state.robot_level = s_robot_level;
    s_chassis_state.is_power_measured = s_power_meter_is_online;
    s_chassis_state.is_referee_valid = s_referee_is_online;
    s_chassis_state.is_chassis_output_enabled = s_is_chassis_output_enabled;
}

static void power_limit(void)
{
    float control_target = fmaxf(
        s_power_target_w - SENTRY_POWER_LIMIT_RESERVE_W, 0.0f);

    if (control_target <= 0.0f) {
        lib_pid_reset(&s_pid_power);
        s_power_scale = 0.0f;
    } else if (s_power_meter_is_online) {
        /* 每个功率计采样只参与一次积分，避免 1 kHz 循环重复使用旧值。 */
        if (s_power_meter_rx_tick != s_power_feedback_tick) {
            s_power_feedback_tick = s_power_meter_rx_tick;
            if (s_power_measured_w
                > control_target + SENTRY_POWER_ERROR_DEADBAND_W) {
                float correction = lib_pid_calc(
                    &s_pid_power, control_target, s_power_measured_w);
                s_power_scale = fminf(
                    s_power_scale,
                    lib_math_clamp(1.0f + correction, 0.0f, 1.0f));
            } else if (s_power_measured_w
                       < control_target - SENTRY_POWER_ERROR_DEADBAND_W) {
                /* 低于目标时清除负积分，并缓慢恢复可用电流。 */
                lib_pid_reset(&s_pid_power);
                s_power_scale = lib_math_clamp(
                    s_power_scale + SENTRY_POWER_SCALE_RECOVERY,
                    0.0f, 1.0f);
            }
        }
    } else {
        /* 测量掉线时限制到保守档，避免失去反馈后电流突然恢复到 100%。 */
        lib_pid_reset(&s_pid_power);
        if (s_power_scale <= 0.0f
            || s_power_scale > SENTRY_POWER_METER_FALLBACK_SCALE) {
            s_power_scale = SENTRY_POWER_METER_FALLBACK_SCALE;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_current[i] = (int16_t)(s_motor_current[i] * s_power_scale);
    }
}

static void chassis_send_can2(void)
{
    uint8_t frame[8];
    bsp_can_tx_status_t status;

    /* 0x200: 驱动电流 [R_H,R_L, L_H,L_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[M_DRIVE_R]);
    frame[1] = LIB_LO_BYTE(s_motor_current[M_DRIVE_R]);
    frame[2] = LIB_HI_BYTE(s_motor_current[M_DRIVE_L]);
    frame[3] = LIB_LO_BYTE(s_motor_current[M_DRIVE_L]);
    status = bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_DRIVE, frame);
    if (status != BSP_CAN_TX_OK) {
        s_can_tx_error_count++;
    }
/* 0x1FF: 转向 [SL_H,SL_L, SR_H,SR_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[M_STEER_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[M_STEER_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[M_STEER_R]);
    frame[3] = LIB_LO_BYTE(s_motor_current[M_STEER_R]);
    status = bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_STEER, frame);
    if (status != BSP_CAN_TX_OK) {
        s_can_tx_error_count++;
    }
}

static void chassis_send_can1(void)
{
    int16_t power_x100 = (int16_t)lib_math_clamp(
        s_power_current_w * 100.0f, -32768.0f, 32767.0f);

    if (app_chassis_comm_send_power_feedback(power_x100)) {
        s_can_tx_error_count++;
    }
    if (app_chassis_comm_send_omega_feedback(s_chassis_state.omega_z)) {
        s_can_tx_error_count++;
    }
}

static void on_power_measure_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    drv_power_data_t measured_power;
    uint32_t irq_state;

    if (std_id != DRV_POWER_CAN_ID || !data || len != 8U) {
        return;
    }
    drv_power_solve(data, &measured_power);
    if (measured_power.bat_v < 500U || measured_power.bat_v > 6000U
        || !isfinite(measured_power.power)) {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    s_power_meter_rx = measured_power;
    s_power_meter_rx_tick = drv_power_port_get_tick();
    s_power_meter_rx_is_valid = 1U;
    __set_PRIMASK(irq_state);
}

static void on_gimbal_angle_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) {
        return;
    }
    float yaw_rad, pitch_rad;
    memcpy(&yaw_rad,   data,     sizeof(float));
    memcpy(&pitch_rad, data + 4, sizeof(float));
    (void)pitch_rad;
    if (!isfinite(yaw_rad)) {
        return;
    }
    s_yaw_from_can_rad = yaw_rad;
    s_yaw_from_can_valid = 1;
    s_last_yaw_tick      = drv_motor_port_get_tick();
}
