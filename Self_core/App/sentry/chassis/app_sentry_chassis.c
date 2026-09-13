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
 *   CAN1 0x111 → body-frame目标速度
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

#include "drv_motor.h"
#include "drv_power_measure.h"

#include "app_chassis_comm.h"
#include "drv_referee.h"

#include <math.h>
#include <string.h>

#define MOTOR_COUNT                        4
#define SENTRY_POWER_MAX             100.0f//底盘功率上限
#define SENTRY_POWER_RESERVE       5.0f//功率余量
#define SENTRY_POWER_DEADZONE      1.0f//功率死区
#define SENTRY_POWER_SCALE_RECOVERY        0.005f
#define SENTRY_POWER_METER_TIMEOUT_MS      50U
#define SENTRY_POWER_METER_FALLBACK_SCALE  0.3f
#define SENTRY_REFEREE_TIMEOUT_MS          500U
#define SENTRY_CAN_CMD_TIMEOUT_MS          200U

enum { M_DRIVE_R=0, M_DRIVE_L=1, M_STEER_L=2, M_STEER_R=3 };

static drv_motor_data_t s_motor[MOTOR_COUNT];
static int16_t          s_motor_current[MOTOR_COUNT];     /* 电流输出      */
static int16_t          s_motor_speed_rpm[MOTOR_COUNT];   /* DJI 反馈边界：RPM */
static drv_motor_data_t s_motor_rx[MOTOR_COUNT];
static uint32_t s_motor_rx_tick[MOTOR_COUNT];
static uint8_t s_motor_rx_is_valid[MOTOR_COUNT];
static const uint8_t s_drive_index[2] = { M_DRIVE_L, M_DRIVE_R };
static const float s_drive_direction[2] = { -1.0f, 1.0f };
static int16_t s_drive_current_ff[2];          /* 力/力矩分配后的驱动电流前馈 */

static app_sentry_swerve_wheel_t s_swerve_cur[2];  /**< 当前状态 [0]=左 [1]=右  */
static app_sentry_swerve_wheel_t s_swerve_tar[2];  /**< 目标状态                 */
static app_sentry_chassis_speed_t s_body_tar;      /**< body-frame 目标速度      */
static app_sentry_chassis_speed_t s_body_cur;      /**< body-frame 估算速度      */
static app_sentry_chassis_state_t s_chassis_state; /**< 全局状态                 */

static float s_force, s_force_angle, s_torque;     /**< PID输出的力/力矩         */

static drv_power_data_t s_power_meter_rx;
static uint32_t s_power_meter_rx_tick;
static uint8_t s_power_meter_rx_is_valid;
static uint32_t s_power_filter_tick;
static uint32_t s_power_feedback_tick;
static lib_lpf_t s_power_measure_lpf;

static float s_power_scale = 1.0f;      /**< 实测功率 PI 得到的电流缩放系数 */
static float s_power_measured_w;        /**< 功率计滤波后的实测功率          */
static float s_power_current_w;         /**< 对外发布的实测功率              */
static float s_power_limit_w = SENTRY_POWER_MAX;
static float s_power_target_w = SENTRY_POWER_MAX;
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

static uint8_t  s_speed_cmd_is_valid;     /**< 是否收到未超时的 CAN1 底盘速度指令 */
static uint32_t s_can_tx_error_count;     /**< CAN 发送失败计数         */

// ─── 私有函数声明 ─────────────────────────

static void pid_init_all(void);
static float calc_logical_angle(uint16_t raw_enc, uint16_t offset);
static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd);
static void forward_kinematics(void);
static void chassis_pid(void);
static void chassis_wrench_allocate(void);
static void wheel_control(void);
static void power_input_update(uint32_t now, float command_ratio);
static uint8_t power_measure_read_snapshot(drv_power_data_t *power,
                                           uint32_t *sample_tick,
                                           uint32_t now);
static void power_state_update(void);
static void power_limit(void);
static void chassis_send_can2(void);
static void chassis_send_can1(void);
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
    memset(s_drive_current_ff,        0, sizeof(s_drive_current_ff));
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
    lib_lpf_init(&s_power_measure_lpf, 0.2f);
    s_power_scale = SENTRY_POWER_METER_FALLBACK_SCALE;
    s_power_measured_w = 0.0f;
    s_power_current_w = 0.0f;
    s_power_limit_w = SENTRY_POWER_MAX;
    s_power_target_w = SENTRY_POWER_MAX;
    s_battery_voltage_v = 0.0f;
    s_battery_current_a = 0.0f;
    s_power_meter_is_online = 0U;
    s_referee_is_online = 0U;
    s_robot_level = 0U;
    s_is_chassis_output_enabled = 1U;
    s_force = s_force_angle = s_torque = 0;
    s_can_tx_error_count = 0;
    s_speed_cmd_is_valid = 0;

    pid_init_all();

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R,
                                 app_chassis_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L,
                                 app_chassis_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_L,
                                 app_chassis_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_R,
                                 app_chassis_on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_POWER,
                                 on_power_measure_feedback);

}
void app_sentry_chassis_ctrl(void)
{
    //获得tick
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

    //获取命令
    app_chassis_speed_cmd_t can_cmd;
    s_speed_cmd_is_valid = app_chassis_comm_read_speed_cmd(&can_cmd, SENTRY_CAN_CMD_TIMEOUT_MS);
    uint8_t is_emergency_stop = s_speed_cmd_is_valid && can_cmd.vx == SENTRY_CHASSIS_ESTOP_VX_RAW;
    power_input_update(now, 1.0f);

    //重启
    if (!is_motor_online || !s_speed_cmd_is_valid || is_emergency_stop) {
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
        chassis_send_can1();
        return;
    }

    //命令转接与限幅
    s_body_tar.v_x = (float)can_cmd.vx;
    s_body_tar.v_y = (float)can_cmd.vy;
    s_body_tar.v_w = (float)can_cmd.vz * SENTRY_CHASSIS_OMEGA_RAD_S_PER_LSB;
    s_body_tar.v_x = lib_clamp(s_body_tar.v_x, -SENTRY_CHASSIS_MAX_VX_MM_S, SENTRY_CHASSIS_MAX_VX_MM_S);
    s_body_tar.v_y = lib_clamp(s_body_tar.v_y, -SENTRY_CHASSIS_MAX_VY_MM_S, SENTRY_CHASSIS_MAX_VY_MM_S);
    s_body_tar.v_w = lib_clamp(s_body_tar.v_w, -SENTRY_CHASSIS_MAX_OMEGA_RAD_S, SENTRY_CHASSIS_MAX_OMEGA_RAD_S);

    //舵轮状态
    s_swerve_cur[0].angle = calc_logical_angle(s_motor[M_STEER_L].angle, SENTRY_SWERVE_0_OFFSET);
    s_swerve_cur[1].angle = calc_logical_angle(s_motor[M_STEER_R].angle, SENTRY_SWERVE_1_OFFSET);
    s_swerve_cur[0].speed = -lib_motor_rpm_to_mm_s((float)s_motor_speed_rpm[M_DRIVE_L], SENTRY_WHEEL_RADIUS_MM, SENTRY_REDUCTION_RATIO);
    s_swerve_cur[1].speed =  lib_motor_rpm_to_mm_s((float)s_motor_speed_rpm[M_DRIVE_R], SENTRY_WHEEL_RADIUS_MM, SENTRY_REDUCTION_RATIO);
    s_swerve_cur[0].rev = 1;
    s_swerve_cur[1].rev = 1;

    //运动控制
    inverse_kinematics(&s_body_tar);//逆运动学解算
    forward_kinematics();//正运动学解算
    chassis_pid();//底盘速度PID
    chassis_wrench_allocate();//力分配
    wheel_control();//轮组控制

    //功率控制
    power_limit();

    //电机命令发送
    chassis_send_can2();
    
    //云台控制反馈与功率
    chassis_send_can1();

    //状态更新
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

static void pid_init_all(void)
{
    //输出力矩/电流，x/y等效
    lib_pid_init(&s_pid_x, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_pid_y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_pid_w, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //舵轮PID，等效
    lib_pid_init(&s_pid_steer[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_pid_steer[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //驱动轮PID，等效
    lib_pid_init(&s_pid_drive[0], 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                SENTRY_M3508_CURRENT_MAX, 0.0f,
                SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_pid_drive[1], 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                SENTRY_M3508_CURRENT_MAX, 0.0f,
                SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX, 0.0f);
    //功率PID，输出缩小系数
    lib_pid_init(&s_pid_power, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
    return lib_enc_conv((float)diff, LIB_ENC13_TO_RAD);
}

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd)
{
    float vx = lib_clamp(body_spd->v_x, -SENTRY_CHASSIS_MAX_VX_MM_S, SENTRY_CHASSIS_MAX_VX_MM_S);
    float vy = lib_clamp(body_spd->v_y, -SENTRY_CHASSIS_MAX_VY_MM_S, SENTRY_CHASSIS_MAX_VY_MM_S);
    float vw = lib_clamp(body_spd->v_w, -SENTRY_CHASSIS_MAX_OMEGA_RAD_S, SENTRY_CHASSIS_MAX_OMEGA_RAD_S);

    for (int i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float ix = vx - sign * vw * SENTRY_WHEEL_HALF_TRACK_MM;
        float iy = vy + sign * vw * SENTRY_WHEEL_HALF_BASE_MM;

        float raw_speed = sqrtf(ix * ix + iy * iy);
        if (raw_speed < 1.0f) {
            s_swerve_tar[i].angle = s_swerve_cur[i].angle;
            s_swerve_tar[i].speed = 0.0f;
            s_swerve_tar[i].rev = 1;
            continue;
        }
        float raw_angle = atan2f(iy, ix);

        float diff = lib_get_shortest_path(raw_angle, s_swerve_cur[i].angle);

        if (fabsf(diff) > LIB_PI * 0.5f) {
            s_swerve_tar[i].rev  = -1;
            s_swerve_tar[i].angle = lib_rad_norm(
                raw_angle + (diff > 0.0f ? -LIB_PI : LIB_PI));
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
        float speed = lib_motor_rpm_to_mm_s(
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

static void chassis_wrench_allocate(void)
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

        float current_ff = (fix * cosf(angle_rad) + fiy * sinf(angle_rad))
                         * SENTRY_WHEEL_RADIUS_MM / SENTRY_REDUCTION_RATIO;
        s_drive_current_ff[i] = (int16_t)lib_clamp(current_ff,
            SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX);
    }
}

static void wheel_control(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        uint8_t drive_index = s_drive_index[i];
        float target_speed = s_swerve_tar[i].speed * s_drive_direction[i];
        float measured_speed = lib_motor_rpm_to_mm_s((float)s_motor_speed_rpm[drive_index], 
        SENTRY_WHEEL_RADIUS_MM,SENTRY_REDUCTION_RATIO);
        float drive_current_ff = (float)s_drive_current_ff[i]
                               * s_drive_direction[i];
        float drive_current = lib_pid_ff_calc(&s_pid_drive[i], target_speed,
                                               measured_speed, drive_current_ff, 0.0f);
        drive_current = lib_clamp(drive_current, SENTRY_M3508_CURRENT_MIN, SENTRY_M3508_CURRENT_MAX);
        s_motor_current[drive_index] = (int16_t)drive_current;

        float angle_err = lib_get_shortest_path(
            s_swerve_tar[i].angle, s_swerve_cur[i].angle);

        float steer_current = lib_pid_calc(&s_pid_steer[i], angle_err, 0.0f);
        steer_current = lib_clamp(steer_current, SENTRY_GM6020_CURRENT_MIN, SENTRY_GM6020_CURRENT_MAX);
        s_motor_current[M_STEER_L + i] = (int16_t)steer_current;
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
        s_power_limit_w = SENTRY_POWER_MAX;
    }
    s_power_target_w = s_power_limit_w * command_ratio;

    s_power_meter_is_online = power_measure_read_snapshot(
        &measured_power, &sample_tick, now);
    if (s_power_meter_is_online) {
        if (!was_power_meter_online) {
            s_power_measure_lpf.out = measured_power.power;
            s_power_filter_tick = sample_tick;
        } else if (sample_tick != s_power_filter_tick) {
            s_power_measure_lpf.out = lib_lpf_update(
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
    float control_target = s_power_target_w - SENTRY_POWER_RESERVE;

    if (control_target <= 0.0f) {
        lib_pid_reset(&s_pid_power);
        s_power_scale = 0.0f;
    } else if (s_power_meter_is_online) {
        if (s_power_meter_rx_tick != s_power_feedback_tick) {
            s_power_feedback_tick = s_power_meter_rx_tick;
            if (s_power_measured_w
                > control_target + SENTRY_POWER_DEADZONE) {
                float correction = lib_pid_calc(
                    &s_pid_power, control_target, s_power_measured_w);
                s_power_scale = fminf(
                    s_power_scale,
                    lib_clamp(1.0f + correction, 0.0f, 1.0f));
            } else if (s_power_measured_w
                       < control_target - SENTRY_POWER_DEADZONE) {
                /* 低于目标时清除负积分，并缓慢恢复可用电流。 */
                lib_pid_reset(&s_pid_power);
                s_power_scale = lib_clamp(
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

    /* 0x200: 驱动电流 [R_H,R_L, L_H,L_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[M_DRIVE_R]);
    frame[1] = LIB_LO_BYTE(s_motor_current[M_DRIVE_R]);
    frame[2] = LIB_HI_BYTE(s_motor_current[M_DRIVE_L]);
    frame[3] = LIB_LO_BYTE(s_motor_current[M_DRIVE_L]);
    if (bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_DRIVE, frame)) {
        s_can_tx_error_count++;
    }
/* 0x1FF: 转向 [SL_H,SL_L, SR_H,SR_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_current[M_STEER_L]);
    frame[1] = LIB_LO_BYTE(s_motor_current[M_STEER_L]);
    frame[2] = LIB_HI_BYTE(s_motor_current[M_STEER_R]);
    frame[3] = LIB_LO_BYTE(s_motor_current[M_STEER_R]);
    if (bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_STEER, frame)) {
        s_can_tx_error_count++;
    }
}

static void chassis_send_can1(void)
{
    s_chassis_state.omega_z = s_body_cur.v_w;

    int16_t power_x100 = (int16_t)lib_clamp(
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

    if (std_id != SENTRY_CAN_CHASSIS_POWER || !data || len != 8U) {
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
