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
#define SENTRY_CAN_CMD_TIMEOUT_MS          200U

enum { M_DRIVE_R=0, M_DRIVE_L=1, M_STEER_L=2, M_STEER_R=3 };

typedef struct {
    drv_motor_data_t feedback[MOTOR_COUNT]; // 控制循环使用的稳定反馈快照
    drv_motor_data_t rx[MOTOR_COUNT];       // CAN 中断写入的反馈缓冲
    uint32_t rx_tick_ms[MOTOR_COUNT];       // 每路电机最近接收时刻
    uint8_t rx_valid[MOTOR_COUNT];          // 每路电机是否收到过反馈
    int16_t current[MOTOR_COUNT];           // 本周期输出电流
    int16_t speed_rpm[MOTOR_COUNT];         // 反馈转速
    int16_t drive_current_ff[2];            // 驱动轮前馈电流
} app_sentry_chassis_motor_state_t;

typedef struct {
    app_sentry_swerve_wheel_t wheel_current[2]; // 当前舵轮状态
    app_sentry_swerve_wheel_t wheel_target[2];  // 目标舵轮状态
    app_sentry_chassis_speed_t body_target;     // 车体目标速度
    app_sentry_chassis_speed_t body_current;    // 车体估算速度
    float force;                                // 合力大小
    float force_angle;                          // 合力方向
    float torque;                               // 绕 z 轴力矩
} app_sentry_chassis_motion_state_t;

typedef struct {
    drv_power_data_t rx;               // 功率计 CAN 接收缓存
    uint32_t rx_tick_ms;                // 最近接收时刻
    uint32_t sample_tick_ms;            // 最近一次参与滤波和 PI 的样本时刻
    lib_lpf_t measure_lpf;              // 实测功率低通滤波器
    float measured_w;                   // 滤波后的实测功率 (W)
    float limit_w;                      // 裁判系统功率上限 (W)
    float target_w;                     // 功率控制目标 (W)
    float scale;                        // 四路电机电流缩放系数
    float battery_v;
    float battery_curr;
    uint8_t robot_level;
    uint8_t chassis_output_enabled;
} app_sentry_chassis_power_state_t;

typedef struct {
    lib_pid_t x;
    lib_pid_t y;
    lib_pid_t w;
    lib_pid_t steer[2];
    lib_pid_t drive[2];
    lib_pid_t power;
    uint8_t speed_cmd_valid;
} app_sentry_chassis_control_state_t;

static app_sentry_chassis_motor_state_t s_motor_state;
static app_sentry_chassis_motion_state_t s_motion_state;
static app_sentry_chassis_power_state_t s_power_state = {
    .sample_tick_ms = 0xFFFFFFFFU,
    .scale = 1.0f,
    .chassis_output_enabled = 1U,
};
static app_sentry_chassis_control_state_t s_control_state;
static app_sentry_chassis_state_t s_chassis_state;

static const uint8_t s_drive_index[2] = { M_DRIVE_L, M_DRIVE_R };
static const float s_drive_direction[2] = { -1.0f, 1.0f };
// ─── 私有函数声明 ─────────────────────────

static void pid_init_all(void);

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd);
static void forward_kinematics(void);
static void chassis_pid(void);
static void chassis_wrench_allocate(void);
static void wheel_control(void);

static uint8_t power_update(void);
static void power_measure_read_snapshot(drv_power_data_t *power, uint32_t *sample_tick);
static void power_apply_limit(uint8_t is_new_sample);
static void power_publish(void);

static void chassis_can2_tx(void);
static void chassis_can1_tx(void);

static void on_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
static void on_power_measure_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

// ─── 公有接口实现 ─────────────────────────

void app_sentry_chassis_init(void)
{
    lib_lpf_init(&s_power_state.measure_lpf, 0.2f);
    pid_init_all();

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_POWER, on_power_measure_feedback);

}
void app_sentry_chassis_ctrl(void)
{
    //获得tick
    uint32_t now;
    uint8_t is_motor_online = 1;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_state.speed_rpm[i] = s_motor_state.feedback[i].speed;
        if (!s_motor_state.rx_valid[i] || (uint32_t)(now - s_motor_state.rx_tick_ms[i]) > SENTRY_MOTOR_TIMEOUT) {
            is_motor_online = 0;
        }
    }
    __set_PRIMASK(irq_state);

    //获取命令
    app_chassis_speed_cmd_t can_cmd;
    s_control_state.speed_cmd_valid = app_chassis_comm_read_speed_cmd(&can_cmd, SENTRY_CAN_CMD_TIMEOUT_MS);
    uint8_t is_emergency_stop = s_control_state.speed_cmd_valid && can_cmd.vx == SENTRY_CHASSIS_ESTOP;
    uint8_t is_new_power_sample = power_update();

    //重启
    if (!is_motor_online || !s_control_state.speed_cmd_valid || is_emergency_stop) {
        memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
        memset(&s_motion_state.body_target, 0, sizeof(s_motion_state.body_target));
        memset(&s_motion_state.body_current, 0, sizeof(s_motion_state.body_current));
        memset(&s_chassis_state, 0, sizeof(s_chassis_state));
        s_power_state.scale = 0.0f;
        pid_init_all();
        power_publish();
        chassis_can2_tx();
        chassis_can1_tx();
        return;
    }

    //命令转接与限幅
    s_motion_state.body_target.v_x = (float)can_cmd.vx;
    s_motion_state.body_target.v_y = (float)can_cmd.vy;
    s_motion_state.body_target.v_w = (float)can_cmd.vz * APP_CHASSIS_OMEGA_RAD_S_PER_LSB;
    s_motion_state.body_target.v_x = lib_clamp(s_motion_state.body_target.v_x, -SENTRY_CHASSIS_MAX_VX, SENTRY_CHASSIS_MAX_VX);
    s_motion_state.body_target.v_y = lib_clamp(s_motion_state.body_target.v_y, -SENTRY_CHASSIS_MAX_VY, SENTRY_CHASSIS_MAX_VY);
    s_motion_state.body_target.v_w = lib_clamp(s_motion_state.body_target.v_w, -SENTRY_CHASSIS_MAX_VW, SENTRY_CHASSIS_MAX_VW);

    //舵轮状态
    s_motion_state.wheel_current[0].angle = lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_L].angle, SENTRY_SWERVE_0_OFFSET);
    s_motion_state.wheel_current[1].angle = lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_R].angle, SENTRY_SWERVE_1_OFFSET);
    s_motion_state.wheel_current[0].speed = -lib_motor_rpm_to_mm_s((float)s_motor_state.speed_rpm[M_DRIVE_L], SENTRY_WHEEL_RADIUS, DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_current[1].speed =  lib_motor_rpm_to_mm_s((float)s_motor_state.speed_rpm[M_DRIVE_R], SENTRY_WHEEL_RADIUS, DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_current[0].rev = 1;
    s_motion_state.wheel_current[1].rev = 1;

    //运动控制
    inverse_kinematics(&s_motion_state.body_target);//逆运动学解算
    forward_kinematics();//正运动学解算
    chassis_pid();//底盘速度PID
    chassis_wrench_allocate();//力分配
    wheel_control();//轮组控制

    //功率控制
    power_apply_limit(is_new_power_sample);

    //电机命令发送
    chassis_can2_tx();
    
    //云台控制反馈与功率
    chassis_can1_tx();

    //状态更新
    memcpy(s_chassis_state.wheel, s_motion_state.wheel_current, sizeof(s_motion_state.wheel_current));
    s_chassis_state.speed = s_motion_state.body_current;
    power_publish();
}

static void on_motor_feedback(uint32_t std_id, uint8_t *data,
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
        drv_motor_solve_dji(data, &s_motor_state.rx[motor_index]);
        s_motor_state.rx_tick_ms[motor_index] = HAL_GetTick();
        s_motor_state.rx_valid[motor_index] = 1;
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
    s_power_state.rx = measured_power;
    s_power_state.rx_tick_ms = HAL_GetTick();
    __set_PRIMASK(irq_state);
}


const app_sentry_chassis_state_t *app_chassis_get_state(void)
{
    return &s_chassis_state;
}

static void pid_init_all(void)
{
    //输出力矩/电流，x/y等效
    lib_pid_init(&s_control_state.x, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.w, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //舵轮PID，等效
    lib_pid_init(&s_control_state.steer[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.steer[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //驱动轮PID，等效
    lib_pid_init(&s_control_state.drive[0], 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                DRV_MOTOR_M3508_CURRENT_MAX, 0.0f,
                DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_control_state.drive[1], 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                DRV_MOTOR_M3508_CURRENT_MAX, 0.0f,
                DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
    //功率PID，输出缩小系数
    lib_pid_init(&s_control_state.power, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd)
{
    float vx = lib_clamp(body_spd->v_x, -SENTRY_CHASSIS_MAX_VX, SENTRY_CHASSIS_MAX_VX);
    float vy = lib_clamp(body_spd->v_y, -SENTRY_CHASSIS_MAX_VY, SENTRY_CHASSIS_MAX_VY);
    float vw = lib_clamp(body_spd->v_w, -SENTRY_CHASSIS_MAX_VW, SENTRY_CHASSIS_MAX_VW);

    for (int i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float ix = vx - sign * vw * SENTRY_WHEEL_HALF_TRACK;
        float iy = vy + sign * vw * SENTRY_WHEEL_HALF_BASE;

        float raw_speed = sqrtf(ix * ix + iy * iy);
        if (raw_speed < 1.0f) {
            s_motion_state.wheel_target[i].angle = s_motion_state.wheel_current[i].angle;
            s_motion_state.wheel_target[i].speed = 0.0f;
            s_motion_state.wheel_target[i].rev = 1;
            continue;
        }
        float raw_angle = atan2f(iy, ix);

        float diff = lib_get_shortest_path(raw_angle, s_motion_state.wheel_current[i].angle);

        if (fabsf(diff) > LIB_PI * 0.5f) {
            s_motion_state.wheel_target[i].rev  = -1;
            s_motion_state.wheel_target[i].angle = lib_rad_norm(
                raw_angle + (diff > 0.0f ? -LIB_PI : LIB_PI));
        } else {
            s_motion_state.wheel_target[i].rev  = 1;
            s_motion_state.wheel_target[i].angle = raw_angle;
        }
        s_motion_state.wheel_target[i].speed = raw_speed * (float)s_motion_state.wheel_target[i].rev;
    }
}

static void forward_kinematics(void)
{
    float velocity_x[2], velocity_y[2];
    for (int i = 0; i < 2; i++) {
        float speed = lib_motor_rpm_to_mm_s(
            (float)s_motor_state.speed_rpm[s_drive_index[i]], SENTRY_WHEEL_RADIUS,
            DRV_MOTOR_M3508_REDUCTION) * s_drive_direction[i];
        float angle_rad = s_motion_state.wheel_current[i].angle;
        velocity_x[i] = speed * cosf(angle_rad);
        velocity_y[i] = speed * sinf(angle_rad);
    }
    float half_track = SENTRY_WHEEL_HALF_TRACK;
    float half_base = SENTRY_WHEEL_HALF_BASE;
    s_motion_state.body_current.v_x = (velocity_x[0] + velocity_x[1]) * 0.5f;
    s_motion_state.body_current.v_y = (velocity_y[0] + velocity_y[1]) * 0.5f;
    s_motion_state.body_current.v_w = (half_track * (velocity_x[1] - velocity_x[0])
                     + half_base * (velocity_y[0] - velocity_y[1]))
                    / (2.0f * (half_track * half_track + half_base * half_base));
}

static void chassis_pid(void)
{
    float fx = lib_pid_calc(&s_control_state.x, s_motion_state.body_target.v_x, s_motion_state.body_current.v_x);
    float fy = lib_pid_calc(&s_control_state.y, s_motion_state.body_target.v_y, s_motion_state.body_current.v_y);
    s_motion_state.force       = sqrtf(fx * fx + fy * fy);
    s_motion_state.force_angle = atan2f(fy, fx);
    s_motion_state.torque      = lib_pid_calc(&s_control_state.w, s_motion_state.body_target.v_w, s_motion_state.body_current.v_w);
}

static void chassis_wrench_allocate(void)
{
    float fx = s_motion_state.force * cosf(s_motion_state.force_angle);
    float fy = s_motion_state.force * sinf(s_motion_state.force_angle);
    float half_track = SENTRY_WHEEL_HALF_TRACK;
    float half_base = SENTRY_WHEEL_HALF_BASE;
    float lever_squared = half_track * half_track + half_base * half_base;

    int i;
    for (i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float fix = fx * 0.5f - sign * s_motion_state.torque * half_track / (2.0f * lever_squared);
        float fiy = fy * 0.5f + sign * s_motion_state.torque * half_base / (2.0f * lever_squared);
        float angle_rad = s_motion_state.wheel_current[i].angle;

        float current_ff = (fix * cosf(angle_rad) + fiy * sinf(angle_rad))
                         * SENTRY_WHEEL_RADIUS / DRV_MOTOR_M3508_REDUCTION;
        s_motor_state.drive_current_ff[i] = (int16_t)lib_clamp(current_ff,
            DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX);
    }
}

static void wheel_control(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        uint8_t drive_index = s_drive_index[i];
        float target_speed = s_motion_state.wheel_target[i].speed * s_drive_direction[i];
        float measured_speed = lib_motor_rpm_to_mm_s((float)s_motor_state.speed_rpm[drive_index], 
        SENTRY_WHEEL_RADIUS,DRV_MOTOR_M3508_REDUCTION);
        float drive_current_ff = (float)s_motor_state.drive_current_ff[i]
                               * s_drive_direction[i];
        float drive_current = lib_pid_ff_calc(&s_control_state.drive[i], target_speed,
                                               measured_speed, drive_current_ff, 0.0f);
        drive_current = lib_clamp(drive_current, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX);
        s_motor_state.current[drive_index] = (int16_t)drive_current;

        float angle_err = lib_get_shortest_path(
            s_motion_state.wheel_target[i].angle, s_motion_state.wheel_current[i].angle);

        float steer_current = lib_pid_calc(&s_control_state.steer[i], angle_err, 0.0f);
        steer_current = lib_clamp(steer_current, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
        s_motor_state.current[M_STEER_L + i] = (int16_t)steer_current;
    }
}

static uint8_t power_update(void)
{
    const drv_referee_global_t *referee = drv_referee_get_data();
    drv_power_data_t measured_power;
    uint32_t sample_tick;
    uint8_t is_new_sample;

    s_power_state.robot_level = referee->robot_status.robot_level;
    s_power_state.chassis_output_enabled =
        referee->robot_status.power_management_chassis_output;
    s_power_state.limit_w = (float)referee->robot_status.chassis_power_limit;
    s_power_state.target_w = s_power_state.limit_w;

    power_measure_read_snapshot(&measured_power, &sample_tick);
    is_new_sample = sample_tick != s_power_state.sample_tick_ms;
    if (is_new_sample) {
        if (s_power_state.sample_tick_ms == 0xFFFFFFFFU) {
            s_power_state.measure_lpf.out = measured_power.power;
        } else {
            s_power_state.measure_lpf.out = lib_lpf_update(
                &s_power_state.measure_lpf, measured_power.power);
        }
        s_power_state.sample_tick_ms = sample_tick;
    }

    s_power_state.measured_w = fmaxf(s_power_state.measure_lpf.out, 0.0f);
    s_power_state.battery_v = measured_power.bat_v / 100.0f;
    s_power_state.battery_curr = measured_power.bat_i / 100.0f;
    return is_new_sample;
}

static void power_measure_read_snapshot(drv_power_data_t *power,
                                        uint32_t *sample_tick)
{
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    *power = s_power_state.rx;
    *sample_tick = s_power_state.rx_tick_ms;
    __set_PRIMASK(irq_state);
}

static void power_apply_limit(uint8_t is_new_sample)
{
    if (s_power_state.target_w <= 0.0f) {
        lib_pid_reset(&s_control_state.power);
        s_power_state.scale = 0.0f;
    } else if (is_new_sample) {
        float correction = lib_pid_calc(&s_control_state.power,
                                        s_power_state.target_w,
                                        s_power_state.measured_w);
        s_power_state.scale = lib_clamp(1.0f + correction, 0.0f, 1.0f);
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_state.current[i] = (int16_t)(s_motor_state.current[i]
                                            * s_power_state.scale);
    }
}

static void power_publish(void)
{
    s_chassis_state.power_w = s_power_state.measured_w;
    s_chassis_state.power_limit = s_power_state.limit_w;
    s_chassis_state.power_target = s_power_state.target_w;
    s_chassis_state.power_scale = s_power_state.scale;
    s_chassis_state.battery_v = s_power_state.battery_v;
    s_chassis_state.battery_curr = s_power_state.battery_curr;
    s_chassis_state.robot_level = s_power_state.robot_level;
    s_chassis_state.is_chassis_output_enabled = s_power_state.chassis_output_enabled;
}

static void chassis_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x200: 驱动电流 [R_H,R_L, L_H,L_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[M_DRIVE_R]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[M_DRIVE_R]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[M_DRIVE_L]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[M_DRIVE_L]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_DRIVE, frame);
/* 0x1FF: 转向 [SL_H,SL_L, SR_H,SR_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[M_STEER_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[M_STEER_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[M_STEER_R]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[M_STEER_R]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_STEER, frame);
}

static void chassis_can1_tx(void)
{
    s_chassis_state.omega_z = s_motion_state.body_current.v_w;

    int16_t power_x100 = (int16_t)lib_clamp(
        s_power_state.measured_w * 100.0f, -32768.0f, 32767.0f);

    (void)app_chassis_comm_power_tx(power_x100);
    (void)app_chassis_comm_omega_tx(s_chassis_state.omega_z);
}

