/**
 * @file    app_sentry_chassis.c
 * @brief   哨兵双舵轮底盘控制实现
 * @note    控制流程：底盘速度指令 → 平移/旋转斜坡 → 逆运动学 → 正运动学 → 车体 PID →
 *          力/力矩分配 → 驱动前馈与轮级 PID → 功率缩放 → CAN2 电流输出。
 */
#include "app_sentry_chassis.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"

#include "drv_motor.h"
#include "drv_power_measure.h"

#include "app_chassis_comm.h"

#include <math.h>
#include <string.h>

volatile app_sentry_chassis_debug_t app_sentry_chassis_debug;

static app_sentry_chassis_motor_state_t s_motor_state;
static app_sentry_chassis_motion_state_t s_motion_state;
static app_sentry_chassis_power_state_t s_power_state = {
    .tar_power = SENTRY_CHASSIS_TAR_POWER,
    .scale = 1.0f,
};
static app_sentry_chassis_control_state_t s_control_state;
static app_sentry_chassis_state_t s_chassis_state;

/* 仅单向发布副本；控制、通信及中断均只访问上面的独立状态。 */
static void chassis_debug_publish(void)
{
    uint32_t irq_state = __get_PRIMASK();

    /* 避免复制接收缓冲时被 CAN 中断打断。 */
    __disable_irq();
    app_sentry_chassis_debug.motor = s_motor_state;
    app_sentry_chassis_debug.power = s_power_state;
    __set_PRIMASK(irq_state);

    app_sentry_chassis_debug.motion = s_motion_state;
    app_sentry_chassis_debug.pid = s_control_state;
    app_sentry_chassis_debug.state = s_chassis_state;
}

static const app_sentry_chassis_config_t s_config = {
    .omega_cutoff_hz = 20.0f,
    .power_cutoff_hz = 35.0f,
};

static uint8_t s_can1_tx_divider;

static const uint8_t s_drive_index[2] = { M_DRIVE_L, M_DRIVE_R };
static const float s_drive_direction[2] = { -1.0f, 1.0f };
static const float s_steer_direction[2] = {
    SENTRY_STEER_L_DIRECTION, SENTRY_STEER_R_DIRECTION,
};

static void pid_init_all(void)
{

    lib_pid_init(&s_control_state.x, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.w, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lib_pid_init(&s_control_state.steer[0], 20000.0f, 20.0f, 10.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_control_state.steer[1], 20000.0f, 20.0f, 10.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_control_state.drive[0], 20.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_control_state.drive[1], 20.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MAX, 0.0f,
                 DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_control_state.power, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}
static void chassis_reset(void);
static void chassis_ramp_update(float dt);
static void inverse_kinematics(const app_sentry_chassis_speed_t *tar_speed);
static void forward_kinematics(void);
static void chassis_pid(void);
static void chassis_wrench_allocate(float fx, float fy, float torque);
static void wheel_control(void);
static void power_update(void);
static void power_measure_read_snapshot(drv_power_data_t *power);
static void power_apply_limit(void);
static void power_publish(void);
static void chassis_can2_tx(void);
static void chassis_can1_tx(void);
static void on_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
static void on_power_measure_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
void app_sentry_chassis_init(void)
{
    memset(&s_motion_state.cmd_speed, 0, sizeof(s_motion_state.cmd_speed));
    memset(&s_motion_state.tar_speed, 0, sizeof(s_motion_state.tar_speed));
    lib_lpf_init(&s_power_state.measure_lpf, s_config.power_cutoff_hz);
    lib_lpf_init(&s_motion_state.omega_tx_lpf, s_config.omega_cutoff_hz);
    pid_init_all();
    s_can1_tx_divider = 0U;

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_POWER, on_power_measure_feedback);
    chassis_debug_publish();
}
void app_sentry_chassis_ctrl(void)
{
    app_chassis_comm_rx_t comm_rx;
    app_chassis_speed_cmd_t can_cmd;
    uint32_t irq_state = __get_PRIMASK();

    /* 复制 CAN 中断写入的反馈，确保本周期使用一致的数据。 */
    __disable_irq();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    __set_PRIMASK(irq_state);

    (void)app_chassis_comm_read_rx(&comm_rx);
    can_cmd = comm_rx.speed;
    uint8_t is_emergency_stop = can_cmd.vx == SENTRY_CHASSIS_ESTOP;
    power_update();

    /* 1 kHz 控制循环中每 5 次发送一次 CAN1 状态帧，即 200 Hz。 */
    uint8_t is_can1_tx_due = ++s_can1_tx_divider >= 5U;
    if (is_can1_tx_due) {
        s_can1_tx_divider = 0U;
    }

    if (is_emergency_stop) {
        chassis_reset();
        power_publish();
        chassis_can2_tx();
        if (is_can1_tx_due) {
            chassis_can1_tx();
        }
        chassis_debug_publish();
        return;
    }

    /* 将 0x111 命令解码为车体坐标系目标速度并限幅。 */
    s_motion_state.cmd_speed.vx_speed = lib_clamp((float)can_cmd.vx,
        -SENTRY_CHASSIS_VX_TAR_SPEED_MAX, SENTRY_CHASSIS_VX_TAR_SPEED_MAX);
    s_motion_state.cmd_speed.vy_speed = lib_clamp((float)can_cmd.vy,
        -SENTRY_CHASSIS_VY_TAR_SPEED_MAX, SENTRY_CHASSIS_VY_TAR_SPEED_MAX);
    s_motion_state.cmd_speed.vw_speed = lib_clamp(
        (float)can_cmd.vz * APP_CHASSIS_OMEGA_RAD_S_PER_LSB,
        -SENTRY_CHASSIS_VW_TAR_SPEED_MAX, SENTRY_CHASSIS_VW_TAR_SPEED_MAX);
    chassis_ramp_update(0.001f);

    /* 根据转向编码器和驱动转速更新两个舵轮当前状态。 */
    s_motion_state.wheel_cur[0].steer_angle = lib_rad_norm(
        SENTRY_SWERVE_L_CAL_ANGLE + SENTRY_STEER_L_DIRECTION
        * lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_L].angle,
                                 SENTRY_SWERVE_0_OFFSET));
    s_motion_state.wheel_cur[1].steer_angle = lib_rad_norm(
        SENTRY_SWERVE_R_CAL_ANGLE + SENTRY_STEER_R_DIRECTION
        * lib_enc13_relative_rad(s_motor_state.feedback[M_STEER_R].angle,
                                 SENTRY_SWERVE_1_OFFSET));
    s_motion_state.wheel_cur[0].drive_speed = -lib_motor_rpm_to_mm_s(
        (float)s_motor_state.feedback[M_DRIVE_L].speed, SENTRY_WHEEL_RADIUS,
        DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[1].drive_speed = lib_motor_rpm_to_mm_s(
        (float)s_motor_state.feedback[M_DRIVE_R].speed, SENTRY_WHEEL_RADIUS,
        DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[0].drive_reversed = 0U;
    s_motion_state.wheel_cur[1].drive_reversed = 0U;

    inverse_kinematics(&s_motion_state.tar_speed);
    forward_kinematics();
    (void)lib_lpf_update(&s_motion_state.omega_tx_lpf,
                         s_motion_state.cur_speed.vw_speed, 0.001f);
    chassis_pid();
    wheel_control();
    power_apply_limit();

    chassis_can2_tx();
    if (is_can1_tx_due) {
        chassis_can1_tx();
    }

    memcpy(s_chassis_state.wheel_cur, s_motion_state.wheel_cur,
           sizeof(s_motion_state.wheel_cur));
    s_chassis_state.cur_speed = s_motion_state.cur_speed;
    power_publish();
    chassis_debug_publish();
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
        drv_motor_data_t decoded = {0};
        drv_motor_solve_dji(data, &decoded);
        s_motor_state.rx[motor_index].angle = decoded.angle;
        s_motor_state.rx[motor_index].speed = decoded.speed;
    }
}

static void on_power_measure_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    drv_power_data_t measured_power;

    if (std_id != SENTRY_CAN_CHASSIS_POWER || !data || len != 8U) {
        return;
    }
    drv_power_solve(data, &measured_power);
    if (measured_power.bat_v < 500U || measured_power.bat_v > 6000U
        || !isfinite(measured_power.power)) {
        return;
    }

    s_power_state.rx = measured_power;
}

const app_sentry_chassis_state_t *app_chassis_get_state(void)
{
    return &s_chassis_state;
}

static void chassis_reset(void)
{
    memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
    memset(&s_motion_state.cmd_speed, 0, sizeof(s_motion_state.cmd_speed));
    memset(&s_motion_state.tar_speed, 0, sizeof(s_motion_state.tar_speed));
    memset(&s_motion_state.cur_speed, 0, sizeof(s_motion_state.cur_speed));
    lib_lpf_init(&s_motion_state.omega_tx_lpf, s_config.omega_cutoff_hz);
    memset(&s_chassis_state, 0, sizeof(s_chassis_state));
    s_power_state.scale = 0.0f;
    pid_init_all();
}

static void chassis_ramp_update(float dt)
{
#if SENTRY_CHASSIS_RAMP_ENABLE
    float dx = s_motion_state.cmd_speed.vx_speed - s_motion_state.tar_speed.vx_speed;
    float dy = s_motion_state.cmd_speed.vy_speed - s_motion_state.tar_speed.vy_speed;
    float distance = sqrtf(dx * dx + dy * dy);
    float max_step = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * dt;

    /* x/y 共用比例，斜向变化率不会增大到 sqrt(2) 倍。 */
    if (distance > max_step) {
        float scale = max_step / distance;
        s_motion_state.tar_speed.vx_speed += dx * scale;
        s_motion_state.tar_speed.vy_speed += dy * scale;
    } else {
        s_motion_state.tar_speed.vx_speed = s_motion_state.cmd_speed.vx_speed;
        s_motion_state.tar_speed.vy_speed = s_motion_state.cmd_speed.vy_speed;
    }
    float dw = s_motion_state.cmd_speed.vw_speed - s_motion_state.tar_speed.vw_speed;
    float max_angular_step = SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 * dt;
    s_motion_state.tar_speed.vw_speed += lib_clamp(dw, -max_angular_step, max_angular_step);
#else
    (void)dt;
    s_motion_state.tar_speed = s_motion_state.cmd_speed;
#endif
}

static void inverse_kinematics(const app_sentry_chassis_speed_t *tar_speed)
{
    float vx_tar_speed = lib_clamp(tar_speed->vx_speed, -SENTRY_CHASSIS_VX_TAR_SPEED_MAX, SENTRY_CHASSIS_VX_TAR_SPEED_MAX);
    float vy_tar_speed = lib_clamp(tar_speed->vy_speed, -SENTRY_CHASSIS_VY_TAR_SPEED_MAX, SENTRY_CHASSIS_VY_TAR_SPEED_MAX);
    float vw_tar_speed = lib_clamp(tar_speed->vw_speed, -SENTRY_CHASSIS_VW_TAR_SPEED_MAX, SENTRY_CHASSIS_VW_TAR_SPEED_MAX);

    for (int i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float wheel_vx_tar_speed = vx_tar_speed - sign * vw_tar_speed * SENTRY_WHEEL_HALF_TRACK;
        float wheel_vy_tar_speed = vy_tar_speed + sign * vw_tar_speed * SENTRY_WHEEL_HALF_BASE;

        float wheel_tar_speed = sqrtf(wheel_vx_tar_speed * wheel_vx_tar_speed + wheel_vy_tar_speed * wheel_vy_tar_speed);
        if (wheel_tar_speed < 1.0f) {
            s_motion_state.wheel_tar[i].steer_angle = s_motion_state.wheel_cur[i].steer_angle;
            s_motion_state.wheel_tar[i].drive_speed = 0.0f;
            s_motion_state.wheel_tar[i].drive_reversed = 0U;
            continue;
        }
        float wheel_tar_angle = atan2f(wheel_vy_tar_speed, wheel_vx_tar_speed);

        float steer_angle_error = lib_get_shortest_path(wheel_tar_angle, s_motion_state.wheel_cur[i].steer_angle);

        if (fabsf(steer_angle_error) > LIB_PI * 0.5f) {
            s_motion_state.wheel_tar[i].drive_reversed = 1U;
            s_motion_state.wheel_tar[i].steer_angle = lib_rad_norm(
                wheel_tar_angle + (steer_angle_error > 0.0f ? -LIB_PI : LIB_PI));
        } else {
            s_motion_state.wheel_tar[i].drive_reversed = 0U;
            s_motion_state.wheel_tar[i].steer_angle = wheel_tar_angle;
        }
        s_motion_state.wheel_tar[i].drive_speed = (s_motion_state.wheel_tar[i].drive_reversed ? -wheel_tar_speed : wheel_tar_speed);
    }
}

static void forward_kinematics(void)
{
    float wheel_vx_cur_speed[2], wheel_vy_cur_speed[2];
    for (int i = 0; i < 2; i++) {
        float wheel_cur_speed = lib_motor_rpm_to_mm_s(
            (float)s_motor_state.feedback[s_drive_index[i]].speed, SENTRY_WHEEL_RADIUS,
            DRV_MOTOR_M3508_REDUCTION) * s_drive_direction[i];
        float wheel_cur_angle = s_motion_state.wheel_cur[i].steer_angle;
        wheel_vx_cur_speed[i] = wheel_cur_speed * cosf(wheel_cur_angle);
        wheel_vy_cur_speed[i] = wheel_cur_speed * sinf(wheel_cur_angle);
    }
    float half_track = SENTRY_WHEEL_HALF_TRACK;
    float half_base = SENTRY_WHEEL_HALF_BASE;
    s_motion_state.cur_speed.vx_speed = (wheel_vx_cur_speed[0] + wheel_vx_cur_speed[1]) * 0.5f;
    s_motion_state.cur_speed.vy_speed = (wheel_vy_cur_speed[0] + wheel_vy_cur_speed[1]) * 0.5f;
    s_motion_state.cur_speed.vw_speed = (half_track * (wheel_vx_cur_speed[1] - wheel_vx_cur_speed[0])
                     + half_base * (wheel_vy_cur_speed[0] - wheel_vy_cur_speed[1]))
                    / (2.0f * (half_track * half_track + half_base * half_base));
}

static void chassis_pid(void)
{
    float fx = lib_pid_calc(&s_control_state.x, s_motion_state.tar_speed.vx_speed, s_motion_state.cur_speed.vx_speed, 0.001f);
    float fy = lib_pid_calc(&s_control_state.y, s_motion_state.tar_speed.vy_speed, s_motion_state.cur_speed.vy_speed, 0.001f);
    float torque = lib_pid_calc(&s_control_state.w, s_motion_state.tar_speed.vw_speed, s_motion_state.cur_speed.vw_speed, 0.001f);
    chassis_wrench_allocate(fx, fy, torque);
}

static void chassis_wrench_allocate(float fx, float fy, float torque)
{
    float half_track = SENTRY_WHEEL_HALF_TRACK;
    float half_base = SENTRY_WHEEL_HALF_BASE;
    float lever_squared = half_track * half_track + half_base * half_base;

    int i;
    for (i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float fix = fx * 0.5f - sign * torque * half_track / (2.0f * lever_squared);
        float fiy = fy * 0.5f + sign * torque * half_base / (2.0f * lever_squared);
        float wheel_cur_angle = s_motion_state.wheel_cur[i].steer_angle;

        float current_ff = (fix * cosf(wheel_cur_angle) + fiy * sinf(wheel_cur_angle))
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
        float drive_tar_speed = s_motion_state.wheel_tar[i].drive_speed * s_drive_direction[i];
        float drive_cur_speed = lib_motor_rpm_to_mm_s((float)s_motor_state.feedback[drive_index].speed,
        SENTRY_WHEEL_RADIUS,DRV_MOTOR_M3508_REDUCTION);
        float drive_current_ff = (float)s_motor_state.drive_current_ff[i]
                               * s_drive_direction[i];
        float drive_current = lib_pid_ff_calc(&s_control_state.drive[i], drive_tar_speed,
                                               drive_cur_speed, drive_current_ff, 0.0f, 0.001f);
        drive_current = lib_clamp(drive_current, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX);
        s_motor_state.current[drive_index] = (int16_t)drive_current;

        float steer_measure = s_motion_state.wheel_cur[i].steer_angle;
        /* 将目标展开到反馈附近，让 PID 的 target - measure 始终取最短角差。 */
        float steer_target = steer_measure + lib_get_shortest_path(
            s_motion_state.wheel_tar[i].steer_angle, steer_measure);
        float steer_speed = (float)s_motor_state.feedback[M_STEER_L + i].speed;

        float steer_current = lib_pid_pos_calc(
            &s_control_state.steer[i], steer_target, steer_measure,
            0.0f, 0.0f, steer_speed, 0.001f);
        steer_current *= s_steer_direction[i];
        steer_current = lib_clamp(steer_current, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
        s_motor_state.current[M_STEER_L + i] = (int16_t)steer_current;
    }
}

static void power_update(void)
{
    drv_power_data_t measured_power;

    power_measure_read_snapshot(&measured_power);
    (void)lib_lpf_update(
        &s_power_state.measure_lpf, measured_power.power, 0.001f);
    s_power_state.measured_power = fmaxf(s_power_state.measure_lpf.out, 0.0f);
    s_chassis_state.battery_voltage = measured_power.bat_v / 100.0f;
    s_chassis_state.battery_current = measured_power.bat_i / 100.0f;
}

static void power_measure_read_snapshot(drv_power_data_t *power)
{
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    *power = s_power_state.rx;
    __set_PRIMASK(irq_state);
}

static void power_apply_limit(void)
{
    if (s_power_state.tar_power <= 0.0f) {
        lib_pid_reset(&s_control_state.power);
        s_power_state.scale = 0.0f;
    } else {
        float correction = lib_pid_calc(&s_control_state.power,
                                        s_power_state.tar_power,
                                        s_power_state.measured_power, 0.001f);
        s_power_state.scale = lib_clamp(1.0f + correction, 0.0f, 1.0f);
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_state.current[i] = (int16_t)(s_motor_state.current[i]
                                            * s_power_state.scale);
    }
}

static void power_publish(void)
{
    s_chassis_state.cur_power = s_power_state.measured_power;
    s_chassis_state.tar_power = s_power_state.tar_power;
    s_chassis_state.power_scale = s_power_state.scale;
}

static void chassis_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x200：[右驱动、左驱动]电流帧。 */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[M_DRIVE_R]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[M_DRIVE_R]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[M_DRIVE_L]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[M_DRIVE_L]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_DRIVE, frame);
/* 0x1FF：[左转向、右转向]电流帧。 */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[M_STEER_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[M_STEER_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[M_STEER_R]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[M_STEER_R]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_CHASSIS_TX_STEER, frame);
}

static void chassis_can1_tx(void)
{

    int16_t power_x100 = (int16_t)lib_clamp(
        s_power_state.measured_power * 100.0f, -32768.0f, 32767.0f);

    (void)app_chassis_comm_power_tx(power_x100);
    (void)app_chassis_comm_omega_tx(s_motion_state.omega_tx_lpf.out);
}
