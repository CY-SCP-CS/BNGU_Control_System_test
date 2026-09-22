/**
 * @file    app_sentry_chassis.c
 * @brief   哨兵双舵轮底盘控制实现
 * @note    控制流程：底盘目标速度 → 逆运动学 → 正运动学 → 车体 PID →
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

#define MOTOR_COUNT  4U


#define SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ  20.0f

/** 底盘电机在反馈表和电流帧中的索引。 */
enum { M_DRIVE_R = 0, M_DRIVE_L = 1, M_STEER_L = 2, M_STEER_R = 3 };

/** 电机反馈快照、CAN 接收缓冲、电流输出和驱动前馈。 */
typedef struct {
    drv_motor_data_t feedback[MOTOR_COUNT]; // 控制循环使用的稳定反馈快照。
    drv_motor_data_t rx[MOTOR_COUNT];       // CAN 中断写入的反馈缓冲。
    int16_t current[MOTOR_COUNT];           // 本周期电流输出。
    int16_t cur_speed[MOTOR_COUNT];         // 电机当前转速，RPM。
    int16_t drive_current_ff[2];            // 两个驱动轮的电流前馈。
} app_sentry_chassis_motor_state_t;

/** 底盘目标/当前状态和动力学中间量。 */
typedef struct {
    app_sentry_swerve_wheel_t wheel_cur[2]; // 当前舵轮状态。
    app_sentry_swerve_wheel_t wheel_tar[2]; // 目标舵轮状态。
    app_sentry_chassis_speed_t tar_speed;   // 车体目标速度。
    app_sentry_chassis_speed_t cur_speed;   // 正运动学估算的车体速度。
    lib_lpf_t omega_tx_lpf;                 // CAN1 角速度反馈低通滤波器。
    float force;                            // 车体合力大小。
    float force_angle;                      // 车体合力方向，rad。
    float torque;                           // 绕 z 轴目标力矩。
} app_sentry_chassis_motion_state_t;

/** 功率计反馈、滤波状态和电流缩放系数。 */
typedef struct {
    drv_power_data_t rx;      // 功率计 CAN 接收缓冲。
    lib_lpf_t measure_lpf;    // 实测功率低通滤波器。
    float measured_power;     // 滤波后的实测功率，W。
    float tar_power;          // 功率控制目标，W。
    float scale;              // 电机电流缩放系数，范围 0~1。
    float battery_voltage;    // 电池电压，V。
    float battery_current;    // 电池电流，A。
} app_sentry_chassis_power_state_t;

/** 底盘各级 PID 状态。 */
typedef struct {
    lib_pid_t x;        // 车体 x 方向速度环。
    lib_pid_t y;        // 车体 y 方向速度环。
    lib_pid_t w;        // 车体角速度环。
    lib_pid_t steer[2]; // 两个转向角度环。
    lib_pid_t drive[2]; // 两个驱动速度环。
    lib_pid_t power;    // 功率缩放环。
} app_sentry_chassis_control_state_t;

/**
 * @brief 底盘全局调试快照。
 *
 * 保留外部链接便于调试器 Watch 在任意断点查看；不在头文件声明，
 * 不作为模块间访问接口。
 */
typedef struct {
    app_sentry_chassis_motor_state_t motor;
    app_sentry_chassis_motion_state_t motion;
    app_sentry_chassis_power_state_t power;
    app_sentry_chassis_control_state_t pid;
    app_sentry_chassis_state_t state;
} app_sentry_chassis_debug_t;

app_sentry_chassis_debug_t app_sentry_chassis_debug = {
    .power = {
        .tar_power = SENTRY_CHASSIS_TAR_POWER,
        .scale = 1.0f,
    },
};

#define s_motor_state   app_sentry_chassis_debug.motor
#define s_motion_state  app_sentry_chassis_debug.motion
#define s_power_state   app_sentry_chassis_debug.power
#define s_control_state app_sentry_chassis_debug.pid
#define s_chassis_state app_sentry_chassis_debug.state

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
    lib_pid_init(&s_control_state.steer[0], 10000.0f, 10.0f, 10.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 0.0f);
    lib_pid_init(&s_control_state.steer[1], 10000.0f, 10.0f, 10.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 0.0f);
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
static void inverse_kinematics(const app_sentry_chassis_speed_t *tar_speed);
static void forward_kinematics(void);
static void chassis_pid(void);
static void chassis_wrench_allocate(void);
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
    lib_lpf_init(&s_power_state.measure_lpf, 35.0f);
    lib_lpf_init(&s_motion_state.omega_tx_lpf, SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ);
    pid_init_all();
    s_can1_tx_divider = 0U;

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_L, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_STEER_R, on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_CHASSIS_POWER, on_power_measure_feedback);

}
void app_sentry_chassis_ctrl(void)
{
    app_chassis_comm_rx_t comm_rx;
    app_chassis_speed_cmd_t can_cmd;
    uint32_t irq_state = __get_PRIMASK();

    /* 复制 CAN 中断写入的反馈，确保本周期使用一致的数据。 */
    __disable_irq();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_motor_state.cur_speed[i] = s_motor_state.feedback[i].speed;
    }
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
        return;
    }

    /* 将 0x111 命令解码为车体坐标系目标速度并限幅。 */
    s_motion_state.tar_speed.vx_speed = lib_clamp((float)can_cmd.vx,
        -SENTRY_CHASSIS_VX_TAR_SPEED_MAX, SENTRY_CHASSIS_VX_TAR_SPEED_MAX);
    s_motion_state.tar_speed.vy_speed = lib_clamp((float)can_cmd.vy,
        -SENTRY_CHASSIS_VY_TAR_SPEED_MAX, SENTRY_CHASSIS_VY_TAR_SPEED_MAX);
    s_motion_state.tar_speed.vw_speed = lib_clamp(
        (float)can_cmd.vz * APP_CHASSIS_OMEGA_RAD_S_PER_LSB,
        -SENTRY_CHASSIS_VW_TAR_SPEED_MAX, SENTRY_CHASSIS_VW_TAR_SPEED_MAX);

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
        (float)s_motor_state.cur_speed[M_DRIVE_L], SENTRY_WHEEL_RADIUS,
        DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[1].drive_speed = lib_motor_rpm_to_mm_s(
        (float)s_motor_state.cur_speed[M_DRIVE_R], SENTRY_WHEEL_RADIUS,
        DRV_MOTOR_M3508_REDUCTION);
    s_motion_state.wheel_cur[0].drive_rev = 1;
    s_motion_state.wheel_cur[1].drive_rev = 1;

    inverse_kinematics(&s_motion_state.tar_speed);
    forward_kinematics();
    (void)lib_lpf_update(&s_motion_state.omega_tx_lpf,
                         s_motion_state.cur_speed.vw_speed, 0.001f);
    chassis_pid();
    chassis_wrench_allocate();
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
    memset(&s_motion_state.tar_speed, 0, sizeof(s_motion_state.tar_speed));
    memset(&s_motion_state.cur_speed, 0, sizeof(s_motion_state.cur_speed));
    lib_lpf_init(&s_motion_state.omega_tx_lpf, SENTRY_CHASSIS_OMEGA_TX_LPF_CUTOFF_HZ);
    memset(&s_chassis_state, 0, sizeof(s_chassis_state));
    s_power_state.scale = 0.0f;
    pid_init_all();
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
            s_motion_state.wheel_tar[i].drive_rev = 1;
            continue;
        }
        float wheel_tar_angle = atan2f(wheel_vy_tar_speed, wheel_vx_tar_speed);

        float steer_angle_error = lib_get_shortest_path(wheel_tar_angle, s_motion_state.wheel_cur[i].steer_angle);

        if (fabsf(steer_angle_error) > LIB_PI * 0.5f) {
            s_motion_state.wheel_tar[i].drive_rev  = -1;
            s_motion_state.wheel_tar[i].steer_angle = lib_rad_norm(
                wheel_tar_angle + (steer_angle_error > 0.0f ? -LIB_PI : LIB_PI));
        } else {
            s_motion_state.wheel_tar[i].drive_rev  = 1;
            s_motion_state.wheel_tar[i].steer_angle = wheel_tar_angle;
        }
        s_motion_state.wheel_tar[i].drive_speed = wheel_tar_speed * (float)s_motion_state.wheel_tar[i].drive_rev;
    }
}

static void forward_kinematics(void)
{
    float wheel_vx_cur_speed[2], wheel_vy_cur_speed[2];
    for (int i = 0; i < 2; i++) {
        float wheel_cur_speed = lib_motor_rpm_to_mm_s(
            (float)s_motor_state.cur_speed[s_drive_index[i]], SENTRY_WHEEL_RADIUS,
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
    s_motion_state.force       = sqrtf(fx * fx + fy * fy);
    s_motion_state.force_angle = atan2f(fy, fx);
    s_motion_state.torque      = lib_pid_calc(&s_control_state.w, s_motion_state.tar_speed.vw_speed, s_motion_state.cur_speed.vw_speed, 0.001f);
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
        float drive_cur_speed = lib_motor_rpm_to_mm_s((float)s_motor_state.cur_speed[drive_index], 
        SENTRY_WHEEL_RADIUS,DRV_MOTOR_M3508_REDUCTION);
        float drive_current_ff = (float)s_motor_state.drive_current_ff[i]
                               * s_drive_direction[i];
        float drive_current = lib_pid_ff_calc(&s_control_state.drive[i], drive_tar_speed,
                                               drive_cur_speed, drive_current_ff, 0.0f, 0.001f);
        drive_current = lib_clamp(drive_current, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX);
        s_motor_state.current[drive_index] = (int16_t)drive_current;

        float steer_target = s_motion_state.wheel_tar[i].steer_angle;
        float steer_measure = s_motion_state.wheel_cur[i].steer_angle;
        float steer_speed = (float)s_motor_state.cur_speed[M_STEER_L + i];

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
    s_power_state.measure_lpf.out = lib_lpf_update(
        &s_power_state.measure_lpf, measured_power.power, 0.001f);
    s_power_state.measured_power = fmaxf(s_power_state.measure_lpf.out, 0.0f);
    s_power_state.battery_voltage = measured_power.bat_v / 100.0f;
    s_power_state.battery_current = measured_power.bat_i / 100.0f;
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
    s_chassis_state.battery_voltage = s_power_state.battery_voltage;
    s_chassis_state.battery_current = s_power_state.battery_current;
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
