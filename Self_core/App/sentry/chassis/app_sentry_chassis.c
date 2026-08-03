/**
 * @file    app_sentry_chassis.c
 * @brief   Sentry 底盘控制实现 — 麦轮运动学 + 级联PID
 * @note    Ported from Chassis_RD1/libs/CONTROL.c + DATA.c + PID.c
 *          CAN2 = 板内电机控制, CAN1 = 板间数据交换
 *
 * 控制流水线 (每 1ms):
 *   DBUS → body-frame目标速度
 *   → Inverse Kinematics → Motor_Speed_tar (RPM)
 *   → Forward Kinematics (body) → body_speed_cur
 *   → 底盘级PID → 极坐标力/力矩
 *   → Force_To_Torque → 每轮前馈
 *   → 轮级FF-PID → 电流输出 (int16)
 *   → CAN2发送
 *
 * 板间数据:
 *   CAN1 RX 0x124: 云台→底盘 yaw角度 (用于世界坐标估算, 替代GM6020编码器)
 *   CAN1 TX 0x112: 底盘→云台 功率反馈 + omega_z (VMC前馈用)
 */
#include "app_sentry_chassis.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"       /* CAN1 角度反馈ID  */
#include "drv_dbus.h"
#include "drv_motor.h"
#include "lib_math.h"
#include "bsp_can.h"
#include "app_gimbal_comm.h"

#include <math.h>
#include <string.h>

/* ── 字节打包 ─────────────────────────────────── */
#define SENTRY_HI_BYTE(x)  ((uint8_t)((x) >> 8))
#define SENTRY_LO_BYTE(x)  ((uint8_t)(x))

/* ════════════════════════════════════════════════════
 * 电机数据 (CAN2)
 * ════════════════════════════════════════════════════ */

#define SENTRY_CHASSIS_MOTOR_COUNT  5   /* M3508×4 + GM6020×1 */

static drv_motor_data_t  s_motor_data[SENTRY_CHASSIS_MOTOR_COUNT];
static int16_t           s_motor_speed_cur[SENTRY_CHASSIS_MOTOR_COUNT];
static int16_t           s_motor_speed_tar[SENTRY_CHASSIS_MOTOR_COUNT];
static int16_t           s_motor_ff[4];            /* 前馈扭矩 (仅4轮)          */
static int16_t           s_motor_output[SENTRY_CHASSIS_MOTOR_COUNT];

/* ════════════════════════════════════════════════════
 * 底盘速度
 * ════════════════════════════════════════════════════ */

static app_sentry_chassis_speed_t s_speed_body_cur;  /**< 车体估算速度 (body-frame) */
static app_sentry_chassis_speed_t s_speed_body_tar;  /**< DBUS目标速度 (body-frame) */
static app_sentry_chassis_speed_t s_speed_world_cur; /**< 世界坐标速度 (输出用)     */
static app_sentry_chassis_force_t s_target_force;

/* yaw (来自云台 CAN1 0x124, 初值0) */
static float s_yaw_angle_deg;          /**< 底盘yaw角度 (deg, 来自云台BMI088)  */
static float s_chassis_omega_z;        /**< 底盘角速度 ωz (rad/s, 前向运动学)  */

/* ════════════════════════════════════════════════════
 * PID 实例 (from Chassis_RD1)
 * ════════════════════════════════════════════════════ */

static lib_pid_t s_pid_speed_x;    /**< x速度→力        Kp=1.0 Ki=1.0 Kd=1.0     */
static lib_pid_t s_pid_speed_y;    /**< y速度→力        Kp=1.0 Ki=1.0 Kd=1.0     */
static lib_pid_t s_pid_speed_w;    /**< 角速度→力矩     Kp=1.0 Ki=1.0 Kd=1.0     */
static lib_pid_t s_pid_wheel[4];   /**< 轮级FF-PID      Kp=3.0 Ki=0.01 Kff_v=0.8 */

/* yaw角度PID: Kp=200, Ki=1, Kd=30000 (degree-scale)
 *   目标来自DBUS CH[0], 反馈来自云台CAN1 0x124 */
static lib_pid_t s_pid_yaw_angle;

/* ════════════════════════════════════════════════════
 * 私有函数
 * ════════════════════════════════════════════════════ */

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_speed,
                               int16_t *motor_rpm);
static void forward_kinematics(const int16_t *motor_rpm,
                               app_sentry_chassis_speed_t *body_speed,
                               float *omega_z);
static void chassis_pid_to_force(const app_sentry_chassis_speed_t *cur,
                                  const app_sentry_chassis_speed_t *tar);
static void force_to_torque(float force, float angle, float torque,
                            int16_t *ff_out);
static void chassis_send_can2(void);
static void on_gimbal_angle_feedback(uint32_t std_id, uint8_t *data, uint8_t len);

/* ════════════════════════════════════════════════════
 * PID 初始化
 * ════════════════════════════════════════════════════ */

static void pid_init_all(void)
{
    int i;

    /* 速度环: Kp=1, Ki=1, Kd=1, out±3000, i_max=500 */
    lib_pid_init(&s_pid_speed_x, 1.0f, 1.0f, 1.0f, 0, 0, 0, 0, -3000, 3000, 500);
    lib_pid_init(&s_pid_speed_y, 1.0f, 1.0f, 1.0f, 0, 0, 0, 0, -3000, 3000, 500);
    lib_pid_init(&s_pid_speed_w, 1.0f, 1.0f, 1.0f, 0, 0, 0, 0, -3000, 3000, 500);

    /* 轮级: Kp=3, Ki=0.01, Kff_v=0.8, out±3000, i_max=2000 */
    for (i = 0; i < 4; i++) {
        lib_pid_init(&s_pid_wheel[i], 3.0f, 0.01f, 0.0f, 0.8f, 0,
                     3000, 0, -3000, 3000, 2000);
    }

    /* yaw角度: Kp=200, Ki=1, Kd=30000 (degree-scale), out±6000, i_max=2000 */
    lib_pid_init(&s_pid_yaw_angle, 200.0f, 1.0f, 30000.0f,
                 0, 0, 0, 0, -6000, 6000, 2000);
}

/* ════════════════════════════════════════════════════
 * 逆运动学: body-frame速度 → 4轮RPM
 * ════════════════════════════════════════════════════ */

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_speed,
                               int16_t *motor_rpm)
{
    float vx = lib_math_clamp(body_speed->v_x,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vy = lib_math_clamp(body_speed->v_y,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vw = lib_math_clamp(body_speed->v_w,
                              -SENTRY_MAX_OMEGA, SENTRY_MAX_OMEGA);

    float f = SENTRY_MECANUM_FACTOR;
    float r = SENTRY_WHEEL_BASE_RADIUS_MM;

    float w0 = -f * vx + f * vy + vw * r;   /* 右前 */
    float w1 = -f * vx - f * vy + vw * r;   /* 左前 */
    float w2 =  f * vx - f * vy + vw * r;   /* 左后 */
    float w3 =  f * vx + f * vy + vw * r;   /* 右后 */

    float rpm[4] = { w0, w1, w2, w3 };
    int i;
    for (i = 0; i < 4; i++) {
        motor_rpm[i] = (int16_t)lib_math_clamp(
            rpm[i] * SENTRY_MMPS_TO_RPM(1.0f),
            -SENTRY_MAX_MOTOR_RPM, SENTRY_MAX_MOTOR_RPM);
    }
}

/* ════════════════════════════════════════════════════
 * 正运动学: 4轮RPM → 车体速度 (body-frame)
 * ════════════════════════════════════════════════════ */

static void forward_kinematics(const int16_t *motor_rpm,
                               app_sentry_chassis_speed_t *body_speed,
                               float *omega_z)
{
    int i;
    float v[4];
    for (i = 0; i < 4; i++) {
        v[i] = (float)motor_rpm[i] * SENTRY_RPM_TO_MMPS(1.0f);
    }

    float f = SENTRY_MECANUM_FACTOR;
    float r = SENTRY_WHEEL_BASE_RADIUS_MM;

    body_speed->v_x = (-v[0] - v[1] + v[2] + v[3]) / (4.0f * f);
    body_speed->v_y = ( v[0] - v[1] - v[2] + v[3]) / (4.0f * f);
    body_speed->v_w = ( v[0] + v[1] + v[2] + v[3]) / (4.0f * r);

    /* 角速度输出 (rad/s, 供VMC前馈用) */
    if (omega_z) {
        *omega_z = body_speed->v_w;  /* v_w 已经是 rad/s */
    }
}

/* ════════════════════════════════════════════════════
 * 底盘级PID: body-frame速度误差 → 极坐标力/力矩
 * ════════════════════════════════════════════════════ */

static void chassis_pid_to_force(const app_sentry_chassis_speed_t *cur,
                                  const app_sentry_chassis_speed_t *tar)
{
    float fx = lib_pid_calc(&s_pid_speed_x, tar->v_x, cur->v_x);
    float fy = lib_pid_calc(&s_pid_speed_y, tar->v_y, cur->v_y);

    s_target_force.force  = sqrtf(fx * fx + fy * fy);
    s_target_force.angle  = atan2f(fy, fx);
    s_target_force.torque = lib_pid_calc(&s_pid_speed_w, tar->v_w, cur->v_w);
}

/* ════════════════════════════════════════════════════
 * 力分解: 极坐标力/力矩 → 每轮前馈扭矩
 * ════════════════════════════════════════════════════ */

static void force_to_torque(float force, float angle, float torque,
                            int16_t *ff_out)
{
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float f = SENTRY_MECANUM_FACTOR;

    float w0 = f * (-cos_a + sin_a) + torque;
    float w1 = f * (-cos_a - sin_a) + torque;
    float w2 = f * ( cos_a - sin_a) + torque;
    float w3 = f * ( cos_a + sin_a) + torque;

    ff_out[0] = (int16_t)lib_math_clamp(w0, -3000, 3000);
    ff_out[1] = (int16_t)lib_math_clamp(w1, -3000, 3000);
    ff_out[2] = (int16_t)lib_math_clamp(w2, -3000, 3000);
    ff_out[3] = (int16_t)lib_math_clamp(w3, -3000, 3000);
}

/* ════════════════════════════════════════════════════
 * CAN2 发送 (板内电机)
 * ════════════════════════════════════════════════════ */

static void chassis_send_can2(void)
{
    uint8_t frame[8];

    /* M3508×4 电流帧 0x200: [M0_H,M0_L, M1_H,M1_L, M2_H,M2_L, M3_H,M3_L] */
    memset(frame, 0, sizeof(frame));
    drv_motor_build_dji_frame_init(frame);
    int i;
    for (i = 0; i < 4; i++) {
        drv_motor_build_dji_frame_set(frame, (uint8_t)i, s_motor_output[i]);
    }
    bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_M3508, frame);

    /* GM6020 yaw 电流帧 0x1FF: [Y_H,Y_L, 0,0,0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = SENTRY_HI_BYTE(s_motor_output[4]);
    frame[1] = SENTRY_LO_BYTE(s_motor_output[4]);
    bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_YAW, frame);
}

/* ════════════════════════════════════════════════════
 * CAN1 板间接收: 云台→底盘 yaw角度 (0x124)
 * ════════════════════════════════════════════════════ */

static void on_gimbal_angle_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    float yaw_rad, pitch_rad;
    memcpy(&yaw_rad,   data,     sizeof(float));
    memcpy(&pitch_rad, data + 4, sizeof(float));
    (void)pitch_rad;
    s_yaw_angle_deg = yaw_rad * (180.0f / (float)LIB_MATH_PI);
}

/* ════════════════════════════════════════════════════
 * 公有接口
 * ════════════════════════════════════════════════════ */

void app_sentry_chassis_init(void)
{
    memset(s_motor_data,     0, sizeof(s_motor_data));
    memset(s_motor_speed_cur, 0, sizeof(s_motor_speed_cur));
    memset(s_motor_speed_tar, 0, sizeof(s_motor_speed_tar));
    memset(s_motor_ff,       0, sizeof(s_motor_ff));
    memset(s_motor_output,   0, sizeof(s_motor_output));
    memset(&s_speed_body_cur, 0, sizeof(s_speed_body_cur));
    memset(&s_speed_body_tar, 0, sizeof(s_speed_body_tar));
    memset(&s_speed_world_cur, 0, sizeof(s_speed_world_cur));
    memset(&s_target_force,  0, sizeof(s_target_force));
    s_yaw_angle_deg   = 0;
    s_chassis_omega_z = 0;

    pid_init_all();

    /* ── 注册 CAN2 电机反馈回调 (板内) ── */
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_M3508_BASE + 0,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_M3508_BASE + 1,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_M3508_BASE + 2,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_M3508_BASE + 3,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_GM6020_YAW,
                                 app_sentry_chassis_motor_feedback);

    /* ── 注册 CAN1 板间接收: 云台 yaw 角度 (0x124) ── */
    bsp_can_register_rx_callback(&hcan1, APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK,
                                 on_gimbal_angle_feedback);
}

void app_sentry_chassis_control(void)
{
    int i;

    /* ── 1. DBUS → body-frame 目标速度 ── */
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
    if (dbus) {
        s_speed_body_tar.v_x = (float)dbus->rc.ch[3]
                             * SENTRY_MAX_LINEAR_SPEED / 660.0f;
        s_speed_body_tar.v_y = (float)dbus->rc.ch[2]
                             * SENTRY_MAX_LINEAR_SPEED / 660.0f;
        s_speed_body_tar.v_w = (float)dbus->rc.ch[1]
                             * SENTRY_MAX_OMEGA / 660.0f;

        /* yaw角度PID: CH[0] → target±180°, 误差=deg scale */
        float yaw_target_deg = (float)dbus->rc.ch[0] * 180.0f / 660.0f;
        float yaw_error_deg  = lib_math_get_shortest_path(
            lib_math_deg2rad(yaw_target_deg),
            lib_math_deg2rad(s_yaw_angle_deg));
        yaw_error_deg *= (180.0f / (float)LIB_MATH_PI);   /* rad→deg */

        s_motor_output[4] = (int16_t)lib_pid_calc(
            &s_pid_yaw_angle, 0, yaw_error_deg);
    }

    /* ── 2. 逆运动学: body-frame → RPM ── */
    inverse_kinematics(&s_speed_body_tar, s_motor_speed_tar);

    /* ── 3. 正运动学: RPM → body-frame估算速度 ── */
    forward_kinematics(s_motor_speed_cur, &s_speed_body_cur, &s_chassis_omega_z);

    /* ── 4. 世界坐标 (用云台yaw旋转) ── */
    if (s_yaw_angle_deg != 0) {
        float yaw_rad = lib_math_deg2rad(s_yaw_angle_deg);
        float cos_y = cosf(yaw_rad), sin_y = sinf(yaw_rad);
        s_speed_world_cur.v_x = s_speed_body_cur.v_x * cos_y
                              - s_speed_body_cur.v_y * sin_y;
        s_speed_world_cur.v_y = s_speed_body_cur.v_x * sin_y
                              + s_speed_body_cur.v_y * cos_y;
        s_speed_world_cur.v_w = s_speed_body_cur.v_w;
    }

    /* ── 5. 底盘级PID: body-frame速度误差 → 力/力矩 ── */
    chassis_pid_to_force(&s_speed_body_cur, &s_speed_body_tar);

    /* ── 6. 力分解 → 每轮前馈 ── */
    force_to_torque(s_target_force.force, s_target_force.angle,
                    s_target_force.torque, s_motor_ff);

    /* ── 7. 轮级FF-PID → 电流 ── */
    for (i = 0; i < 4; i++) {
        s_motor_output[i] = (int16_t)lib_pid_ff_calc(
            &s_pid_wheel[i],
            (float)s_motor_speed_tar[i],
            (float)s_motor_speed_cur[i],
            (float)s_motor_ff[i], 0);
    }

    /* ── 8. CAN2 发送电机电流 ── */
    chassis_send_can2();
}

void app_sentry_chassis_motor_feedback(uint32_t std_id, uint8_t *data,
                                       uint8_t len)
{
    (void)len;

    if (std_id >= SENTRY_CAN_CHASSIS_M3508_BASE
        && std_id <= SENTRY_CAN_CHASSIS_M3508_BASE + 3) {
        uint8_t idx = (uint8_t)(std_id - SENTRY_CAN_CHASSIS_M3508_BASE);
        if (idx < 4) {
            drv_motor_solve_dji_data(data, &s_motor_data[idx]);
            s_motor_speed_cur[idx] = s_motor_data[idx].speed;
        }
    } else if (std_id == SENTRY_CAN_CHASSIS_GM6020_YAW) {
        drv_motor_solve_dji_data(data, &s_motor_data[4]);
        s_motor_speed_cur[4] = s_motor_data[4].speed;
    }
}

const app_sentry_chassis_speed_t *app_sentry_chassis_get_speed(void)
{
    return &s_speed_world_cur;
}

const app_sentry_chassis_speed_t *app_sentry_chassis_get_target(void)
{
    return &s_speed_body_tar;
}
