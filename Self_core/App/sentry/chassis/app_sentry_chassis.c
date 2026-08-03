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
 *   G0=云台yaw (0x207) — 提供底盘朝向
 *
 * 控制流水线 (1kHz):
 *   CAN1 0x113 或 DBUS → body-frame目标速度
 *   → G0 yaw旋转 → world-frame
 *   → 逆运动学 (每轮角度+速度, >90°反转优化)
 *   → 正运动学 → body-frame估算速度 + ωz
 *   → 底盘PID → 极坐标力/力矩
 *   → 力分配 → 每轮驱动前馈
 *   → 驱动FF-PID + 转向角度PID
 *   → 功率限制 → CAN2发送
 */
#include "app_sentry_chassis.h"

#include "app_chassis_comm.h"     /* CAN1 0x113 阿克曼指令  */
#include "app_gimbal_comm.h"      /* CAN1 0x124 角度反馈ID   */
#include "drv_dbus.h"
#include "drv_motor.h"
#include "lib_filter.h"
#include "lib_math.h"
#include "bsp_can.h"

#include <math.h>
#include <string.h>

/* ── 字节打包 ─────────────────────────────────── */
#define SENTRY_HI(x)  ((uint8_t)((x) >> 8))
#define SENTRY_LO(x)  ((uint8_t)(x))

/* ════════════════════════════════════════════════════
 * 电机索引
 * ════════════════════════════════════════════════════ */

#define MOTOR_COUNT  5
enum { M_DRIVE_R=0, M_DRIVE_L=1, M_STEER_L=2, M_STEER_R=3, M_GIMBAL_G0=4 };

static drv_motor_data_t s_motor[MOTOR_COUNT];
static int16_t          s_motor_current[MOTOR_COUNT];     /* 电流输出      */
static int16_t          s_motor_speed_cur[MOTOR_COUNT];   /* 当前 RPM      */
static int16_t          s_motor_speed_tar[MOTOR_COUNT];   /* 目标 RPM      */
static int16_t          s_motor_ff[2];                    /* 驱动前馈扭矩   */

/* ════════════════════════════════════════════════════
 * 舵轮状态
 * ════════════════════════════════════════════════════ */

static app_sentry_swerve_wheel_t s_swerve_cur[2];  /**< 当前状态 [0]=左 [1]=右  */
static app_sentry_swerve_wheel_t s_swerve_tar[2];  /**< 目标状态                 */
static app_sentry_chassis_speed_t s_body_tar;      /**< body-frame 目标速度      */
static app_sentry_chassis_speed_t s_body_cur;      /**< body-frame 估算速度      */
static app_sentry_chassis_state_t s_chassis_state; /**< 全局状态                 */

static float s_force, s_force_angle, s_torque;     /**< PID输出的力/力矩         */

/* ════════════════════════════════════════════════════
 * 功率限制 (from Steering_wheel_Chasssis_test POWER.c)
 * ════════════════════════════════════════════════════ */

#define SENTRY_TARGET_POWER   70.0f    /**< 目标功率 (W)                    */
#define SENTRY_POWER_MAX_ATTEN 0.3f    /**< 最低功率衰减系数                 */
static float s_power_scale = 1.0f;     /**< 功率缩放系数                     */
static float s_power_cur   = 0.0f;     /**< 当前功率估算 (W)                */

/* ── M3508 功率模型系数 (多项式) ── */
static const float s_power_coeff_m3508[6] = {
    2.4003793160f, 0.0017839099f, 0.0005151336f,
    0.0000027461f, 0.0000003893f, 0.0000014012f
};

/* ════════════════════════════════════════════════════
 * PID (from Steering_wheel_Chasssis_test INIT.c)
 * ════════════════════════════════════════════════════ */

static lib_pid_t s_pid_x;        /**< vx PID: Kp=10 Ki=0.5 Kd=0 out±15000 i=1000 */
static lib_pid_t s_pid_y;        /**< vy PID  同上                                 */
static lib_pid_t s_pid_w;        /**< vw PID  同上                                 */
static lib_pid_t s_pid_steer[2]; /**< 转向角度 PID: Kp=500 Ki=0.5 Kd=0 out±20000 i=2000 */
static lib_pid_t s_pid_drive[2]; /**< 驱动转速 PID: Kp=1.0 Kff_v=0.8 out±12000 i=2000 */

/* ════════════════════════════════════════════════════
 * 私有函数
 * ════════════════════════════════════════════════════ */

static float calc_logical_angle(uint16_t raw_enc, uint16_t offset);
static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd,
                               float yaw_deg);
static void forward_kinematics(void);
static void chassis_pid(void);
static void force_distribute(void);
static void wheel_control(void);
static float motor_power_model(float current, float speed, const float *coeff);
static void power_limit(void);
static void chassis_send_can2(void);
static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len);

/* ════════════════════════════════════════════════════
 * PID 初始化 (from Steering_wheel_Chasssis_test INIT.c)
 * ════════════════════════════════════════════════════ */

static void pid_init_all(void)
{
    /* 底盘速度PID */
    lib_pid_init(&s_pid_x, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);
    lib_pid_init(&s_pid_y, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);
    lib_pid_init(&s_pid_w, 10.0f, 0.5f, 0.0f, 0, 0, 0, 0, -15000, 15000, 1000);

    /* 转向角度PID: Kp=500, Ki=0.5, out±20000, i=2000 */
    int i;
    for (i = 0; i < 2; i++) {
        lib_pid_init(&s_pid_steer[i], 500.0f, 0.5f, 0.0f,
                     0, 0, 0, 0, -20000, 20000, 2000);
    }

    /* 驱动转速FF-PID: Kp=1.0, Kff_v=0.8, out±12000, i=2000 */
    for (i = 0; i < 2; i++) {
        lib_pid_init(&s_pid_drive[i], 1.0f, 0.0f, 0.0f, 0.8f, 0,
                     10000, 0, -12000, 12000, 2000);
    }
}

/* ════════════════════════════════════════════════════
 * 编码器→逻辑角度 (from DATA.c Calculate_Logical_Angle)
 * ════════════════════════════════════════════════════ */

static float calc_logical_angle(uint16_t raw_enc, uint16_t offset)
{
    int32_t diff = (int32_t)raw_enc - (int32_t)offset;
    if (diff > 4096)       diff -= 8192;
    else if (diff < -4095) diff += 8192;
    return (float)diff * 360.0f / 8192.0f;
}

/* ════════════════════════════════════════════════════
 * 逆运动学: body速度 → 每轮角度+转速
 * ════════════════════════════════════════════════════ */

static void inverse_kinematics(const app_sentry_chassis_speed_t *body_spd,
                               float yaw_deg)
{
    float vx = lib_math_clamp(body_spd->v_x,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vy = lib_math_clamp(body_spd->v_y,
                              -SENTRY_MAX_LINEAR_SPEED, SENTRY_MAX_LINEAR_SPEED);
    float vw = lib_math_clamp(body_spd->v_w,
                              -SENTRY_MAX_OMEGA, SENTRY_MAX_OMEGA);

    /* body→world 旋转 */
    float yaw_rad = lib_math_deg2rad(yaw_deg);
    float vx_w = vx * cosf(yaw_rad) - vy * sinf(yaw_rad);
    float vy_w = vx * sinf(yaw_rad) + vy * cosf(yaw_rad);

    float L = SENTRY_WHEEL_HALF_TRACK_MM;
    int i;
    for (i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;   /* 左轮+, 右轮- */
        float ix = vx_w - sign * vw * L;
        float iy = vy_w + sign * vw * L;

        float raw_speed = sqrtf(ix * ix + iy * iy);
        float raw_angle = atan2f(iy, ix);       /* rad */

        /* >90°优化: 反转方向, 减小转向行程 */
        float cur_deg = s_swerve_cur[i].angle;
        float diff = lib_math_get_shortest_path(raw_angle,
                     lib_math_deg2rad(cur_deg));
        float diff_deg = diff * (180.0f / (float)LIB_MATH_PI);

        if (fabsf(diff_deg) > 90.0f) {
            s_swerve_tar[i].rev  = -1;
            s_swerve_tar[i].angle = lib_math_rad2deg(raw_angle)
                                  + (diff_deg > 0 ? -180.0f : 180.0f);
        } else {
            s_swerve_tar[i].rev  = 1;
            s_swerve_tar[i].angle = lib_math_rad2deg(raw_angle);
        }
        s_swerve_tar[i].speed = raw_speed * SENTRY_MMPS_TO_RPM(1.0f)
                              * (float)s_swerve_tar[i].rev;
    }
}

/* ════════════════════════════════════════════════════
 * 正运动学: 每轮状态 → body速度
 * ════════════════════════════════════════════════════ */

static void forward_kinematics(void)
{
    float v[2], a[2], rev[2];
    int i;
    for (i = 0; i < 2; i++) {
        v[i]   = (float)s_motor_speed_cur[i] * SENTRY_RPM_TO_MMPS(1.0f)
               * (float)s_swerve_cur[i].rev;
        a[i]   = lib_math_deg2rad(s_swerve_cur[i].angle);
    }

    float L = SENTRY_WHEEL_HALF_TRACK_MM;

    float v0x = v[0] * cosf(a[0]), v0y = v[0] * sinf(a[0]);
    float v1x = v[1] * cosf(a[1]), v1y = v[1] * sinf(a[1]);

    s_body_cur.v_x = (v0x + v1x) * 0.5f;
    s_body_cur.v_y = (v0y + v1y) * 0.5f;
    s_body_cur.v_w = (v1x - v0x + v0y - v1y) / (4.0f * L);
}

/* ════════════════════════════════════════════════════
 * 底盘PID: 速度误差 → 力/力矩极坐标
 * ════════════════════════════════════════════════════ */

static void chassis_pid(void)
{
    float fx = lib_pid_calc(&s_pid_x, s_body_tar.v_x, s_body_cur.v_x);
    float fy = lib_pid_calc(&s_pid_y, s_body_tar.v_y, s_body_cur.v_y);
    s_force       = sqrtf(fx * fx + fy * fy);
    s_force_angle = atan2f(fy, fx);
    s_torque      = lib_pid_calc(&s_pid_w, s_body_tar.v_w, s_body_cur.v_w);
}

/* ════════════════════════════════════════════════════
 * 力分配: 极坐标力/力矩 → 每轮驱动前馈 (考虑转向角)
 * ════════════════════════════════════════════════════ */

static void force_distribute(void)
{
    float fx = s_force * cosf(s_force_angle);
    float fy = s_force * sinf(s_force_angle);
    float L  = SENTRY_WHEEL_HALF_TRACK_MM;

    int i;
    for (i = 0; i < 2; i++) {
        float sign = (i == 0) ? 1.0f : -1.0f;
        float fix = fx * 0.5f - sign * s_torque / (4.0f * L);
        float fiy = fy * 0.5f + sign * s_torque / (4.0f * L);
        float a   = lib_math_deg2rad(s_swerve_cur[i].angle);

        /* 投影到车轮前进方向 */
        s_motor_ff[i] = (int16_t)lib_math_clamp(
            (fix * cosf(a) + fiy * sinf(a))
            * SENTRY_WHEEL_RADIUS_MM / SENTRY_REDUCTION_RATIO,
            -10000, 10000);
    }
}

/* ════════════════════════════════════════════════════
 * 轮级控制: 驱动FF-PID + 转向角度PID → 电流
 * ════════════════════════════════════════════════════ */

static void wheel_control(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        /* 驱动 FF-PID: 目标RPM vs 当前RPM + 前馈扭矩 */
        float target_rpm = s_swerve_tar[i].speed;
        /* 注意: 左驱动(M0)在参考代码中取反目标, 因为安装方向相反
         *   FF_PID_Update(&FeedforwardPID_M0, -Swerve_State_tar.Speed_0, ...) */
        if (i == 0) target_rpm = -target_rpm;

        s_motor_current[i] = (int16_t)lib_pid_ff_calc(
            &s_pid_drive[i], target_rpm,
            (float)s_motor_speed_cur[i],
            (float)s_motor_ff[i], 0);

        /* 转向角度 PID */
        float angle_err = lib_math_get_shortest_path(
            lib_math_deg2rad(s_swerve_tar[i].angle),
            lib_math_deg2rad(s_swerve_cur[i].angle));
        float angle_err_deg = angle_err * (180.0f / (float)LIB_MATH_PI);

        s_motor_current[i + 2] = (int16_t)lib_pid_calc(
            &s_pid_steer[i], 0, angle_err_deg);
    }
}

/* ════════════════════════════════════════════════════
 * 功率模型: M3508多项式 (from Steering_wheel_Chasssis_test)
 *   power = k0 + k1*I + k2*ω + k3*I² + k4*ω² + k5*I*ω
 * ════════════════════════════════════════════════════ */

static float motor_power_model(float current, float speed_rpm,
                               const float *coeff)
{
    return coeff[0] + coeff[1] * current + coeff[2] * speed_rpm
         + coeff[3] * current * current
         + coeff[4] * speed_rpm * speed_rpm
         + coeff[5] * current * speed_rpm;
}

/* ════════════════════════════════════════════════════
 * 功率限制 (from POWER.c Chassis_Power_Limit_Direct)
 * ════════════════════════════════════════════════════ */

static void power_limit(void)
{
    int i;
    /* 估算当前总功率 */
    float total_power = 0;
    for (i = 0; i < 4; i++) {
        total_power += motor_power_model(
            (float)s_motor_current[i],
            (float)s_motor_speed_cur[i],
            s_power_coeff_m3508);
    }
    s_power_cur = total_power;

    /* 功率超过目标时衰减 */
    if (total_power > SENTRY_TARGET_POWER) {
        s_power_scale -= 0.001f * (total_power - SENTRY_TARGET_POWER);
    } else {
        s_power_scale += 0.001f;
    }
    s_power_scale = lib_math_clamp(s_power_scale,
                                   SENTRY_POWER_MAX_ATTEN, 1.0f);

    /* 应用缩放 */
    for (i = 0; i < 4; i++) {
        s_motor_current[i] = (int16_t)((float)s_motor_current[i]
                                       * s_power_scale);
    }
}

/* ════════════════════════════════════════════════════
 * CAN2 发送 (板内电机)
 * ════════════════════════════════════════════════════ */

static void chassis_send_can2(void)
{
    uint8_t frame[8];

    /* 0x200: 驱动电流 [R_H,R_L, L_H,L_L, 0,0,0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = SENTRY_HI(s_motor_current[M_DRIVE_R]);
    frame[1] = SENTRY_LO(s_motor_current[M_DRIVE_R]);
    frame[2] = SENTRY_HI(s_motor_current[M_DRIVE_L]);
    frame[3] = SENTRY_LO(s_motor_current[M_DRIVE_L]);
    bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_DRIVE, frame);

    /* 0x1FF: 转向+G0 [SL_H,SL_L, SR_H,SR_L, G0_H,G0_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = SENTRY_HI(s_motor_current[M_STEER_L]);
    frame[1] = SENTRY_LO(s_motor_current[M_STEER_L]);
    frame[2] = SENTRY_HI(s_motor_current[M_STEER_R]);
    frame[3] = SENTRY_LO(s_motor_current[M_STEER_R]);
    frame[4] = SENTRY_HI(s_motor_current[M_GIMBAL_G0]);
    frame[5] = SENTRY_LO(s_motor_current[M_GIMBAL_G0]);
    bsp_can_send(&hcan2, SENTRY_CAN_CHASSIS_TX_STEER, frame);
}

/* ════════════════════════════════════════════════════
 * CAN1 发送 (板间: 底盘→云台/上位机)
 *   0x112: 功率+ωz [power_x100(int16), omega_z(float)]
 * ════════════════════════════════════════════════════ */

static void chassis_send_can1(void)
{
    uint8_t frame[8];
    memset(frame, 0, sizeof(frame));

    /* 前2字节: 功率×100 (int16 LE) */
    int16_t power_x100 = (int16_t)(s_power_cur * 100.0f);
    memcpy(frame, &power_x100, sizeof(int16_t));

    /* 后4字节: ωz (float LE, VMC前馈用) */
    memcpy(frame + 4, &s_chassis_state.omega_z, sizeof(float));

    bsp_can_send(&hcan1, APP_CHASSIS_CAN_ID_POWER_FEEDBACK, frame);
}

/* ════════════════════════════════════════════════════
 * CAN1 接收: 小电脑阿克曼指令 0x113
 * ════════════════════════════════════════════════════ */

static void on_ackermann_cmd(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    if (len < 8) return;
    float speed, steer;
    memcpy(&speed, data,     sizeof(float));
    memcpy(&steer, data + 4, sizeof(float));

    /* speed [-100,100] → mm/s, steer [-PI,PI] rad → deg */
    s_body_tar.v_x = speed * (SENTRY_MAX_LINEAR_SPEED / 100.0f);
    s_body_tar.v_y = 0;
    s_body_tar.v_w = 0;   /* omega由舵轮转弯自然产生 */
}

/* ════════════════════════════════════════════════════
 * 公有接口
 * ════════════════════════════════════════════════════ */

void app_sentry_chassis_init(void)
{
    memset(s_motor,           0, sizeof(s_motor));
    memset(s_motor_current,   0, sizeof(s_motor_current));
    memset(s_motor_speed_cur, 0, sizeof(s_motor_speed_cur));
    memset(s_motor_speed_tar, 0, sizeof(s_motor_speed_tar));
    memset(s_motor_ff,        0, sizeof(s_motor_ff));
    memset(s_swerve_cur,      0, sizeof(s_swerve_cur));
    memset(s_swerve_tar,      0, sizeof(s_swerve_tar));
    memset(&s_body_tar,       0, sizeof(s_body_tar));
    memset(&s_body_cur,       0, sizeof(s_body_cur));
    memset(&s_chassis_state,  0, sizeof(s_chassis_state));
    s_power_scale = 1.0f;
    s_force = s_force_angle = s_torque = 0;

    pid_init_all();

    /* 注册 CAN2 电机反馈 (板内) */
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_R,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_DRIVE_L,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_STEER_L,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_STEER_R,
                                 app_sentry_chassis_motor_feedback);
    bsp_can_register_rx_callback(&hcan2, SENTRY_CAN_CHASSIS_GIMBAL_G0,
                                 app_sentry_chassis_motor_feedback);

    /* 注册 CAN1 板间: 小电脑阿克曼指令 0x113 */
    bsp_can_register_rx_callback(&hcan1, APP_CHASSIS_CAN_ID_ACKERMANN_CMD,
                                 on_ackermann_cmd);
}

void app_sentry_chassis_control(void)
{
    /* ── 1. 指令来源: CAN1 0x113 (已在回调中写入 s_body_tar) ── */
    /*    DBUS 手动模式 (CAN无数据时用DBUS) */
    const app_chassis_ackermann_cmd_t *can_cmd =
        app_chassis_comm_get_ackermann_cmd();
    if (can_cmd->speed == 0 && can_cmd->steer_angle == 0) {
        const drv_dbus_data_t *dbus = drv_dbus_port_get_data();
        if (dbus) {
            s_body_tar.v_x = (float)dbus->rc.ch[3]
                           * SENTRY_MAX_LINEAR_SPEED / 660.0f;
            s_body_tar.v_y = (float)dbus->rc.ch[2]
                           * SENTRY_MAX_LINEAR_SPEED / 660.0f;
            s_body_tar.v_w = (float)dbus->rc.ch[1]
                           * SENTRY_MAX_OMEGA / 660.0f;
        }
    }

    /* ── 2. 当前舵轮状态 (编码器→角度) ── */
    s_swerve_cur[0].angle = calc_logical_angle(s_motor[M_STEER_L].angle,
                                               SENTRY_SWERVE_0_OFFSET);
    s_swerve_cur[1].angle = calc_logical_angle(s_motor[M_STEER_R].angle,
                                               SENTRY_SWERVE_1_OFFSET);
    s_swerve_cur[0].speed = (float)s_motor_speed_cur[M_DRIVE_L];
    s_swerve_cur[1].speed = (float)s_motor_speed_cur[M_DRIVE_R];
    s_swerve_cur[0].rev = s_swerve_cur[1].rev = 1;   /* 正运动学用 */

    /* G0 yaw 角度 */
    s_chassis_state.yaw_deg = SENTRY_ENC_TO_DEG(s_motor[M_GIMBAL_G0].angle);

    /* ── 3. 逆运动学: body目标 → 每轮角度+RPM ── */
    inverse_kinematics(&s_body_tar, s_chassis_state.yaw_deg);

    /* ── 4. 正运动学: 每轮状态 → body估算速度 + ωz ── */
    forward_kinematics();
    s_chassis_state.omega_z = s_body_cur.v_w;

    /* ── 5. 底盘PID: 速度误差 → 力/力矩 ── */
    chassis_pid();

    /* ── 6. 力分配 → 驱动前馈 ── */
    force_distribute();

    /* ── 7. 轮级控制: 驱动FF-PID + 转向角度PID ── */
    wheel_control();

    /* ── 8. 功率限制 ── */
    power_limit();

    /* ── 9. CAN2 发送 ── */
    chassis_send_can2();

    /* ── 10. CAN1 发送 (功率+ωz给云台VMC前馈) ── */
    chassis_send_can1();

    /* ── 11. 更新全局状态 ── */
    s_chassis_state.speed    = s_body_cur;
    s_chassis_state.power_w  = s_power_cur;
}

void app_sentry_chassis_motor_feedback(uint32_t std_id, uint8_t *data,
                                       uint8_t len)
{
    (void)len;
    int idx = -1;
    switch (std_id) {
    case SENTRY_CAN_CHASSIS_DRIVE_R:   idx = M_DRIVE_R;   break;
    case SENTRY_CAN_CHASSIS_DRIVE_L:   idx = M_DRIVE_L;   break;
    case SENTRY_CAN_CHASSIS_STEER_L:   idx = M_STEER_L;   break;
    case SENTRY_CAN_CHASSIS_STEER_R:   idx = M_STEER_R;   break;
    case SENTRY_CAN_CHASSIS_GIMBAL_G0: idx = M_GIMBAL_G0; break;
    default: return;
    }
    if (idx >= 0 && idx < MOTOR_COUNT) {
        drv_motor_solve_dji_data(data, &s_motor[idx]);
        s_motor_speed_cur[idx] = s_motor[idx].speed;
    }
}

const app_sentry_chassis_state_t *app_sentry_chassis_get_state(void)
{
    return &s_chassis_state;
}
