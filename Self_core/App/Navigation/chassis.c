//
// Created by 33140 on 2026/11/3.
//

#include "chassis.h"
#include "Navigation_cfg.h"
#include "bsp_can.h"
#include "Lib/Lib_math/lib_math.h"

#include <math.h>

NAV_M3508_wheel_drive NAV_MOTOR_DRIVE_FL = {   // 前左，4 号电机
    .feedback_id = NAV_MOTOR_DRIVE_FL_FEEDBACKID,
};

NAV_M3508_wheel_drive NAV_MOTOR_DRIVE_RL = {   // 后左，2 号电机
    .feedback_id = NAV_MOTOR_DRIVE_RL_FEEDBACKID,
};

NAV_M3508_wheel_drive NAV_MOTOR_DRIVE_FR = {   // 前右，1 号电机
    .feedback_id = NAV_MOTOR_DRIVE_FR_FEEDBACKID,
};

NAV_M3508_wheel_drive NAV_MOTOR_DRIVE_RR = {   // 后右，3 号电机
    .feedback_id = NAV_MOTOR_DRIVE_RR_FEEDBACKID,
};

static uint8_t s_chassis_initialized = 0U;

static void chassis_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (!data || len != 8U) {return;}

    drv_motor_data_t *cur;
    switch (std_id) {
    case NAV_MOTOR_DRIVE_FL_FEEDBACKID:
        cur = &NAV_MOTOR_DRIVE_FL.motor_data; break;
    case NAV_MOTOR_DRIVE_RL_FEEDBACKID:
        cur = &NAV_MOTOR_DRIVE_RL.motor_data; break;
    case NAV_MOTOR_DRIVE_FR_FEEDBACKID:
        cur = &NAV_MOTOR_DRIVE_FR.motor_data; break;
    case NAV_MOTOR_DRIVE_RR_FEEDBACKID:
        cur = &NAV_MOTOR_DRIVE_RR.motor_data; break;
    default:
        cur = NULL; break;
    }

    if (cur == NULL) return;
    drv_motor_solve_dji(data, cur);
}

static void chassis_pid_init(void)
{
    // 旧工程 RPM 参数换算为 rad/s 参数，积分输出限幅为 0.6 × 360 = 216。
    lib_pid_init(&NAV_MOTOR_DRIVE_FL.pid,52.52113f,5.729578f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 216.0f);
    lib_pid_init(&NAV_MOTOR_DRIVE_RL.pid,52.52113f,5.729578f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 216.0f);
    lib_pid_init(&NAV_MOTOR_DRIVE_FR.pid,52.52113f,5.729578f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 216.0f);
    lib_pid_init(&NAV_MOTOR_DRIVE_RR.pid,52.52113f,5.729578f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 216.0f);
}

static void chassis_can_init(void)
{
    bsp_can_rx_reg(&hcan2,NAV_MOTOR_DRIVE_FL_FEEDBACKID,chassis_feedback);
    bsp_can_rx_reg(&hcan2,NAV_MOTOR_DRIVE_RL_FEEDBACKID,chassis_feedback);
    bsp_can_rx_reg(&hcan2,NAV_MOTOR_DRIVE_FR_FEEDBACKID,chassis_feedback);
    bsp_can_rx_reg(&hcan2,NAV_MOTOR_DRIVE_RR_FEEDBACKID,chassis_feedback);
}

void chassis_init(void)
{
    s_chassis_initialized = 0U;

    chassis_pid_init();
    chassis_can_init();

    s_chassis_initialized = 1U;
}

/** 四轮速度环：目标与反馈均为电机转子角速度 rad/s，每 5 ms 调用一次。 */
static void chassis_pid_ctrl(void)
{
    if (!isfinite(NAV_MOTOR_DRIVE_FL.tar_omega_speed) || !isfinite(NAV_MOTOR_DRIVE_RL.tar_omega_speed) ||
        !isfinite(NAV_MOTOR_DRIVE_FR.tar_omega_speed) || !isfinite(NAV_MOTOR_DRIVE_RR.tar_omega_speed)) {
        lib_pid_reset(&NAV_MOTOR_DRIVE_FL.pid); NAV_MOTOR_DRIVE_FL.send_current = 0;
        lib_pid_reset(&NAV_MOTOR_DRIVE_RL.pid); NAV_MOTOR_DRIVE_RL.send_current = 0;
        lib_pid_reset(&NAV_MOTOR_DRIVE_FR.pid); NAV_MOTOR_DRIVE_FR.send_current = 0;
        lib_pid_reset(&NAV_MOTOR_DRIVE_RR.pid); NAV_MOTOR_DRIVE_RR.send_current = 0;
        return;
    }

    float max_abs = lib_max_abs4(NAV_MOTOR_DRIVE_FL.tar_omega_speed,NAV_MOTOR_DRIVE_RL.tar_omega_speed,
                                NAV_MOTOR_DRIVE_FR.tar_omega_speed,NAV_MOTOR_DRIVE_RR.tar_omega_speed);
    if (max_abs > NAV_DRIVE_MAX_OMEGA_RAD_S) {
        float scale_factor = NAV_DRIVE_MAX_OMEGA_RAD_S / max_abs;

        NAV_MOTOR_DRIVE_FL.tar_omega_speed *= scale_factor;
        NAV_MOTOR_DRIVE_RL.tar_omega_speed *= scale_factor;
        NAV_MOTOR_DRIVE_FR.tar_omega_speed *= scale_factor;
        NAV_MOTOR_DRIVE_RR.tar_omega_speed *= scale_factor;
    }

    NAV_MOTOR_DRIVE_FL.send_current = (int16_t)lib_pid_calc(&NAV_MOTOR_DRIVE_FL.pid,NAV_MOTOR_DRIVE_FL.tar_omega_speed,
        lib_rpm_to_rad_s(NAV_MOTOR_DRIVE_FL.motor_data.speed),NAV_CHASSIS_CTRL_DT_S);
    NAV_MOTOR_DRIVE_RL.send_current = (int16_t)lib_pid_calc(&NAV_MOTOR_DRIVE_RL.pid,NAV_MOTOR_DRIVE_RL.tar_omega_speed,
        lib_rpm_to_rad_s(NAV_MOTOR_DRIVE_RL.motor_data.speed),NAV_CHASSIS_CTRL_DT_S);
    NAV_MOTOR_DRIVE_FR.send_current = (int16_t)lib_pid_calc(&NAV_MOTOR_DRIVE_FR.pid,NAV_MOTOR_DRIVE_FR.tar_omega_speed,
        lib_rpm_to_rad_s(NAV_MOTOR_DRIVE_FR.motor_data.speed),NAV_CHASSIS_CTRL_DT_S);
    NAV_MOTOR_DRIVE_RR.send_current = (int16_t)lib_pid_calc(&NAV_MOTOR_DRIVE_RR.pid,NAV_MOTOR_DRIVE_RR.tar_omega_speed,
        lib_rpm_to_rad_s(NAV_MOTOR_DRIVE_RR.motor_data.speed),NAV_CHASSIS_CTRL_DT_S);
}

static uint8_t chassis_current_send(void)
{
    if (!s_chassis_initialized) {return 1U;}

    uint8_t frame[8] = {0};

    // 0x200 的四个槽位依次对应电调 ID 1、2、3、4。
    drv_motor_set_dji(frame , 0 , NAV_MOTOR_DRIVE_FR.send_current);
    drv_motor_set_dji(frame , 1 , NAV_MOTOR_DRIVE_RL.send_current);
    drv_motor_set_dji(frame , 2 , NAV_MOTOR_DRIVE_RR.send_current);
    drv_motor_set_dji(frame , 3 , NAV_MOTOR_DRIVE_FL.send_current);

    return bsp_can_tx(&hcan2 , NAV_M3508_SEND_ID , frame);
}

/**
 * @brief 将车体目标速度逆解算成四个 3508 电机的目标转子角速度，单位 rad/s。
 * @param vx 车体 x 方向目标速度（向前为正，m/s）
 * @param vy 车体 y 方向目标速度（向左为正，m/s）
 * @param wz 底盘绕中心的目标角速度（俯视逆时针为正，rad/s）
 * @note 普通全向轮驱动方向按 45° 安装；左前/右后朝前右，左后/右前朝前左。
 */
static void chassis_inverse_kinematics(float vx , float vy , float wz)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        !isfinite(NAV_MOTOR_DRIVE_WHEEL_RADIUS) || NAV_MOTOR_DRIVE_WHEEL_RADIUS <= 0.0f ||
        !isfinite(NAV_CHASSIS_ROTATION_COEFF_MM) || NAV_CHASSIS_ROTATION_COEFF_MM <= 0.0f) {
        NAV_MOTOR_DRIVE_FL.tar_omega_speed = 0.0f;
        NAV_MOTOR_DRIVE_RL.tar_omega_speed = 0.0f;
        NAV_MOTOR_DRIVE_FR.tar_omega_speed = 0.0f;
        NAV_MOTOR_DRIVE_RR.tar_omega_speed = 0.0f;
        return;
    }

    float wheel_lin_v1, wheel_lin_v2 ,wheel_omega_speed1 , wheel_omega_speed2 ,
            wz_lin_v1 , wz_lin_v2 , wz_omega_speed1 , wz_omega_speed2;

    // 普通全向轮：将平移速度投影到 ±45° 驱动方向，单位 m/s。
    wheel_lin_v1 = (vx - vy) * 0.70710678118f;
    wheel_lin_v2 = (vx + vy) * 0.70710678118f;

    // 自旋在驱动方向上的投影：±(半轴距 + 半轮距) / sqrt(2) * wz，单位 mm/s。
    wz_lin_v1 = -NAV_CHASSIS_ROTATION_COEFF_MM * 0.70710678118f * wz;
    wz_lin_v2 =  NAV_CHASSIS_ROTATION_COEFF_MM * 0.70710678118f * wz;

    // 轮缘线速度 / 轮子半径 * 减速比 = 电机转子角速度，单位 rad/s。
    wheel_omega_speed1 = wheel_lin_v1 / NAV_MOTOR_DRIVE_WHEEL_RADIUS * 1000.0f * DRV_MOTOR_M3508_REDUCTION;
    wheel_omega_speed2 = wheel_lin_v2 / NAV_MOTOR_DRIVE_WHEEL_RADIUS * 1000.0f * DRV_MOTOR_M3508_REDUCTION;

    wz_omega_speed1 = wz_lin_v1 / NAV_MOTOR_DRIVE_WHEEL_RADIUS * DRV_MOTOR_M3508_REDUCTION;
    wz_omega_speed2 = wz_lin_v2 / NAV_MOTOR_DRIVE_WHEEL_RADIUS * DRV_MOTOR_M3508_REDUCTION;

    float omega_FL = wheel_omega_speed1 + wz_omega_speed1;
    float omega_RL = wheel_omega_speed2 + wz_omega_speed1;
    float omega_FR = -(wheel_omega_speed2 + wz_omega_speed2);
    float omega_RR = -(wheel_omega_speed1 + wz_omega_speed2);

    // 检查平移与旋转叠加后的最终结果，任一路异常时四路目标全部清零。
    if (!isfinite(omega_FL) || !isfinite(omega_RL) || !isfinite(omega_FR) || !isfinite(omega_RR)) {
        omega_FL = 0.0f;
        omega_RL = 0.0f;
        omega_FR = 0.0f;
        omega_RR = 0.0f;
    }

    NAV_MOTOR_DRIVE_FL.tar_omega_speed = omega_FL;
    NAV_MOTOR_DRIVE_RL.tar_omega_speed = omega_RL;
    NAV_MOTOR_DRIVE_FR.tar_omega_speed = omega_FR;
    NAV_MOTOR_DRIVE_RR.tar_omega_speed = omega_RR;
}

/**
 * @brief 将世界坐标系目标线速度转换为车体坐标系目标线速度，单位 m/s。
 * @param vx_world 世界坐标系 x 方向目标速度
 * @param vy_world 世界坐标系 y 方向目标速度（+x 逆时针转 90° 为 +y）
 * @param yaw 底盘偏航角，单位度；车头沿世界 +x 为 0°，俯视逆时针为正
 * @param vx_body 输出车体 x 方向目标速度，向前为正
 * @param vy_body 输出车体 y 方向目标速度，向左为正
 */
static void chassis_world_to_body_velocity(float vx_world,float vy_world,float yaw,
                                            float *vx_body,float *vy_body)
{
    if (!vx_body || !vy_body) {return;}

    *vx_body = 0.0f;
    *vy_body = 0.0f;

    if (!isfinite(vx_world) || !isfinite(vy_world) || !isfinite(yaw)) {return;}

    float c = cosf(lib_deg_to_rad(yaw)); float s = sinf(lib_deg_to_rad(yaw));
    float vx =  vx_world * c + vy_world * s;
    float vy = -vx_world * s + vy_world * c;

    // 防止有限输入在坐标转换中溢出。
    if (!isfinite(vx) || !isfinite(vy)) {return;}

    *vx_body = vx;
    *vy_body = vy;
}

uint8_t chassis_ctrl_world(float vx_world, float vy_world, float wz, float yaw)
{
    if (!s_chassis_initialized) {return 1U;}

    if (!isfinite(vx_world) || !isfinite(vy_world) || !isfinite(wz) || !isfinite(yaw)) {
        return chassis_stop();
    }

    float vx_car = 0.0f , vy_car = 0.0f;
    chassis_world_to_body_velocity(vx_world , vy_world , yaw , &vx_car , &vy_car);
    chassis_inverse_kinematics(vx_car , vy_car , wz);
    chassis_pid_ctrl();
    return chassis_current_send();
}

uint8_t chassis_ctrl_body(float vx_body, float vy_body, float wz)
{
    if (!s_chassis_initialized) {return 1U;}

    if (!isfinite(vx_body) || !isfinite(vy_body) || !isfinite(wz)) {
        return chassis_stop();
    }

    chassis_inverse_kinematics(vx_body , vy_body , wz);
    chassis_pid_ctrl();
    return chassis_current_send();
}

uint8_t chassis_stop(void)
{
    NAV_MOTOR_DRIVE_FL.tar_omega_speed = 0.0f;
    NAV_MOTOR_DRIVE_RL.tar_omega_speed = 0.0f;
    NAV_MOTOR_DRIVE_FR.tar_omega_speed = 0.0f;
    NAV_MOTOR_DRIVE_RR.tar_omega_speed = 0.0f;

    NAV_MOTOR_DRIVE_FL.send_current = 0;
    NAV_MOTOR_DRIVE_RL.send_current = 0;
    NAV_MOTOR_DRIVE_FR.send_current = 0;
    NAV_MOTOR_DRIVE_RR.send_current = 0;

    lib_pid_reset(&NAV_MOTOR_DRIVE_FL.pid);
    lib_pid_reset(&NAV_MOTOR_DRIVE_RL.pid);
    lib_pid_reset(&NAV_MOTOR_DRIVE_FR.pid);
    lib_pid_reset(&NAV_MOTOR_DRIVE_RR.pid);

    return chassis_current_send();
}
