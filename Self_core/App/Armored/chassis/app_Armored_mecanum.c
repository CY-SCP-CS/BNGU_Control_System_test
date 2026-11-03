//
// Created by 33140 on 2026/10/26.
//

#include "App/Armored/Armored_cfg.h"
#include "app_Armored_mecanum.h"
#include "drv_motor.h"
#include "bsp_can.h"
#include "Lib/Lib_math/lib_math.h"

#include <math.h>
#include <string.h>

M3508_wheel_drive MOTOR_DRIVE_FL = {   //前左
    .feedback_id = MOTOR_DRIVE_FL_FEEDBACKID,
};

M3508_wheel_drive MOTOR_DRIVE_RL = {   //后左
    .feedback_id = MOTOR_DRIVE_RL_FEEDBACKID,
};

M3508_wheel_drive MOTOR_DRIVE_FR = {   //前右
    .feedback_id = MOTOR_DRIVE_FR_FEEDBACKID,
};

M3508_wheel_drive MOTOR_DRIVE_RR = {   //后右
    .feedback_id = MOTOR_DRIVE_RR_FEEDBACKID,
};

static uint8_t  s_mecanum_initialized = 0U;

static void mecnum_feedback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (!data || len != 8U) {return;}

    drv_motor_data_t * cur;
    switch (std_id) {
    case MOTOR_DRIVE_FL_FEEDBACKID:
        cur = &MOTOR_DRIVE_FL.motor_data; break;
    case MOTOR_DRIVE_RL_FEEDBACKID:
        cur = &MOTOR_DRIVE_RL.motor_data; break;
    case MOTOR_DRIVE_FR_FEEDBACKID:
        cur = &MOTOR_DRIVE_FR.motor_data; break;
    case MOTOR_DRIVE_RR_FEEDBACKID:
        cur = &MOTOR_DRIVE_RR.motor_data; break;
    default:
        cur = NULL; break;

    }
    if (cur == NULL) return;
    drv_motor_solve_dji(data, cur);
}

static void mecanum_pid_init(void){

    lib_pid_init(&MOTOR_DRIVE_FL.pid,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 0.0f);
    lib_pid_init(&MOTOR_DRIVE_RL.pid,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 0.0f);
    lib_pid_init(&MOTOR_DRIVE_FR.pid,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 0.0f);
    lib_pid_init(&MOTOR_DRIVE_RR.pid,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
                    DRV_MOTOR_M3508_CURRENT_MIN ,DRV_MOTOR_M3508_CURRENT_MAX , 0.0f);
}

static void mecanum_can_init(void)
{
    bsp_can_rx_reg(&hcan2,MOTOR_DRIVE_FL_FEEDBACKID,mecnum_feedback);
    bsp_can_rx_reg(&hcan2,MOTOR_DRIVE_RL_FEEDBACKID,mecnum_feedback);
    bsp_can_rx_reg(&hcan2,MOTOR_DRIVE_FR_FEEDBACKID,mecnum_feedback);
    bsp_can_rx_reg(&hcan2,MOTOR_DRIVE_RR_FEEDBACKID,mecnum_feedback);
}

void mecnum_init(void)
{
    s_mecanum_initialized = 0U;

    mecanum_pid_init();
    mecanum_can_init();

    s_mecanum_initialized = 1U;
}

static void mecnum_pid_ctrl(float pitch)
{
    if (!isfinite(MOTOR_DRIVE_FL.tar_omega_speed) || !isfinite(MOTOR_DRIVE_RL.tar_omega_speed)||
        !isfinite(MOTOR_DRIVE_FR.tar_omega_speed) || !isfinite(MOTOR_DRIVE_RR.tar_omega_speed)|| !isfinite(pitch)) {
        lib_pid_reset(&MOTOR_DRIVE_FL.pid); MOTOR_DRIVE_FL.send_current = 0;
        lib_pid_reset(&MOTOR_DRIVE_RL.pid); MOTOR_DRIVE_RL.send_current = 0;
        lib_pid_reset(&MOTOR_DRIVE_FR.pid); MOTOR_DRIVE_FR.send_current = 0;
        lib_pid_reset(&MOTOR_DRIVE_RR.pid); MOTOR_DRIVE_RR.send_current = 0;
        return;}

    float scale_factor = 1 , max_abs; //缩放比例 , 最大速度
    if ((max_abs = lib_max_abs4(MOTOR_DRIVE_FL.tar_omega_speed,MOTOR_DRIVE_RL.tar_omega_speed,
        MOTOR_DRIVE_FR.tar_omega_speed,MOTOR_DRIVE_RR.tar_omega_speed))>DRIVE_MAX_OMEGA_RAD_S) {
        scale_factor = DRIVE_MAX_OMEGA_RAD_S / max_abs;

        MOTOR_DRIVE_FL.tar_omega_speed = scale_factor * MOTOR_DRIVE_FL.tar_omega_speed;
        MOTOR_DRIVE_RL.tar_omega_speed = scale_factor * MOTOR_DRIVE_RL.tar_omega_speed;
        MOTOR_DRIVE_FR.tar_omega_speed = scale_factor * MOTOR_DRIVE_FR.tar_omega_speed;
        MOTOR_DRIVE_RR.tar_omega_speed = scale_factor * MOTOR_DRIVE_RR.tar_omega_speed;
    }

    float ff_g_comp_Force = sinf(lib_deg_to_rad(pitch)); //和真实的力差常数倍
    float ff_g_comp_torque = ff_g_comp_Force * MOTOR_DRIVE_WHEEL_RADIUS*0.001f;  //和真实力矩差常数倍

    MOTOR_DRIVE_FL.send_current = (int16_t)lib_pid_ff_calc(&MOTOR_DRIVE_FL.pid,MOTOR_DRIVE_FL.tar_omega_speed,
        lib_rpm_to_rad_s(MOTOR_DRIVE_FL.motor_data.speed) , ff_g_comp_torque , 0 , 0.004);
    MOTOR_DRIVE_RL.send_current = (int16_t)lib_pid_ff_calc(&MOTOR_DRIVE_RL.pid,MOTOR_DRIVE_RL.tar_omega_speed,
        lib_rpm_to_rad_s(MOTOR_DRIVE_RL.motor_data.speed) , ff_g_comp_torque , 0 , 0.004);
    MOTOR_DRIVE_FR.send_current = (int16_t)lib_pid_ff_calc(&MOTOR_DRIVE_FR.pid,MOTOR_DRIVE_FR.tar_omega_speed,
        lib_rpm_to_rad_s(MOTOR_DRIVE_FR.motor_data.speed) , -ff_g_comp_torque , 0 , 0.004);
    MOTOR_DRIVE_RR.send_current = (int16_t)lib_pid_ff_calc(&MOTOR_DRIVE_RR.pid,MOTOR_DRIVE_RR.tar_omega_speed,
        lib_rpm_to_rad_s(MOTOR_DRIVE_RR.motor_data.speed) , -ff_g_comp_torque , 0 , 0.004);
}

static uint8_t mecnum_current_send(void)
{
    uint8_t frame[8] = {0};

    if (s_mecanum_initialized){
        drv_motor_set_dji(frame , 0 , MOTOR_DRIVE_FL.send_current);
        drv_motor_set_dji(frame , 1 , MOTOR_DRIVE_RL.send_current);
        drv_motor_set_dji(frame , 2 , MOTOR_DRIVE_FR.send_current);
        drv_motor_set_dji(frame , 3 , MOTOR_DRIVE_RR.send_current);
    }

    return bsp_can_tx(&hcan2 , M3508_SEND_ID , frame);
}

/**
 * @brief 将车体目标 x 和车体目标 y 和大地目标角速度 wz 逆解算成轮速电机目标角速度
 * @param vx 车体 x 方向目标速度（向前为正 m/s）
 * @param vy 车体 y 方向目标速度（向左为正 m/s）
 * @param wz 大地角速度 wz （从上往下看逆时针为正 rad/s）
 */
static void mecanum_inverse_kinematics(float vx , float vy , float wz)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) {
        MOTOR_DRIVE_FL.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_RL.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_FR.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_RR.tar_omega_speed = 0.0f;
        return;
    }

    float wheel_lin_v1, wheel_lin_v2 ,wheel_omega_speed1 , wheel_omega_speed2 ,
            wz_lin_v1 , wz_lin_v2 , wz_omega_speed1 , wz_omega_speed2;

    wheel_lin_v1 = vx - vy;
    wheel_lin_v2 = vx + vy;

    wz_lin_v1 = -(CHASSIS_ROTATION_COEFF_MM)* wz;
    wz_lin_v2 = (CHASSIS_ROTATION_COEFF_MM)* wz;

    wheel_omega_speed1 = wheel_lin_v1 / MOTOR_DRIVE_WHEEL_RADIUS * 1000 * DRV_MOTOR_M3508_REDUCTION ;
    wheel_omega_speed2 = wheel_lin_v2 / MOTOR_DRIVE_WHEEL_RADIUS * 1000 * DRV_MOTOR_M3508_REDUCTION ;

    wz_omega_speed1 = wz_lin_v1 / MOTOR_DRIVE_WHEEL_RADIUS * DRV_MOTOR_M3508_REDUCTION ;
    wz_omega_speed2 = wz_lin_v2 / MOTOR_DRIVE_WHEEL_RADIUS * DRV_MOTOR_M3508_REDUCTION ;

    float omega_FL = wheel_omega_speed1 + wz_omega_speed1;
    float omega_RL = wheel_omega_speed2 + wz_omega_speed1;
    float omega_FR = - (wheel_omega_speed2 + wz_omega_speed2);
    float omega_RR = - (wheel_omega_speed1 + wz_omega_speed2);

    // 检查平移与旋转叠加后的最终结果，任一路异常时四路目标全部清零。
    if (!isfinite(omega_FL) || !isfinite(omega_RL)|| !isfinite(omega_FR) || !isfinite(omega_RR)) {
        omega_FL = 0.0f;
        omega_RL = 0.0f;
        omega_FR = 0.0f;
        omega_RR = 0.0f;
    }

    MOTOR_DRIVE_FL.tar_omega_speed = omega_FL;
    MOTOR_DRIVE_RL.tar_omega_speed = omega_RL;
    MOTOR_DRIVE_FR.tar_omega_speed = omega_FR;
    MOTOR_DRIVE_RR.tar_omega_speed = omega_RR;

}

/** 世界坐标系转车体坐标系  yaw是度 其他单位与上述相同 */
static void mecanum_world_to_body_velocity(float vx_world,float vy_world,float yaw,
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

uint8_t mecanum_ctrl(float vx_world, float vy_world, float wz,float yaw, float pitch)
{
    if (!isfinite(vx_world) || !isfinite(vy_world) || !isfinite(wz)
        || !isfinite(yaw) || !isfinite(pitch)) {
        lib_pid_reset(&MOTOR_DRIVE_FL.pid);
        lib_pid_reset(&MOTOR_DRIVE_RL.pid);
        lib_pid_reset(&MOTOR_DRIVE_FR.pid);
        lib_pid_reset(&MOTOR_DRIVE_RR.pid);

        MOTOR_DRIVE_FL.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_RL.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_FR.tar_omega_speed = 0.0f;
        MOTOR_DRIVE_RR.tar_omega_speed = 0.0f;

        MOTOR_DRIVE_FL.send_current = 0;
        MOTOR_DRIVE_RL.send_current = 0;
        MOTOR_DRIVE_FR.send_current = 0;
        MOTOR_DRIVE_RR.send_current = 0;
        return mecnum_current_send();
    }

    float vx_car = 0 , vy_car = 0;
    mecanum_world_to_body_velocity(vx_world , vy_world , yaw , &vx_car , & vy_car);
    mecanum_inverse_kinematics(vx_car , vy_car , wz);
    mecnum_pid_ctrl(pitch);
    return mecnum_current_send();

}

uint8_t mecanum_stop(void)
{
    MOTOR_DRIVE_FL.tar_omega_speed = 0.0f;
    MOTOR_DRIVE_RL.tar_omega_speed = 0.0f;
    MOTOR_DRIVE_FR.tar_omega_speed = 0.0f;
    MOTOR_DRIVE_RR.tar_omega_speed = 0.0f;

    MOTOR_DRIVE_FL.send_current = 0;
    MOTOR_DRIVE_RL.send_current = 0;
    MOTOR_DRIVE_FR.send_current = 0;
    MOTOR_DRIVE_RR.send_current = 0;

    lib_pid_reset(&MOTOR_DRIVE_FL.pid);
    lib_pid_reset(&MOTOR_DRIVE_RL.pid);
    lib_pid_reset(&MOTOR_DRIVE_FR.pid);
    lib_pid_reset(&MOTOR_DRIVE_RR.pid);

    if (!s_mecanum_initialized) {
        return 1U;
    }

    return mecnum_current_send();
}
