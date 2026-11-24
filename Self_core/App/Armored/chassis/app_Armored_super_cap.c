//
// Created by 33140 on 2026/11/23.
//

#include "app_Armored_super_cap.h"
#include "Drv/Drv_power_measure/drv_power_measure.h"
#include "Drv/Drv_super_cap/drv_super_cap.h"
#include "Drv/Drv_referee/drv_referee.h"
#include "Bsp/bsp_tim.h"
#include "can.h"
#include <math.h>

#define ARMORED_REFEREE_POWER_LIMIT_W \
    ((float)drv_referee_get_data()->robot_status.chassis_power_limit)

/* 单轮功率增量 P_i = a_i * max(0, I_i * omega_i) + b_i * I_i^2。
 * 四轮总输入功率 P_total = P_idle + sum(P_i)。
 * I 为带方向的转矩电流，单位 A。omega 为同一正方向下的电机转子角速度，单位 rad/s。
 * 功率单位为 W，系数待实测标定。 */
const armored_wheel_power_coeffs_t armored_wheel_power_coeffs[4] = {
    {.a = 0.0f, .b = 0.0f}, // FL
    {.a = 0.0f, .b = 0.0f}, // RL
    {.a = 0.0f, .b = 0.0f}, // FR
    {.a = 0.0f, .b = 0.0f}, // RR
};
const float armored_wheel_power_idle_w = 0.0f; // 四个电调合计的静态输入功率，W



float app_armored_wheel_power_estimate(const float current_a[4],
                                       const float rotor_speed_rad_s[4])
{
    if (!current_a || !rotor_speed_rad_s) {
        return NAN;
    }

    float total_power_w = armored_wheel_power_idle_w;
    for (unsigned int i = 0; i < 4U; ++i) {
        const float current = current_a[i];
        const float omega = rotor_speed_rad_s[i];
        if (!isfinite(current) || !isfinite(omega)) {
            return NAN;
        }

        const armored_wheel_power_coeffs_t *coeffs = &armored_wheel_power_coeffs[i];
        const float drive_term = coeffs->a * fmaxf(0.0f, current * omega);
        const float current_loss = coeffs->b * (current * current);
        total_power_w += drive_term + current_loss;
    }
    return total_power_w;
}

drv_power_data_t s_power_meter1; //暂时放裁判系统端
xtqz_powermeter_data_t s_power_meter2;  //暂时放合流端
drv_super_cap_t s_super_cap; //超电返回的信息
drv_super_cap_control_t s_super_cap_control; //控制超电的结构体

static uint8_t s_super_cap_feedback_count_after_request = 0U;//超电使能请求后计数
static uint8_t s_super_cap_enable_request_submitted = 0U; // 本轮超电开启请求已有一帧提交给 CAN 发送。

drv_referee_robot_status_t reference_status; //裁判系统状态

static void armored_power_meter_can_rx(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if ((std_id != DRV_POWER_CAN_ID && std_id != DRV_POWER_CAN_ID_XTQZ) || !data || len != 8U) {
        return;
    }

    if (std_id == DRV_POWER_CAN_ID) {
        drv_power_data_t decoded;
        drv_power_solve(data, &decoded);
        s_power_meter1 = decoded;
    }
    else if (std_id == DRV_POWER_CAN_ID_XTQZ) {
        xtqz_powermeter_data_t decoded;
        drv_xtqz_power_solve(data, &decoded);
        s_power_meter2 = decoded;
    }
}

static void armored_super_cap_can_rx(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (std_id != DRV_SUPER_CAP_CAN_ID_FEEDBACK || !data || len != 8U) {return;}
    (void)drv_super_cap_solve(&s_super_cap,std_id,data,len, bsp_tim_get_tick_ms());
    if (s_super_cap_control.enabled == 1 && s_super_cap_enable_request_submitted
                                && s_super_cap_feedback_count_after_request < 3U) {
        s_super_cap_feedback_count_after_request++;
    }

}


void app_armored_super_cap_init(void)
{
    drv_super_cap_init(&s_super_cap);         // 清空超电反馈、接收时间和在线标志
    s_super_cap_control = (drv_super_cap_control_t){0};
    s_power_meter1 = (drv_power_data_t){0};  // 清空功率计数据

    drv_super_cap_port_can_init(&hcan2, armored_super_cap_can_rx);      // 0x211
    drv_power_port_can_init(&hcan2, DRV_POWER_CAN_ID, armored_power_meter_can_rx);
    drv_power_port_can_init(&hcan2, DRV_POWER_CAN_ID_XTQZ, armored_power_meter_can_rx);

}

static uint8_t armored_referee_status_is_fresh(drv_referee_robot_status_t *status)
{
    static uint32_t last_update_count;
    static uint32_t last_rx_ms;
    static uint8_t has_status;

    // 在同一次快照中读取状态和更新计数。
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const drv_referee_global_t *referee = drv_referee_get_data();
    *status = referee->robot_status;
    const uint32_t update_count = referee->robot_status_update_count;
    __set_PRIMASK(irq_state);

    const uint32_t now_ms = bsp_tim_get_tick_ms();
    if (update_count != last_update_count) {
        last_update_count = update_count;
        last_rx_ms = now_ms;
        has_status = 1U;
    }

    return has_status && (uint32_t)(now_ms - last_rx_ms) < 1000U;
}

float armored_wheel_power_limit_step(const float current_a[4],
    const float rotor_speed_rad_s[4],float margin_w)
{

    const uint8_t referee_online =armored_referee_status_is_fresh(&reference_status);
    const float referee_limit_w = referee_online?
    ((float)reference_status.chassis_power_limit- margin_w): 50.0f;

    float drive_w = 0.0f;        // A
    float current_loss_w = 0.0f; // B
    for (unsigned int i = 0; i < 4U; ++i) {
        const float current = current_a[i];
        const float omega = rotor_speed_rad_s[i];
        const armored_wheel_power_coeffs_t *coeff = &armored_wheel_power_coeffs[i];

        drive_w += coeff->a * fmaxf(0.0f, current * omega);
        current_loss_w += coeff->b * (current * current);
    }
    const float requested_w = armored_wheel_power_idle_w + drive_w + current_loss_w;


    if (referee_online && !reference_status.power_management_chassis_output) {return 0.0f;}

    float limit_w = referee_limit_w;  // 裁判在线用实际限额，离线用 50 W

    s_super_cap_control.power_limit = limit_w;
    if (requested_w < limit_w) {
        s_super_cap_control.enabled = 0;
        s_super_cap_feedback_count_after_request = 0;
    }
    else{s_super_cap_control.enabled = 1;}

    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint8_t super_cap_online = drv_super_cap_is_online(&s_super_cap, bsp_tim_get_tick_ms(), 300U);
    if (!super_cap_online) {s_super_cap_feedback_count_after_request = 0U;}
    __set_PRIMASK(irq_state);

    if (referee_limit_w > 0.0f && s_super_cap_control.enabled && s_super_cap.has_feedback &&
        super_cap_online && s_super_cap_feedback_count_after_request >= 3U) {
        limit_w = fmaxf(0.0f, s_super_cap.feedback.max_output_power - margin_w);}

    if (requested_w <= limit_w) {return 1.0f;}  // 四轮保持原来的电流指令

    const float d = limit_w - armored_wheel_power_idle_w;
    if (d <= 0.0f) {return 0.0f;}  // 四轮命令电流归零

    return (2.0f * d) /(drive_w + sqrtf(drive_w * drive_w + 4.0f * current_loss_w * d));

}

void app_armored_super_cap_tx_step(void)
{
    static uint32_t last_tx_ms;
    static uint8_t has_sent;

    const uint32_t now_ms = bsp_tim_get_tick_ms();

    if (s_super_cap_control.enabled == 0U) {
        s_super_cap_enable_request_submitted = 0U;
        s_super_cap_feedback_count_after_request = 0U;
    }

    if (has_sent && (uint32_t)(now_ms - last_tx_ms) < 100U) {return;}

    if (!drv_super_cap_port_can_tx(&hcan2, &s_super_cap_control)) {return;}

    last_tx_ms = now_ms;
    has_sent = 1U;

    if (s_super_cap_control.enabled == 1U) {
        s_super_cap_enable_request_submitted = 1U;
    }
}