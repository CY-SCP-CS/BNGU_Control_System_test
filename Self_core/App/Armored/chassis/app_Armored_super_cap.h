//
// Created by 33140 on 2026/11/23.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_SUPER_CAP_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_SUPER_CAP_H

/** 单个轮电机的功率模型系数。 */
typedef struct {
    float a; // max(0, I * omega) 项的系数
    float b; // I^2 项的系数
} armored_wheel_power_coeffs_t;

extern const armored_wheel_power_coeffs_t armored_wheel_power_coeffs[4]; // FL、RL、FR、RR
extern const float armored_wheel_power_idle_w; // 四个电调合计的静态输入功率，W

/** 估算四轮电调合计的输入功率（W）。
 *  @param current_a 四轮带方向的转矩电流（A），顺序 FL、RL、FR、RR。
 *  @param rotor_speed_rad_s 四轮电机的转子角速度（rad/s），顺序及符号约定同电流。
 *  @return 估算功率。传入空指针或非有限输入时返回 NAN。
 */
float app_armored_wheel_power_estimate(const float current_a[4],
                                       const float rotor_speed_rad_s[4]);

#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_SUPER_CAP_H
