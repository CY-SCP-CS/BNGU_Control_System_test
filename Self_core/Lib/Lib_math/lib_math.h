/**
 * @file    lib_math.h
 * @brief   数学工具：限幅、角度、转速和编码器单位转换
 */
#ifndef LIB_MATH_H
#define LIB_MATH_H

#include "lib_typedef.h"

#define LIB_PI  3.14159265358979323846f

#define LIB_ENC13_TO_RAD  0U  // 13 位编码器值转换为 rad。
#define LIB_RAD_TO_ENC13  1U  // rad 转换为 13 位编码器计数。
#define LIB_ENC16_TO_RAD  2U  // 16 位编码器值转换为 rad。
#define LIB_RAD_TO_ENC16  3U  // rad 转换为 16 位编码器计数。

/**
 * @brief  将值限制在闭区间 [min, max] 内。
 * @param  value 输入值。
 * @param  min   下限。
 * @param  max   上限。
 * @return 限幅后的值。
 */
float lib_clamp(float value, float min, float max);

/**
 * @brief  将输入区间线性映射到输出区间，同时限制输入范围。
 * @param  value   输入值。
 * @param  in_min  输入下限。
 * @param  in_max  输入上限，必须大于 in_min。
 * @param  out_min 输入为 in_min 时的输出值。
 * @param  out_max 输入为 in_max 时的输出值。
 * @return 映射后的输出值。
 */
float lib_remap_clamp(float value, float in_min, float in_max,
                      float out_min, float out_max);

/**
 * @brief  计算目标角度相对测量角度的最短有符号差。
 * @param  target  目标角度，单位 rad。
 * @param  measure 测量角度，单位 rad。
 * @return target - measure 归一化后的结果，范围 [-PI, PI)。
 */
float lib_get_shortest_path(float target, float measure);

/**
 * @brief  将弧度归一化到 [-PI, PI)。
 * @param  rad 输入角度，单位 rad。
 * @return 归一化后的角度。
 */
float lib_rad_norm(float rad);

/**
 * @brief  角度转换为弧度。
 * @param  deg 角度，单位 °。
 * @return 弧度值。
 */
float lib_deg_to_rad(float deg);

/**
 * @brief  弧度转换为角度。
 * @param  rad 弧度值。
 * @return 角度，单位 °。
 */
float lib_rad_to_deg(float rad);

/**
 * @brief  转速转换为角速度。
 * @param  rpm 转速，单位 RPM。
 * @return 角速度，单位 rad/s。
 */
float lib_rpm_to_rad_s(float rpm);

/**
 * @brief  角速度转换为转速。
 * @param  rad_s 角速度，单位 rad/s。
 * @return 转速，单位 RPM。
 */
float lib_rad_s_to_rpm(float rad_s);

/**
 * @brief  电机轴转速转换为轮缘线速度。
 * @param  motor_rpm         电机轴转速，单位 RPM。
 * @param  wheel_radius_mm   车轮半径，单位 mm。
 * @param  reduction_ratio   电机转速与车轮转速的比值。
 * @return 轮缘线速度，单位 mm/s；几何参数无效时返回 0。
 */
float lib_motor_rpm_to_mm_s(float motor_rpm, float wheel_radius_mm, float reduction_ratio);

/**
 * @brief  轮缘线速度转换为电机轴转速。
 * @param  speed_mm_s        轮缘线速度，单位 mm/s。
 * @param  wheel_radius_mm   车轮半径，单位 mm。
 * @param  reduction_ratio   电机转速与车轮转速的比值。
 * @return 电机轴转速，单位 RPM；几何参数无效时返回 0。
 */
float lib_motor_mm_s_to_rpm(float speed_mm_s, float wheel_radius_mm, float reduction_ratio);

/**
 * @brief  计算快速 Sigmoid 近似值。
 * @param  x 输入值。
 * @return 近似 Sigmoid(x)，范围 (0, 1)。
 */
float lib_fast_sigmoid(float x);

/**
 * @brief  将 13 位编码器值换算为相对机械零位的有符号角度。
 * @param  raw_enc  当前编码器值，范围 0~8191。
 * @param  zero_enc 机械零位编码器值，范围 0~8191。
 * @return 相对机械零位的角度，范围约为 [-PI, PI)，单位 rad。
 */
float lib_enc13_relative_rad(uint16_t raw_enc, uint16_t zero_enc);

/**
 * @brief  在编码器计数与弧度之间转换。
 * @param  value 编码器计数或弧度值，由 dir 决定。
 * @param  dir   转换方向，取 LIB_ENC13_TO_RAD、LIB_RAD_TO_ENC13、
 *               LIB_ENC16_TO_RAD 或 LIB_RAD_TO_ENC16。
 * @return 转换后的值；方向无效时返回 0。
 */
float lib_enc_conv(float value, uint8_t dir);

#endif