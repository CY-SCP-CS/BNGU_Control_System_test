/**
 * @file    lib_math.h
 * @brief   数学工具：限幅/最短路径/弧度归一化/角度弧度转换/编码器转换
 */
#ifndef LIB_MATH_H
#define LIB_MATH_H

#include "lib_typedef.h"

#define LIB_PI       3.14159265358979323846f

#define LIB_ENC13_TO_RAD  0u   // 0-8191   → rad  
#define LIB_RAD_TO_ENC13  1u   // rad       → 0-8191 
#define LIB_ENC16_TO_RAD  2u   // 0-65535  → rad
#define LIB_RAD_TO_ENC16  3u   // rad       → 0-65535 

/**
 * @brief  限幅函数
 * @param  value 输入值
 * @param  min   最小值
 * @param  max   最大值
 * @return 限幅后的值
 */
float lib_clamp(float value, float min, float max);
/**
 * @brief 将输入区间线性映射到输出区间，并把输入限制在输入区间内。
 * @param value    输入值。
 * @param in_min   输入下限。
 * @param in_max   输入上限，必须大于 in_min。
 * @param out_min  输入为 in_min 时的输出值。
 * @param out_max  输入为 in_max 时的输出值。
 */
float lib_remap_clamp(float value, float in_min, float in_max,
                      float out_min, float out_max);

/**
 * @brief  获取最短路径 (弧度差值)
 * @param  target 目标角度 (弧度)
 * @param  measure 测量角度 (弧度)
 * @return 最短路径 (弧度差值, 范围 (-PI, PI])
 */
float lib_get_shortest_path(float target, float measure);

/**
 * @brief  弧度归一化到 (-PI, PI]
 * @param  rad 输入弧度
 * @return 归一化后的弧度
 */
float lib_rad_norm(float rad);

/**
 * @brief  角度弧度转换
 * @param  deg 输入角度 (度)
 * @return 转换后的弧度
 */
float lib_deg_to_rad(float deg);

/**
 * @brief  弧度角度转换
 * @param  rad 输入弧度
 * @return 转换后的角度 (度)
 */
float lib_rad_to_deg(float rad);
/*
 * @brief  转速与角速度转换
 * @param  rpm 输入转速 (rpm)
 * @return 转换后的角速度 (rad/s)
 */
float lib_rpm_to_rad_s(float rpm);
/*
 * @brief  角速度与转速转换
 * @param  rad_s 输入角速度 (rad/s)
 * @return 转换后的转速 (rpm)
 */
float lib_rad_s_to_rpm(float rad_s);

/**
 * @brief 电机转速转换为轮缘线速度。
 * @param motor_rpm 电机轴转速，单位 RPM。
 * @param wheel_radius_mm 车轮半径，单位 mm。
 * @param reduction_ratio 电机转速/车轮转速。
 * @return 轮缘线速度，单位 mm/s；参数无效时返回 0。
 */
float lib_motor_rpm_to_mm_s(float motor_rpm, float wheel_radius_mm, float reduction_ratio);

/**
 * @brief 轮缘线速度转换为电机转速。
 * @param speed_mm_s 轮缘线速度，单位 mm/s。
 * @param wheel_radius_mm 车轮半径，单位 mm。
 * @param reduction_ratio 电机转速/车轮转速。
 * @return 电机轴转速，单位 RPM；参数无效时返回 0。
 */
float lib_motor_mm_s_to_rpm(float speed_mm_s, float wheel_radius_mm, float reduction_ratio);

/**
 * @brief  快速 Sigmoid 近似 (分段有理函数)
 * @param  x 输入值
 * @return Sigmoid(x), 范围 (0, 1)
 */
float lib_fast_sigmoid(float x);

/**
 * @brief  编码器值与弧度互转
 * @param  value  编码器值或弧度值 (由 dir 决定)
 * @param  dir    转换方向, 见本文件的宏定义
 * @return 转换结果 (弧度或编码器值)
 */
/**
 * @brief  将 13 位编码器值转换为相对机械零位的有符号角度。
 * @param  raw_enc   当前编码器值，范围 0~8191
 * @param  zero_enc  机械零位编码器值，范围 0~8191
 * @return 相对零位的角度，范围 [-PI, PI]，单位 rad
 */
float lib_enc13_relative_rad(uint16_t raw_enc, uint16_t zero_enc);
float lib_enc_conv(float value, uint8_t dir);

#endif
