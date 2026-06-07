/**
 * @file    lib_math.h
 * @brief   数学工具：限幅/最短路径/弧度归一化/角度弧度转换/编码器转换
 */
#ifndef LIB_MATH_H
#define LIB_MATH_H

#include "lib_typedef.h"

// ─── 编码器转换方向 ──────────────────────────────

#define LIB_MATH_ENC13_TO_RAD  0u   /* 0-8191   → rad */
#define LIB_MATH_RAD_TO_ENC13  1u   /* rad       → 0-8191 */
#define LIB_MATH_ENC16_TO_RAD  2u   /* 0-65535  → rad */
#define LIB_MATH_RAD_TO_ENC16  3u   /* rad       → 0-65535 */

// ─── 限幅 ─────────────────────────────────────────

float lib_math_clamp(float value, float min, float max);

// ─── 角度最短路径误差 (单位: rad) ─────────────────

float lib_math_get_shortest_path(float target, float measure);

// ─── 弧度归一化到 (-PI, PI] ──────────────────────

float lib_math_rad_normalize(float rad);

// ─── 角度弧度转换 ────────────────────────────────

float lib_math_deg2rad(float deg);
float lib_math_rad2deg(float rad);

// ─── 快速 Sigmoid ────────────────────────────────

/**
 * @brief  快速 Sigmoid 近似 (分段有理函数)
 * @param  x 输入值
 * @return Sigmoid(x), 范围 (0, 1)
 */
float lib_math_fast_sigmoid(float x);

// ─── 编码器值与弧度转换 ─────────────────────────

/**
 * @brief  编码器值与弧度互转
 * @param  value  编码器值或弧度值 (由 dir 决定)
 * @param  dir    转换方向, 见 LIB_MATH_ENCxx_TO_RAD / RAD_TO_ENCxx
 * @return 转换结果 (弧度或编码器值)
 */
float lib_math_enc_convert(float value, uint8_t dir);

#endif
