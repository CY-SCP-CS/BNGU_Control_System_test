/**
 * @file    lib_math.c
 * @brief   数学工具实现
 */
#include "lib_math.h"
#include <math.h>

// ─── 接口实现 ─────────────────────────────────────

float lib_math_clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float lib_math_get_shortest_path(float target, float measure)
{
    return lib_math_rad_normalize(target - measure);
}

float lib_math_rad_normalize(float rad)
{
    while (rad > (float)M_PI) rad -= 2.0f * (float)M_PI;
    while (rad <= (float)-M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

float lib_math_deg2rad(float deg)
{
    return deg * (float)(M_PI / 180.0);
}

float lib_math_rad2deg(float rad)
{
    return rad * (float)(180.0 / M_PI);
}

// ─── 快速 Sigmoid ───────────────────────────────

float lib_math_fast_sigmoid(float x)
{
    if (x > 6.0f)  x = 6.0f;
    if (x < -6.0f) x = -6.0f;
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

// ─── 编码器值与弧度转换 ─────────────────────────

float lib_math_enc_convert(float value, uint8_t dir)
{
    switch (dir) {
    case LIB_MATH_ENC13_TO_RAD:
        return value * (float)(2.0 * M_PI / 8192.0);
    case LIB_MATH_RAD_TO_ENC13:
        return value * (float)(8192.0 / (2.0 * M_PI));
    case LIB_MATH_ENC16_TO_RAD:
        return value * (float)(2.0 * M_PI / 65536.0);
    case LIB_MATH_RAD_TO_ENC16:
        return value * (float)(65536.0 / (2.0 * M_PI));
    default:
        return 0.0f;
    }
}
