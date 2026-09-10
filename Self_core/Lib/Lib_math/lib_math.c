/**
 * @file    lib_math.c
 * @brief   数学工具实现
 */
#include "lib_math.h"
#include <math.h>

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
    rad = fmodf(rad + LIB_MATH_PI, 2.0f * LIB_MATH_PI);
    if (rad < 0.0f) rad += 2.0f * LIB_MATH_PI;
    return rad - LIB_MATH_PI;
}

float lib_math_deg_to_rad(float deg)
{
    return deg * (LIB_MATH_PI / 180.0f);
}

float lib_math_rad_to_deg(float rad)
{
    return rad * (float)(180.0f / LIB_MATH_PI);
}

float lib_math_fast_sigmoid(float x)
{
    if (x > 6.0f)  x = 6.0f;
    if (x < -6.0f) x = -6.0f;
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

float lib_math_enc_convert(float value, uint8_t dir)
{
    switch (dir) {
    case LIB_MATH_ENC13_TO_RAD:
        return value * (float)(2.0 * LIB_MATH_PI / 8192.0f);
    case LIB_MATH_RAD_TO_ENC13:
        return value * (float)(8192.0 / (2.0 * LIB_MATH_PI));
    case LIB_MATH_ENC16_TO_RAD:
        return value * (float)(2.0 * LIB_MATH_PI / 65536.0f);
    case LIB_MATH_RAD_TO_ENC16:
        return value * (float)(65536.0 / (2.0 * LIB_MATH_PI));
    default:
        return 0.0f;
    }
}
