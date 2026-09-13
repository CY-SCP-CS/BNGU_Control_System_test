/**
 * @file    lib_math.c
 * @brief   数学工具实现
 */
#include "lib_math.h"
#include <math.h>

float lib_clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float lib_get_shortest_path(float target, float measure)
{
    return lib_rad_norm(target - measure);
}

float lib_rad_norm(float rad)
{
    rad = fmodf(rad + LIB_PI, 2.0f * LIB_PI);
    if (rad < 0.0f) rad += 2.0f * LIB_PI;
    return rad - LIB_PI;
}

float lib_deg_to_rad(float deg)
{
    return deg * (LIB_PI / 180.0f);
}

float lib_rad_to_deg(float rad)
{
    return rad * (float)(180.0f / LIB_PI);
}

float lib_rpm_to_rad_s(float rpm)
{
    return rpm * (2.0f * LIB_PI / 60.0f);
}

float lib_rad_s_to_rpm(float rad_s)
{
    return rad_s * (60.0f / (2.0f * LIB_PI));
}

float lib_motor_rpm_to_mm_s(float motor_rpm, float wheel_radius_mm, float reduction_ratio)
{
    if (wheel_radius_mm <= 0.0f || reduction_ratio <= 0.0f) {
        return 0.0f;
    }
    return motor_rpm * 2.0f * LIB_PI * wheel_radius_mm
           / (60.0f * reduction_ratio);
}

float lib_motor_mm_s_to_rpm(float speed_mm_s, float wheel_radius_mm, float reduction_ratio)
{
    if (wheel_radius_mm <= 0.0f || reduction_ratio <= 0.0f) {
        return 0.0f;
    }
    return speed_mm_s * 60.0f * reduction_ratio
           / (2.0f * LIB_PI * wheel_radius_mm);
}

float lib_fast_sigmoid(float x)
{
    if (x > 6.0f)  x = 6.0f;
    if (x < -6.0f) x = -6.0f;
    return 0.5f * (x / (1.0f + fabsf(x))) + 0.5f;
}

float lib_enc13_relative_rad(uint16_t raw_enc, uint16_t zero_enc)
{
    int32_t diff = (int32_t)raw_enc - (int32_t)zero_enc;

    if (diff > 4096) {
        diff -= 8192;
    } else if (diff < -4095) {
        diff += 8192;
    }
    return lib_enc_conv((float)diff, LIB_ENC13_TO_RAD);
}
float lib_enc_conv(float value, uint8_t dir)
{
    switch (dir) {
    case LIB_ENC13_TO_RAD:
        return value * (float)(2.0 * LIB_PI / 8192.0f);
    case LIB_RAD_TO_ENC13:
        return value * (float)(8192.0 / (2.0 * LIB_PI));
    case LIB_ENC16_TO_RAD:
        return value * (float)(2.0 * LIB_PI / 65536.0f);
    case LIB_RAD_TO_ENC16:
        return value * (float)(65536.0 / (2.0 * LIB_PI));
    default:
        return 0.0f;
    }
}
