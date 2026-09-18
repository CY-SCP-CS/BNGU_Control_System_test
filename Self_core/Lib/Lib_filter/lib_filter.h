/**
 * @file    lib_filter.h
 * @brief   滤波器：一阶低通与滑动窗口滤波
 */
#ifndef LIB_FILTER_H
#define LIB_FILTER_H

#include "lib_typedef.h"

/** 一阶低通滤波器状态。 */
typedef struct {
    float out;          // 上一次滤波输出。
    float cutoff_hz;    // -3 dB 截止频率，单位 Hz。
    uint8_t initialized;// 首次采样标志。
} lib_lpf_t;

/** 滑动窗口滤波器状态；缓冲区由调用者提供。 */
typedef struct {
    float    *buf;      // 环形缓冲区。
    uint16_t  len;      // 窗口长度。
    uint16_t  idx;      // 当前写入位置。
    float     sum;      // 窗口内样本总和。
    uint8_t   filled;   // 缓冲区是否已填满。
} lib_swf_t;

/**
 * @brief  初始化一阶低通滤波器。
 * @param  lpf       滤波器状态。
 * @param  cutoff_hz -3 dB 截止频率，单位 Hz。
 * @note   首次调用 lib_lpf_update 时直接采用输入值，避免上电瞬态。
 */
void lib_lpf_init(lib_lpf_t *lpf, float cutoff_hz);

/**
 * @brief  更新一阶低通滤波器。
 * @param  lpf   滤波器状态。
 * @param  input 当前输入值。
 * @param  dt    当前采样周期，单位 s，必须大于 0。
 * @return 滤波后的输出值。
 */
float lib_lpf_update(lib_lpf_t *lpf, float input, float dt);

/**
 * @brief  初始化滑动窗口滤波器。
 * @param  swf 滤波器状态。
 * @param  buf 外部缓冲区，长度至少为 len。
 * @param  len 窗口长度。
 */
void lib_swf_init(lib_swf_t *swf, float *buf, uint16_t len);

/**
 * @brief  向滑动窗口写入一个样本并计算平均值。
 * @param  swf   滤波器状态。
 * @param  input 当前输入值。
 * @return 当前窗口内样本的平均值。
 */
float lib_swf_update(lib_swf_t *swf, float input);

#endif