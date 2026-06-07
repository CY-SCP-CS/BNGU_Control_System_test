/**
 * @file    lib_filter.h
 * @brief   滤波器：一阶低通/互补滤波/滑动窗口滤波
 */
#ifndef LIB_FILTER_H
#define LIB_FILTER_H

#include "lib_typedef.h"

// ─── 一阶低通滤波器 ──────────────────────────────

typedef struct {
    float out;      /* 上一拍输出值 */
    float alpha;    /* 滤波系数 (0~1), 越小越平滑 */
} lib_filter_lpf_t;

void lib_filter_lpf_init(lib_filter_lpf_t *lpf, float alpha);
float lib_filter_lpf_update(lib_filter_lpf_t *lpf, float input);

// ─── 滑动窗口滤波 (移动平均) ────────────────────

typedef struct {
    float    *buf;      /* 环形缓冲区 (外部提供) */
    uint16_t  len;      /* 窗口长度 */
    uint16_t  idx;      /* 当前写入位置 */
    float     sum;      /* 窗口内总和 */
    uint8_t   filled;   /* 缓冲区是否已填满 */
} lib_filter_swf_t;

/**
 * @brief  滑动窗口滤波器初始化
 * @param  swf  滑动窗口滤波器结构体指针
 * @param  buf  外部提供的缓冲区 (长度 >= len)
 * @param  len  窗口长度
 */
void lib_filter_swf_init(lib_filter_swf_t *swf, float *buf, uint16_t len);

/**
 * @brief  滑动窗口滤波器更新
 * @param  swf   滑动窗口滤波器结构体指针
 * @param  input 输入值
 * @return 窗口内平均值
 */
float lib_filter_swf_update(lib_filter_swf_t *swf, float input);

#endif
