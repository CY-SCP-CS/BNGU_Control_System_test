/**
 * @file    lib_filter.h
 * @brief   滤波器：一阶低通/互补滤波/滑动窗口滤波
 */
#ifndef LIB_FILTER_H
#define LIB_FILTER_H

#include "lib_typedef.h"

typedef struct {
    float out; //上一次输出
    float alpha; //滤波系数
} lib_filter_lpf_t;//一阶低通滤波结构体

typedef struct {
    float    *buf;      /* 环形缓冲区 (外部提供) */
    uint16_t  len;      /* 窗口长度 */
    uint16_t  idx;      /* 当前写入位置 */
    float     sum;      /* 窗口内总和 */
    uint8_t   filled;   /* 缓冲区是否已填满 */
} lib_filter_swf_t;//滑动窗口滤波器结构体


/**
 * @brief  一阶低通滤波器初始化
 * @param  lpf   一阶低通滤波器结构体指针
 * @param  alpha 滤波系数
 */
void lib_filter_lpf_init(lib_filter_lpf_t *lpf, float alpha);

/**
 * @brief  一阶低通滤波器更新
 * @param  lpf   一阶低通滤波器结构体指针
 * @param  input 输入值
 * @return 滤波后的值
 */
float lib_filter_lpf_update(lib_filter_lpf_t *lpf, float input);

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
