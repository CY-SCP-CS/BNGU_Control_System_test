/**
 * @file    lib_filter.c
 * @brief   滤波器实现
 */
#include "lib_filter.h"
#include <string.h>


void lib_filter_lpf_init(lib_filter_lpf_t *lpf, float alpha)
{
    lpf->out   = 0.0f;
    lpf->alpha = alpha;
}

float lib_filter_lpf_update(lib_filter_lpf_t *lpf, float input)
{
    lpf->out = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->out;
    return lpf->out;
}

void lib_filter_swf_init(lib_filter_swf_t *swf, float *buf, uint16_t len)
{
    memset(buf, 0, len * sizeof(float));
    swf->buf    = buf;
    swf->len    = len;
    swf->idx    = 0;
    swf->sum    = 0.0f;
    swf->filled = 0;
}

float lib_filter_swf_update(lib_filter_swf_t *swf, float input)
{
    uint16_t count = swf->idx + 1;

    swf->sum -= swf->buf[swf->idx];
    swf->buf[swf->idx] = input;
    swf->sum += input;

    swf->idx++;
    if (swf->idx >= swf->len) {
        swf->idx    = 0;
        swf->filled = 1;
    }

    if (swf->filled) {
        return swf->sum / (float)swf->len;
    } else {
        return swf->sum / (float)count;
    }
}
