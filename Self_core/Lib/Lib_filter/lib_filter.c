#include "lib_filter.h"
#include <math.h>
#include <string.h>

void lib_lpf_init(lib_lpf_t *lpf, float cutoff_hz)
{
    if (!lpf) {
        return;
    }
    lpf->out = 0.0f;
    lpf->cutoff_hz = cutoff_hz;
    lpf->initialized = 0U;
}

float lib_lpf_update(lib_lpf_t *lpf, float input, float dt)
{
    float alpha;
    if (!lpf || dt <= 0.0f || lpf->cutoff_hz <= 0.0f) {
        return input;
    }
    if (!lpf->initialized) {
        lpf->out = input;
        lpf->initialized = 1U;
        return lpf->out;
    }
    alpha = 1.0f - expf(-6.28318530718f * lpf->cutoff_hz * dt);
    lpf->out += alpha * (input - lpf->out);
    return lpf->out;
}

void lib_swf_init(lib_swf_t *swf, float *buf, uint16_t len)
{
    if (!swf || !buf || len == 0U) {
        return;
    }
    memset(buf, 0, len * sizeof(float));
    swf->buf = buf;
    swf->len = len;
    swf->idx = 0U;
    swf->sum = 0.0f;
    swf->filled = 0U;
}

float lib_swf_update(lib_swf_t *swf, float input)
{
    uint16_t count;
    if (!swf || !swf->buf || swf->len == 0U) {
        return input;
    }
    count = swf->idx + 1U;
    swf->sum -= swf->buf[swf->idx];
    swf->buf[swf->idx] = input;
    swf->sum += input;
    swf->idx++;
    if (swf->idx >= swf->len) {
        swf->idx = 0U;
        swf->filled = 1U;
    }
    return swf->sum / (float)(swf->filled ? swf->len : count);
}