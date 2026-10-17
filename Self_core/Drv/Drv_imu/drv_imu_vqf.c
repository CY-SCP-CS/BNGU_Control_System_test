// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/** @file drv_imu_vqf.c
 *  @brief VQF 六轴 C 移植：惯性系加速度滤波、倾角修正、运动/静止零偏估计。
 *  @note  基于 dlaidig/vqf 的 vqf/cpp/vqf.cpp，版本与许可见 VQF_LICENSE.txt。
 *         输入加速度为 g，内部换算为 m/s^2。未接入磁力计和磁场干扰拒绝。
 */
#include "drv_imu_filter_internal.h"

enum { VQF_LP_ACC, VQF_LP_REST_GYR, VQF_LP_REST_ACC, VQF_LP_MOTION_R, VQF_LP_MOTION_BIAS };

static float vqf_square(float x)
{
    return x*x;
}

void drv_imu_vqf_default_config(drv_imu_vqf_config_t *config)
{
    static const drv_imu_vqf_config_t defaults = {
        3.0f, 0.5f, 100.0f, 2.0f, 0.1f, 0.0001f, 0.03f,
        1.5f, 0.5f, 2.0f, 0.5f, 1U, 1U
    };
    if (config) {
        *config = defaults;
    }
}

static uint8_t vqf_in_range(float x, float low, float high)
{
    return isfinite(x) && x >= low && x <= high;
}

uint8_t drv_imu_vqf_configure(drv_imu_t *imu, const drv_imu_vqf_config_t *config)
{
    drv_imu_vqf_config_t c;
    if (!imu) {
        return 0U;
    }
    if (config) {
        c = *config;
    } else {
        drv_imu_vqf_default_config(&c);
    }
    if (!vqf_in_range(c.tau_acc, 0.0001f, 3600.0f)
        || !vqf_in_range(c.bias_sigma_init, 0.00001f, 100.0f)
        || !vqf_in_range(c.bias_forgetting_time, 0.001f, 1e6f)
        || !vqf_in_range(c.bias_clip, 0.00001f, 1000.0f)
        || !vqf_in_range(c.bias_sigma_motion, 0.00001f, 100.0f)
        || !vqf_in_range(c.bias_vertical_forgetting_factor, 1e-10f, 1.0f)
        || !vqf_in_range(c.bias_sigma_rest, 0.00001f, 100.0f)
        || !vqf_in_range(c.rest_min_time, 0.0f, 3600.0f)
        || !vqf_in_range(c.rest_filter_tau, 0.0001f, 3600.0f)
        || !vqf_in_range(c.rest_threshold_gyro, 0.00001f, 1000.0f)
        || !vqf_in_range(c.rest_threshold_acc, 0.00001f, 100.0f)
        || c.motion_bias_enabled > 1U || c.rest_bias_enabled > 1U) {
        return 0U;
    }
    imu->vqf_config = c;
    if (imu->algorithm == DRV_IMU_ALGO_VQF) {
        drv_imu_reset_filter(imu);
    }
    return 1U;
}

static void vqf_reset_lpf(double state[], unsigned int count)
{
    for (unsigned int i = 0U; i < count; ++i) {
        state[i] = NAN;
    }
}

void drv_imu_vqf_reset(drv_imu_t *imu)
{
    drv_imu_vqf_state_t *s = &imu->filter.vqf;
    memset(s, 0, sizeof(*s));
    s->gyr_quat = imu->quat;
    s->acc_quat.q0 = 1.0f;
    s->bias_p0 = vqf_square(imu->vqf_config.bias_sigma_init*100.0f);
    s->bias_p[0] = s->bias_p[4] = s->bias_p[8] = s->bias_p0;
    vqf_reset_lpf(s->acc_lp_state, 6U);
    vqf_reset_lpf(s->rest_gyr_state, 6U);
    vqf_reset_lpf(s->rest_acc_state, 6U);
    vqf_reset_lpf(s->motion_r_state, 18U);
    vqf_reset_lpf(s->motion_bias_state, 4U);
}

static void vqf_filter_coefficients(float tau, float dt, double b[3], double a[2])
{
    if (tau < dt/2.0f) {
        b[0] = 1.0;
        b[1] = b[2] = a[0] = a[1] = 0.0;
        return;
    }
    double c = tan(1.4142135623730950488*(double)dt/(2.0*(double)tau));
    double d = c*c+1.4142135623730950488*c+1.0;
    b[0] = c*c/d;
    b[1] = 2.0*b[0];
    b[2] = b[0];
    a[0] = 2.0*(c*c-1.0)/d;
    a[1] = (1.0-1.4142135623730950488*c+c*c)/d;
}

/** 改变 dt 时转换滤波历史，避免二阶滤波器输出跳变。last=NULL 时按上游方法近似末次输出。 */
static void vqf_adapt_lpf(const float last[], unsigned int n, const double old_b[3], const double old_a[2],
                          const double new_b[3], const double new_a[2], double state[])
{
    if (isnan(state[0])) {
        return;
    }
    for (unsigned int i = 0U; i < n; ++i) {
        double y = last ? (double)last[i] : state[2U*i];
        state[2U*i] += (old_b[0]-new_b[0])*y;
        state[2U*i+1U] += (old_b[1]-new_b[1]-old_a[0]+new_a[0])*y;
    }
}

static uint8_t vqf_update_coefficients(drv_imu_t *imu, float dt)
{
    drv_imu_vqf_state_t *s = &imu->filter.vqf;
    const drv_imu_vqf_config_t *c = &imu->vqf_config;
    double acc_b[3], acc_a[2], rest_b[3], rest_a[2];
    float v, motion_w, vertical_w, rest_w, p;
    if (s->coeff_dt == dt) {
        return 1U;
    }
    v = 100.0f*dt/c->bias_forgetting_time;
    p = vqf_square(c->bias_sigma_motion*100.0f);
    motion_w = p*p/v+p;
    vertical_w = motion_w/c->bias_vertical_forgetting_factor;
    p = vqf_square(c->bias_sigma_rest*100.0f);
    rest_w = p*p/v+p;
    if (!imu_filter_positive(v) || !isfinite(motion_w) || !isfinite(vertical_w) || !isfinite(rest_w)) {
        return 0U;
    }
    vqf_filter_coefficients(c->tau_acc, dt, acc_b, acc_a);
    vqf_filter_coefficients(c->rest_filter_tau, dt, rest_b, rest_a);
    if (s->coeff_dt > 0.0f) {
        vqf_adapt_lpf(s->last_acc_lp, 3U, s->acc_b, s->acc_a, acc_b, acc_a, s->acc_lp_state);
        vqf_adapt_lpf(NULL, 9U, s->acc_b, s->acc_a, acc_b, acc_a, s->motion_r_state);
        vqf_adapt_lpf(NULL, 2U, s->acc_b, s->acc_a, acc_b, acc_a, s->motion_bias_state);
        vqf_adapt_lpf(s->rest_gyr_lp, 3U, s->rest_b, s->rest_a, rest_b, rest_a, s->rest_gyr_state);
        vqf_adapt_lpf(s->rest_acc_lp, 3U, s->rest_b, s->rest_a, rest_b, rest_a, s->rest_acc_state);
    }
    memcpy(s->acc_b, acc_b, sizeof(acc_b));
    memcpy(s->acc_a, acc_a, sizeof(acc_a));
    memcpy(s->rest_b, rest_b, sizeof(rest_b));
    memcpy(s->rest_a, rest_a, sizeof(rest_a));
    s->bias_v = v;
    s->bias_motion_w = motion_w;
    s->bias_vertical_w = vertical_w;
    s->bias_rest_w = rest_w;
    s->coeff_dt = dt;
    return 1U;
}

/** 二阶 Butterworth，先对 tau 时间内的样本求平均以建立稳态历史。n >= 2。 */
static void vqf_filter_vec(const float x[], unsigned int n, float tau, float dt,
                           const double b[3], const double a[2], double state[], double *init_time, float out[])
{
    if (isnan(state[0])) {
        if (isnan(state[1])) {
            state[1] = 0.0;
            for (unsigned int i = 0U; i < n; ++i) {
                state[2U+i] = 0.0;
            }
        }
        state[1] += 1.0;
        *init_time += (double)dt;
        for (unsigned int i = 0U; i < n; ++i) {
            state[2U+i] += (double)x[i];
            out[i] = (float)(state[2U+i]/state[1]);
        }
        if ((float)*init_time >= tau) {
            for (unsigned int i = 0U; i < n; ++i) {
                state[2U*i] = (double)out[i]*(1.0-b[0]);
                state[2U*i+1U] = (double)out[i]*(b[2]-a[1]);
            }
        }
        return;
    }
    for (unsigned int i = 0U; i < n; ++i) {
        double input = (double)x[i];
        double y = b[0]*input+state[2U*i];
        state[2U*i] = b[1]*input-a[0]*y+state[2U*i+1U];
        state[2U*i+1U] = b[2]*input-a[1]*y;
        out[i] = (float)y;
    }
}

static void vqf_matrix_multiply(const float a[9], const float b[9], uint8_t transpose_a,
                               uint8_t transpose_b, float out[9])
{
    float tmp[9];
    for (unsigned int i = 0U; i < 3U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            float v = 0.0f;
            for (unsigned int k = 0U; k < 3U; ++k) {
                v += a[transpose_a ? 3U*k+i : 3U*i+k]*b[transpose_b ? 3U*j+k : 3U*k+j];
            }
            tmp[3U*i+j] = v;
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

static void vqf_rotation_matrix(const drv_imu_quat_t *q, float r[9])
{
    r[0] = 1.0f-2.0f*vqf_square(q->q2)-2.0f*vqf_square(q->q3);
    r[1] = 2.0f*(q->q2*q->q1-q->q0*q->q3);
    r[2] = 2.0f*(q->q0*q->q2+q->q3*q->q1);
    r[3] = 2.0f*(q->q0*q->q3+q->q2*q->q1);
    r[4] = 1.0f-2.0f*vqf_square(q->q1)-2.0f*vqf_square(q->q3);
    r[5] = 2.0f*(q->q2*q->q3-q->q1*q->q0);
    r[6] = 2.0f*(q->q3*q->q1-q->q0*q->q2);
    r[7] = 2.0f*(q->q0*q->q1+q->q3*q->q2);
    r[8] = 1.0f-2.0f*vqf_square(q->q1)-2.0f*vqf_square(q->q2);
}

static void vqf_update_bias(drv_imu_t *imu, const float acc_earth[3], float dt)
{
    drv_imu_vqf_state_t *s = &imu->filter.vqf;
    const drv_imu_vqf_config_t *c = &imu->vqf_config;
    float r[9], bias_lp[2], w[3], e[3], gain[9];
    float clip = c->bias_clip*IMU_FILTER_DEG_TO_RAD;
    if (!c->motion_bias_enabled && !c->rest_bias_enabled) {
        return;
    }
    vqf_rotation_matrix(&imu->quat, r);
    bias_lp[0] = r[0]*s->bias[0]+r[1]*s->bias[1]+r[2]*s->bias[2];
    bias_lp[1] = r[3]*s->bias[0]+r[4]*s->bias[1]+r[5]*s->bias[2];
    vqf_filter_vec(r, 9U, c->tau_acc, dt, s->acc_b, s->acc_a, s->motion_r_state,
                   &s->init_time[VQF_LP_MOTION_R], r);
    vqf_filter_vec(bias_lp, 2U, c->tau_acc, dt, s->acc_b, s->acc_a, s->motion_bias_state,
                   &s->init_time[VQF_LP_MOTION_BIAS], bias_lp);
    if (s->rest_detected && c->rest_bias_enabled) {
        for (unsigned int i = 0U; i < 3U; ++i) {
            e[i] = s->rest_gyr_lp[i]-s->bias[i];
            w[i] = s->bias_rest_w;
        }
        memset(r, 0, sizeof(r));
        r[0] = r[4] = r[8] = 1.0f;
    } else if (c->motion_bias_enabled) {
        e[0] = -acc_earth[1]/dt+bias_lp[0]-r[0]*s->bias[0]-r[1]*s->bias[1]-r[2]*s->bias[2];
        e[1] = acc_earth[0]/dt+bias_lp[1]-r[3]*s->bias[0]-r[4]*s->bias[1]-r[5]*s->bias[2];
        e[2] = -r[6]*s->bias[0]-r[7]*s->bias[1]-r[8]*s->bias[2];
        w[0] = w[1] = s->bias_motion_w;
        w[2] = s->bias_vertical_w;
    } else {
        w[0] = -1.0f;
    }
    for (unsigned int i = 0U; i < 9U; i += 4U) {
        if (s->bias_p[i] < s->bias_p0) {
            s->bias_p[i] += s->bias_v;
        }
    }
    if (w[0] < 0.0f) {
        return;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        e[i] = imu_filter_clip(e[i], -clip, clip);
    }
    /* K = P R^T (R P R^T + W)^-1，沿用 VQF 的协方差缩放。 */
    vqf_matrix_multiply(s->bias_p, r, 0U, 1U, gain);
    vqf_matrix_multiply(r, gain, 0U, 0U, gain);
    gain[0] += w[0];
    gain[4] += w[1];
    gain[8] += w[2];
    if (!imu_filter_inverse3(gain, gain)) {
        return;
    }
    vqf_matrix_multiply(r, gain, 1U, 0U, gain);
    vqf_matrix_multiply(s->bias_p, gain, 0U, 0U, gain);
    for (unsigned int i = 0U; i < 3U; ++i) {
        s->bias[i] += gain[3U*i]*e[0]+gain[3U*i+1U]*e[1]+gain[3U*i+2U]*e[2];
        s->bias[i] = imu_filter_clip(s->bias[i], -clip, clip);
    }
    vqf_matrix_multiply(gain, r, 0U, 0U, gain);
    vqf_matrix_multiply(gain, s->bias_p, 0U, 0U, gain);
    for (unsigned int i = 0U; i < 9U; ++i) {
        s->bias_p[i] -= gain[i];
    }
}

static void vqf_update_gyro(drv_imu_t *imu, const float gyr[3], float dt)
{
    drv_imu_vqf_state_t *s = &imu->filter.vqf;
    const drv_imu_vqf_config_t *c = &imu->vqf_config;
    float rate[3] = {gyr[0]-s->bias[0], gyr[1]-s->bias[1], gyr[2]-s->bias[2]};
    if (c->rest_bias_enabled) {
        float deviation = 0.0f;
        float clip = c->bias_clip*IMU_FILTER_DEG_TO_RAD;
        vqf_filter_vec(gyr, 3U, c->rest_filter_tau, dt, s->rest_b, s->rest_a,
                       s->rest_gyr_state, &s->init_time[VQF_LP_REST_GYR], s->rest_gyr_lp);
        for (unsigned int i = 0U; i < 3U; ++i) {
            deviation += vqf_square(gyr[i]-s->rest_gyr_lp[i]);
        }
        s->rest_deviation[0] = deviation;
        if (deviation >= vqf_square(c->rest_threshold_gyro*IMU_FILTER_DEG_TO_RAD)
            || fabsf(s->rest_gyr_lp[0]) > clip || fabsf(s->rest_gyr_lp[1]) > clip
            || fabsf(s->rest_gyr_lp[2]) > clip) {
            s->rest_time = 0.0f;
            s->rest_detected = 0U;
        }
    }
    imu_filter_quat_integrate(&s->gyr_quat, rate, dt);
}

static void vqf_update_acc(drv_imu_t *imu, const float acc[3], float dt)
{
    drv_imu_vqf_state_t *s = &imu->filter.vqf;
    const drv_imu_vqf_config_t *c = &imu->vqf_config;
    float earth[3], norm = imu_filter_norm(acc);
    drv_imu_quat_t correction;
    if (!imu_filter_vec_valid(acc) || !isfinite(norm) || norm < 1e-6f) {
        s->rest_time = 0.0f;
        s->rest_detected = 0U;
        return;
    }
    if (c->rest_bias_enabled) {
        float deviation = 0.0f;
        vqf_filter_vec(acc, 3U, c->rest_filter_tau, dt, s->rest_b, s->rest_a,
                       s->rest_acc_state, &s->init_time[VQF_LP_REST_ACC], s->rest_acc_lp);
        for (unsigned int i = 0U; i < 3U; ++i) {
            deviation += vqf_square(acc[i]-s->rest_acc_lp[i]);
        }
        s->rest_deviation[1] = deviation;
        if (deviation >= vqf_square(c->rest_threshold_acc)) {
            s->rest_time = 0.0f;
            s->rest_detected = 0U;
        } else {
            s->rest_time += dt;
            s->rest_detected = s->rest_time >= c->rest_min_time;
        }
    }
    imu_filter_quat_rotate(&s->gyr_quat, acc, earth);
    vqf_filter_vec(earth, 3U, c->tau_acc, dt, s->acc_b, s->acc_a,
                   s->acc_lp_state, &s->init_time[VQF_LP_ACC], s->last_acc_lp);
    imu_filter_quat_rotate(&s->acc_quat, s->last_acc_lp, earth);
    norm = imu_filter_norm(earth);
    if (!isfinite(norm) || norm < 1e-6f) {
        return;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        earth[i] /= norm;
    }
    correction.q0 = sqrtf(0.5f*(imu_filter_clip(earth[2], -1.0f, 1.0f)+1.0f));
    if (correction.q0 > 1e-6f) {
        correction.q1 = 0.5f*earth[1]/correction.q0;
        correction.q2 = -0.5f*earth[0]/correction.q0;
        correction.q3 = 0.0f;
    } else {
        correction.q0 = correction.q2 = correction.q3 = 0.0f;
        correction.q1 = 1.0f;
    }
    imu_filter_quat_multiply(&correction, &s->acc_quat, &s->acc_quat);
    imu_filter_quat_normalize(&s->acc_quat);
    imu_filter_quat_multiply(&s->acc_quat, &s->gyr_quat, &imu->quat);
    vqf_update_bias(imu, earth, dt);
}

void drv_imu_vqf_update(drv_imu_t *imu, float dt)
{
    float gyr[3], acc[3];
    if (!imu_filter_get_sample(imu, dt, gyr, acc)) {
        return;
    }
    drv_imu_set_algorithm(imu, DRV_IMU_ALGO_VQF);
    if (!vqf_update_coefficients(imu, dt)) {
        return;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        acc[i] *= IMU_FILTER_GRAVITY;
    }
    vqf_update_gyro(imu, gyr, dt);
    vqf_update_acc(imu, acc, dt);
    imu_filter_quat_multiply(&imu->filter.vqf.acc_quat, &imu->filter.vqf.gyr_quat, &imu->quat);
    imu_filter_quat_normalize(&imu->quat);
}
