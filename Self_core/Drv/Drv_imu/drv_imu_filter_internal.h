/** @file drv_imu_filter_internal.h
 *  @brief 姿态滤波内部数学工具；不依赖 HAL，不使用共享可变状态。
 */
#ifndef DRV_IMU_FILTER_INTERNAL_H
#define DRV_IMU_FILTER_INTERNAL_H

#include "drv_imu.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define IMU_FILTER_PI 3.14159265358979323846f
#define IMU_FILTER_DEG_TO_RAD (IMU_FILTER_PI / 180.0f)
#define IMU_FILTER_GRAVITY 9.80665f
#define IMU_FILTER_MAX_DT 0.1f

void drv_imu_ekf_reset(drv_imu_t *imu);
void drv_imu_vqf_reset(drv_imu_t *imu);

static inline float imu_filter_clip(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

static inline uint8_t imu_filter_positive(float x)
{
    return isfinite(x) && x > 0.0f;
}

static inline uint8_t imu_filter_vec_valid(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static inline float imu_filter_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static inline uint8_t imu_filter_get_sample(const drv_imu_t *imu, float dt,
                                          float gyr[3], float acc[3])
{
    if (!imu || !imu_filter_positive(dt) || dt > IMU_FILTER_MAX_DT) {
        return 0U;
    }
    gyr[0] = imu->gyro.x - imu->gyro_offset.x;
    gyr[1] = imu->gyro.y - imu->gyro_offset.y;
    gyr[2] = imu->gyro.z - imu->gyro_offset.z;
    acc[0] = imu->acc.x - imu->acc_offset.x;
    acc[1] = imu->acc.y - imu->acc_offset.y;
    acc[2] = imu->acc.z - imu->acc_offset.z;
    return imu_filter_vec_valid(gyr) && isfinite(imu_filter_norm(gyr));
}

static inline void imu_filter_quat_normalize(drv_imu_quat_t *q)
{
    float n = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    if (!isfinite(n) || n < 1e-6f) {
        q->q0 = 1.0f;
        q->q1 = q->q2 = q->q3 = 0.0f;
        return;
    }
    q->q0 /= n;
    q->q1 /= n;
    q->q2 /= n;
    q->q3 /= n;
}

static inline void imu_filter_quat_multiply(const drv_imu_quat_t *a, const drv_imu_quat_t *b,
                                          drv_imu_quat_t *out)
{
    drv_imu_quat_t q;
    q.q0 = a->q0*b->q0 - a->q1*b->q1 - a->q2*b->q2 - a->q3*b->q3;
    q.q1 = a->q0*b->q1 + a->q1*b->q0 + a->q2*b->q3 - a->q3*b->q2;
    q.q2 = a->q0*b->q2 - a->q1*b->q3 + a->q2*b->q0 + a->q3*b->q1;
    q.q3 = a->q0*b->q3 + a->q1*b->q2 - a->q2*b->q1 + a->q3*b->q0;
    *out = q;
}

static inline void imu_filter_quat_rotate(const drv_imu_quat_t *q, const float v[3], float out[3])
{
    float x = (1.0f-2.0f*(q->q2*q->q2+q->q3*q->q3))*v[0]
            + 2.0f*(q->q1*q->q2-q->q0*q->q3)*v[1] + 2.0f*(q->q0*q->q2+q->q1*q->q3)*v[2];
    float y = 2.0f*(q->q0*q->q3+q->q1*q->q2)*v[0]
            + (1.0f-2.0f*(q->q1*q->q1+q->q3*q->q3))*v[1] + 2.0f*(q->q2*q->q3-q->q0*q->q1)*v[2];
    float z = 2.0f*(q->q1*q->q3-q->q0*q->q2)*v[0] + 2.0f*(q->q0*q->q1+q->q2*q->q3)*v[1]
            + (1.0f-2.0f*(q->q1*q->q1+q->q2*q->q2))*v[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

/** q <- q * Exp(rate * dt)，零角速度时也保持归一化。 */
static inline void imu_filter_quat_integrate(drv_imu_quat_t *q, const float rate[3], float dt)
{
    float n = imu_filter_norm(rate);
    float half = 0.5f*n*dt;
    float scale = n > 1e-6f ? sinf(half)/n : 0.5f*dt;
    drv_imu_quat_t dq = {cosf(half), scale*rate[0], scale*rate[1], scale*rate[2]};
    imu_filter_quat_multiply(q, &dq, q);
    imu_filter_quat_normalize(q);
}

/** 支持原位运算；double 余子式避免小量测协方差的行列式下溢。 */
static inline uint8_t imu_filter_inverse3(const float m[9], float out[9])
{
    double c[9], det;
    c[0] = (double)m[4]*m[8] - (double)m[5]*m[7];
    c[1] = (double)m[2]*m[7] - (double)m[1]*m[8];
    c[2] = (double)m[1]*m[5] - (double)m[2]*m[4];
    c[3] = (double)m[5]*m[6] - (double)m[3]*m[8];
    c[4] = (double)m[0]*m[8] - (double)m[2]*m[6];
    c[5] = (double)m[2]*m[3] - (double)m[0]*m[5];
    c[6] = (double)m[3]*m[7] - (double)m[4]*m[6];
    c[7] = (double)m[1]*m[6] - (double)m[0]*m[7];
    c[8] = (double)m[0]*m[4] - (double)m[1]*m[3];
    det = (double)m[0]*c[0] + (double)m[1]*c[3] + (double)m[2]*c[6];
    if (!isfinite(det) || fabs(det) < DBL_MIN) {
        return 0U;
    }
    for (unsigned int i = 0U; i < 9U; ++i) {
        if (!isfinite(c[i]/det) || fabs(c[i]/det) > FLT_MAX) {
            return 0U;
        }
    }
    for (unsigned int i = 0U; i < 9U; ++i) {
        out[i] = (float)(c[i]/det);
    }
    return 1U;
}

#endif
