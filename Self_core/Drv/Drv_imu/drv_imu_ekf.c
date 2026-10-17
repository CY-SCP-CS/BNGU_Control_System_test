/** @file drv_imu_ekf.c
 *  @brief 六维局部误差状态 EKF：四元数姿态 + 三轴陀螺仪残余零偏。
 *  @note  右乘误差定义 q_true = q * Exp(dtheta)，重力观测 h = R(q)^T * ez。
 *         参考 Solà, Quaternion kinematics for the error-state Kalman filter,
 *         https://arxiv.org/abs/1711.02508 。六轴观测无法确定绝对航向。
 */
#include "drv_imu_filter_internal.h"

void drv_imu_ekf_default_config(drv_imu_ekf_config_t *config)
{
    static const drv_imu_ekf_config_t defaults = {
        0.015f, 0.0001f, 0.035f, 0.35f, 0.0174532925f,
        0.2f, 11.345f, 0.0872664626f
    };
    if (config) {
        *config = defaults;
    }
}

uint8_t drv_imu_ekf_configure(drv_imu_t *imu, const drv_imu_ekf_config_t *config)
{
    drv_imu_ekf_config_t c;
    if (!imu) {
        return 0U;
    }
    if (config) {
        c = *config;
    } else {
        drv_imu_ekf_default_config(&c);
    }
    if (!imu_filter_positive(c.gyro_noise) || c.gyro_noise > 10.0f
        || !imu_filter_positive(c.bias_noise) || c.bias_noise > 10.0f
        || !imu_filter_positive(c.acc_noise) || c.acc_noise > 1.0f
        || !imu_filter_positive(c.initial_attitude_std) || c.initial_attitude_std > IMU_FILTER_PI
        || !imu_filter_positive(c.initial_bias_std) || c.initial_bias_std > 10.0f
        || !imu_filter_positive(c.acc_rejection) || c.acc_rejection >= 1.0f
        || !imu_filter_positive(c.innovation_gate) || c.innovation_gate > 1e6f
        || !imu_filter_positive(c.bias_limit) || c.bias_limit > 10.0f
        || c.acc_noise < 1e-6f || c.gyro_noise < 1e-9f || c.bias_noise < 1e-9f
        || c.initial_attitude_std < 1e-6f || c.initial_bias_std < 1e-9f) {
        return 0U;
    }
    imu->ekf_config = c;
    if (imu->algorithm == DRV_IMU_ALGO_EKF) {
        drv_imu_reset_filter(imu);
    }
    return 1U;
}

void drv_imu_ekf_reset(drv_imu_t *imu)
{
    drv_imu_ekf_state_t *s = &imu->filter.ekf;
    memset(s, 0, sizeof(*s));
    for (unsigned int i = 0U; i < 3U; ++i) {
        s->covariance[i][i] = imu->ekf_config.initial_attitude_std * imu->ekf_config.initial_attitude_std;
        s->covariance[i+3U][i+3U] = imu->ekf_config.initial_bias_std * imu->ekf_config.initial_bias_std;
    }
}

static void ekf_skew(const float v[3], float out[3][3])
{
    out[0][0] = 0.0f;  out[0][1] = -v[2]; out[0][2] = v[1];
    out[1][0] = v[2];  out[1][1] = 0.0f;  out[1][2] = -v[0];
    out[2][0] = -v[1]; out[2][1] = v[0];  out[2][2] = 0.0f;
}

/** P <- F P F^T；临时存储仅 6x6，避免中断栈中保存多份矩阵。 */
static void ekf_transform_covariance(float p[6][6], float f[6][6])
{
    float tmp[6][6] = {{0}};
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 6U; ++j) {
            for (unsigned int k = 0U; k < 6U; ++k) {
                tmp[i][j] += f[i][k]*p[k][j];
            }
        }
    }
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 6U; ++j) {
            float sum = 0.0f;
            for (unsigned int k = 0U; k < 6U; ++k) {
                sum += tmp[i][k]*f[j][k];
            }
            p[i][j] = sum;
        }
    }
}

static void ekf_symmetrize(float p[6][6])
{
    for (unsigned int i = 0U; i < 6U; ++i) {
        p[i][i] = fmaxf(p[i][i], 1e-12f);
        for (unsigned int j = i+1U; j < 6U; ++j) {
            float v = 0.5f*(p[i][j]+p[j][i]);
            p[i][j] = p[j][i] = v;
        }
    }
}

static void ekf_predict(drv_imu_t *imu, const float gyr[3], float dt)
{
    drv_imu_ekf_state_t *s = &imu->filter.ekf;
    float rate[3] = {gyr[0]-s->bias[0], gyr[1]-s->bias[1], gyr[2]-s->bias[2]};
    float skew[3][3], f[6][6] = {{0}};
    float n = imu_filter_norm(rate);
    float angle = n*dt;
    float a, b, c;

    imu_filter_quat_integrate(&imu->quat, rate, dt);
    ekf_skew(rate, skew);
    if (angle < 0.001f) {
        a = dt;
        b = 0.5f*dt*dt;
        c = dt*dt*dt/6.0f;
    } else {
        a = sinf(angle)/n;
        b = (1.0f-cosf(angle))/(n*n);
        c = (angle-sinf(angle))/(n*n*n);
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            float square = 0.0f;
            float identity = i == j ? 1.0f : 0.0f;
            for (unsigned int k = 0U; k < 3U; ++k) {
                square += skew[i][k]*skew[k][j];
            }
            f[i][j] = identity-a*skew[i][j]+b*square;
            f[i][j+3U] = -dt*identity+b*skew[i][j]-c*square;
        }
        f[i+3U][i+3U] = 1.0f;
    }
    ekf_transform_covariance(s->covariance, f);
    float qg = imu->ekf_config.gyro_noise*imu->ekf_config.gyro_noise;
    float qb = imu->ekf_config.bias_noise*imu->ekf_config.bias_noise;
    for (unsigned int i = 0U; i < 3U; ++i) {
        s->covariance[i][i] += qg*dt + qb*dt*dt*dt/3.0f;
        s->covariance[i][i+3U] -= qb*dt*dt/2.0f;
        s->covariance[i+3U][i] -= qb*dt*dt/2.0f;
        s->covariance[i+3U][i+3U] += qb*dt;
    }
    ekf_symmetrize(s->covariance);
}

static void ekf_correct(drv_imu_t *imu, const float acc[3])
{
    drv_imu_ekf_state_t *s = &imu->filter.ekf;
    const drv_imu_ekf_config_t *c = &imu->ekf_config;
    const drv_imu_quat_t *q = &imu->quat;
    float norm = imu_filter_norm(acc);
    float h[3] = {2.0f*(q->q1*q->q3-q->q0*q->q2),
                  2.0f*(q->q0*q->q1+q->q2*q->q3),
                  1.0f-2.0f*(q->q1*q->q1+q->q2*q->q2)};
    float jac[3][3], residual[3], pht[6][3] = {{0}}, gain[6][3] = {{0}};
    float innovation[9] = {0}, inverse[9], dx[6] = {0}, reset[6][6] = {{0}};
    float r = c->acc_noise*c->acc_noise;

    if (!imu_filter_vec_valid(acc) || !isfinite(norm) || norm < 1e-6f
        || fabsf(norm-1.0f) > c->acc_rejection) {
        return;
    }
    ekf_skew(h, jac);
    for (unsigned int i = 0U; i < 3U; ++i) {
        residual[i] = acc[i]/norm-h[i];
    }
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            for (unsigned int k = 0U; k < 3U; ++k) {
                pht[i][j] += s->covariance[i][k]*jac[j][k];
            }
        }
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            float v = i == j ? r : 0.0f;
            for (unsigned int k = 0U; k < 3U; ++k) {
                v += jac[i][k]*pht[k][j];
            }
            innovation[3U*i+j] = v;
        }
    }
    if (!imu_filter_inverse3(innovation, inverse)) {
        return;
    }
    s->innovation_squared = 0.0f;
    for (unsigned int i = 0U; i < 3U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            s->innovation_squared += residual[i]*inverse[3U*i+j]*residual[j];
        }
    }
    if (!isfinite(s->innovation_squared) || s->innovation_squared > c->innovation_gate) {
        return;
    }
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            for (unsigned int k = 0U; k < 3U; ++k) {
                gain[i][j] += pht[i][k]*inverse[3U*k+j];
            }
            dx[i] += gain[i][j]*residual[j];
        }
    }
    /* Joseph 形式，P <- (I-KH) P (I-KH)^T + K R K^T。 */
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 6U; ++j) {
            reset[i][j] = i == j ? 1.0f : 0.0f;
            if (j < 3U) {
                for (unsigned int k = 0U; k < 3U; ++k) {
                    reset[i][j] -= gain[i][k]*jac[k][j];
                }
            }
        }
    }
    ekf_transform_covariance(s->covariance, reset);
    for (unsigned int i = 0U; i < 6U; ++i) {
        for (unsigned int j = 0U; j < 6U; ++j) {
            for (unsigned int k = 0U; k < 3U; ++k) {
                s->covariance[i][j] += r*gain[i][k]*gain[j][k];
            }
        }
    }
    imu_filter_quat_integrate(&imu->quat, dx, 1.0f);
    for (unsigned int i = 0U; i < 3U; ++i) {
        s->bias[i] = imu_filter_clip(s->bias[i]+dx[i+3U], -c->bias_limit, c->bias_limit);
    }
    /* 注入误差后，协方差转回新的局部切空间。 */
    ekf_skew(dx, jac);
    memset(reset, 0, sizeof(reset));
    for (unsigned int i = 0U; i < 6U; ++i) {
        reset[i][i] = 1.0f;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        for (unsigned int j = 0U; j < 3U; ++j) {
            reset[i][j] -= 0.5f*jac[i][j];
        }
    }
    ekf_transform_covariance(s->covariance, reset);
    ekf_symmetrize(s->covariance);
    s->acc_used = 1U;
}

void drv_imu_ekf_update(drv_imu_t *imu, float dt)
{
    float gyr[3], acc[3];
    if (!imu_filter_get_sample(imu, dt, gyr, acc)) {
        return;
    }
    drv_imu_set_algorithm(imu, DRV_IMU_ALGO_EKF);
    imu->filter.ekf.acc_used = 0U;
    imu->filter.ekf.innovation_squared = 0.0f;
    ekf_predict(imu, gyr, dt);
    ekf_correct(imu, acc);
}
