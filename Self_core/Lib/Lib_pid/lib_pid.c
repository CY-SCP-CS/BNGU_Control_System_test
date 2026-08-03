/**
 * @file    lib_pid.c
 * @brief   PID 控制器实现
 */
#include "lib_pid.h"
#include "lib_math.h"
#include <string.h>


void lib_pid_init(lib_pid_t *p, float kp, float ki, float kd,
                  float kff_g, float kff_y,
                  float max_ff_g, float max_ff_y,
                  float min_out, float max_out, float max_iout)
{
    memset(p, 0, sizeof(lib_pid_t));
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->kff_g = kff_g; p->kff_y = kff_y;
    p->max_ff_g = max_ff_g; p->max_ff_y = max_ff_y;
    p->min_out = min_out; p->max_out = max_out;
    p->max_iout = max_iout;
    p->weight_p = 1.0f;
    p->weight_d = 1.0f;
    /* speed_lpf 默认 alpha=0, 需外部另行配置 */
    lib_filter_lpf_init(&p->speed_lpf, 0.1f);
}

float lib_pid_calc(lib_pid_t *pid, float target, float measure)
{
    float error = target - measure;

    /* P */
    float p_term = pid->kp * error;

    /* I */
    pid->integral += error;
    pid->integral = lib_math_clamp(pid->integral, -pid->max_iout, pid->max_iout);
    float i_term = pid->ki * pid->integral;

    /* D */
    float d_term = pid->kd * (error - pid->last_err);
    pid->last_err = error;

    pid->out = lib_math_clamp(p_term + i_term + d_term,
                              pid->min_out, pid->max_out);
    return pid->out;
}

float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure,
                      float ff_g, float ff_y)
{
    lib_pid_calc(pid, target, measure);

    float feedforward_g = lib_math_clamp(ff_g * pid->kff_g,
                                         -pid->max_ff_g, pid->max_ff_g);
    float feedforward_y = lib_math_clamp(ff_y * pid->kff_y,
                                         -pid->max_ff_y, pid->max_ff_y);

    pid->out = lib_math_clamp(pid->out + feedforward_g + feedforward_y,
                              pid->min_out, pid->max_out);
    return pid->out;
}

float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure,
                       float ff_g, float ff_y, float speed)
{
    float error = target - measure;

    float p_term = pid->kp * error;

    pid->integral += error;
    pid->integral = lib_math_clamp(pid->integral, -pid->max_iout, pid->max_iout);
    float i_term = pid->ki * pid->integral;

    float filtered_v = lib_filter_lpf_update(&pid->speed_lpf, speed);
    float d_term = -pid->kd * filtered_v;

    float feedforward_g = lib_math_clamp(ff_g * pid->kff_g,
                                         -pid->max_ff_g, pid->max_ff_g);
    float feedforward_y = lib_math_clamp(ff_y * pid->kff_y,
                                         -pid->max_ff_y, pid->max_ff_y);

        pid->last_err = error;
pid->out = lib_math_clamp(p_term + i_term + d_term + feedforward_g + feedforward_y,
                              pid->min_out, pid->max_out);

    return pid->out;
}

void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d)
{
    pid->weight_p = weight_p;
    pid->weight_d = weight_d;
}

float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure)
{
    float error = target - measure;

    float p_term = pid->kp * error;

    pid->integral += error;
    pid->integral = lib_math_clamp(pid->integral, -pid->max_iout, pid->max_iout);
    float i_term = pid->ki * pid->integral;

    float d_term = -pid->kd * (measure - pid->last_meas);
    pid->last_meas = measure;
    pid->last_err   = error;

    pid->out = lib_math_clamp(p_term + i_term + d_term,
                              pid->min_out, pid->max_out);
    return pid->out;
}

// ─── 模糊 PID 默认规则表 ─────────────────────────

static const int8_t fuzzy_rule_kp[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 6, 6, 5, 5, 4, 3, 3 },
    { 6, 6, 5, 4, 4, 3, 1 },
    { 5, 5, 5, 4, 3, 1, 1 },
    { 5, 5, 4, 3, 1, 0, 0 },
    { 4, 4, 3, 1, 1, 0, 0 },
    { 4, 3, 1, 0, 0, 0, 0 },
    { 3, 3, 1, 0, 0, 0, 0 },
};

static const int8_t fuzzy_rule_ki[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 0, 0, 1, 1, 2, 3, 3 },
    { 0, 0, 1, 2, 2, 3, 3 },
    { 0, 1, 2, 2, 3, 4, 4 },
    { 1, 1, 2, 3, 4, 5, 5 },
    { 2, 2, 3, 4, 4, 5, 6 },
    { 3, 3, 4, 4, 5, 6, 6 },
    { 3, 3, 4, 5, 5, 6, 6 },
};

static const int8_t fuzzy_rule_kd[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 4, 1, 0, 0, 0, 1, 4 },
    { 4, 1, 0, 1, 1, 2, 3 },
    { 3, 1, 1, 1, 2, 2, 3 },
    { 3, 2, 2, 2, 2, 2, 3 },
    { 3, 3, 3, 3, 3, 3, 3 },
    { 6, 1, 4, 4, 4, 4, 6 },
    { 6, 5, 5, 5, 4, 4, 6 },
};

// ─── 量化辅助 ───────────────────────────────────

static int8_t fuzzy_quantize(float val, float gain)
{
    float q = val * gain;

    if (q > 6.0f)  q = 6.0f;
    if (q < -6.0f) q = -6.0f;
    return (int8_t)((q + 6.0f) / 2.0f);
}

// ─── 模糊 PID 接口实现 ───────────────────────────

void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd,
                        float min_out, float max_out, float max_iout)
{
    memset(fpid, 0, sizeof(lib_pid_fuzzy_t));
    fpid->kp_base  = kp;
    fpid->ki_base  = ki;
    fpid->kd_base  = kd;
    fpid->min_out  = min_out;
    fpid->max_out  = max_out;
    fpid->max_iout = max_iout;
    fpid->kp_cur   = kp;
    fpid->ki_cur   = ki;
    fpid->kd_cur   = kd;
}

void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid,
                       float ke, float kec,
                       float kp_delta, float ki_delta, float kd_delta)
{
    fpid->ke       = ke;
    fpid->kec      = kec;
    fpid->kp_delta = kp_delta;
    fpid->ki_delta = ki_delta;
    fpid->kd_delta = kd_delta;
}

void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid)
{
    fpid->rule_kp = fuzzy_rule_kp;
    fpid->rule_ki = fuzzy_rule_ki;
    fpid->rule_kd = fuzzy_rule_kd;
}

float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure)
{
    float error = target - measure;
    float ec    = error - fpid->last_err;
    int8_t e_idx, ec_idx;
    int8_t dkp, dki, dkd;

    e_idx  = fuzzy_quantize(error, fpid->ke);
    ec_idx = fuzzy_quantize(ec, fpid->kec);

    if (e_idx  < 0) e_idx  = 0;
    if (e_idx  >= LIB_PID_FUZZY_LEVELS) e_idx  = LIB_PID_FUZZY_LEVELS - 1;
    if (ec_idx < 0) ec_idx = 0;
    if (ec_idx >= LIB_PID_FUZZY_LEVELS) ec_idx = LIB_PID_FUZZY_LEVELS - 1;

    dkp = fpid->rule_kp[e_idx][ec_idx] - 3;
    dki = fpid->rule_ki[e_idx][ec_idx] - 3;
    dkd = fpid->rule_kd[e_idx][ec_idx] - 3;

    fpid->kp_cur = fpid->kp_base + (float)dkp * fpid->kp_delta / 3.0f;
    fpid->ki_cur = fpid->ki_base + (float)dki * fpid->ki_delta / 3.0f;
    fpid->kd_cur = fpid->kd_base + (float)dkd * fpid->kd_delta / 3.0f;

    float p_term = fpid->kp_cur * error;

    fpid->integral += error;
    fpid->integral = lib_math_clamp(fpid->integral, -fpid->max_iout, fpid->max_iout);
    float i_term = fpid->ki_cur * fpid->integral;

    float d_term = fpid->kd_cur * (error - fpid->last_err);
    fpid->last_err = error;

    fpid->out = lib_math_clamp(p_term + i_term + d_term,
                               fpid->min_out, fpid->max_out);
    return fpid->out;
}
