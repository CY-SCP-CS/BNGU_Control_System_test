/**
 * @file    lib_pid.c
 * @brief   PID 控制算法实现。
 */
#include "lib_pid.h"

#include "lib_math.h"
#include <string.h>

static float pid_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float pid_clamp_symmetric(float value, float max_abs)
{
    max_abs = pid_abs(max_abs);
    return lib_clamp(value, -max_abs, max_abs);
}

static void pid_update_error(lib_pid_t *pid, float error)
{
    pid->last_err = pid->err;
    pid->err = error;
}

static float pid_apply(lib_pid_t *pid, float error, float p_term,
                       float d_term, float ff_term, float dt)
{
    float base_output = p_term + d_term + ff_term;
    float integral_prev;
    float integral_next;
    float i_term_prev;
    float i_term_next;
    float output_raw;

    if (pid->ki == 0.0f) {
        pid->integral = 0.0f;
        pid->out = lib_clamp(base_output, pid->min_out, pid->max_out);
        return pid->out;
    }

    i_term_prev = pid_clamp_symmetric(pid->ki * pid->integral, pid->max_iout);
    integral_prev = i_term_prev / pid->ki;
    integral_next = integral_prev + error * dt;
    i_term_next = pid_clamp_symmetric(pid->ki * integral_next, pid->max_iout);
    integral_next = i_term_next / pid->ki;
    output_raw = base_output + i_term_next;

    if (output_raw > pid->max_out && i_term_next > i_term_prev) {
        float i_term_limit = pid_clamp_symmetric(pid->max_out - base_output,
                                                  pid->max_iout);
        i_term_next = i_term_limit > i_term_prev ? i_term_limit : i_term_prev;
        integral_next = i_term_next / pid->ki;
        output_raw = base_output + i_term_next;
    } else if (output_raw < pid->min_out && i_term_next < i_term_prev) {
        float i_term_limit = pid_clamp_symmetric(pid->min_out - base_output,
                                                  pid->max_iout);
        i_term_next = i_term_limit < i_term_prev ? i_term_limit : i_term_prev;
        integral_next = i_term_next / pid->ki;
        output_raw = base_output + i_term_next;
    }

    pid->integral = integral_next;
    pid->out = lib_clamp(output_raw, pid->min_out, pid->max_out);
    return pid->out;
}

static float pid_standard_calc(lib_pid_t *pid, float target, float measure,
                               float ff_g, float ff_y, float dt)
{
    float error = target - measure;
    float p_term = pid->kp * error;
    float d_term = pid->kd * (error - pid->err) / dt;
    float ff_term = pid_clamp_symmetric(ff_g * pid->kff_g, pid->max_ff_g)
                  + pid_clamp_symmetric(ff_y * pid->kff_y, pid->max_ff_y);

    pid_update_error(pid, error);
    return pid_apply(pid, error, p_term, d_term, ff_term, dt);
}

void lib_pid_reset(lib_pid_t *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->last_meas = 0.0f;
    pid->last_d_input = 0.0f;
    pid->out = 0.0f;
    pid->speed_lpf.out = 0.0f;
    pid->speed_lpf.initialized = 0U;
}

void lib_pid_init(lib_pid_t *p, float kp, float ki, float kd,
                  float kff_g, float kff_y,
                  float max_ff_g, float max_ff_y,
                  float min_out, float max_out, float max_iout)
{
    float swap;

    if (!p) {
        return;
    }
    if (min_out > max_out) {
        swap = min_out;
        min_out = max_out;
        max_out = swap;
    }

    memset(p, 0, sizeof(lib_pid_t));
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->kff_g = kff_g;
    p->kff_y = kff_y;
    p->max_ff_g = pid_abs(max_ff_g);
    p->max_ff_y = pid_abs(max_ff_y);
    p->min_out = min_out;
    p->max_out = max_out;
    p->max_iout = pid_abs(max_iout);
    p->weight_p = 1.0f;
    p->weight_d = 1.0f;
    lib_lpf_init(&p->speed_lpf, 25.0f);
}

float lib_pid_calc(lib_pid_t *pid, float target, float measure, float dt)
{
    if (!pid) {
        return 0.0f;
    }
    if (!(dt > 0.0f)) {
        return pid->out;
    }
    return pid_standard_calc(pid, target, measure, 0.0f, 0.0f, dt);
}

float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure,
                      float ff_g, float ff_y, float dt)
{
    if (!pid) {
        return 0.0f;
    }
    if (!(dt > 0.0f)) {
        return pid->out;
    }
    return pid_standard_calc(pid, target, measure, ff_g, ff_y, dt);
}

float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure,
                       float ff_g, float ff_y, float speed, float dt)
{
    float error;
    float p_term;
    float d_term;
    float ff_term;

    if (!pid) {
        return 0.0f;
    }
    if (!(dt > 0.0f)) {
        return pid->out;
    }

    error = target - measure;
    p_term = pid->kp * error;
    d_term = -pid->kd * lib_lpf_update(&pid->speed_lpf, speed, dt);
    ff_term = pid_clamp_symmetric(ff_g * pid->kff_g, pid->max_ff_g)
            + pid_clamp_symmetric(ff_y * pid->kff_y, pid->max_ff_y);
    pid_update_error(pid, error);
    return pid_apply(pid, error, p_term, d_term, ff_term, dt);
}

void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d)
{
    if (!pid) {
        return;
    }
    pid->weight_p = lib_clamp(weight_p, 0.0f, 1.0f);
    pid->weight_d = lib_clamp(weight_d, 0.0f, 1.0f);
}

float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure, float dt)
{
    float error;
    float p_term;
    float d_input;
    float d_term;

    if (!pid) {
        return 0.0f;
    }
    if (!(dt > 0.0f)) {
        return pid->out;
    }

    error = target - measure;
    p_term = pid->kp * (pid->weight_p * target - measure);
    d_input = pid->weight_d * target - measure;
    d_term = pid->kd * (d_input - pid->last_d_input) / dt;
    pid->last_d_input = d_input;
    pid_update_error(pid, error);
    return pid_apply(pid, error, p_term, d_term, 0.0f, dt);
}

float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure, float dt)
{
    float error;
    float p_term;
    float d_term;

    if (!pid) {
        return 0.0f;
    }
    if (!(dt > 0.0f)) {
        return pid->out;
    }

    error = target - measure;
    p_term = pid->kp * error;
    d_term = -pid->kd * (measure - pid->last_meas) / dt;
    pid->last_meas = measure;
    pid_update_error(pid, error);
    return pid_apply(pid, error, p_term, d_term, 0.0f, dt);
}

/* ─── 模糊 PID 默认规则 ─── */
static const int8_t s_fuzzy_rule_kp[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 6, 6, 5, 5, 4, 3, 3 }, { 6, 6, 5, 4, 4, 3, 1 },
    { 5, 5, 5, 4, 3, 1, 1 }, { 5, 5, 4, 3, 1, 0, 0 },
    { 4, 4, 3, 1, 1, 0, 0 }, { 4, 3, 1, 0, 0, 0, 0 },
    { 3, 3, 1, 0, 0, 0, 0 },
};

static const int8_t s_fuzzy_rule_ki[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 0, 0, 1, 1, 2, 3, 3 }, { 0, 0, 1, 2, 2, 3, 3 },
    { 0, 1, 2, 2, 3, 4, 4 }, { 1, 1, 2, 3, 4, 5, 5 },
    { 2, 2, 3, 4, 4, 5, 6 }, { 3, 3, 4, 4, 5, 6, 6 },
    { 3, 3, 4, 5, 5, 6, 6 },
};

static const int8_t s_fuzzy_rule_kd[LIB_PID_FUZZY_LEVELS][LIB_PID_FUZZY_LEVELS] = {
    { 4, 1, 0, 0, 0, 1, 4 }, { 4, 1, 0, 1, 1, 2, 3 },
    { 3, 1, 1, 1, 2, 2, 3 }, { 3, 2, 2, 2, 2, 2, 3 },
    { 3, 3, 3, 3, 3, 3, 3 }, { 6, 1, 4, 4, 4, 4, 6 },
    { 6, 5, 5, 5, 4, 4, 6 },
};

/* ─── 模糊量化 ─── */
static int8_t fuzzy_quantize(float val, float gain)
{
    float q = val * gain;
    if (q > 6.0f) q = 6.0f;
    if (q < -6.0f) q = -6.0f;
    return (int8_t)((q + 6.0f) / 2.0f);
}

static float fuzzy_apply(lib_pid_fuzzy_t *fpid, float error,
                         float p_term, float d_term, float dt)
{
    float base_output = p_term + d_term;
    float integral_prev;
    float integral_next;
    float i_term_prev;
    float i_term_next;
    float output_raw;

    if (fpid->ki_cur == 0.0f) {
        fpid->integral = 0.0f;
        fpid->out = lib_clamp(base_output, fpid->min_out, fpid->max_out);
        return fpid->out;
    }

    i_term_prev = pid_clamp_symmetric(fpid->ki_cur * fpid->integral,
                                      fpid->max_iout);
    integral_prev = i_term_prev / fpid->ki_cur;
    integral_next = integral_prev + error * dt;
    i_term_next = pid_clamp_symmetric(fpid->ki_cur * integral_next,
                                      fpid->max_iout);
    integral_next = i_term_next / fpid->ki_cur;
    output_raw = base_output + i_term_next;

    if (output_raw > fpid->max_out && i_term_next > i_term_prev) {
        float i_term_limit = pid_clamp_symmetric(fpid->max_out - base_output,
                                                  fpid->max_iout);
        i_term_next = i_term_limit > i_term_prev ? i_term_limit : i_term_prev;
        integral_next = i_term_next / fpid->ki_cur;
        output_raw = base_output + i_term_next;
    } else if (output_raw < fpid->min_out && i_term_next < i_term_prev) {
        float i_term_limit = pid_clamp_symmetric(fpid->min_out - base_output,
                                                  fpid->max_iout);
        i_term_next = i_term_limit < i_term_prev ? i_term_limit : i_term_prev;
        integral_next = i_term_next / fpid->ki_cur;
        output_raw = base_output + i_term_next;
    }

    fpid->integral = integral_next;
    fpid->out = lib_clamp(output_raw, fpid->min_out, fpid->max_out);
    return fpid->out;
}

/* ─── 模糊 PID 初始化 ─── */
void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd,
                        float min_out, float max_out, float max_iout)
{
    float swap;

    if (!fpid) {
        return;
    }
    if (min_out > max_out) {
        swap = min_out;
        min_out = max_out;
        max_out = swap;
    }

    memset(fpid, 0, sizeof(lib_pid_fuzzy_t));
    fpid->kp_base = kp;
    fpid->ki_base = ki;
    fpid->kd_base = kd;
    fpid->min_out = min_out;
    fpid->max_out = max_out;
    fpid->max_iout = pid_abs(max_iout);
    fpid->kp_cur = kp;
    fpid->ki_cur = ki;
    fpid->kd_cur = kd;
    lib_pid_fuzzy_load_default_rules(fpid);
}

void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid, float ke, float kec,
                       float kp_delta, float ki_delta, float kd_delta)
{
    if (!fpid) {
        return;
    }
    fpid->ke = ke;
    fpid->kec = kec;
    fpid->kp_delta = kp_delta;
    fpid->ki_delta = ki_delta;
    fpid->kd_delta = kd_delta;
}

void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid)
{
    if (!fpid) {
        return;
    }
    fpid->rule_kp = s_fuzzy_rule_kp;
    fpid->rule_ki = s_fuzzy_rule_ki;
    fpid->rule_kd = s_fuzzy_rule_kd;
}

float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure, float dt)
{
    float error;
    float ec;
    int8_t e_idx;
    int8_t ec_idx;
    int8_t dkp;
    int8_t dki;
    int8_t dkd;
    float p_term;
    float d_term;

    if (!fpid) {
        return 0.0f;
    }
    if (!(dt > 0.0f) || !fpid->rule_kp || !fpid->rule_ki || !fpid->rule_kd) {
        return fpid->out;
    }

    error = target - measure;
    ec = (error - fpid->last_err) / dt;
    e_idx = fuzzy_quantize(error, fpid->ke);
    ec_idx = fuzzy_quantize(ec, fpid->kec);

    if (e_idx < 0) e_idx = 0;
    if (e_idx >= LIB_PID_FUZZY_LEVELS) e_idx = LIB_PID_FUZZY_LEVELS - 1;
    if (ec_idx < 0) ec_idx = 0;
    if (ec_idx >= LIB_PID_FUZZY_LEVELS) ec_idx = LIB_PID_FUZZY_LEVELS - 1;

    dkp = fpid->rule_kp[e_idx][ec_idx] - 3;
    dki = fpid->rule_ki[e_idx][ec_idx] - 3;
    dkd = fpid->rule_kd[e_idx][ec_idx] - 3;
    fpid->kp_cur = fpid->kp_base + (float)dkp * fpid->kp_delta / 3.0f;
    fpid->ki_cur = fpid->ki_base + (float)dki * fpid->ki_delta / 3.0f;
    fpid->kd_cur = fpid->kd_base + (float)dkd * fpid->kd_delta / 3.0f;

    p_term = fpid->kp_cur * error;
    d_term = fpid->kd_cur * (error - fpid->last_err) / dt;
    fpid->last_err = error;
    return fuzzy_apply(fpid, error, p_term, d_term, dt);
}