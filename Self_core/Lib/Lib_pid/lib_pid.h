/**
 * @file    lib_pid.h
 * @brief   PID 控制器：标准、前馈、位置环、二自由度与模糊 PID
 */
#ifndef LIB_PID_H
#define LIB_PID_H

#include "lib_typedef.h"
#include "lib_filter.h"

/** 标准 PID 控制器状态与配置。 */
typedef struct {
    float kp;          // 比例增益。
    float ki;          // 积分增益。
    float kd;          // 微分增益。
    float kff_g;       // 第一前馈增益。
    float kff_y;       // 第二前馈增益。
    float max_ff_g;    // 第一前馈限幅。
    float max_ff_y;    // 第二前馈限幅。
    float min_out;     // 总输出下限。
    float max_out;     // 总输出上限。
    float max_iout;    // 积分输出绝对值上限。
    float integral;    // 积分状态。
    float err;         // 本周期误差。
    float last_err;    // 上一周期误差。
    float last_meas;   // 上一周期测量值。
    float last_d_input; // 二自由度 PID 的上一周期微分输入。
    float out;          // 本周期输出。
    float weight_p;    // 二自由度 P 项设定值权重，1 表示标准 PID。
    float weight_d;    // 二自由度 D 项设定值权重，1 表示标准 PID。
    lib_lpf_t speed_lpf;// 位置环微分项使用的速度低通滤波器。
} lib_pid_t;

/**
 * @brief  初始化 PID 的增益、限幅和内部滤波器。
 * @param  pid      PID 状态。
 * @param  kp       比例增益。
 * @param  ki       积分增益。
 * @param  kd       微分增益。
 * @param  kff_g    第一前馈增益。
 * @param  kff_y    第二前馈增益。
 * @param  max_ff_g 第一前馈输出的绝对值上限。
 * @param  max_ff_y 第二前馈输出的绝对值上限。
 * @param  min_out  总输出下限。
 * @param  max_out  总输出上限。
 * @param  max_iout 积分输出绝对值上限。
 */
void lib_pid_init(lib_pid_t *pid, float kp, float ki, float kd,
                  float kff_g, float kff_y,
                  float max_ff_g, float max_ff_y,
                  float min_out, float max_out, float max_iout);

/**
 * @brief  清除 PID 动态状态，保留增益、限幅和滤波配置。
 * @param  pid PID 状态。
 */
void lib_pid_reset(lib_pid_t *pid);

/**
 * @brief  计算标准 PID 输出。
 * @param  pid     PID 状态。
 * @param  target  目标值。
 * @param  measure 测量值。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 限幅后的 PID 输出。
 */
float lib_pid_calc(lib_pid_t *pid, float target, float measure, float dt);

/**
 * @brief  计算带两路前馈的 PID 输出。
 * @param  pid     PID 状态。
 * @param  target  目标值。
 * @param  measure 测量值。
 * @param  ff_g    第一前馈量。
 * @param  ff_y    第二前馈量。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 前馈与 PID 叠加、限幅后的输出。
 */
float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure,
                      float ff_g, float ff_y, float dt);

/**
 * @brief  计算位置环 PID 输出，微分项使用实测速度并经过低通滤波。
 * @param  pid     PID 状态。
 * @param  target  目标位置。
 * @param  measure 测量位置。
 * @param  ff_g    第一前馈量。
 * @param  ff_y    第二前馈量。
 * @param  speed   当前实测速度。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 前馈与位置环 PID 叠加、限幅后的输出。
 */
float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure,
                       float ff_g, float ff_y, float speed, float dt);

/**
 * @brief  设置二自由度 PID 的设定值权重。
 * @param  pid      PID 状态。
 * @param  weight_p P 项设定值权重，范围 0~1；1 为标准 PID。
 * @param  weight_d D 项设定值权重，范围 0~1；1 为标准 PID。
 */
void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d);

/**
 * @brief  计算微分先行 PID；微分项对测量值求导，以减小目标跳变冲击。
 * @param  pid     PID 状态。
 * @param  target  目标值。
 * @param  measure 测量值。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 限幅后的 PID 输出。
 */
float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure, float dt);

/**
 * @brief  计算二自由度 PID 输出。
 * @param  pid     PID 状态。
 * @param  target  目标值。
 * @param  measure 测量值。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 限幅后的 PID 输出。
 */
float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure, float dt);

#define LIB_PID_FUZZY_LEVELS  7U  // NB、NM、NS、ZO、PS、PM、PB。

/** 模糊 PID 的配置、状态和规则表。 */
typedef struct {
    float kp_base;     // Kp 基准值。
    float ki_base;     // Ki 基准值。
    float kd_base;     // Kd 基准值。
    float min_out;     // 输出下限。
    float max_out;     // 输出上限。
    float max_iout;    // 积分输出绝对值上限。
    float integral;    // 积分状态。
    float last_err;    // 上一周期误差。
    float out;         // 上一周期输出。
    float ke;          // 误差量化因子。
    float kec;         // 误差变化率量化因子。
    float kp_delta;    // ΔKp 输出幅度。
    float ki_delta;    // ΔKi 输出幅度。
    float kd_delta;    // ΔKd 输出幅度。
    float kp_cur;      // 当前 Kp。
    float ki_cur;      // 当前 Ki。
    float kd_cur;      // 当前 Kd。
    const int8_t (*rule_kp)[LIB_PID_FUZZY_LEVELS]; // Kp 规则表，值域 -3~3。
    const int8_t (*rule_ki)[LIB_PID_FUZZY_LEVELS]; // Ki 规则表，值域 -3~3。
    const int8_t (*rule_kd)[LIB_PID_FUZZY_LEVELS]; // Kd 规则表，值域 -3~3。
} lib_pid_fuzzy_t;

/**
 * @brief  初始化模糊 PID 的基准增益与限幅。
 * @param  fpid     模糊 PID 状态。
 * @param  kp       Kp 基准值。
 * @param  ki       Ki 基准值。
 * @param  kd       Kd 基准值。
 * @param  min_out  输出下限。
 * @param  max_out  输出上限。
 * @param  max_iout 积分输出绝对值上限。
 */
void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd,
                        float min_out, float max_out, float max_iout);

/**
 * @brief  配置模糊 PID 的量化和增益调整幅度。
 * @param  fpid     模糊 PID 状态。
 * @param  ke       误差量化因子。
 * @param  kec      误差变化率量化因子。
 * @param  kp_delta ΔKp 幅度。
 * @param  ki_delta ΔKi 幅度。
 * @param  kd_delta ΔKd 幅度。
 */
void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid, float ke, float kec,
                       float kp_delta, float ki_delta, float kd_delta);

/**
 * @brief  载入内置的 7×7 模糊规则表。
 * @param  fpid 模糊 PID 状态。
 */
void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid);

/**
 * @brief  根据误差和误差变化率计算模糊 PID 输出。
 * @param  fpid    模糊 PID 状态。
 * @param  target  目标值。
 * @param  measure 测量值。
 * @param  dt      控制周期，单位 s，必须大于 0。
 * @return 限幅后的 PID 输出。
 */
float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure, float dt);

#endif