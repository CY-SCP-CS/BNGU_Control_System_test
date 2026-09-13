/**
 * @file    lib_pid.h
 * @brief   PID 控制器: 标准PID / 前馈PID / 位置环PID / 模糊PID
 */
#ifndef LIB_PID_H
#define LIB_PID_H

#include "lib_typedef.h"
#include "lib_filter.h"
//模糊PID和二自由度还没细究过，现在只是AI写的暂不使用
typedef struct {

    float kp;
    float ki;
    float kd;

    float kff_g;
    float kff_y;

    float max_ff_g;
    float max_ff_y;
    float min_out;
    float max_out;
    float max_iout;

    float integral;
    float last_err;
    float last_meas;
    float out;

    float weight_p;            //二自由度 P 项权重 (0~1), 1=标准PID */
    float weight_d;            //二自由度 D 项权重 (0~1), 1=标准PID */

    lib_lpf_t speed_lpf;//速度低通滤波，供微分项使用（位置环PID）
} lib_pid_t;//PID 控制器结构体


/**
 * @brief  PID 控制器初始化
 * @param  p         PID 结构体指针
 * @param  kp        比例增益
 * @param  ki        积分增益
 * @param  kd        微分增益
 * @param  kff_g     前馈一增益
 * @param  kff_y     前馈二增益
 * @param  max_ff_g  前馈一上限
 * @param  max_ff_y  前馈二上限
 * @param  min_out   输出下限
 * @param  max_out   输出上限
 * @param  max_iout  积分上限
 */
void lib_pid_init(lib_pid_t *p, float kp, float ki, float kd,
                  float kff_g, float kff_y,
                  float max_ff_g, float max_ff_y,
                  float min_out, float max_out, float max_iout);

/**
 * @brief  标准 PID 计算
 * @param  pid     PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @return PID 输出
 */
float lib_pid_calc(lib_pid_t *pid, float target, float measure);

/** 
 * @brief 清除动态状态，保留增益、限幅和滤波系数
 * @param  pid     要清除的PID 结构体指针
 */
void lib_pid_reset(lib_pid_t *pid);

/**
 * @brief  前馈 PID 计算
 * @param  pid     PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @param  ff_g    前馈量1
 * @param  ff_y    前馈量2
 * @return PID 输出
 */
float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure,
                      float ff_g, float ff_y);

/**
 * @brief  位置环 PID 计算 (含速度微分项 + 前馈)
 * @param  pid     PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @param  ff_g    前馈一量
 * @param  ff_y    前馈二量
 * @param  speed   当前速度 (用于微分项)
 * @return PID 输出
 */
float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure,
                       float ff_g, float ff_y, float speed);

/**
 * @brief  设置二自由度 PID 设定值加权系数
 * @param  pid  PID 结构体指针
 * @param  weight_p    P 项设定值权重 (0~1), 1 为标准 PID
 * @param  weight_d    D 项设定值权重 (0~1), 1 为标准 PID
 */
void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d);

/**
 * @brief  微分先行 PID 计算 (D 对测量值求导, 避免目标跳变冲击)
 * @param  pid     PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @return PID 输出
 */
float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure);

/**
 * @brief  二自由度 PID 计算 (设定值加权, 独立调节跟踪与抗扰)
 * @param  pid     PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @return PID 输出
 */
float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure);

// ─── 模糊 PID ────────────────────────────────────

#define LIB_PID_FUZZY_LEVELS  7   /* NB, NM, NS, ZO, PS, PM, PB */

typedef struct {
    /* 基础 PID 增益 (被模糊调整的基准) */
    float kp_base;
    float ki_base;
    float kd_base;

    /* 限幅 */
    float min_out;
    float max_out;
    float max_iout;

    /* 状态量 */
    float integral;
    float last_err;
    float out;

    /* 模糊量化与比例因子 */
    float ke;        /* 误差量化因子            */
    float kec;       /* 误差变化率量化因子       */
    float kp_delta;  /* ΔKp 输出幅度            */
    float ki_delta;  /* ΔKi 输出幅度            */
    float kd_delta;  /* ΔKd 输出幅度            */

    /* 当前实际使用的增益 */
    float kp_cur;
    float ki_cur;
    float kd_cur;

    /* 规则表 (7×7, 值域 -3~3) */
    const int8_t (*rule_kp)[LIB_PID_FUZZY_LEVELS];
    const int8_t (*rule_ki)[LIB_PID_FUZZY_LEVELS];
    const int8_t (*rule_kd)[LIB_PID_FUZZY_LEVELS];
} lib_pid_fuzzy_t;

/**
 * @brief  模糊 PID 初始化
 * @param  fpid     模糊 PID 结构体指针
 * @param  kp       Kp 基准值
 * @param  ki       Ki 基准值
 * @param  kd       Kd 基准值
 * @param  min_out  输出下限
 * @param  max_out  输出上限
 * @param  max_iout 积分上限
 */
void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd,
                        float min_out, float max_out, float max_iout);

/**
 * @brief  配置模糊 PID 量化与比例因子
 * @param  fpid     模糊 PID 结构体指针
 * @param  ke       误差量化因子
 * @param  kec      误差变化率量化因子
 * @param  kp_delta ΔKp 幅度
 * @param  ki_delta ΔKi 幅度
 * @param  kd_delta ΔKd 幅度
 */
void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid,
                       float ke, float kec,
                       float kp_delta, float ki_delta, float kd_delta);

/**
 * @brief  载入默认模糊规则表
 * @param  fpid  模糊 PID 结构体指针
 */
void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid);

/**
 * @brief  模糊 PID 计算
 * @param  fpid    模糊 PID 结构体指针
 * @param  target  目标值
 * @param  measure 测量值
 * @return PID 输出
 */
float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure);

#endif
