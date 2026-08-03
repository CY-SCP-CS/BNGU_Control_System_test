/**
 * @file    drv_buzzer.c
 * @brief   蜂鸣器驱动实现 (纯逻辑层)
 */
#include "drv_buzzer.h"

// ─── 私有变量 ────────────────────────────────────

static drv_buzzer_set_fn_t  s_set;
static drv_buzzer_freq_fn_t s_set_freq;
static drv_buzzer_duty_fn_t s_set_duty;
static uint8_t              s_state;    /* 0=关, 1=开 */
static uint16_t             s_freq_hz;  /* 当前频率    */

// ─── 私有函数声明 ────────────────────────────────

static void buzzer_write(uint8_t state);

// ─── 接口实现 ─────────────────────────────────────

void drv_buzzer_init(drv_buzzer_set_fn_t set, drv_buzzer_freq_fn_t set_freq,
                     drv_buzzer_duty_fn_t set_duty)
{
    s_set      = set;
    s_set_freq = set_freq;
    s_set_duty = set_duty;
    s_state    = 0;
    s_freq_hz  = 0;

    buzzer_write(0);  /* 默认关闭 */
}

void drv_buzzer_on(void)
{
    buzzer_write(1);
}

void drv_buzzer_off(void)
{
    buzzer_write(0);
}

void drv_buzzer_toggle(void)
{
    buzzer_write(s_state ? 0 : 1);
}

void drv_buzzer_set_freq(uint16_t freq_hz)
{
    if (!s_set_freq) return;

    s_freq_hz = freq_hz;
    s_set_freq(freq_hz);
}

void drv_buzzer_set_duty(uint16_t duty)
{
    if (!s_set_duty) return;

    s_set_duty(duty);
}

// ─── 私有函数定义 ─────────────────────────────────

static void buzzer_write(uint8_t state)
{
    if (!s_set) return;

    s_state = state;
    s_set(state);
}