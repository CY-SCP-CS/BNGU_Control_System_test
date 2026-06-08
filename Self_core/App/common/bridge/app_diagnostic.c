/**
 * @file    app_diagnostic.c
 * @brief   离线检测汇总实现
 */
#include "app_diagnostic.h"

#include "app_motor.h"
#include "app_power_measure.h"

#include "app_led.h"
#include "app_buzzer.h"

// ─── 告警参数 ────────────────────────────────────

#define ALERT_BLINK_MS   250     /* 告警闪烁间隔 (ms) */

// ─── 私有变量 ────────────────────────────────────

static uint32_t s_last_blink;
static uint8_t  s_prev_all_online = 1;   /* 上一拍全在线 */
static uint8_t  s_blink_state;           /* 用于蜂鸣器间歇 */

// ─── 接口实现 ─────────────────────────────────────

void app_diagnostic_init(void)
{
    s_last_blink      = HAL_GetTick();
    s_prev_all_online = 1;
    s_blink_state     = 0;
}

void app_diagnostic_update(app_diagnostic_result_t *result)
{
    uint8_t   i;
    uint8_t   all_online = 1;
    uint32_t  now        = HAL_GetTick();

    /* 刷新各个模块的在线状态 */
    app_motor_refresh_online();
    app_power_measure_refresh_online();

    /* 汇总结果 */
    if (result) {
        result->motor_count = app_motor_get_count();
        result->power_online = app_power_measure_is_online();

        for (i = 0; i < result->motor_count; i++) {
            result->motor_online[i] = app_motor_is_online(i);
        }
    }

    /* 检查全在线 */
    if (!app_power_measure_is_online()) all_online = 0;

    for (i = 0; i < app_motor_get_count(); i++) {
        if (!app_motor_is_online(i)) {
            all_online = 0;
            break;
        }
    }

    if (result) {
        result->all_online = all_online;
    }

    /* ── 状态切换时的动作 ── */
    if (all_online) {
        if (!s_prev_all_online) {
            /* 刚恢复 → 灭灯, 关蜂鸣器 */
            app_led_rgb(0, 1, 0);   /* 绿灯 */
            app_buzzer_off();
            s_blink_state = 0;
        }
    } else {
        if (s_prev_all_online) {
            /* 刚掉线 → 立即亮红灯 + 蜂鸣器 */
            app_led_rgb(1, 0, 0);   /* 红灯 */
            app_buzzer_on();
            s_blink_state = 1;
            s_last_blink  = now;
        } else {
            /* 持续离线状态 → 闪烁 */
            if (now - s_last_blink >= ALERT_BLINK_MS) {
                s_last_blink = now;
                s_blink_state = !s_blink_state;

                if (s_blink_state) {
                    app_led_rgb(1, 0, 0);   /* 红灯亮 */
                    app_buzzer_on();
                } else {
                    app_led_rgb(0, 0, 0);   /* 全灭 */
                    app_buzzer_off();
                }
            }
        }
    }

    s_prev_all_online = all_online;
}
