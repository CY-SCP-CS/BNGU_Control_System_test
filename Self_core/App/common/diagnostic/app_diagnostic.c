/**
 * @file    app_diagnostic.c
 * @brief   通用设备注册表实现 — 心跳跟踪 / 超时判断 / LED/蜂鸣器 告警。
 */
#include "app_diagnostic.h"

#include "drv_led.h"
#include "drv_buzzer.h"
#include "drv_motor.h"
#include "bsp_cfg.h"

#include <string.h>

// === 私有类型 ===

typedef struct {
    app_diagnostic_device_type_t type;
    uint8_t                      index;
    uint32_t                     timeout_ms;
    uint32_t                     last_heartbeat;
    uint8_t                      online;
    uint8_t                      used;
    uint8_t                      consecutive_offline;
} app_diagnostic_entry_t;

// === 私有宏 ===

#define APP_DIAGNOSTIC_ALERT_BLINK_MS   250
#define APP_DIAGNOSTIC_DEBOUNCE_COUNT  3   /* 连续超时次数 >= 此值才判离线，防 CAN 抖动 */



// === 私有变量 ===

static app_diagnostic_entry_t s_diagnostic_registry[APP_DIAGNOSTIC_MAX_DEVICES];
static uint8_t                s_diagnostic_device_count;
static uint32_t               s_diagnostic_last_blink;
static uint8_t                s_diagnostic_prev_all_online = 1;
static uint8_t                s_diagnostic_blink_state;

// === 私有函数声明 ===

static app_diagnostic_entry_t *app_diagnostic_find_entry(
    app_diagnostic_device_type_t type, uint8_t index);

// === 公有接口 ===

void app_diagnostic_init(void)
{
    memset(s_diagnostic_registry, 0, sizeof(s_diagnostic_registry));
    s_diagnostic_device_count    = 0;
    s_diagnostic_last_blink      = HAL_GetTick();
    s_diagnostic_prev_all_online = 1;
    s_diagnostic_blink_state     = 0;
}

int app_diagnostic_register(app_diagnostic_device_type_t type,
                            uint8_t index, uint32_t timeout_ms)
{
    uint8_t i;

    /* 1. 若该 (type,index) 已注册 → 直接返回成功, 避免重复占槽 (D2) */
    for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
        if (s_diagnostic_registry[i].used
            && s_diagnostic_registry[i].type == type
            && s_diagnostic_registry[i].index == index) {
            s_diagnostic_registry[i].timeout_ms = timeout_ms;
            return 0;
        }
    }

    /* 2. 找空闲槽注册 */
    for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
        if (!s_diagnostic_registry[i].used) {
            s_diagnostic_registry[i].type           = type;
            s_diagnostic_registry[i].index          = index;
            s_diagnostic_registry[i].timeout_ms     = timeout_ms;
            s_diagnostic_registry[i].last_heartbeat = HAL_GetTick();
            s_diagnostic_registry[i].online         = 1;
            s_diagnostic_registry[i].used           = 1;
            s_diagnostic_device_count++;
            return 0;
        }
    }

    return -1;
}

void app_diagnostic_heartbeat(app_diagnostic_device_type_t type,
                              uint8_t index)
{
    app_diagnostic_entry_t *entry = app_diagnostic_find_entry(type, index);
    if (entry) {
        entry->last_heartbeat = HAL_GetTick();
        entry->online         = 1;
    }
}

uint8_t app_diagnostic_is_online(app_diagnostic_device_type_t type,
                                 uint8_t index)
{
    app_diagnostic_entry_t *entry = app_diagnostic_find_entry(type, index);
    return entry ? entry->online : 0;
}

void app_diagnostic_update(app_diagnostic_result_t *result)
{
    uint8_t   i;
    uint8_t   all_online = 1;
    uint32_t  now        = HAL_GetTick();

    for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
        if (!s_diagnostic_registry[i].used) continue;
        if (now - s_diagnostic_registry[i].last_heartbeat
            > s_diagnostic_registry[i].timeout_ms) {
            s_diagnostic_registry[i].consecutive_offline++;
        } else {
            s_diagnostic_registry[i].consecutive_offline = 0;
        }
        /* 连续超时达到阈值才判离线（防抖） */
        if (s_diagnostic_registry[i].consecutive_offline >= APP_DIAGNOSTIC_DEBOUNCE_COUNT) {
            s_diagnostic_registry[i].online = 0;
        }
    }

    if (result) {
        uint8_t count = 0;
        for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
            if (!s_diagnostic_registry[i].used) continue;
            result->devices[count].type     = s_diagnostic_registry[i].type;
            result->devices[count].index    = s_diagnostic_registry[i].index;
            result->devices[count].online   = s_diagnostic_registry[i].online;
            result->devices[count].check_ok = 1;
            count++;
        }
        result->device_count = count;
    }

    for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
        if (s_diagnostic_registry[i].used
            && !s_diagnostic_registry[i].online) {
            all_online = 0;
            break;
        }
    }

    if (result) {
        result->all_online = all_online;
    }

    if (all_online) {
        if (!s_diagnostic_prev_all_online) {
            drv_led_rgb(0, 1, 0);
            drv_buzzer_off();
            s_diagnostic_blink_state = 0;
        }
    } else {
        if (s_diagnostic_prev_all_online) {
            drv_led_rgb(1, 0, 0);
            drv_buzzer_on();
            s_diagnostic_blink_state = 1;
            s_diagnostic_last_blink  = now;
        } else {
            if (now - s_diagnostic_last_blink
                >= APP_DIAGNOSTIC_ALERT_BLINK_MS) {
                s_diagnostic_last_blink = now;
                s_diagnostic_blink_state = !s_diagnostic_blink_state;
                if (s_diagnostic_blink_state) {
                    drv_led_rgb(1, 0, 0);
                    drv_buzzer_on();
                } else {
                    drv_led_rgb(0, 0, 0);
                    drv_buzzer_off();
                }
            }
        }
    }

    s_diagnostic_prev_all_online = all_online;
}

// === 私有函数定义 ===

static app_diagnostic_entry_t *app_diagnostic_find_entry(
    app_diagnostic_device_type_t type, uint8_t index)
{
    uint8_t i;

    for (i = 0; i < APP_DIAGNOSTIC_MAX_DEVICES; i++) {
        if (s_diagnostic_registry[i].used
            && s_diagnostic_registry[i].type == type
            && s_diagnostic_registry[i].index == index) {
            return &s_diagnostic_registry[i];
        }
    }

    return NULL;
}

