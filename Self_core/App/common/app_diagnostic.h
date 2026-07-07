/**
 * @file    app_diagnostic.h
 * @brief   通用设备注册表 — 统一管理所有设备在线状态, 驱动 LED/蜂鸣器告警
 * @note    任何设备注册后, 由本模块统一管理心跳和超时判断,
 *          成为在线状态的唯一数据源。不涉及具体设备的数据解析。
 */
#ifndef APP_DIAGNOSTIC_H
#define APP_DIAGNOSTIC_H

#include "lib_typedef.h"

// ─── 注册表 ──────────────────────────────────────

#define APP_DIAGNOSTIC_MAX_DEVICES      16

typedef enum {
    APP_DIAGNOSTIC_DEVICE_MOTOR = 0,
    APP_DIAGNOSTIC_DEVICE_POWER = 1,
    APP_DIAGNOSTIC_DEVICE_IMU   = 2,
    APP_DIAGNOSTIC_DEVICE_DBUS  = 3,
    APP_DIAGNOSTIC_DEVICE_COUNT
} app_diagnostic_device_type_t;

typedef struct {
    app_diagnostic_device_type_t type;
    uint8_t                      index;
    uint8_t                      online;
    uint8_t                      check_ok;
} app_diagnostic_device_result_t;

typedef struct {
    app_diagnostic_device_result_t devices[APP_DIAGNOSTIC_MAX_DEVICES];
    uint8_t                        device_count;
    uint8_t                        all_online;
} app_diagnostic_result_t;

void     app_diagnostic_init(void);
int      app_diagnostic_register(app_diagnostic_device_type_t type,
                                 uint8_t index, uint32_t timeout_ms);
void     app_diagnostic_heartbeat(app_diagnostic_device_type_t type,
                                  uint8_t index);
uint8_t  app_diagnostic_is_online(app_diagnostic_device_type_t type,
                                  uint8_t index);
void     app_diagnostic_update(app_diagnostic_result_t *result);

#endif
