/**
 * @file    app_diagnostic.h
 * @brief   离线检测汇总 — 周期性检查所有设备在线状态, 触发 LED/蜂鸣器告警
 * @note    本模块汇总 app_motor / app_power_measure 的在线状态,
 *          设备离线时亮红灯 + 蜂鸣器间歇鸣叫; 全在线时亮绿灯
 */
#ifndef APP_DIAGNOSTIC_H
#define APP_DIAGNOSTIC_H

#include "lib_typedef.h"

/**
 * @brief  诊断结果结构体
 */
typedef struct {
    uint8_t motor_online[8];    /* 各电机在线标志 */
    uint8_t motor_count;        /* 电机总数        */
    uint8_t power_online;       /* 功率计在线标志   */
    uint8_t all_online;         /* 全在线 = 1      */
} app_diagnostic_result_t;

/**
 * @brief  初始化诊断模块
 * @note   需在 app_motor_init / app_power_measure_init 之后调用
 */
void app_diagnostic_init(void);

/**
 * @brief  更新诊断 (需周期性调用, 建议 20~50ms)
 * @param  result  输出当前诊断结果, 可传 NULL
 * @note   内部自动调用 app_motor_refresh_online 和
 *         app_power_measure_refresh_online;
 *         设备离线时驱动 LED 红灯 + 蜂鸣器间歇提示
 */
void app_diagnostic_update(app_diagnostic_result_t *result);

#endif
