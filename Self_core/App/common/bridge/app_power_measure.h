/**
 * @file    app_power_measure.h
 * @brief   功率计管理桥接层 — CAN 回调分发 / 在线检测
 * @note    功率计以 CAN ID 0x212 周期性广播, 本模块被动监听,
 *          超过 APP_POWER_TIMEOUT_MS 未收到数据即判离线
 */
#ifndef APP_POWER_MEASURE_H
#define APP_POWER_MEASURE_H

#include "lib_typedef.h"
#include "bsp_cfg.h"
#include "drv_power_measure.h"

#define APP_POWER_TIMEOUT_MS    100     /* 功率计 ~100Hz, 100ms 超时 */

/**
 * @brief  初始化功率计管理
 * @param  hcan  CAN 句柄 (接收此 CAN 上的 0x212 帧)
 */
void app_power_measure_init(CAN_HandleTypeDef *hcan);

/**
 * @brief  获取功率计数据
 * @param  out  输出缓冲区
 * @return 0=成功, -1=无数据 (离线)
 */
int app_power_measure_get_data(drv_power_data_t *out);

/**
 * @brief  查询功率计在线状态
 * @return 0=离线, 1=在线
 */
uint8_t app_power_measure_is_online(void);

/**
 * @brief  更新在线状态 (需周期性调用)
 */
void app_power_measure_refresh_online(void);

#endif
