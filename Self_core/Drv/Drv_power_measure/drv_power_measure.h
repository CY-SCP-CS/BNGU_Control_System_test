/**
 * @file    drv_power_measure.h
 * @brief   功率计数据解析 (CAN ID 0x212, 只收不发)
 * @note    纯数据层, 不涉及 CAN 收发硬件
 */
#ifndef DRV_POWER_MEASURE_H
#define DRV_POWER_MEASURE_H

#include "lib_typedef.h"

// ─── 功率计数据 ─────────────────────────────────

#define DRV_POWER_CAN_ID    0x212

typedef struct {
    int16_t bat_v;      /* 电池电压 (0.01V)   */
    int16_t bat_i;      /* 电池电流 (0.01A)   */
    int16_t cap_v;      /* 电容电压 (0.01V)   */
    int16_t ch_i;       /* 通道电流 (0.01A)   */
    float   power;      /* 功率 (W)           */
} drv_power_data_t;

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  解析功率计 CAN 数据
 * @param  can_data  CAN 数据场 (8 字节)
 * @param  data      输出数据结构体
 */
void drv_power_solve(const uint8_t can_data[8], drv_power_data_t *data);

#endif
