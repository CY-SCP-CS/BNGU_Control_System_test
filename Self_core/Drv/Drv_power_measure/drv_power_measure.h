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
    uint16_t bat_v;     /* 电池电压 (0.01V)                   */
    uint16_t bat_i;     /* 电池电流 (0.01A)                   */
    uint16_t reserved0; /* 当前功率计固件固定发送 0xFFFF      */
    uint16_t reserved1; /* 当前功率计固件固定发送 0xFFFF      */
    float    power;     /* 电池侧实测功率 (W), bat_v * bat_i */
} drv_power_data_t;

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  解析功率计 CAN 数据
 * @param  can_data  CAN 数据场 (8 字节)
 * @param  data      输出数据结构体
 */
void drv_power_solve(const uint8_t can_data[8], drv_power_data_t *data);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  CAN 初始化 (注册接收回调)
 * @param  hcan   CAN 句柄 (void*)
 * @param  rx_cb  接收回调
 */
void drv_power_port_can_init(void *hcan,
                              void (*rx_cb)(uint32_t, uint8_t*, uint8_t));

#endif
