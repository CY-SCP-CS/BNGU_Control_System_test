/**
 * @file    drv_power_measure_port.c
 * @brief   BSP 适配 — drv_power_measure 的硬件端口挂接
 */
#include "drv_power_measure.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_can.h"

void drv_power_port_can_init(void *hcan,
                              void (*rx_cb)(uint32_t, uint8_t*, uint8_t))
{
    bsp_can_rx_reg((CAN_HandleTypeDef *)hcan,
                                 DRV_POWER_CAN_ID,
                                 (bsp_can_rx_callback_t)rx_cb);
}
