/**
 * @file    drv_motor_port.c
 * @brief   BSP 适配 — drv_motor 的硬件端口挂接
 */
#include "drv_motor.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_cfg.h"
#include "bsp_can.h"

void drv_motor_port_can_init(void *hcan, uint32_t can_id,
                              void (*rx_cb)(uint32_t, uint8_t*, uint8_t))
{
    bsp_can_register_rx_callback((CAN_HandleTypeDef *)hcan, can_id,
                                 (bsp_can_rx_callback_t)rx_cb);
}

void drv_motor_port_can_send(void *hcan, uint32_t std_id, uint8_t *data)
{
    bsp_can_send((CAN_HandleTypeDef *)hcan, std_id, data);
}

uint32_t drv_motor_port_get_tick(void)
{
    return HAL_GetTick();
}
