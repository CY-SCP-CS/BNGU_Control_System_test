/**
 * @file    drv_super_cap_port.c
 * @brief   超级电容驱动的 BSP CAN 端口适配。
 */
#include "drv_super_cap.h"
#include "bsp_can.h"

void drv_super_cap_port_can_init(void *hcan,
                                 void (*rx_cb)(uint32_t, uint8_t *, uint8_t))
{
    if (!hcan || !rx_cb) {
        return;
    }
    bsp_can_rx_reg((CAN_HandleTypeDef *)hcan, DRV_SUPER_CAP_CAN_ID_FEEDBACK, rx_cb);
}

uint8_t drv_super_cap_port_can_tx(void *hcan, const drv_super_cap_control_t *control)
{
    uint8_t frame[8];
    if (!hcan || !drv_super_cap_pack(control, frame)) {
        return 0U;
    }
    return (uint8_t)(bsp_can_tx((CAN_HandleTypeDef *)hcan,
                              DRV_SUPER_CAP_CAN_ID_CONTROL, frame) == 0U);
}
