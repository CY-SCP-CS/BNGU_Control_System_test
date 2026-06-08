/**
 * @file    app_power_measure.c
 * @brief   功率计管理桥接层实现
 */
#include "app_power_measure.h"

#include "bsp_can.h"

// ─── 私有变量 ────────────────────────────────────

static drv_power_data_t s_data;
static uint32_t         s_last_tick;
static uint8_t          s_online;
static uint8_t          s_inited;

// ─── CAN 接收回调 ────────────────────────────────

static void power_rx_callback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    (void)std_id;
    (void)len;
    drv_power_solve(data, &s_data);
    s_last_tick = HAL_GetTick();
    s_online    = 1;
}

// ─── 接口实现 ─────────────────────────────────────

void app_power_measure_init(CAN_HandleTypeDef *hcan)
{
    if (s_inited) return;
    s_inited = 1;

    bsp_can_register_rx_callback(hcan, DRV_POWER_CAN_ID, power_rx_callback);
}

int app_power_measure_get_data(drv_power_data_t *out)
{
    if (!out) return -1;

    *out = s_data;
    return s_online ? 0 : -1;
}

uint8_t app_power_measure_is_online(void)
{
    return s_online;
}

void app_power_measure_refresh_online(void)
{
    if (!s_inited) return;

    if (HAL_GetTick() - s_last_tick > APP_POWER_TIMEOUT_MS) {
        s_online = 0;
    }
}
