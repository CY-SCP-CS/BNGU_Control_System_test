/**
 * @file    drv_super_cap.c
 * @brief   超级电容协议解析与控制帧构建。
 */
#include "drv_super_cap.h"

#include <math.h>
#include <string.h>

/** 读取小端序的 16 位无符号数。 */
static uint16_t super_cap_read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

void drv_super_cap_init(drv_super_cap_t *super_cap)
{
    if (super_cap) {
        memset(super_cap, 0, sizeof(*super_cap));
    }
}

uint8_t drv_super_cap_solve(drv_super_cap_t *super_cap, uint32_t std_id,
                            const uint8_t *data, uint8_t len, uint32_t now_ms)
{
    if (!super_cap || !data || std_id != DRV_SUPER_CAP_CAN_ID_FEEDBACK || len != 8U) {
        return 0U;
    }

    uint16_t raw_current = super_cap_read_u16(data + 2);
    int32_t signed_current = raw_current;
    if ((raw_current & 0x8000U) != 0U) {
        signed_current -= 65536;
    }

    super_cap->feedback.battery_voltage = (float)super_cap_read_u16(data) * 0.01f;
    super_cap->feedback.battery_current = (float)signed_current * 0.01f;
    super_cap->feedback.cap_voltage = (float)super_cap_read_u16(data + 4) * 0.01f;
    super_cap->feedback.max_output_power = (float)super_cap_read_u16(data + 6) * 0.01f;
    super_cap->rx_tick_ms = now_ms;
    super_cap->has_feedback = 1U;
    return 1U;
}

uint8_t drv_super_cap_pack(const drv_super_cap_control_t *control, uint8_t frame[8])
{
    if (!control || !frame || !isfinite(control->power_limit)
        || control->power_limit < 0.0f || control->power_limit > 655.35f
        || control->enabled > 1U) {
        return 0U;
    }

    uint16_t power_limit = (uint16_t)(control->power_limit * 100.0f + 0.5f);
    uint8_t discharge_strategy = control->discharge_strategy;
    uint8_t disable = control->enabled ? 0U : 1U;
    memset(frame, 0, 8U);
    frame[0] = LIB_LO_BYTE(power_limit);
    frame[1] = LIB_HI_BYTE(power_limit);
    frame[2] = discharge_strategy;
    frame[4] = disable;
    return 1U;
}

uint8_t drv_super_cap_is_online(const drv_super_cap_t *super_cap,
                                uint32_t now_ms, uint32_t timeout_ms)
{
    if (!super_cap || !super_cap->has_feedback || timeout_ms == 0U) {
        return 0U;
    }
    return (uint8_t)((uint32_t)(now_ms - super_cap->rx_tick_ms) < timeout_ms);
}
