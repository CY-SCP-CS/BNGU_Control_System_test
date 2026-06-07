/**
 * @file    drv_power_measure.c
 * @brief   功率计数据解析实现
 */
#include "drv_power_measure.h"

void drv_power_solve(const uint8_t can_data[8], drv_power_data_t *data)
{
    data->bat_v = (int16_t)((can_data[1] << 8) | can_data[0]);
    data->bat_i = (int16_t)((can_data[3] << 8) | can_data[2]);
    data->cap_v = (int16_t)((can_data[5] << 8) | can_data[4]);
    data->ch_i  = (int16_t)((can_data[7] << 8) | can_data[6]);

    data->power = (data->bat_v / 100.0f) * (data->bat_i / 100.0f);
}
