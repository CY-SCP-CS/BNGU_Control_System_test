/**
 * @file    drv_motor.c
 * @brief   电机协议层实现
 */
#include "drv_motor.h"

#include <string.h>

// ─── 接口实现 ─────────────────────────────────────

void drv_motor_solve_dji_data(const uint8_t *data, drv_motor_data_t *cur)
{
    cur->angle       = (uint16_t)data[0] << 8 | data[1];
    cur->speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    cur->current     = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    cur->temperature = data[6];//大端序
}

void drv_motor_build_dji_frame_init(uint8_t *frame)
{
    memset(frame, 0, 8);
}

void drv_motor_build_dji_frame_set(uint8_t *frame, uint8_t slot,
                                   int16_t current)
{
    if (slot >= DRV_MOTOR_DJI_FRAME_MAX) return;
    frame[slot * 2]     = LIB_HI_BYTE(current);
    frame[slot * 2 + 1] = LIB_LO_BYTE(current);
}

void drv_motor_solve_lk_data(const uint8_t *data, drv_motor_data_t *cur)
{
    cur->cmd_id      = data[0];
    cur->temperature = data[1];
    cur->current     = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8);
    cur->speed       = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
    cur->angle       = (uint16_t)data[6] | (uint16_t)data[7] << 8;//小端序
}

void drv_motor_build_lk_read_frame(uint8_t *frame)
{
    memset(frame, 0, 8);
    frame[0] = 0x9C;
}

void drv_motor_build_lk_frame(uint8_t *frame, int16_t current)
{
    memset(frame, 0, 8);
    frame[0] = 0xA1;
    frame[4] = LIB_LO_BYTE(current);
    frame[5] = LIB_HI_BYTE(current);
}