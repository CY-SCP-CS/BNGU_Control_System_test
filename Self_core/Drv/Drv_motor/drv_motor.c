/**
 * @file    drv_motor.c
 * @brief   电机协议层实现
 */
#include "drv_motor.h"

#include <stdint.h>
#include <string.h>

// ─── 字节打包辅助 ───────────────────────────────

#define HI_BYTE(x)  ((uint8_t)((x) >> 8))
#define LO_BYTE(x)  ((uint8_t)(x))

// ─── 接口实现 ─────────────────────────────────────

void drv_motor_solve_dji_data(const uint8_t *data, drv_motor_data_t *cur)
{
    /* DJI 回传协议: 大端 */
    cur->angle       = (uint16_t)data[0] << 8 | data[1];
    cur->speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    cur->current     = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    cur->temperature = data[6];
}

void drv_motor_build_dji_frame_init(uint8_t *frame)
{
    memset(frame, 0, 8);
}

void drv_motor_build_dji_frame_set(uint8_t *frame, uint8_t slot,
                                   int16_t current)
{
    if (slot >= DRV_MOTOR_DJI_FRAME_MAX) return;
    frame[slot * 2]     = HI_BYTE(current);
    frame[slot * 2 + 1] = LO_BYTE(current);
}

void drv_motor_solve_lk_data(const uint8_t *data, drv_motor_data_t *cur)
{
    /* 翎控回传协议: 小端 */
    cur->cmd_id      = data[0];
    cur->temperature = data[1];
    cur->current     = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8);
    cur->speed       = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
    cur->angle       = (uint16_t)data[6] | (uint16_t)data[7] << 8;
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
    frame[4] = LO_BYTE(current);
    frame[5] = HI_BYTE(current);
}
