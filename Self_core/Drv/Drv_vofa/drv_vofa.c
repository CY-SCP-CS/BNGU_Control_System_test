/**
 * @file    drv_vofa.c
 * @brief   VOFA+ 协议实现 (纯数据层)
 */
#include "drv_vofa.h"
#include <string.h>

/* VOFA JustFloat 使用 IEEE-754 binary32。目标 STM32 工具链必须满足该协议。 */
typedef char drv_vofa_requires_32_bit_float[(sizeof(float) == 4U) ? 1 : -1];

// ─── 接口实现 ─────────────────────────────────────

drv_vofa_status_t drv_vofa_pack(const float data[DRV_VOFA_CHANNEL_COUNT],
                                uint8_t frame[DRV_VOFA_FRAME_SIZE])
{
    return drv_vofa_pack_channels(data, DRV_VOFA_CHANNEL_COUNT, frame);
}

drv_vofa_status_t drv_vofa_pack_channels(const float *data, size_t channel_count, uint8_t *frame)
{
    static const uint8_t tail[DRV_VOFA_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7fU};

    if (!data || !frame || channel_count == 0U || channel_count > DRV_VOFA_CHANNEL_COUNT) {
        return DRV_VOFA_ERR_ARGUMENT;
    }

    const size_t data_size = channel_count * sizeof(*data);
    memcpy(frame, data, data_size);
    memcpy(&frame[data_size], tail, sizeof(tail));
    return DRV_VOFA_OK;
}
