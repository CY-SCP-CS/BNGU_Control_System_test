/**
 * @file    drv_vofa.c
 * @brief   VOFA+ 协议实现 (纯数据层)
 */
#include "drv_vofa.h"
#include <string.h>

// ─── 私有变量 ────────────────────────────────────

static uint8_t          s_ch_count;
static volatile uint8_t s_busy;     /* 1 = 上次发送未完成 */

// ─── 接口实现 ─────────────────────────────────────

void drv_vofa_init(uint8_t ch_count)
{
    if (ch_count > DRV_VOFA_CH_MAX) ch_count = DRV_VOFA_CH_MAX;
    if (ch_count < 1)              ch_count = 1;

    s_ch_count = ch_count;
    s_busy     = 0;
}

int drv_vofa_pack(float *fdata, uint8_t *buf, uint16_t *len)
{
    uint16_t data_len;

    /* 上一帧还在发送中 → 直接丢弃新帧 (不会阻塞主循环) */
    if (s_busy) return -1;

    data_len = s_ch_count * 4;

    /* 打包浮点数据 (小端) */
    memcpy(buf, fdata, data_len);

    /* 附加帧尾 {0x00, 0x00, 0x80, 0x7f} */
    buf[data_len]     = 0x00;
    buf[data_len + 1] = 0x00;
    buf[data_len + 2] = 0x80;
    buf[data_len + 3] = 0x7f;

    *len = data_len + 4;
    s_busy = 1;
    return 0;
}

void drv_vofa_tx_complete(void)
{
    s_busy = 0;
}