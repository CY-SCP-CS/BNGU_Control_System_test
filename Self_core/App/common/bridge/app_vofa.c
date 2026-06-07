/**
 * @file    app_vofa.c
 * @brief   VOFA+ 上层胶水 — 串联 BSP UART 与 DRV VOFA
 */
#include "app_vofa.h"

#include "drv_vofa.h"

// ─── 私有变量 ────────────────────────────────────

static UART_HandleTypeDef *s_huart;
static uint8_t             s_buf[DRV_VOFA_CH_MAX * 4 + 4];

// ─── 接口实现 ─────────────────────────────────────

void app_vofa_init(UART_HandleTypeDef *huart, uint8_t ch_count)
{
    s_huart = huart;

    drv_vofa_init(ch_count);

    /* 注册发送完成回调 → 自动释放忙标志 */
    bsp_uart_register_tx_callback(huart, drv_vofa_tx_complete);
}

void app_vofa_send(float *fdata)
{
    uint16_t len;

    if (drv_vofa_pack(fdata, s_buf, &len) != 0)
        return;     /* 忙, 丢弃 */

    bsp_uart_send(s_huart, s_buf, len);
}
