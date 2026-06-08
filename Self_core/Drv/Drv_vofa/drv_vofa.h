/**
 * @file    drv_vofa.h
 * @brief   VOFA+ 上位机数据协议 (纯数据层)
 * @note    协议: float[CH_COUNT] + tail{0x00,0x00,0x80,0x7f}
 *          帧尾为 +inf 的 IEEE 754 小端编码, VOFA 据此切帧
 *          本模块只负责数据打包, 不关心具体发送途径
 */
#ifndef DRV_VOFA_H
#define DRV_VOFA_H

#include "lib_typedef.h"

// ─── 默认与上限 ──────────────────────────────────

#define DRV_VOFA_CH_MAX     32      /* 最多 32 通道浮点 */
#define DRV_VOFA_TAIL       {0x00, 0x00, 0x80, 0x7f}

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  初始化 VOFA 驱动
 * @param  ch_count  通道数 (每帧发送的 float 个数)
 */
void drv_vofa_init(uint8_t ch_count);

/**
 * @brief  打包一帧数据
 * @param  fdata  待发送的浮点数组 (需 >= ch_count 个元素)
 * @param  buf    输出缓冲区 (需 >= ch_count * 4 + 4 字节)
 * @param  len    输出帧长度 (字节)
 * @return 0=打包成功, -1=上次发送未完成 (帧被丢弃)
 * @note   返回 0 时 buf/len 有效, 调用者负责将 buf 发送出去
 */
int drv_vofa_pack(float *fdata, uint8_t *buf, uint16_t *len);

/**
 * @brief  发送完成通知 (由外部中断入口调用)
 * @note   释放忙标志, 允许发送下一帧
 */
void drv_vofa_tx_complete(void);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  初始化 VOFA (BSP 适配版)
 * @param  huart     UART 句柄 (void*)
 * @param  tx_cb     发送完成回调
 * @param  ch_count  通道数
 */
void drv_vofa_port_init(void *huart, void (*tx_cb)(void), uint8_t ch_count);

/**
 * @brief  UART 发送 (非阻塞)
 * @param  huart  UART 句柄 (void*)
 * @param  data   数据缓冲区
 * @param  len    数据长度
 */
void drv_vofa_port_send(void *huart, uint8_t *data, uint16_t len);

#endif
