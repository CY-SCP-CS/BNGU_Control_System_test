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

// ─── 固定协议布局 ─────────────────────────────────

#define DRV_VOFA_CHANNEL_COUNT  9U
#define DRV_VOFA_TAIL_SIZE      4U
#define DRV_VOFA_DATA_SIZE      (DRV_VOFA_CHANNEL_COUNT * sizeof(float))
#define DRV_VOFA_FRAME_SIZE     (DRV_VOFA_DATA_SIZE + DRV_VOFA_TAIL_SIZE)

typedef enum {
    DRV_VOFA_OK = 0,
    DRV_VOFA_ERR_BUSY = -1,
    DRV_VOFA_ERR_ARGUMENT = -2,
    DRV_VOFA_ERR_UART = -3
} drv_vofa_status_t;

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  打包一帧数据
 * @param  data  至少包含 DRV_VOFA_CHANNEL_COUNT 个 float 的输入数组
 * @param  frame 至少 DRV_VOFA_FRAME_SIZE 字节的输出数组
 * @return DRV_VOFA_OK=成功，参数为空时返回 DRV_VOFA_ERR_ARGUMENT
 * @note   纯打包接口；异步发送状态由 port 层管理
 */
drv_vofa_status_t drv_vofa_pack(const float data[DRV_VOFA_CHANNEL_COUNT],
                                uint8_t frame[DRV_VOFA_FRAME_SIZE]);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  初始化 VOFA (BSP 适配版)
 */
void drv_vofa_port_init(void);

/**
 * @brief  打包并通过 UART1 非阻塞发送一帧 VOFA 数据
 * @param  data 至少包含 DRV_VOFA_CHANNEL_COUNT 个 float 的输入数组
 * @return DRV_VOFA_OK=已启动发送；DRV_VOFA_ERR_BUSY=上一帧仍在发送
 *         其他返回值表示参数或 UART 错误
 * @note   发送缓冲区由 port 层持有，直到 UART 发送完成
 */
drv_vofa_status_t drv_vofa_port_send(const float data[DRV_VOFA_CHANNEL_COUNT]);

#endif
