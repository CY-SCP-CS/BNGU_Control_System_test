//
// Created by 33140 on 2026/11/7.
//

#ifndef BNGU_CONTROL_SYSTEM_DRV_GAME_CONTROLLER_H
#define BNGU_CONTROL_SYSTEM_DRV_GAME_CONTROLLER_H

#include "Bsp/bsp_uart.h"
#include <stdint.h>

typedef struct {
    float lx;
    float ly;
    float rx;
    float ry;

    float lt;
    uint8_t lb;
    float rt;
    uint8_t rb;

    uint8_t a;
    uint8_t b;
    uint8_t x;
    uint8_t y;

    uint8_t up;
    uint8_t down;
    uint8_t left;
    uint8_t right;

    uint8_t view;   // 视图键，两个方块
    uint8_t menu;   // 菜单键，三条横线
    uint8_t share;  // 分享键，上传图标
    uint8_t xbox;
}drv_game_controller_buttons_t;

/**
 * @brief 第一版：解析 JOY,lx,ly,rx,ry 文本帧，后续协议与接口还会修改。
 * @param buffer 完整帧，以 '\0' 结束；四轴范围 [-1, 1]。
 * @param data 成功时仅更新 lx、ly、rx、ry，其余字段不变。
 * @return 1 成功；0 失败，原数据不变。
 */
uint8_t drv_game_controller_decode(const uint8_t *buffer, drv_game_controller_buttons_t *data);



// ─── Port: BSP 适配 ─────────────────────────────

/** @brief 绑定已配置 RX DMA 的串口并初始化状态；1 成功，0 失败。启动时调用一次。 */
uint8_t drv_game_controller_uart_init(UART_HandleTypeDef *huart);

/**
 * @brief 主循环周期调用，接收、拼帧并解码；不可重入。
 * @return 0 无新有效帧；1 至少解出一帧；2 接收错误（可能已更新数据）。
 * @note 接收失败后，后续调用会重试。需由串口 IRQ 调用 bsp_uart_irq_handler()。
 */
uint8_t drv_game_controller_uart_process(void);

/** @brief 复制最新快照；未收有效帧、接收已停止或超时返回 0，正常返回 1，out 保持不变。 */
uint8_t drv_game_controller_uart_read(drv_game_controller_buttons_t *out,
                                     uint32_t timeout_ms);

#endif // BNGU_CONTROL_SYSTEM_DRV_GAME_CONTROLLER_H
