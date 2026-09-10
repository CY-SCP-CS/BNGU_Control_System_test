/**
 * @file    bsp_can.h
 * @brief   CAN 驱动: 初始化 / 发送 / 回调订阅
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_cfg.h"

// ─── 回调注册 ────────────────────────────────────

/* 每总线回调上限 (CAN1/CAN2 各自独立, 互不占用) */
#define BSP_CAN_RX_CALLBACK_MAX  16

typedef void (*bsp_can_rx_callback_t)(uint32_t std_id, uint8_t *data, uint8_t len);

// ─── 发送状态 ────────────────────────────────────

typedef enum {
    BSP_CAN_TX_OK    = 0,
    BSP_CAN_TX_BUSY  = 1,           /* 三个 mailbox 全满 */
    BSP_CAN_TX_ERROR = 2
} bsp_can_tx_status_t;

// ─── 接口声明 ─────────────────────────────────────

/**
 * @brief  启动 CAN (配置滤波器 + 开启中断)
 * @param  hcan               CAN 句柄
 * @param  filter_bank        滤波器组号
 * @param  slave_filter_bank  CAN2 滤波器起始组号 (仅双 CAN 时 CAN1 需要, 单 CAN 传 0)
 * @return HAL_StatusTypeDef
 *///当时没有考虑过滤器配置，暂时不加入
HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank,
                                uint8_t slave_filter_bank);

/**
 * @brief  发送 CAN 帧 (非阻塞, 固定 8 字节)
 * @param  hcan   CAN 句柄
 * @param  std_id 标准 ID
 * @param  data   数据 (8 字节)
 * @return 发送状态
 */
bsp_can_tx_status_t bsp_can_send(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                 uint8_t data[8]);

/**
 * @brief  注册 CAN 接收回调 (中断中直接分发)
 * @param  hcan     CAN 句柄
 * @param  std_id   要监听的标准 ID
 * @param  callback 回调函数
 * @note   用法示例:
 *         static void on_motor_rx(uint32_t std_id, uint8_t *data, uint8_t len) {
 *             if (std_id == 0x201) drv_motor_solve_dji_data(data, &s_motor);
 *         }
 *         bsp_can_register_rx_callback(&hcan2, 0x201, on_motor_rx);
 *///回调函数里面别加delay，做个解报就够了
void bsp_can_register_rx_callback(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                  bsp_can_rx_callback_t callback);


/**
 * @brief  CAN 接收中断入口 (在 HAL 回调中调用)
 * @param  hcan   CAN 句柄
 */
void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan);

#endif