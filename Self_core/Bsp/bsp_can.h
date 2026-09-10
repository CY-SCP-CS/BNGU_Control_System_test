/**
 * @file    bsp_can.h
 * @brief   CAN 驱动: 初始化 / 发送 / 回调订阅
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_cfg.h"

#define BSP_CAN_RX_CALLBACK_MAX  16//CAN1/CAN2最大回调注册数量

typedef void (*bsp_can_rx_callback_t)(uint32_t std_id, uint8_t *data, uint8_t len);//CAN接收回调函数指针


typedef enum {
    BSP_CAN_TX_OK    = 0,
    BSP_CAN_TX_BUSY  = 1,
    BSP_CAN_TX_ERROR = 2
} bsp_can_tx_status_t;//CAN发送状态


/**
 * @brief  CAN 初始化
 * @param  hcan               CAN 句柄
 * @param  filter_bank        滤波器组号
 * @param  slave_filter_bank  CAN2 滤波器起始组号 (仅双 CAN 时 CAN1 需要, 单 CAN 传 0)
 * @return HAL_StatusTypeDef
 * @note  滤波器用 16-bit IDMASK 模式, 接收所有 ID，还没留接口，感觉不太需要
 */
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
    * @note   同一总线同 ID 只允许注册一个回调, 重复注册会覆盖旧回调
    * @note   回调函数在中断中执行, 尽量短小, 避免阻塞
 */
void bsp_can_register_rx_callback(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                  bsp_can_rx_callback_t callback);


/**
 * @brief  CAN 接收中断入口 (在 HAL 回调中调用)
 * @param  hcan   CAN 句柄
 */
void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan);

#endif