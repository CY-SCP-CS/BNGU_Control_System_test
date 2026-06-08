/**
 * @file    bsp_can.c
 * @brief   CAN 驱动实现: 初始化 / 发送 / 回调分发
 * @note    滤波器用 16-bit IDMASK 模式, 接收所有 ID
 *          所有接收走回调注册, 无轮询缓冲区
 */
#include "bsp_can.h"

// ─── 回调表 ──────────────────────────────────────

typedef struct {
    CAN_HandleTypeDef      *hcan;
    uint32_t                std_id;
    bsp_can_rx_callback_t   callback;
} bsp_can_callback_entry_t;

static bsp_can_callback_entry_t s_callbacks[BSP_CAN_RX_CALLBACK_MAX];
static uint8_t             s_can_callback_count;

// ─── 发送: 找空闲 mailbox ─────────────────────────

static uint32_t get_free_mbox(CAN_HandleTypeDef *hcan)
{
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0))
        return CAN_TX_MAILBOX0;
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX1))
        return CAN_TX_MAILBOX1;
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX2))
        return CAN_TX_MAILBOX2;
    return 0xFFFFFFFFu;
}

// ─── 接口实现 ─────────────────────────────────────

HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank)
{
    CAN_FilterTypeDef filter;
    HAL_StatusTypeDef can_status_return;

    filter.FilterActivation       = CAN_FILTER_ENABLE;
    filter.FilterBank             = filter_bank;
    filter.FilterMode             = CAN_FILTERMODE_IDMASK;
    filter.FilterScale            = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh           = 0x0000;
    filter.FilterIdLow            = 0x0000;
    filter.FilterMaskIdHigh       = 0x0000;
    filter.FilterMaskIdLow        = 0x0000;
    filter.FilterFIFOAssignment   = CAN_RX_FIFO0;
    filter.SlaveStartFilterBank   = 14;

    can_status_return = HAL_CAN_ConfigFilter(hcan, &filter);
    if (can_status_return != HAL_OK) return can_status_return;

    can_status_return = HAL_CAN_Start(hcan);
    if (can_status_return != HAL_OK) return can_status_return;

    can_status_return = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return can_status_return;
}

bsp_can_tx_status_t bsp_can_send(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                 uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_hdr;
    uint32_t            mbox;

    mbox = get_free_mbox(hcan);
    if (mbox == 0xFFFFFFFFu) return BSP_CAN_TX_BUSY;

    tx_hdr.StdId = std_id;
    tx_hdr.IDE   = CAN_ID_STD;
    tx_hdr.RTR   = CAN_RTR_DATA;
    tx_hdr.DLC   = 8;
    if (HAL_CAN_AddTxMessage(hcan, &tx_hdr, data, &mbox) != HAL_OK) {
        return BSP_CAN_TX_ERROR;
    }
    return BSP_CAN_TX_OK;
}

void bsp_can_register_rx_callback(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                  bsp_can_rx_callback_t callback)
{
    uint8_t i;

    if (s_can_callback_count >= BSP_CAN_RX_CALLBACK_MAX) return;

    for (i = 0; i < s_can_callback_count; i++) {
        if (s_callbacks[i].hcan == hcan && s_callbacks[i].std_id == std_id) {
            s_callbacks[i].callback = callback;
            return;
        }
    }

    s_callbacks[s_can_callback_count].hcan     = hcan;
    s_callbacks[s_can_callback_count].std_id   = std_id;
    s_callbacks[s_can_callback_count].callback = callback;
    s_can_callback_count++;
}

void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t             data[8];
    uint8_t             i;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, data) != HAL_OK) {
        return;
    }

    for (i = 0; i < s_can_callback_count; i++) {
        if (s_callbacks[i].hcan == hcan &&
            s_callbacks[i].std_id == rx_hdr.StdId) {
            s_callbacks[i].callback(rx_hdr.StdId, data, rx_hdr.DLC);
            return;
        }
    }
}