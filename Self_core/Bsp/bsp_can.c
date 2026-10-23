/**
 * @file    bsp_can.c
 * @brief   CAN 驱动实现: 初始化 / 发送 / 回调分发
 * @note    滤波器用 16-bit IDMASK 模式, 接收所有 ID
 *          所有接收走回调注册, 无轮询缓冲区
 *          CAN1 / CAN2 使用独立回调表, 互不占用
 */
#include "bsp_can.h"
#include <string.h>

#define BSP_CAN1_TX_QUEUE_SIZE 16U

typedef struct {
    uint32_t std_id;
    uint8_t data[8];
} bsp_can_tx_entry_t;

/** CAN1 发送队列；同一 ID 的待发送数据保留最新值。 */
typedef struct {
    bsp_can_tx_entry_t queue[BSP_CAN1_TX_QUEUE_SIZE];
    uint8_t head;
    uint8_t count;
} bsp_can1_tx_queue_t;

static bsp_can1_tx_queue_t s_can1_tx_queue;


typedef struct {
    uint32_t                std_id;
    bsp_can_rx_callback_t   callback;
} bsp_can_callback_entry_t;//CAN回调表条目

typedef struct {
    bsp_can_callback_entry_t entries[BSP_CAN_REG_MAX];
    uint8_t                  count;
} bsp_can_callback_table_t;//CAN回调表

static bsp_can_callback_table_t s_can1_table;//CAN1回调表
static bsp_can_callback_table_t s_can2_table;//CAN2回调表

/**
 * @brief  根据 hcan 选择对应回调表
 * @return 回调表指针, 未知总线返回 NULL
 */
static bsp_can_callback_table_t *get_table(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) return &s_can1_table;
    if (hcan == &hcan2) return &s_can2_table;
    return NULL;
}

/**
 * @brief  获取空闲邮箱号
 * @return 邮箱号 (0/1/2), 无空闲返回 0xFFFFFFFF
 */
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

/** 排空 CAN1 软件队列；最多填满三个硬件邮箱，不阻塞等待总线。 */
static void can1_tx_drain(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    CAN_TxHeaderTypeDef header = {0};
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    while (s_can1_tx_queue.count > 0U) {
        uint32_t mailbox = get_free_mbox(&hcan1);
        bsp_can_tx_entry_t *entry = &s_can1_tx_queue.queue[s_can1_tx_queue.head];
        if (mailbox == 0xFFFFFFFFU) {
            break;
        }
        header.StdId = entry->std_id;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, entry->data, &mailbox) != HAL_OK) {
            break; /* 保留队首，下次发送调用或 TX 中断再尝试。 */
        }
        s_can1_tx_queue.head = (s_can1_tx_queue.head + 1U) % BSP_CAN1_TX_QUEUE_SIZE;
        s_can1_tx_queue.count--;
    }
    __set_PRIMASK(irq_state);
}


HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank,
                                uint8_t slave_filter_bank)
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
    filter.SlaveStartFilterBank   = slave_filter_bank;

    can_status_return = HAL_CAN_ConfigFilter(hcan, &filter);
    if (can_status_return != HAL_OK) return can_status_return;//检查寄存器配置是否成功

    if (hcan == &hcan1) {
        memset(&s_can1_tx_queue, 0, sizeof(s_can1_tx_queue));
    }

    can_status_return = HAL_CAN_Start(hcan);
    if (can_status_return != HAL_OK) return can_status_return;//检查CAN启动是否成功

    uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING;
    if (hcan == &hcan1) {
        notifications |= CAN_IT_TX_MAILBOX_EMPTY;
    }
    can_status_return = HAL_CAN_ActivateNotification(hcan, notifications);
    return can_status_return;
}

uint8_t bsp_can_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    HAL_StatusTypeDef   hal_status;
    uint32_t            irq_state;
    uint32_t            mbox;

    if ((hcan != &hcan1 && hcan != &hcan2) || !data || std_id > 0x7FFU) return 1U;

    tx_hdr.StdId = std_id;
    tx_hdr.IDE   = CAN_ID_STD;
    tx_hdr.RTR   = CAN_RTR_DATA;
    tx_hdr.DLC   = 8;

    /* 邮箱检查与设置 TXRQ 必须连续完成，防止中断嵌套发送使用同一邮箱*/
    irq_state = __get_PRIMASK();
    __disable_irq();

    if (hcan == &hcan1) {
        /* 避免总线暂时拥塞时积压多份旧速度或旧姿态。 */
        uint8_t index;
        for (index = 0U; index < s_can1_tx_queue.count; index++) {
            uint8_t slot = (s_can1_tx_queue.head + index) % BSP_CAN1_TX_QUEUE_SIZE;
            if (s_can1_tx_queue.queue[slot].std_id == std_id) {
                memcpy(s_can1_tx_queue.queue[slot].data, data, 8U);
                break;
            }
        }
        if (index == s_can1_tx_queue.count) {
            if (s_can1_tx_queue.count >= BSP_CAN1_TX_QUEUE_SIZE) {
                __set_PRIMASK(irq_state);
                return 1U;
            }
            uint8_t tail = (s_can1_tx_queue.head + s_can1_tx_queue.count) % BSP_CAN1_TX_QUEUE_SIZE;
            s_can1_tx_queue.queue[tail].std_id = std_id;
            memcpy(s_can1_tx_queue.queue[tail].data, data, 8U);
            s_can1_tx_queue.count++;
        }
        __set_PRIMASK(irq_state);
        can1_tx_drain(hcan);
        return 0U;
    }

    mbox = get_free_mbox(hcan);
    if (mbox == 0xFFFFFFFFU) {
        __set_PRIMASK(irq_state);
        return 1U;
    }
    hal_status = HAL_CAN_AddTxMessage(hcan, &tx_hdr, data, &mbox);
    __set_PRIMASK(irq_state);

    return hal_status == HAL_OK ? 0U : 1U;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can1_tx_drain(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can1_tx_drain(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can1_tx_drain(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) { can1_tx_drain(hcan); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) { can1_tx_drain(hcan); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) { can1_tx_drain(hcan); }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    can1_tx_drain(hcan);
}

void bsp_can_rx_reg(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                  bsp_can_rx_callback_t callback)
{
    bsp_can_callback_table_t *table = get_table(hcan);
    uint8_t i;

    if (!table || !callback) return;

    for (i = 0; i < table->count; i++) {
        if (table->entries[i].std_id == std_id) {
            table->entries[i].callback = callback;
            return;//覆盖旧回调
        }
    }//

    if (table->count >= BSP_CAN_REG_MAX) {
        return;
    }//超出限制静默失败

    table->entries[table->count].std_id   = std_id;
    table->entries[table->count].callback = callback;
    table->count++;//正常添加
}

void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t             data[8];
    bsp_can_callback_table_t *table = get_table(hcan);
    uint8_t             i;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, data) != HAL_OK) {
        return;
    }

    for (i = 0; i < table->count; i++) {
        if (table->entries[i].std_id == rx_hdr.StdId) {
            table->entries[i].callback(rx_hdr.StdId, data, rx_hdr.DLC);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    bsp_can_rx_irq_handler(hcan);
}
