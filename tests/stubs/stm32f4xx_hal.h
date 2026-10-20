/** @file stm32f4xx_hal.h
 *  @brief 主机回归测试使用的 HAL 替身；不进入固件编译。
 */
#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; uint32_t idle; } UART_HandleTypeDef;
typedef struct { int instance; uint32_t ErrorCode; } CAN_HandleTypeDef;
typedef struct { int instance; } I2C_HandleTypeDef;
typedef struct { int instance; } GPIO_TypeDef;
typedef struct {
    uint32_t FilterActivation, FilterBank, FilterMode, FilterScale;
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct {
    uint32_t StdId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, DLC; } CAN_RxHeaderTypeDef;
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_FILTER_ENABLE 1U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_16BIT 0U
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_TX_MAILBOX_EMPTY 2U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
typedef struct { int instance; } SPI_HandleTypeDef;
typedef struct { int instance; } TIM_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define RESET 0
#define UART_FLAG_IDLE 1
#define UART_IT_IDLE 1
#define __HAL_UART_ENABLE_IT(port, flag) ((void)(port))
#define __HAL_UART_GET_FLAG(port, flag) ((port)->idle)
#define __HAL_UART_CLEAR_IDLEFLAG(port) ((port)->idle = 0)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t state) { (void)state; }
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *port);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *port, uint8_t *data, uint16_t len);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can, uint32_t mailbox);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t notifications);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data);
#endif
