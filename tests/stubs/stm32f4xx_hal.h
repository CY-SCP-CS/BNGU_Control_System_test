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
typedef struct { int instance; } CAN_HandleTypeDef;
typedef struct { int instance; } SPI_HandleTypeDef;
typedef struct { int instance; } TIM_HandleTypeDef;
#define HAL_OK 0
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
#endif
