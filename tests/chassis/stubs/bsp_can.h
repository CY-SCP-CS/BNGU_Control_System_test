#ifndef TEST_BSP_CAN_H
#define TEST_BSP_CAN_H
#include <stdint.h>
typedef struct { int unused; } CAN_HandleTypeDef;
CAN_HandleTypeDef hcan1, hcan2;
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline void bsp_can_rx_reg(CAN_HandleTypeDef *can, uint32_t id,
                                 void (*callback)(uint32_t, uint8_t *, uint8_t))
{ (void)can; (void)id; (void)callback; }
static inline uint8_t bsp_can_tx(CAN_HandleTypeDef *can, uint32_t id, uint8_t *frame)
{ (void)can; (void)id; (void)frame; return 0U; }
#endif
