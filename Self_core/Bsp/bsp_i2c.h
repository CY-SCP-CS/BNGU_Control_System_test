/**
 * @file    bsp_i2c.h
 * @brief   I2C 收发
 */
#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "bsp_cfg.h"
#include "stm32f4xx_hal.h"

typedef void (*bsp_i2c_async_callback_t)(I2C_HandleTypeDef *hi2c,
                                          HAL_StatusTypeDef status,
                                          void *context);


typedef enum {
    BSP_I2C_READ = 0U,
    BSP_I2C_WRITE = 1U
} bsp_i2c_direction_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    bsp_i2c_direction_t direction;
    uint16_t dev_addr;
    uint16_t reg_addr;
    uint16_t reg_addr_size;
    uint8_t *data;
    uint16_t length;
    uint32_t timeout_ms; /* 同步传输使用；异步传输忽略此项。 */
} bsp_i2c_transfer_t;

/* Preserve HAL_OK / HAL_BUSY / HAL_ERROR / HAL_TIMEOUT for the caller. */
HAL_StatusTypeDef bsp_i2c_transfer(const bsp_i2c_transfer_t *transfer);

/* Single asynchronous transaction owner per I2C peripheral. Completion callbacks
 * run from the HAL I2C IRQ and must only record completion state.
 */
HAL_StatusTypeDef bsp_i2c_async_register(I2C_HandleTypeDef *hi2c,
                                         bsp_i2c_async_callback_t callback,
                                         void *context);
HAL_StatusTypeDef bsp_i2c_transfer_async(const bsp_i2c_transfer_t *transfer);

#endif
