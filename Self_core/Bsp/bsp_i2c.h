/**
 * @file    bsp_i2c.h
 * @brief   I2C 收发
 */
#ifndef BSP_I2C_H
#define BSP_I2C_H
#include "bsp_cfg.h"
#include "stm32f4xx_hal.h"


void bsp_i2c_read(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t reg_addr,
    uint16_t reg_addr_size,
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms);

void bsp_i2c_write(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t reg_addr,
    uint16_t reg_addr_size,
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms);

#endif
