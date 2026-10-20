/**
 * @file    bsp_i2c.c
 * @brief   I2C 驱动实现
 */
#include "bsp_i2c.h"

void bsp_i2c_read(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t reg_addr,
    uint16_t reg_addr_size,
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    HAL_I2C_Mem_Read(hi2c,dev_addr,reg_addr,reg_addr_size,
        data,length,timeout_ms);
}

void bsp_i2c_write(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t reg_addr,
    uint16_t reg_addr_size,
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{

    HAL_I2C_Mem_Write(hi2c,dev_addr,reg_addr,reg_addr_size,
        (uint8_t *)data,length,timeout_ms);
}