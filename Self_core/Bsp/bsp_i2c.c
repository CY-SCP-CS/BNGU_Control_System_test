/**
 * @file    bsp_i2c.c
 * @brief   I2C 驱动实现
 */
#include "bsp_i2c.h"

static I2C_HandleTypeDef *s_async_handle;
static bsp_i2c_async_callback_t s_async_callback;
static void *s_async_context;

HAL_StatusTypeDef bsp_i2c_transfer(const bsp_i2c_transfer_t *transfer)
{
    if (transfer == NULL || transfer->hi2c == NULL || transfer->data == NULL ||
        transfer->length == 0U || transfer->timeout_ms == 0U)
        return HAL_ERROR;
    if (transfer->direction == BSP_I2C_READ) {
        return HAL_I2C_Mem_Read(transfer->hi2c, transfer->dev_addr,
                                transfer->reg_addr, transfer->reg_addr_size,
                                transfer->data, transfer->length,
                                transfer->timeout_ms);
    }
    if (transfer->direction == BSP_I2C_WRITE) {
        return HAL_I2C_Mem_Write(transfer->hi2c, transfer->dev_addr,
                                 transfer->reg_addr, transfer->reg_addr_size,
                                 transfer->data, transfer->length,
                                 transfer->timeout_ms);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef bsp_i2c_transfer_async(const bsp_i2c_transfer_t *transfer)
{
    if (transfer == NULL || transfer->hi2c == NULL ||
        transfer->hi2c != s_async_handle || transfer->data == NULL ||
        transfer->length == 0U)
        return HAL_ERROR;
    if (transfer->direction == BSP_I2C_READ) {
        return HAL_I2C_Mem_Read_IT(transfer->hi2c, transfer->dev_addr,
                                   transfer->reg_addr, transfer->reg_addr_size,
                                   transfer->data, transfer->length);
    }
    if (transfer->direction == BSP_I2C_WRITE) {
        return HAL_I2C_Mem_Write_IT(transfer->hi2c, transfer->dev_addr,
                                    transfer->reg_addr, transfer->reg_addr_size,
                                    transfer->data, transfer->length);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef bsp_i2c_async_register(I2C_HandleTypeDef *hi2c,
                                         bsp_i2c_async_callback_t callback,
                                         void *context)
{
    if (hi2c == NULL || callback == NULL) return HAL_ERROR;
    if (hi2c->Instance != I2C3) return HAL_ERROR;
    s_async_handle = hi2c;
    s_async_callback = callback;
    s_async_context = context;
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 1U, 0U);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
    return HAL_OK;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_async_handle && s_async_callback != NULL)
        s_async_callback(hi2c, HAL_OK, s_async_context);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_async_handle && s_async_callback != NULL)
        s_async_callback(hi2c, HAL_OK, s_async_context);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_async_handle && s_async_callback != NULL)
        s_async_callback(hi2c, HAL_ERROR, s_async_context);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_async_handle && s_async_callback != NULL)
        s_async_callback(hi2c, HAL_TIMEOUT, s_async_context);
}

/* CubeMX does not enable I2C3 event/error IRQs for polling mode. This async
 * port owns the handlers and enables their NVIC lines when it registers.
 */
void I2C3_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(s_async_handle); }
void I2C3_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(s_async_handle); }
