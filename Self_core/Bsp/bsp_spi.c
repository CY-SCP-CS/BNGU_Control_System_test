/**
 * @file    bsp_spi.c
 * @brief   SPI 驱动实现
 */
#include "bsp_spi.h"

void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, size, HAL_MAX_DELAY);
}
