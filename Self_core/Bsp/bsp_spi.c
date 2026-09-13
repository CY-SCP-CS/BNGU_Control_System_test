/**
 * @file    bsp_spi.c
 * @brief   SPI 驱动实现
 */
#include "bsp_spi.h"

#define BSP_SPI_TIMEOUT_MS  10

static SPI_HandleTypeDef *s_spi_dma_handle;
static bsp_spi_callback_t s_spi_dma_complete_callback;
static bsp_spi_callback_t s_spi_dma_error_callback;

void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, size, BSP_SPI_TIMEOUT_MS);
}

HAL_StatusTypeDef bsp_spi_transceive_dma(SPI_HandleTypeDef *hspi,
                                         const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    if (!hspi || !hspi->hdmarx || !hspi->hdmatx || !tx || !rx || size == 0U) {
        return HAL_ERROR;
    }
    return HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t *)tx, rx, size);
}

void bsp_spi_reg_dma_callbacks(SPI_HandleTypeDef *hspi,
                                    bsp_spi_callback_t complete,
                                    bsp_spi_callback_t error)
{
    s_spi_dma_handle = hspi;
    s_spi_dma_complete_callback = complete;
    s_spi_dma_error_callback = error;
}

HAL_StatusTypeDef bsp_spi_abort_dma(SPI_HandleTypeDef *hspi)
{
    if (!hspi) {
        return HAL_ERROR;
    }
    return HAL_SPI_Abort_IT(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_complete_callback) {
        s_spi_dma_complete_callback(hspi);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback) {
        s_spi_dma_error_callback(hspi);
    }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_spi_dma_handle && s_spi_dma_error_callback) {
        s_spi_dma_error_callback(hspi);
    }
}
