/**
 * @file    bsp_spi.h
 * @brief   SPI 收发抽象
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "bsp_cfg.h"

typedef void (*bsp_spi_callback_t)(SPI_HandleTypeDef *hspi);

/**
 * @brief  SPI 同步收发 (全双工)
 * @param  hspi  SPI 句柄
 * @param  tx    发送缓冲区
 * @param  rx    接收缓冲区
 * @param  size  收发字节数
 */
void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size);

/** @brief 启动 SPI DMA 全双工收发，缓冲区在完成前必须保持有效。 */
HAL_StatusTypeDef bsp_spi_transceive_dma(SPI_HandleTypeDef *hspi,
                                         const uint8_t *tx, uint8_t *rx, uint16_t size);

/** @brief 注册 SPI DMA 完成和错误回调。 */
void bsp_spi_register_dma_callbacks(SPI_HandleTypeDef *hspi,
                                    bsp_spi_callback_t complete,
                                    bsp_spi_callback_t error);

/** @brief 异步中止 SPI DMA。 */
HAL_StatusTypeDef bsp_spi_abort_dma(SPI_HandleTypeDef *hspi);

#endif
