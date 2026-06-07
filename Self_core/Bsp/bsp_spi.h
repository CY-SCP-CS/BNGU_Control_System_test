/**
 * @file    bsp_spi.h
 * @brief   SPI 收发抽象
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "lib_typedef.h"
#include "bsp_cfg.h"

/**
 * @brief  SPI 同步收发 (全双工)
 * @param  hspi  SPI 句柄
 * @param  tx    发送缓冲区
 * @param  rx    接收缓冲区
 * @param  size  收发字节数
 */
void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size);

#endif
