/**
 * @file    bsp_spi.c
 * @brief   SPI 驱动实现
 */
#include "bsp_spi.h"

/* SPI 传输超时 (ms)
 *   正常 IMU 读 ~8μs, 超时留足余量
 *   若 SPI 硬件故障, 10ms 后返回避免阻塞控制循环 */
#define BSP_SPI_TIMEOUT_MS  10

void bsp_spi_transceive(SPI_HandleTypeDef *hspi,
                        const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, size, BSP_SPI_TIMEOUT_MS);
}
