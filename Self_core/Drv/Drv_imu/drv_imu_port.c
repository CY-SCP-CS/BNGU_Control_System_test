/**
 * @file    drv_imu_port.c
 * @brief   BSP 适配 — drv_imu 的硬件端口挂接
 */
#include "drv_imu.h"
#include "bsp_cfg.h"

// ─── Port: BSP 适配 ─────────────────────────────
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"

static void port_imu_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    (void)ctx;
    bsp_spi_transceive(&hspi1, tx, rx, len);
}

static void port_acc_cs_write(void *ctx, uint8_t state)
{
    (void)ctx;
    bsp_gpio_write_pin(GPIOA, GPIO_PIN_4, state);
}

static void port_gyro_cs_write(void *ctx, uint8_t state)
{
    (void)ctx;
    bsp_gpio_write_pin(GPIOB, GPIO_PIN_0, state);
}

void drv_imu_port_init(drv_imu_t *imu)
{
    drv_imu_bus_t bus;

    bus.spi_ctx     = NULL;
    bus.spi_xfer    = port_imu_spi_xfer;
    bus.acc_cs_ctx  = NULL;
    bus.acc_cs      = port_acc_cs_write;
    bus.gyro_cs_ctx = NULL;
    bus.gyro_cs     = port_gyro_cs_write;
    bus.delay_ms    = drv_imu_port_delay_ms;

    drv_imu_init(imu, &bus);
    drv_imu_start(imu);
}

void drv_imu_port_heater_start(void)
{
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void drv_imu_port_heater_set(uint16_t val)
{
    bsp_tim_pwm_set_compare(&htim10, TIM_CHANNEL_1, val);
}

void drv_imu_port_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
