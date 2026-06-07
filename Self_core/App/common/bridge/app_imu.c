/**
 * @file    app_imu.c
 * @brief   BMI088 上层胶水 — 挂接 BSP SPI/GPIO/TIM 与 DRV IMU
 */
#include "app_imu.h"

#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "bsp_cfg.h"

// ─── SPI 上下文 ──────────────────────────────────

static void *spi_ctx = &hspi1;

static void imu_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    (void)ctx;
    bsp_spi_transceive(&hspi1, tx, rx, len);
}

// ─── CS 上下文 ───────────────────────────────────

/* ACC CS: PA4 */
static void acc_cs_write(void *ctx, uint8_t state)
{
    (void)ctx;
    bsp_gpio_write_pin(GPIOA, GPIO_PIN_4, state);
}

/* GYRO CS: PB0 */
static void gyro_cs_write(void *ctx, uint8_t state)
{
    (void)ctx;
    bsp_gpio_write_pin(GPIOB, GPIO_PIN_0, state);
}

// ─── 延时 ────────────────────────────────────────

static void imu_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

// ─── 加热控制 ────────────────────────────────────
/* TIM10_CH1 PWM 驱动加热电阻 */

static void heater_set(uint16_t val)
{
    bsp_tim_pwm_set_compare(&htim10, TIM_CHANNEL_1, val);
}

// ─── 接口实现 ─────────────────────────────────────

void app_imu_init(drv_imu_t *imu)
{
    drv_imu_bus_t bus;

    bus.spi_ctx     = NULL;          /* 未使用, spi_xfer 内部硬编码 hspi1 */
    bus.spi_xfer    = imu_spi_xfer;
    bus.acc_cs_ctx  = NULL;
    bus.acc_cs      = acc_cs_write;
    bus.gyro_cs_ctx = NULL;
    bus.gyro_cs     = gyro_cs_write;
    bus.delay_ms    = imu_delay_ms;

    drv_imu_init(imu, &bus);
    drv_imu_start(imu);
}

int app_imu_calibrate(drv_imu_t *imu)
{
    /* Step 1: 加热至 40°C (含迟滞) */
    heater_set(1500);   /* 全功率加热 */
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    do {
        drv_imu_read_temp(imu);

        if (imu->temperature < 39.5f)
            heater_set(1500);
        else if (imu->temperature > 40.5f)
            heater_set(0);

        HAL_Delay(5);
    } while (imu->temperature < 39.5f || imu->temperature > 40.5f);

    /* Step 2: 陀螺零偏校准 (500 样本) */
    drv_imu_calibrate_gyro(imu, 500);

    /* Step 3: 初始对准 (基于加速度计) */
    drv_imu_read_acc_raw(imu);
    drv_imu_read_gyro_raw(imu);
    drv_imu_data_convert(imu);
    drv_imu_initial_alignment(imu);

    /* Step 4: Mahony 收敛 (500 次迭代) */
    for (int i = 0; i < 500; i++) {
        drv_imu_read_acc_raw(imu);
        drv_imu_read_gyro_raw(imu);
        drv_imu_data_convert(imu);
        drv_imu_mahony_update(imu, 0.001f);
        HAL_Delay(1);
    }

    /* Step 5: 记录零位欧拉角 */
    drv_imu_quat_to_euler(imu);
    drv_imu_calibrate_pose(imu);

    return 0;
}
