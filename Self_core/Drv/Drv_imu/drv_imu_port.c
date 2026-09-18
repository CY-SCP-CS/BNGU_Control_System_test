/**
 * @file    drv_imu_port.c
 * @brief   BMI088 SPI1 纭欢閫傞厤涓庡弻缂撳啿 DMA 閲囨牱
 */
#include "drv_imu.h"

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tim.h"

#include <string.h>

// 鈹€鈹€鈹€ 绉佹湁瀹?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
#define DRV_IMU_PORT_ACC_FRAME_SIZE  8U//IMU ACC 閲囨牱甯ч暱搴?(1+6+1)
#define DRV_IMU_PORT_GYRO_FRAME_SIZE 7U//IMU GYRO 閲囨牱甯ч暱搴?(1+6)
#define DRV_IMU_PORT_DMA_TIMEOUT_MS  2U//DMA 瓒呮椂鏃堕棿
#define DRV_IMU_PORT_GYRO_CALIBRATION_SAMPLES 500U//闄€铻轰华闆跺亸鏍囧畾鏍锋湰鏁伴噺

// 鈹€鈹€鈹€ 绉佹湁绫诲瀷 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
typedef enum {
    DRV_IMU_PORT_DMA_IDLE = 0,
    DRV_IMU_PORT_DMA_ACC,
    DRV_IMU_PORT_DMA_GYRO,
    DRV_IMU_PORT_DMA_ABORTING
} drv_imu_port_dma_state_t;// IMU DMA 鐘舵€佹満

typedef struct {
    drv_imu_raw_t acc;
    drv_imu_raw_t gyro;
    uint32_t sequence;
    uint32_t tick;
} drv_imu_port_snapshot_t; // IMU 异步接收快照
// 鈹€鈹€鈹€ 绉佹湁鍙橀噺 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
static drv_imu_t *s_imu_port_instance;
static volatile drv_imu_port_dma_state_t s_imu_port_dma_state;
static drv_imu_port_snapshot_t s_imu_port_snapshot[2];
static volatile uint8_t s_imu_port_published_index;
static uint8_t s_imu_port_write_index;
static uint32_t s_imu_port_consumed_sequence;
static uint32_t s_imu_port_sequence;
static uint32_t s_imu_port_dma_start_tick;
static volatile uint32_t s_imu_port_busy_count;
static volatile uint32_t s_imu_port_error_count;

static const uint8_t s_imu_port_acc_tx[DRV_IMU_PORT_ACC_FRAME_SIZE] = {
    0x92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};
static const uint8_t s_imu_port_gyro_tx[DRV_IMU_PORT_GYRO_FRAME_SIZE] = {
    0x82U, 0U, 0U, 0U, 0U, 0U, 0U
};
static uint8_t s_imu_port_acc_rx[DRV_IMU_PORT_ACC_FRAME_SIZE];
static uint8_t s_imu_port_gyro_rx[DRV_IMU_PORT_GYRO_FRAME_SIZE];

// 鈹€鈹€鈹€ 绉佹湁鍑芥暟澹版槑 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
static void drv_imu_port_spi_transfer_complete(SPI_HandleTypeDef *hspi);
static void drv_imu_port_spi_transfer_error(SPI_HandleTypeDef *hspi);
static void drv_imu_port_finish_with_error(void);
static void drv_imu_port_spi_transfer(void *context,
                                      const uint8_t *tx, uint8_t *rx, uint16_t len);
static void drv_imu_port_acc_cs_write(void *context, uint8_t state);
static void drv_imu_port_gyro_cs_write(void *context, uint8_t state);

// 鈹€鈹€鈹€ 鍏湁鎺ュ彛瀹炵幇 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void drv_imu_port_init(drv_imu_t *imu)
{
    drv_imu_bus_t bus;

    memset(&bus, 0, sizeof(bus));
    bus.spi_xfer = drv_imu_port_spi_transfer;
    bus.acc_cs = drv_imu_port_acc_cs_write;
    bus.gyro_cs = drv_imu_port_gyro_cs_write;
    bus.delay_ms = drv_imu_port_delay_ms;

    s_imu_port_instance = imu;
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_IDLE;
    s_imu_port_published_index = 0U;
    s_imu_port_write_index = 1U;
    s_imu_port_consumed_sequence = 0U;
    s_imu_port_sequence = 0U;
    s_imu_port_busy_count = 0U;
    s_imu_port_error_count = 0U;
    memset(s_imu_port_snapshot, 0, sizeof(s_imu_port_snapshot));

    drv_imu_init(imu, &bus);
    drv_imu_start(imu);

    /* 涓婄數鏃朵簯鍙板繀椤讳繚鎸侀潤姝細鍏堟秷闄ら浂鍋忥紝鍐嶇敤閲嶅姏鏂瑰悜寤虹珛 roll/pitch 鍒濆€笺€?*/
    drv_imu_calibrate_gyro(imu, DRV_IMU_PORT_GYRO_CALIBRATION_SAMPLES);
    drv_imu_read_acc_raw(imu);
    drv_imu_data_convert(imu);
    drv_imu_initial_alignment(imu);

    bsp_spi_reg_dma_callbacks(&hspi1,
                                   drv_imu_port_spi_transfer_complete,
                                   drv_imu_port_spi_transfer_error);
    drv_imu_port_async_start();
}

uint8_t drv_imu_port_async_start(void)
{
    uint32_t now = bsp_tim_get_tick_ms();
    if (!s_imu_port_instance) {
        return 0U;
    }

    if (s_imu_port_dma_state != DRV_IMU_PORT_DMA_IDLE) {
        s_imu_port_busy_count++;
        if (s_imu_port_dma_state != DRV_IMU_PORT_DMA_ABORTING
            && (uint32_t)(now - s_imu_port_dma_start_tick) > DRV_IMU_PORT_DMA_TIMEOUT_MS) {
            s_imu_port_dma_state = DRV_IMU_PORT_DMA_ABORTING;
            if (bsp_spi_abort_dma(&hspi1) != HAL_OK) {
                drv_imu_port_finish_with_error();
            }
        }
        return 0U;
    }

    s_imu_port_write_index = (uint8_t)(s_imu_port_published_index ^ 1U);
    s_imu_port_dma_start_tick = now;
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_ACC;
    drv_imu_port_acc_cs_write(NULL, 0U);
    if (bsp_spi_transceive_dma(&hspi1, s_imu_port_acc_tx,
                               s_imu_port_acc_rx, sizeof(s_imu_port_acc_rx)) != HAL_OK) {
        drv_imu_port_finish_with_error();
        return 0U;
    }
    return 1U;
}

uint8_t drv_imu_port_snapshot_update(drv_imu_t *imu)
{
    if (!imu) {
        return 0U;
    }

    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t index = s_imu_port_published_index;
    drv_imu_port_snapshot_t snapshot = s_imu_port_snapshot[index];
    __set_PRIMASK(irq_state);

    if (snapshot.sequence == 0U || snapshot.sequence == s_imu_port_consumed_sequence) {
        return 0U;
    }
    imu->acc_raw = snapshot.acc;
    imu->gyro_raw = snapshot.gyro;
    s_imu_port_consumed_sequence = snapshot.sequence;
    return 1U;
}

uint8_t drv_imu_port_is_online(uint32_t timeout_ms)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    drv_imu_port_snapshot_t snapshot = s_imu_port_snapshot[s_imu_port_published_index];
    __set_PRIMASK(irq_state);
    return snapshot.sequence != 0U
           && (uint32_t)(bsp_tim_get_tick_ms() - snapshot.tick) <= timeout_ms;
}

uint32_t drv_imu_port_get_busy_count(void)
{
    return s_imu_port_busy_count;
}

uint32_t drv_imu_port_get_error_count(void)
{
    return s_imu_port_error_count;
}

void drv_imu_port_heater_start(void)
{
    bsp_tim_it_start(&htim10);
    bsp_tim_pwm_start(&htim10, TIM_CHANNEL_1);
}

void drv_imu_port_heater_set(uint16_t value)
{
    bsp_tim_pwm_set_compare(&htim10, TIM_CHANNEL_1, value);
}

void drv_imu_port_delay_ms(uint32_t ms)
{
    bsp_tim_delay_ms(ms);
}

// 鈹€鈹€鈹€ 绉佹湁鍑芥暟瀹炵幇 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
static void drv_imu_port_spi_transfer_complete(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) {
        return;
    }

    drv_imu_port_snapshot_t *snapshot = &s_imu_port_snapshot[s_imu_port_write_index];
    if (s_imu_port_dma_state == DRV_IMU_PORT_DMA_ACC) {
        drv_imu_port_acc_cs_write(NULL, 1U);
        snapshot->acc.x = (int16_t)((s_imu_port_acc_rx[3] << 8) | s_imu_port_acc_rx[2]);
        snapshot->acc.y = (int16_t)((s_imu_port_acc_rx[5] << 8) | s_imu_port_acc_rx[4]);
        snapshot->acc.z = (int16_t)((s_imu_port_acc_rx[7] << 8) | s_imu_port_acc_rx[6]);

        s_imu_port_dma_state = DRV_IMU_PORT_DMA_GYRO;
        drv_imu_port_gyro_cs_write(NULL, 0U);
        if (bsp_spi_transceive_dma(&hspi1, s_imu_port_gyro_tx,
                                   s_imu_port_gyro_rx, sizeof(s_imu_port_gyro_rx)) != HAL_OK) {
            drv_imu_port_finish_with_error();
        }
        return;
    }

    if (s_imu_port_dma_state == DRV_IMU_PORT_DMA_GYRO) {
        drv_imu_port_gyro_cs_write(NULL, 1U);
        snapshot->gyro.x = (int16_t)((s_imu_port_gyro_rx[2] << 8) | s_imu_port_gyro_rx[1]);
        snapshot->gyro.y = (int16_t)((s_imu_port_gyro_rx[4] << 8) | s_imu_port_gyro_rx[3]);
        snapshot->gyro.z = (int16_t)((s_imu_port_gyro_rx[6] << 8) | s_imu_port_gyro_rx[5]);
        snapshot->tick = bsp_tim_get_tick_ms();
        s_imu_port_sequence++;
        if (s_imu_port_sequence == 0U) {
            s_imu_port_sequence = 1U;
        }
        snapshot->sequence = s_imu_port_sequence;
        __DMB();
        s_imu_port_published_index = s_imu_port_write_index;
        s_imu_port_dma_state = DRV_IMU_PORT_DMA_IDLE;
        return;
    }

    drv_imu_port_finish_with_error();
}

static void drv_imu_port_spi_transfer_error(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1 && s_imu_port_dma_state != DRV_IMU_PORT_DMA_IDLE) {
        drv_imu_port_finish_with_error();
    }
}

static void drv_imu_port_finish_with_error(void)
{
    drv_imu_port_acc_cs_write(NULL, 1U);
    drv_imu_port_gyro_cs_write(NULL, 1U);
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_IDLE;
    s_imu_port_error_count++;
}

static void drv_imu_port_spi_transfer(void *context,
                                      const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    (void)context;
    bsp_spi_transceive(&hspi1, tx, rx, len);
}

static void drv_imu_port_acc_cs_write(void *context, uint8_t state)
{
    (void)context;
    bsp_gpio_write_pin(GPIOA, GPIO_PIN_4, state);
}

static void drv_imu_port_gyro_cs_write(void *context, uint8_t state)
{
    (void)context;
    bsp_gpio_write_pin(GPIOB, GPIO_PIN_0, state);
}
