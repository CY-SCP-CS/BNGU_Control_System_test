/**
 * @file    drv_imu_port.c
 * @brief   BMI088 SPI1 硬件适配与双缓冲 DMA 采样
 */
#include "drv_imu.h"

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tim.h"

#include <string.h>

/* 私有常量。 */
#define DRV_IMU_PORT_ACC_FRAME_SIZE  8U   // 加速度计采样帧长度：命令 1 字节、数据 6 字节、占位 1 字节。
#define DRV_IMU_PORT_GYRO_FRAME_SIZE 7U   // 陀螺仪采样帧长度：命令 1 字节、数据 6 字节。
#define DRV_IMU_PORT_DMA_TIMEOUT_MS  2U   // 单次 DMA 传输超时时间，单位 ms。
#define DRV_IMU_PORT_GYRO_CALIBRATION_SAMPLES 500U // 上电陀螺仪零偏标定样本数。

/* 私有类型。 */
typedef enum {
    DRV_IMU_PORT_DMA_IDLE = 0,
    DRV_IMU_PORT_DMA_ACC,
    DRV_IMU_PORT_DMA_GYRO_PENDING,
    DRV_IMU_PORT_DMA_GYRO,
    DRV_IMU_PORT_DMA_ABORTING
} drv_imu_port_dma_state_t; // IMU DMA 状态机。

typedef struct {
    drv_imu_raw_t acc;
    drv_imu_raw_t gyro;
    uint32_t sequence;
    uint32_t tick;
} drv_imu_port_snapshot_t; // IMU 异步接收快照

/**
 * @brief IMU DMA 采样调试快照。
 *
 * 保留外部链接便于调试器 Watch 直接确认 DMA 状态、完整帧序号、错误计数和最近两帧原始数据。
 * 不在头文件声明，不作为其他模块的访问接口。
 */
typedef struct {
    drv_imu_t *instance;
    volatile drv_imu_port_dma_state_t dma_state;
    drv_imu_port_snapshot_t snapshot[2];
    volatile uint8_t published_index;
    uint8_t write_index;
    uint32_t consumed_sequence;
    volatile uint32_t sequence;
    uint32_t dma_start_tick;
    volatile uint32_t busy_count;
    volatile uint32_t error_count;
    volatile uint32_t start_error_count;
    volatile uint32_t callback_error_count;
    volatile uint32_t abort_count;
    volatile uint32_t last_hal_status;
    volatile uint32_t spi_error_code;
    volatile uint32_t dma_rx_error_code;
    volatile uint32_t dma_tx_error_code;
} drv_imu_port_debug_t;

drv_imu_port_debug_t drv_imu_port_debug;

#define s_imu_port_instance          drv_imu_port_debug.instance
#define s_imu_port_dma_state         drv_imu_port_debug.dma_state
#define s_imu_port_snapshot          drv_imu_port_debug.snapshot
#define s_imu_port_published_index   drv_imu_port_debug.published_index
#define s_imu_port_write_index       drv_imu_port_debug.write_index
#define s_imu_port_consumed_sequence drv_imu_port_debug.consumed_sequence
#define s_imu_port_sequence          drv_imu_port_debug.sequence
#define s_imu_port_dma_start_tick    drv_imu_port_debug.dma_start_tick
#define s_imu_port_busy_count        drv_imu_port_debug.busy_count
#define s_imu_port_error_count       drv_imu_port_debug.error_count
#define s_imu_port_start_error_count drv_imu_port_debug.start_error_count
#define s_imu_port_callback_error_count drv_imu_port_debug.callback_error_count
#define s_imu_port_abort_count       drv_imu_port_debug.abort_count
#define s_imu_port_last_hal_status   drv_imu_port_debug.last_hal_status
#define s_imu_port_spi_error_code    drv_imu_port_debug.spi_error_code
#define s_imu_port_dma_rx_error_code drv_imu_port_debug.dma_rx_error_code
#define s_imu_port_dma_tx_error_code drv_imu_port_debug.dma_tx_error_code

static const uint8_t s_imu_port_acc_tx[DRV_IMU_PORT_ACC_FRAME_SIZE] = {
    0x92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};
static const uint8_t s_imu_port_gyro_tx[DRV_IMU_PORT_GYRO_FRAME_SIZE] = {
    0x82U, 0U, 0U, 0U, 0U, 0U, 0U
};
static uint8_t s_imu_port_acc_rx[DRV_IMU_PORT_ACC_FRAME_SIZE];
static uint8_t s_imu_port_gyro_rx[DRV_IMU_PORT_GYRO_FRAME_SIZE];

/* 私有函数声明。 */
static void drv_imu_port_spi_transfer_complete(SPI_HandleTypeDef *hspi);
static void drv_imu_port_spi_transfer_error(SPI_HandleTypeDef *hspi);
static uint8_t drv_imu_port_start_acc_dma(void);
static uint8_t drv_imu_port_start_gyro_dma(void);
static uint8_t drv_imu_port_spi_dma_is_ready(void);
static void drv_imu_port_abort_with_error(void);
static void drv_imu_port_finish_with_error(void);
static void drv_imu_port_spi_transfer(void *context,
                                      const uint8_t *tx, uint8_t *rx, uint16_t len);
static void drv_imu_port_acc_cs_write(void *context, uint8_t state);
static void drv_imu_port_gyro_cs_write(void *context, uint8_t state);

/* 公有接口实现。 */
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
    s_imu_port_start_error_count = 0U;
    s_imu_port_callback_error_count = 0U;
    s_imu_port_abort_count = 0U;
    s_imu_port_last_hal_status = HAL_OK;
    s_imu_port_spi_error_code = HAL_SPI_ERROR_NONE;
    s_imu_port_dma_rx_error_code = HAL_DMA_ERROR_NONE;
    s_imu_port_dma_tx_error_code = HAL_DMA_ERROR_NONE;
    memset(s_imu_port_snapshot, 0, sizeof(s_imu_port_snapshot));

    drv_imu_init(imu, &bus);
    drv_imu_start(imu);

    /* 上电标定时云台必须保持静止：先校准陀螺仪零偏，再根据重力方向建立 roll/pitch 初值。 */
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

    if (s_imu_port_dma_state == DRV_IMU_PORT_DMA_IDLE) {
        return drv_imu_port_start_acc_dma();
    }

    if (s_imu_port_dma_state == DRV_IMU_PORT_DMA_GYRO_PENDING
        && drv_imu_port_spi_dma_is_ready()) {
        return drv_imu_port_start_gyro_dma();
    }

    s_imu_port_busy_count++;
    if (s_imu_port_dma_state != DRV_IMU_PORT_DMA_ABORTING
        && (uint32_t)(now - s_imu_port_dma_start_tick) > DRV_IMU_PORT_DMA_TIMEOUT_MS) {
        drv_imu_port_abort_with_error();
    }
    return 0U;
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

/* 私有函数实现。 */
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

        s_imu_port_dma_state = DRV_IMU_PORT_DMA_GYRO_PENDING;
        if (drv_imu_port_spi_dma_is_ready()) {
            (void)drv_imu_port_start_gyro_dma();
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
        if (s_imu_port_dma_state != DRV_IMU_PORT_DMA_ABORTING) {
            s_imu_port_callback_error_count++;
        }
        drv_imu_port_finish_with_error();
    }
}

static uint8_t drv_imu_port_start_acc_dma(void)
{
    HAL_StatusTypeDef status;

    s_imu_port_write_index = (uint8_t)(s_imu_port_published_index ^ 1U);
    s_imu_port_dma_start_tick = bsp_tim_get_tick_ms();
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_ACC;
    drv_imu_port_acc_cs_write(NULL, 0U);
    status = bsp_spi_transceive_dma(
        &hspi1, s_imu_port_acc_tx, s_imu_port_acc_rx, sizeof(s_imu_port_acc_rx));
    s_imu_port_last_hal_status = (uint32_t)status;
    if (status != HAL_OK) {
        s_imu_port_start_error_count++;
        drv_imu_port_abort_with_error();
        return 0U;
    }
    return 1U;
}

static uint8_t drv_imu_port_start_gyro_dma(void)
{
    HAL_StatusTypeDef status;

    s_imu_port_dma_start_tick = bsp_tim_get_tick_ms();
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_GYRO;
    drv_imu_port_gyro_cs_write(NULL, 0U);
    status = bsp_spi_transceive_dma(
        &hspi1, s_imu_port_gyro_tx, s_imu_port_gyro_rx, sizeof(s_imu_port_gyro_rx));
    s_imu_port_last_hal_status = (uint32_t)status;
    if (status != HAL_OK) {
        s_imu_port_start_error_count++;
        drv_imu_port_abort_with_error();
        return 0U;
    }
    return 1U;
}

static uint8_t drv_imu_port_spi_dma_is_ready(void)
{
    return hspi1.State == HAL_SPI_STATE_READY
        && hspi1.hdmarx && hspi1.hdmatx
        && HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_READY
        && HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_READY;
}

static void drv_imu_port_abort_with_error(void)
{
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_ABORTING;
    s_imu_port_abort_count++;
    if (bsp_spi_abort_dma(&hspi1) != HAL_OK) {
        drv_imu_port_finish_with_error();
    }
}

static void drv_imu_port_finish_with_error(void)
{
    drv_imu_port_acc_cs_write(NULL, 1U);
    drv_imu_port_gyro_cs_write(NULL, 1U);
    s_imu_port_dma_state = DRV_IMU_PORT_DMA_IDLE;
    s_imu_port_spi_error_code = hspi1.ErrorCode;
    s_imu_port_dma_rx_error_code = hspi1.hdmarx ? hspi1.hdmarx->ErrorCode : HAL_DMA_ERROR_NO_XFER;
    s_imu_port_dma_tx_error_code = hspi1.hdmatx ? hspi1.hdmatx->ErrorCode : HAL_DMA_ERROR_NO_XFER;
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
