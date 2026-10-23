#include "drv_mag_port.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_tim.h"
#include <stddef.h>
#include <string.h>

/* CubeMX configures PA8/PC9 for I2C3, PG6 for active-low reset and PG3 for
 * falling-edge DRDY. This port owns the sensor protocol and sample timing.
 */
#define MAG_RESET_PORT GPIOG
#define MAG_RESET_PIN  GPIO_PIN_6
#define MAG_DRDY_PIN   GPIO_PIN_3
#define MAG_RESET_HOLD_MS   50U
#define MAG_RESET_SETTLE_MS 50U
#define MAG_I2C_TIMEOUT_MS  3U
#define MAG_STALE_MS        100U
#define MAG_RETRY_MS        500U

typedef enum {
    MAG_IO_NONE = 0,
    MAG_IO_ID,
    MAG_IO_CONFIG_WRITE,
    MAG_IO_CONFIG_VERIFY,
    MAG_IO_START,
    MAG_IO_STATUS,
    MAG_IO_DATA
} mag_io_kind_t;

typedef struct { uint8_t reg, value; } mag_setting_t;
static const mag_setting_t s_settings[] = {
    {0x0AU, 0x00U}, {0x0CU, 0x00U}, {0x40U, 0x00U},
    {0x0BU, 0x08U}, {0x41U, 0x24U}, {0x42U, 0xC0U}
};

static struct {
    volatile uint8_t data_ready;
    volatile uint32_t data_ready_tick;
    volatile uint8_t io_done;
    volatile HAL_StatusTypeDef io_status;
    uint8_t io_buffer[7];
    uint8_t tx_byte;
    uint8_t stat1;
    uint8_t io_active, io_aborting;
    mag_io_kind_t io_kind;
    drv_mag_snapshot_t snapshot, published;
    uint32_t state_since_ms, last_poll_ms, io_started_ms, sample_tick_ms;
    uint8_t config_index, config_phase;
} s_magnetometer;

static IST8310_Status_t map_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK: return IST8310_OK;
    case HAL_BUSY: return IST8310_ERR_BUSY;
    case HAL_TIMEOUT: return IST8310_ERR_TIMEOUT;
    default: return IST8310_ERR_IO;
    }
}

static void set_reset(bool asserted)
{
    bsp_gpio_write_pin(MAG_RESET_PORT, MAG_RESET_PIN, asserted ? 0U : 1U);
}

static void data_ready_irq(void *context)
{
    (void)context;
    s_magnetometer.data_ready_tick = bsp_tim_get_tick_ms();
    s_magnetometer.data_ready = 1U;
}

static void i2c_complete(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status,
                         void *context)
{
    (void)context;
    if (hi2c != &hi2c3) return;
    s_magnetometer.io_status = status;
    s_magnetometer.io_done = 1U;
}

static bool take_data_ready(uint32_t *tick_ms)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool ready = s_magnetometer.data_ready != 0U;
    if (ready) {
        if (tick_ms != NULL) *tick_ms = s_magnetometer.data_ready_tick;
        s_magnetometer.data_ready = 0U;
    }
    __set_PRIMASK(mask);
    return ready;
}

static void publish_snapshot(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    s_magnetometer.published = s_magnetometer.snapshot;
    __set_PRIMASK(mask);
}

static void begin_reset(uint32_t now_ms)
{
    (void)take_data_ready(NULL);
    set_reset(true);
    s_magnetometer.state_since_ms = now_ms;
    s_magnetometer.snapshot.state = DRV_MAG_RESET_LOW;
    s_magnetometer.snapshot.online = false;
}

static void recover(IST8310_Status_t error, uint32_t now_ms)
{
    s_magnetometer.io_active = 0U;
    s_magnetometer.io_aborting = 0U;
    s_magnetometer.io_kind = MAG_IO_NONE;
    s_magnetometer.io_done = 0U;
    s_magnetometer.snapshot.last_error = error;
    ++s_magnetometer.snapshot.error_count;
    s_magnetometer.snapshot.online = false;
    s_magnetometer.snapshot.state = DRV_MAG_RECOVERING;
    s_magnetometer.state_since_ms = now_ms;
}

static void io_callback_result(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t done = s_magnetometer.io_done;
    HAL_StatusTypeDef status = s_magnetometer.io_status;
    s_magnetometer.io_done = 0U;
    __set_PRIMASK(mask);
    if (!done) return;
    s_magnetometer.io_active = 0U;
    if (s_magnetometer.io_aborting || status != HAL_OK) {
        recover(s_magnetometer.io_aborting ? IST8310_ERR_TIMEOUT : map_status(status),
                bsp_tim_get_tick_ms());
        return;
    }

    switch (s_magnetometer.io_kind) {
    case MAG_IO_ID:
        s_magnetometer.snapshot.device_id = s_magnetometer.io_buffer[0];
        if (s_magnetometer.snapshot.device_id != IST8310_DEVICE_ID) {
            recover(IST8310_ERR_DEVICE_ID, bsp_tim_get_tick_ms());
            return;
        }
        s_magnetometer.config_index = 0U;
        s_magnetometer.config_phase = 1U; /* write first setting */
        break;
    case MAG_IO_CONFIG_WRITE:
        s_magnetometer.config_phase = 2U; /* read it back next tick */
        break;
    case MAG_IO_CONFIG_VERIFY:
        if (s_magnetometer.io_buffer[0] != s_settings[s_magnetometer.config_index].value) {
            recover(IST8310_ERR_VERIFY, bsp_tim_get_tick_ms());
            return;
        }
        ++s_magnetometer.config_index;
        s_magnetometer.config_phase = 1U;
        break;
    case MAG_IO_START:
        s_magnetometer.state_since_ms = bsp_tim_get_tick_ms();
        s_magnetometer.last_poll_ms = s_magnetometer.state_since_ms;
        s_magnetometer.snapshot.state = DRV_MAG_WAIT;
        break;
    case MAG_IO_STATUS:
        s_magnetometer.stat1 = s_magnetometer.io_buffer[0];
        if ((s_magnetometer.stat1 & 0x01U) != 0U) {
            s_magnetometer.snapshot.state = DRV_MAG_READ_DATA;
        } else if ((uint32_t)(bsp_tim_get_tick_ms() - s_magnetometer.state_since_ms)
                   >= IST8310_DEFAULT_TIMEOUT_MS) {
            recover(IST8310_ERR_TIMEOUT, bsp_tim_get_tick_ms());
        } else {
            s_magnetometer.snapshot.state = DRV_MAG_WAIT;
        }
        break;
    case MAG_IO_DATA: {
        IST8310_Data_t sample;
        IST8310_Status_t status = IST8310_DecodeSample(
            s_magnetometer.stat1, s_magnetometer.io_buffer, &sample);
        if (status != IST8310_OK) {
            recover(status, bsp_tim_get_tick_ms());
            return;
        }
        s_magnetometer.snapshot.sample = sample;
        ++s_magnetometer.snapshot.sample_count;
        s_magnetometer.snapshot.last_update_ms = s_magnetometer.sample_tick_ms;
        s_magnetometer.snapshot.valid = true;
        s_magnetometer.snapshot.online = true;
        s_magnetometer.snapshot.state = DRV_MAG_START;
        break;
    }
    default:
        break;
    }
    s_magnetometer.io_kind = MAG_IO_NONE;
}

static HAL_StatusTypeDef start_read(uint8_t reg, uint8_t length,
                                    mag_io_kind_t kind, uint32_t now_ms)
{
    bsp_i2c_transfer_t transfer = {
        .hi2c = &hi2c3,
        .direction = BSP_I2C_READ,
        .dev_addr = IST8310_I2C_ADDRESS_7BIT << 1U,
        .reg_addr = reg,
        .reg_addr_size = I2C_MEMADD_SIZE_8BIT,
        .data = s_magnetometer.io_buffer,
        .length = length,
    };
    HAL_StatusTypeDef status = bsp_i2c_transfer_async(&transfer);
    if (status == HAL_OK) {
        s_magnetometer.io_active = 1U;
        s_magnetometer.io_started_ms = now_ms;
        s_magnetometer.io_kind = kind;
    }
    return status;
}

static HAL_StatusTypeDef start_write(uint8_t reg, uint8_t value,
                                     mag_io_kind_t kind, uint32_t now_ms)
{
    s_magnetometer.tx_byte = value;
    bsp_i2c_transfer_t transfer = {
        .hi2c = &hi2c3,
        .direction = BSP_I2C_WRITE,
        .dev_addr = IST8310_I2C_ADDRESS_7BIT << 1U,
        .reg_addr = reg,
        .reg_addr_size = I2C_MEMADD_SIZE_8BIT,
        .data = &s_magnetometer.tx_byte,
        .length = 1U,
    };
    HAL_StatusTypeDef status = bsp_i2c_transfer_async(&transfer);
    if (status == HAL_OK) {
        s_magnetometer.io_active = 1U;
        s_magnetometer.io_started_ms = now_ms;
        s_magnetometer.io_kind = kind;
    }
    return status;
}

static void process_io(uint32_t now_ms)
{
    if (s_magnetometer.io_active) {
        io_callback_result();
        if (!s_magnetometer.io_active) return;
        if ((uint32_t)(now_ms - s_magnetometer.io_started_ms) >= MAG_I2C_TIMEOUT_MS
            && !s_magnetometer.io_aborting) {
            HAL_StatusTypeDef status = HAL_I2C_Master_Abort_IT(
                &hi2c3, IST8310_I2C_ADDRESS_7BIT << 1U);
            s_magnetometer.io_aborting = 1U;
            if (status != HAL_OK) recover(IST8310_ERR_TIMEOUT, now_ms);
        }
        io_callback_result();
        return;
    }

    HAL_StatusTypeDef status = HAL_OK;
    switch (s_magnetometer.snapshot.state) {
    case DRV_MAG_CONFIGURE:
        if (s_magnetometer.config_phase == 0U) {
            status = start_read(0x00U, 1U, MAG_IO_ID, now_ms);
            if (status == HAL_OK) s_magnetometer.config_phase = 0xFFU;
        } else if (s_magnetometer.config_phase == 1U) {
            if (s_magnetometer.config_index >= sizeof(s_settings) / sizeof(s_settings[0])) {
                s_magnetometer.snapshot.state = DRV_MAG_START;
                return;
            }
            status = start_write(s_settings[s_magnetometer.config_index].reg,
                                 s_settings[s_magnetometer.config_index].value,
                                 MAG_IO_CONFIG_WRITE, now_ms);
        } else if (s_magnetometer.config_phase == 2U) {
            status = start_read(s_settings[s_magnetometer.config_index].reg, 1U,
                                MAG_IO_CONFIG_VERIFY, now_ms);
        }
        break;
    case DRV_MAG_START:
        (void)take_data_ready(NULL);
        status = start_write(0x0AU, 0x01U, MAG_IO_START, now_ms);
        break;
    case DRV_MAG_WAIT: {
        uint32_t ready_tick = 0U;
        bool ready = take_data_ready(&ready_tick);
        uint32_t elapsed = now_ms - s_magnetometer.state_since_ms;
        if (elapsed < IST8310_MIN_SAMPLE_INTERVAL_MS) return;
        if (!ready && now_ms == s_magnetometer.last_poll_ms) return;
        if (ready) s_magnetometer.sample_tick_ms = ready_tick;
        else s_magnetometer.sample_tick_ms = now_ms;
        s_magnetometer.last_poll_ms = now_ms;
        status = start_read(0x02U, 1U, MAG_IO_STATUS, now_ms);
        break;
    }
    case DRV_MAG_READ_DATA:
        /* STAT1 is captured separately so the seven-byte burst remains atomic. */
        status = start_read(0x03U, 7U, MAG_IO_DATA, now_ms);
        break;
    default:
        break;
    }
    if (status != HAL_OK && status != HAL_BUSY)
        recover(map_status(status), now_ms);
}

IST8310_Status_t drv_mag_port_init(void)
{
    memset(&s_magnetometer, 0, sizeof(s_magnetometer));
    if (bsp_i2c_async_register(&hi2c3, i2c_complete, &s_magnetometer) != HAL_OK)
        return IST8310_ERR_IO;
    if (bsp_gpio_exti_register(MAG_DRDY_PIN, data_ready_irq, &s_magnetometer) != 0U)
        return IST8310_ERR_BUSY;
    s_magnetometer.snapshot.state = DRV_MAG_DISABLED;
    begin_reset(bsp_tim_get_tick_ms());
    publish_snapshot();
    return IST8310_OK;
}

void drv_mag_port_process(void)
{
    uint32_t now_ms = bsp_tim_get_tick_ms();
    switch (s_magnetometer.snapshot.state) {
    case DRV_MAG_RESET_LOW:
        if ((uint32_t)(now_ms - s_magnetometer.state_since_ms) >= MAG_RESET_HOLD_MS) {
            set_reset(false);
            s_magnetometer.state_since_ms = now_ms;
            s_magnetometer.config_phase = 0U;
            s_magnetometer.snapshot.state = DRV_MAG_RESET_SETTLE;
        }
        break;
    case DRV_MAG_RESET_SETTLE:
        if ((uint32_t)(now_ms - s_magnetometer.state_since_ms) >= MAG_RESET_SETTLE_MS)
            s_magnetometer.snapshot.state = DRV_MAG_CONFIGURE;
        break;
    case DRV_MAG_RECOVERING:
        if ((uint32_t)(now_ms - s_magnetometer.state_since_ms) >= MAG_RETRY_MS) {
            ++s_magnetometer.snapshot.recovery_count;
            begin_reset(now_ms);
        }
        break;
    default:
        process_io(now_ms);
        break;
    }
    publish_snapshot();
}

bool drv_mag_port_read_snapshot(drv_mag_snapshot_t *out)
{
    if (out == NULL) return false;
    uint32_t now_ms = bsp_tim_get_tick_ms();
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = s_magnetometer.published;
    __set_PRIMASK(mask);
    out->online = out->online && out->valid &&
                  (uint32_t)(now_ms - out->last_update_ms) < MAG_STALE_MS;
    return out->state != DRV_MAG_DISABLED;
}
