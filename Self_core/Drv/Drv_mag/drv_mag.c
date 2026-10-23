#include "drv_mag.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

enum {
    REG_WHO_AM_I = 0x00, REG_STAT1 = 0x02, REG_DATA = 0x03,
    REG_CNTL1 = 0x0A, REG_CNTL2 = 0x0B, REG_STR = 0x0C,
    REG_TCCNTL = 0x40, REG_AVGCNTL = 0x41, REG_PDCNTL = 0x42
};

static IST8310_Status_t write_reg(IST8310_Handle_t *dev, uint8_t reg, uint8_t value)
{
    return dev->config.write(dev->config.context, reg, &value, 1U);
}

static IST8310_Status_t write_verified(IST8310_Handle_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t actual = 0U;
    IST8310_Status_t status = write_reg(dev, reg, value);
    if (status != IST8310_OK) return status;
    status = dev->config.read(dev->config.context, reg, &actual, 1U);
    if (status != IST8310_OK) return status;
    return actual == value ? IST8310_OK : IST8310_ERR_VERIFY;
}

static IST8310_Status_t check_device(const IST8310_Handle_t *dev)
{
    if (dev == NULL) return IST8310_ERR_ARGUMENT;
    return dev->initialized ? IST8310_OK : IST8310_ERR_NOT_INITIALIZED;
}

static int16_t decode_axis(const uint8_t *bytes)
{
    uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
    return (int16_t)((value & 0x8000U) ? (int32_t)value - 65536 : value);
}

IST8310_Status_t IST8310_Init(IST8310_Handle_t *dev, const IST8310_Config_t *config)
{
    if (dev == NULL) return IST8310_ERR_ARGUMENT;
    if (config == NULL || config->read == NULL || config->write == NULL) {
        dev->initialized = false;
        return IST8310_ERR_ARGUMENT;
    }
    /* Copy first to also allow reinitialization with &dev->config. */
    IST8310_Config_t bus = *config;
    memset(dev, 0, sizeof(*dev));
    dev->config = bus;
    IST8310_Status_t status = bus.read(bus.context, REG_WHO_AM_I, &dev->device_id, 1U);
    if (status != IST8310_OK) return status;
    if (dev->device_id != IST8310_DEVICE_ID) return IST8310_ERR_DEVICE_ID;
    static const uint8_t settings[][2] = {
        {REG_CNTL1, 0x00U}, {REG_STR, 0x00U}, {REG_TCCNTL, 0x00U},
        {REG_CNTL2, 0x08U}, /* DRDY enabled, active low -> falling-edge EXTI. */
        {REG_AVGCNTL, 0x24U}, {REG_PDCNTL, 0xC0U}
    };
    for (unsigned i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        status = write_verified(dev, settings[i][0], settings[i][1]);
        if (status != IST8310_OK) return status;
    }
    dev->initialized = true;
    return IST8310_OK;
}

IST8310_Status_t IST8310_StartMeasurement(IST8310_Handle_t *dev)
{
    IST8310_Status_t status = check_device(dev);
    return status == IST8310_OK ? write_reg(dev, REG_CNTL1, 0x01U) : status;
}

IST8310_Status_t IST8310_Read(IST8310_Handle_t *dev, IST8310_Data_t *data)
{
    if (data == NULL) return IST8310_ERR_ARGUMENT;
    IST8310_Status_t status = check_device(dev);
    if (status != IST8310_OK) return status;
    uint8_t stat1 = 0U;
    status = dev->config.read(dev->config.context, REG_STAT1, &stat1, 1U);
    if (status != IST8310_OK) return status;
    if ((stat1 & 0x01U) == 0U) return IST8310_NOT_READY;
    /* Burst includes all axes and STAT2, releasing the device DRDY. */
    uint8_t bytes[7];
    status = dev->config.read(dev->config.context, REG_DATA, bytes, sizeof(bytes));
    if (status != IST8310_OK) return status;
    status = IST8310_DecodeSample(stat1, bytes, data);
    return status;
}

IST8310_Status_t IST8310_DecodeSample(uint8_t stat1, const uint8_t bytes[7],
                                      IST8310_Data_t *data)
{
    if (bytes == NULL || data == NULL) return IST8310_ERR_ARGUMENT;
    if ((stat1 & 0x01U) == 0U) return IST8310_NOT_READY;
    IST8310_Data_t sample = {0};
    sample.raw.x = decode_axis(&bytes[0]);
    sample.raw.y = decode_axis(&bytes[2]);
    sample.raw.z = decode_axis(&bytes[4]);
    sample.x_uT = sample.raw.x * IST8310_SENSITIVITY_UT_PER_LSB;
    sample.y_uT = sample.raw.y * IST8310_SENSITIVITY_UT_PER_LSB;
    sample.z_uT = sample.raw.z * IST8310_SENSITIVITY_UT_PER_LSB;
    sample.norm_uT = sqrtf(sample.x_uT * sample.x_uT + sample.y_uT * sample.y_uT
                           + sample.z_uT * sample.z_uT);
    sample.stat1 = stat1;
    sample.stat2 = bytes[6];
    sample.overrun = (stat1 & 0x02U) != 0U;
    sample.field_interrupt = (sample.stat2 & 0x08U) != 0U;
    *data = sample;
    return IST8310_OK;
}

IST8310_Status_t IST8310_SoftReset(IST8310_Handle_t *dev)
{
    IST8310_Status_t status = check_device(dev);
    if (status != IST8310_OK) return status;
    dev->initialized = false;
    return write_reg(dev, REG_CNTL2, 0x01U);
}

static IST8310_Status_t diagnostic_measurement(IST8310_Handle_t *dev, IST8310_RawData_t *raw)
{
    IST8310_Status_t status = IST8310_StartMeasurement(dev);
    if (status != IST8310_OK) return status;
    for (uint32_t waited = 0U; waited < IST8310_DEFAULT_TIMEOUT_MS; ++waited) {
        dev->config.delay_ms(dev->config.context, 1U);
        if (waited + 1U < IST8310_MIN_SAMPLE_INTERVAL_MS) continue;
        IST8310_Data_t sample;
        status = IST8310_Read(dev, &sample);
        if (status == IST8310_OK) { *raw = sample.raw; return status; }
        if (status != IST8310_NOT_READY) return status;
    }
    return IST8310_ERR_TIMEOUT;
}

IST8310_Status_t IST8310_SelfTest(IST8310_Handle_t *dev, IST8310_SelfTestResult_t *result)
{
    if (result == NULL) return IST8310_ERR_ARGUMENT;
    IST8310_Status_t status = check_device(dev);
    if (status != IST8310_OK) return status;
    if (dev->config.delay_ms == NULL) return IST8310_ERR_ARGUMENT;
    memset(result, 0, sizeof(*result));
    /* iSentek v1.5 section 3.1.3: disable temperature compensation for self-test. */
    status = write_verified(dev, REG_CNTL1, 0x00U);
    if (status == IST8310_OK) status = write_verified(dev, REG_TCCNTL, 0x01U);
    if (status == IST8310_OK) status = write_verified(dev, REG_STR, 0x00U);
    if (status == IST8310_OK) status = diagnostic_measurement(dev, &result->normal);
    if (status == IST8310_OK) status = write_verified(dev, REG_STR, 0x40U);
    if (status == IST8310_OK) status = diagnostic_measurement(dev, &result->reversed);
    /* Always attempt all restorations, including on a partial I2C failure. */
    IST8310_Status_t restore_mode = write_verified(dev, REG_CNTL1, 0x00U);
    IST8310_Status_t restore_str = write_verified(dev, REG_STR, 0x00U);
    IST8310_Status_t restore_tc = write_verified(dev, REG_TCCNTL, 0x00U);
    if (restore_mode != IST8310_OK || restore_str != IST8310_OK || restore_tc != IST8310_OK) {
        dev->initialized = false;
        return restore_mode != IST8310_OK ? restore_mode :
               (restore_str != IST8310_OK ? restore_str : restore_tc);
    }
    if (status != IST8310_OK) return status;
    result->x_passed = (int32_t)result->normal.x == -(int32_t)result->reversed.x;
    result->y_passed = (int32_t)result->normal.y == -(int32_t)result->reversed.y;
    result->z_passed = (int32_t)result->normal.z == -(int32_t)result->reversed.z;
    if (!result->x_passed || !result->y_passed || !result->z_passed) return IST8310_ERR_SELF_TEST;
    if (result->normal.x == 0 || result->normal.y == 0 || result->normal.z == 0)
        return IST8310_SELF_TEST_INCONCLUSIVE;
    return IST8310_OK;
}
