#include "drv_mag.h"

#include "i2c.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define IST8310_REG_WHO_AM_I 0x00U
#define IST8310_REG_STAT1    0x02U
#define IST8310_REG_DATAXL   0x03U
#define IST8310_REG_STAT2    0x09U
#define IST8310_REG_CNTL1    0x0AU
#define IST8310_REG_CNTL2    0x0BU
#define IST8310_REG_STR      0x0CU
#define IST8310_REG_AVGCNTL  0x41U
#define IST8310_REG_PDCNTL   0x42U


#define IST8310_STAT1_DRDY 0x01U
#define IST8310_STAT1_DOR  0x02U
#define IST8310_STAT2_INT  0x08U

#define IST8310_CNTL1_SINGLE_MEASUREMENT 0x01U
#define IST8310_CNTL2_DRDY_ACTIVE_LOW    0x08U
#define IST8310_CNTL2_SOFT_RESET         0x01U
#define IST8310_STR_SELF_TEST            0x40U
#define IST8310_AVGCNTL_LOW_NOISE        0x24U
#define IST8310_PDCNTL_NORMAL            0xC0U

#define IST8310_HARD_RESET_DELAY_MS 50U
#define IST8310_SOFT_RESET_DELAY_MS 50U
#define IST8310_SELF_TEST_MIN_TOLERANCE_LSB 5U
#define IST8310_SELF_TEST_TOLERANCE_PERCENT 10U

IST8310_Handle_t g_ist8310;
IST8310_Data_t g_ist8310_data;
IST8310_Status_t g_ist8310_status = IST8310_ERROR_NOT_INITIALIZED;
volatile uint32_t g_ist8310_sample_count = 0U;
volatile uint32_t g_ist8310_last_update_ms = 0U;

static volatile uint32_t ist8310_start_stage = 0U;

static volatile IST8310_Status_t ist8310_init_status =
    IST8310_ERROR_NOT_INITIALIZED;

static volatile IST8310_Status_t ist8310_self_test_status =
    IST8310_ERROR_NOT_INITIALIZED;

static IST8310_SelfTestResult_t ist8310_self_test_result;


/* BoardUpdate owns recovery; never reset/delay inside the timer ISR. */
static bool board_restart_pending = false;
static uint32_t board_wait_started_ms = 0U;

//检查基础配置，hi2c、reset端口G、reset引脚6是否有效
static bool IST8310_ConfigIsValid(const IST8310_Handle_t *dev)
{
    return (dev != NULL) &&
           (dev->config.hi2c != NULL) &&
           (dev->config.reset_port != NULL) &&
           (dev->config.reset_pin != 0U);
}

//读取状态寄存器
static IST8310_Status_t IST8310_ReadRegister(IST8310_Handle_t *dev,
                                             uint8_t reg,
                                             uint8_t *value)
{
    if (!IST8310_ConfigIsValid(dev) || (value == NULL))
    {
        return IST8310_ERROR_PARAMETER;
    }
    //读取寄存器，然后储存到value中，读取一个字节
    if (HAL_I2C_Mem_Read(dev->config.hi2c,dev->config.i2c_address, reg, I2C_MEMADD_SIZE_8BIT,value, 1U,dev->config.timeout_ms) != HAL_OK)
    {
        return IST8310_ERROR_I2C;
    }

    return IST8310_STATUS_OK;
}

//读取数据寄存器
static IST8310_Status_t IST8310_ReadRegisters(IST8310_Handle_t *dev,
                                              uint8_t first_reg,
                                              uint8_t *buffer,
                                              uint16_t length)
{
    if (!IST8310_ConfigIsValid(dev) || (buffer == NULL) || (length == 0U))
    {
        return IST8310_ERROR_PARAMETER;
    }

    if (HAL_I2C_Mem_Read(dev->config.hi2c,
                         dev->config.i2c_address,
                         first_reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         length,
                         dev->config.timeout_ms) != HAL_OK)
    {
        return IST8310_ERROR_I2C;
    }

    return IST8310_STATUS_OK;
}

static IST8310_Status_t IST8310_WriteRegister(IST8310_Handle_t *dev,
                                              uint8_t reg,
                                              uint8_t value)
{
    if (!IST8310_ConfigIsValid(dev))
    {
        return IST8310_ERROR_PARAMETER;
    }

    if (HAL_I2C_Mem_Write(dev->config.hi2c,
                          dev->config.i2c_address,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1U,
                          dev->config.timeout_ms) != HAL_OK)
    {
        return IST8310_ERROR_I2C;
    }

    return IST8310_STATUS_OK;
}

//检查寄存器写入是否成功，写入后读取寄存器值进行比对
static IST8310_Status_t IST8310_WriteAndVerify(IST8310_Handle_t *dev,
                                               uint8_t reg,
                                               uint8_t value)
{
    uint8_t readback = 0U;
    IST8310_Status_t status = IST8310_WriteRegister(dev, reg, value);

    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    status = IST8310_ReadRegister(dev, reg, &readback);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    if (readback != value)
    {
        return IST8310_ERROR_REGISTER_VERIFY;
    }

    return IST8310_STATUS_OK;
}

//解码低位在前，高位在后，组成16位有符号整数
static int16_t IST8310_DecodeInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}
//解码
static void IST8310_DecodeRaw(const uint8_t buffer[6],
                              IST8310_RawData_t *raw)
{
    raw->x = IST8310_DecodeInt16(buffer[0], buffer[1]);
    raw->y = IST8310_DecodeInt16(buffer[2], buffer[3]);
    raw->z = IST8310_DecodeInt16(buffer[4], buffer[5]);
}

//判断是否发生正负反转
static bool IST8310_PolarityReversed(int16_t normal, int16_t reversed)
{
    return ((normal > 0) && (reversed < 0)) ||
           ((normal < 0) && (reversed > 0));
}

//比较自检前后绝对值；使用32位运算避免-32768取绝对值时溢出
static uint32_t IST8310_AbsoluteInt16(int16_t value)
{
    const int32_t extended = (int32_t)value;
    return (extended < 0) ? (uint32_t)(-extended) : (uint32_t)extended;
}

//手册要求反转前后绝对值相同，实际测量允许5 LSB或10%的误差
static bool IST8310_SelfTestMagnitudeMatches(int16_t normal,
                                             int16_t reversed)
{
    const uint32_t normal_abs = IST8310_AbsoluteInt16(normal);
    const uint32_t reversed_abs = IST8310_AbsoluteInt16(reversed);
    const uint32_t reference = (normal_abs > reversed_abs) ?
        normal_abs : reversed_abs;
    const uint32_t difference = (normal_abs > reversed_abs) ?
        (normal_abs - reversed_abs) : (reversed_abs - normal_abs);
    uint32_t tolerance =
        (reference * IST8310_SELF_TEST_TOLERANCE_PERCENT) / 100U;

    if (tolerance < IST8310_SELF_TEST_MIN_TOLERANCE_LSB)
    {
        tolerance = IST8310_SELF_TEST_MIN_TOLERANCE_LSB;
    }

    return difference <= tolerance;
}

//判断传感器是否能正常使用（轮询DRDY）
static IST8310_Status_t IST8310_WaitAndReadRaw(IST8310_Handle_t *dev,
                                               IST8310_RawData_t *raw)
{
    uint8_t stat1 = 0U;
    uint8_t buffer[6];
    uint32_t start_tick;
    IST8310_Status_t status;

    if (raw == NULL)
    {
        return IST8310_ERROR_PARAMETER;
    }

    status = IST8310_StartMeasurement(dev);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    //记录开始等待传感器数据的时间点
    start_tick = HAL_GetTick();
    do
    {
        status = IST8310_ReadRegister(dev, IST8310_REG_STAT1, &stat1);
        if (status != IST8310_STATUS_OK)
        {
            return status;
        }

        if ((stat1 & IST8310_STAT1_DRDY) != 0U)
        {
            //一下子读取6个寄存器，DATAXL、DATAXH、DATAYL、DATAYH、DATAZL、DATAZH
            status = IST8310_ReadRegisters(dev,
                                           IST8310_REG_DATAXL,
                                           buffer,
                                           (uint16_t)sizeof(buffer));
            if (status != IST8310_STATUS_OK)
            {
                return status;
            }

            IST8310_DecodeRaw(buffer, raw);
            return IST8310_STATUS_OK;
        }

        HAL_Delay(1U);
    } while ((HAL_GetTick() - start_tick) < dev->config.timeout_ms);

    return IST8310_ERROR_TIMEOUT;
}

//进行初始化，配置I2C、复位引脚、I2C地址、超时时间等参数，进行硬件复位，检查设备ID，配置寄存器
IST8310_Status_t IST8310_Init(IST8310_Handle_t *dev,
                              const IST8310_Config_t *config)
{
    IST8310_Status_t status;

    if ((dev == NULL) || (config == NULL) ||
        (config->hi2c == NULL) || (config->reset_port == NULL) ||
        (config->reset_pin == 0U))
    {
        return IST8310_ERROR_PARAMETER;
    }
    memset(dev, 0, sizeof(*dev));
    dev->config = *config;
    if (dev->config.i2c_address == 0U)
    {
        dev->config.i2c_address = IST8310_I2C_ADDRESS_HAL;
    }
    if (dev->config.timeout_ms == 0U)
    {
        dev->config.timeout_ms = IST8310_DEFAULT_TIMEOUT_MS;
    }

    status = IST8310_HardwareReset(dev);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    status = IST8310_CheckDevice(dev);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    status = IST8310_WriteAndVerify(dev,
                                    IST8310_REG_CNTL2,
                                    IST8310_CNTL2_DRDY_ACTIVE_LOW);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    status = IST8310_WriteAndVerify(dev,
                                    IST8310_REG_AVGCNTL,
                                    IST8310_AVGCNTL_LOW_NOISE);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    status = IST8310_WriteAndVerify(dev,
                                    IST8310_REG_PDCNTL,
                                    IST8310_PDCNTL_NORMAL);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    dev->initialized = true;
    status = IST8310_StartMeasurement(dev);
    if (status != IST8310_STATUS_OK)
    {
        dev->initialized = false;
        return status;
    }

    return IST8310_STATUS_OK;
}

//硬件复位，拉低复位引脚50ms，再拉高50ms
IST8310_Status_t IST8310_HardwareReset(IST8310_Handle_t *dev)
{
    if (!IST8310_ConfigIsValid(dev))
    {
        return IST8310_ERROR_PARAMETER;
    }

    HAL_GPIO_WritePin(dev->config.reset_port,
                      dev->config.reset_pin,
                      GPIO_PIN_RESET);
    HAL_Delay(IST8310_HARD_RESET_DELAY_MS);
    HAL_GPIO_WritePin(dev->config.reset_port,
                      dev->config.reset_pin,
                      GPIO_PIN_SET);
    HAL_Delay(IST8310_HARD_RESET_DELAY_MS);

    dev->initialized = false;
    dev->irq_data_ready = false;
    return IST8310_STATUS_OK;
}

//软件复位，写入寄存器0x0B，拉低复位引脚50ms，再拉高50ms
IST8310_Status_t IST8310_SoftReset(IST8310_Handle_t *dev)
{
    IST8310_Status_t status;

    if (!IST8310_ConfigIsValid(dev))
    {
        return IST8310_ERROR_PARAMETER;
    }

    status = IST8310_WriteRegister(dev,
                                   IST8310_REG_CNTL2,
                                   IST8310_CNTL2_SOFT_RESET);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    HAL_Delay(IST8310_SOFT_RESET_DELAY_MS);
    dev->initialized = false;
    dev->irq_data_ready = false;
    return IST8310_STATUS_OK;
}

//检查设备ID是否正确
IST8310_Status_t IST8310_CheckDevice(IST8310_Handle_t *dev)
{
    uint8_t device_id = 0U;
    IST8310_Status_t status = IST8310_ReadRegister(dev,
                                                   IST8310_REG_WHO_AM_I,
                                                   &device_id);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    dev->device_id = device_id;
    if (device_id != IST8310_DEVICE_ID)
    {
        return IST8310_ERROR_DEVICE_ID;
    }

    return IST8310_STATUS_OK;
}

//写入单次测量模式寄存器CNTL1（0x0A），启动单次测量模式
IST8310_Status_t IST8310_StartMeasurement(IST8310_Handle_t *dev)
{
    if (!IST8310_ConfigIsValid(dev))
    {
        return IST8310_ERROR_PARAMETER;
    }
    if (!dev->initialized)
    {
        return IST8310_ERROR_NOT_INITIALIZED;
    }

    //写入寄存器0x0A，启动单次测量模式
    return IST8310_WriteRegister(dev,
                                 IST8310_REG_CNTL1,
                                 IST8310_CNTL1_SINGLE_MEASUREMENT);
}

//判断数据是否已经准备好了，读取寄存器STAT1（0x02），判断bit0是否为1
IST8310_Status_t IST8310_IsDataReady(IST8310_Handle_t *dev,
                                     bool *ready)
{
    uint8_t stat1 = 0U;
    IST8310_Status_t status;

    if ((dev == NULL) || (ready == NULL))
    {
        return IST8310_ERROR_PARAMETER;
    }
    if (!dev->initialized)
    {
        return IST8310_ERROR_NOT_INITIALIZED;
    }

    status = IST8310_ReadRegister(dev, IST8310_REG_STAT1, &stat1);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    *ready = ((stat1 & IST8310_STAT1_DRDY) != 0U);
    return IST8310_STATUS_OK;
}

//判断dev是否有效
void IST8310_NotifyDataReadyFromISR(IST8310_Handle_t *dev)
{
    if (dev != NULL)
    {
        dev->irq_data_ready = true;
    }
}

//单次测量读取数据
IST8310_Status_t IST8310_Read(IST8310_Handle_t *dev,
                              IST8310_Data_t *data)
{
    uint8_t buffer[6];
    IST8310_Status_t status;
    IST8310_Data_t sample = {0};
    IST8310_Data_t *output = data;

    if ((dev == NULL) || (data == NULL))
    {
        return IST8310_ERROR_PARAMETER;
    }
    if (!dev->initialized)
    {
        return IST8310_ERROR_NOT_INITIALIZED;
    }

    /* Publish only after the entire transaction has succeeded. */
    data = &sample;
    status = IST8310_ReadRegister(dev, IST8310_REG_STAT1, &data->stat1);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    if ((data->stat1 & IST8310_STAT1_DRDY) == 0U)
    {
        dev->irq_data_ready = false;
        return IST8310_STATUS_NOT_READY;
    }

    //判断是否溢出
    data->overrun = ((data->stat1 & IST8310_STAT1_DOR) != 0U);

    status = IST8310_ReadRegisters(dev,
                                   IST8310_REG_DATAXL,
                                   buffer,
                                   (uint16_t)sizeof(buffer));
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }
    IST8310_DecodeRaw(buffer, &data->raw);

    status = IST8310_ReadRegister(dev, IST8310_REG_STAT2, &data->stat2);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }
    //判断磁场中断是否触发
    data->field_interrupt = ((data->stat2 & IST8310_STAT2_INT) != 0U);

    data->x_uT = (float)data->raw.x * IST8310_SENSITIVITY_UT_PER_LSB;
    data->y_uT = (float)data->raw.y * IST8310_SENSITIVITY_UT_PER_LSB;
    data->z_uT = (float)data->raw.z * IST8310_SENSITIVITY_UT_PER_LSB;
    data->norm_uT = sqrtf((data->x_uT * data->x_uT) +
                          (data->y_uT * data->y_uT) +
                          (data->z_uT * data->z_uT));

    dev->irq_data_ready = false;
    status = IST8310_StartMeasurement(dev);
    if (status == IST8310_STATUS_OK)
    {
        *output = sample;
    }
    return status;
}

//自检模式
IST8310_Status_t IST8310_SelfTest(IST8310_Handle_t *dev,
                                  IST8310_SelfTestResult_t *result)
{
    IST8310_Status_t status;
    IST8310_Status_t restore_status;
    bool x_magnitude_matches;
    bool y_magnitude_matches;
    bool z_magnitude_matches;

    if ((dev == NULL) || (result == NULL))
    {
        return IST8310_ERROR_PARAMETER;
    }
    if (!dev->initialized)
    {
        return IST8310_ERROR_NOT_INITIALIZED;
    }

    memset(result, 0, sizeof(*result));

    //测试，防止自检寄存器未关闭
    status = IST8310_WriteRegister(dev, IST8310_REG_STR, 0x00U);
    if (status == IST8310_STATUS_OK)
    {
        status = IST8310_WaitAndReadRaw(dev, &result->normal);
    }
    if (status == IST8310_STATUS_OK)
    {
        status = IST8310_WriteRegister(dev,
                                       IST8310_REG_STR,
                                       IST8310_STR_SELF_TEST);
    }
    if (status == IST8310_STATUS_OK)
    {
        status = IST8310_WaitAndReadRaw(dev, &result->reversed);
    }
    //关闭自检寄存器
    restore_status = IST8310_WriteRegister(dev, IST8310_REG_STR, 0x00U);
    if ((status == IST8310_STATUS_OK) &&
        (restore_status != IST8310_STATUS_OK))
    {
        status = restore_status;
    }

    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    result->x_polarity_reversed = IST8310_PolarityReversed(
        result->normal.x, result->reversed.x);
    result->y_polarity_reversed = IST8310_PolarityReversed(
        result->normal.y, result->reversed.y);
    result->z_polarity_reversed = IST8310_PolarityReversed(
        result->normal.z, result->reversed.z);

    x_magnitude_matches = IST8310_SelfTestMagnitudeMatches(
        result->normal.x, result->reversed.x);
    y_magnitude_matches = IST8310_SelfTestMagnitudeMatches(
        result->normal.y, result->reversed.y);
    z_magnitude_matches = IST8310_SelfTestMagnitudeMatches(
        result->normal.z, result->reversed.z);

    status = IST8310_StartMeasurement(dev);
    if (status != IST8310_STATUS_OK)
    {
        return status;
    }

    if (!result->x_polarity_reversed ||
        !result->y_polarity_reversed ||
        !result->z_polarity_reversed ||
        !x_magnitude_matches ||
        !y_magnitude_matches ||
        !z_magnitude_matches)
    {
        return IST8310_ERROR_SELF_TEST;
    }

    return IST8310_STATUS_OK;
}

//基础配置
IST8310_Status_t IST8310_BoardInit(void)
{
    const IST8310_Config_t config = {
        .hi2c = &hi2c3,
        .reset_port = GPIOG,
        .reset_pin = GPIO_PIN_6,
        .i2c_address = IST8310_I2C_ADDRESS_HAL,
        .timeout_ms = IST8310_DEFAULT_TIMEOUT_MS
    };

    memset(&g_ist8310_data, 0, sizeof(g_ist8310_data));
    g_ist8310_sample_count = 0U;
    g_ist8310_last_update_ms = 0U;
    g_ist8310_status = IST8310_Init(&g_ist8310, &config);
    board_restart_pending = false;
    board_wait_started_ms = HAL_GetTick();
    return g_ist8310_status;
}


void IST8310_BoardIRQHandler(uint16_t gpio_pin)
{
    if (gpio_pin == GPIO_PIN_3)
    {
        IST8310_NotifyDataReadyFromISR(&g_ist8310);
    }
}
IST8310_Status_t IST8310_BoardUpdate(void)
{
    if (!g_ist8310.initialized)
    {
        g_ist8310_status = IST8310_ERROR_NOT_INITIALIZED;
        return g_ist8310_status;
    }

    /* Retry a failed/partially completed transaction on the next timer tick. */
    if (board_restart_pending)
    {
        g_ist8310.irq_data_ready = false;
        g_ist8310_status = IST8310_StartMeasurement(&g_ist8310);
        if (g_ist8310_status == IST8310_STATUS_OK)
        {
            board_restart_pending = false;
            board_wait_started_ms = HAL_GetTick();
            g_ist8310_status = IST8310_STATUS_NOT_READY;
        }
        return g_ist8310_status;
    }

    if (!g_ist8310.irq_data_ready)
    {
        if ((uint32_t)(HAL_GetTick() - board_wait_started_ms) <
            g_ist8310.config.timeout_ms)
        {
            g_ist8310_status = IST8310_STATUS_NOT_READY;
            return g_ist8310_status;
        }

        /* A missing EXTI edge must not permanently stop acquisition. */
        bool ready = false;
        g_ist8310_status = IST8310_IsDataReady(&g_ist8310, &ready);
        if ((g_ist8310_status != IST8310_STATUS_OK) || !ready)
        {
            board_restart_pending = true;
            if (g_ist8310_status == IST8310_STATUS_OK)
            {
                g_ist8310_status = IST8310_ERROR_TIMEOUT;
            }
            return g_ist8310_status;
        }
    }

    g_ist8310_status = IST8310_Read(&g_ist8310, &g_ist8310_data);
    if (g_ist8310_status == IST8310_STATUS_OK)
    {
        ++g_ist8310_sample_count;
        g_ist8310_last_update_ms = HAL_GetTick();
        board_wait_started_ms = g_ist8310_last_update_ms;
    }
    else if (g_ist8310_status < IST8310_STATUS_OK)
    {
        board_restart_pending = true;
    }

    return g_ist8310_status;
}

void IST8310_AppStart(void)
{
    ist8310_start_stage = 1U;
    ist8310_init_status = IST8310_BoardInit();

    if (ist8310_init_status != IST8310_STATUS_OK)
    {
        Error_Handler();
        return;
    }

    ist8310_start_stage = 2U;
    ist8310_self_test_status =
        IST8310_SelfTest(&g_ist8310,
                         &ist8310_self_test_result);

    g_ist8310_status = ist8310_self_test_status;

    if (ist8310_self_test_status != IST8310_STATUS_OK)
    {
        Error_Handler();
        return;
    }

    /*
     * Control System不使用TIM2。
     * 自检函数结束前已经重新启动了一次正常测量。
     */
    ist8310_start_stage = 4U;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    IST8310_BoardIRQHandler(GPIO_Pin);
}
