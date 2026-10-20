#include "drv_mag.h"
#include "bsp_i2c.h"
#include "i2c.h"
#include "main.h"
#include <string.h>

IST8310_Handle_t g_ist8310;
IST8310_Data_t g_ist8310_data;
volatile uint32_t g_ist8310_sample_count = 0U;
volatile uint32_t g_ist8310_last_update_ms = 0U;
static IST8310_SelfTestResult_t g_ist8310_self_test_result;

void IST8310_ReadRegister(IST8310_Handle_t *dev,uint8_t reg, uint8_t *value)
{
    bsp_i2c_read(dev->config.hi2c,dev->config.i2c_address,reg,
        I2C_MEMADD_SIZE_8BIT, value,1U,dev->config.timeout_ms);

}

void IST8310_ReadRegisters(IST8310_Handle_t *dev, uint8_t first_reg, uint8_t *buffer, uint16_t length)
{
    bsp_i2c_read(dev->config.hi2c,dev->config.i2c_address,first_reg,
    I2C_MEMADD_SIZE_8BIT,buffer,length,dev->config.timeout_ms);
}

void IST8310_WriteRegister(IST8310_Handle_t *dev,uint8_t reg,uint8_t value)
{
    bsp_i2c_write(dev->config.hi2c,dev->config.i2c_address,reg,
    I2C_MEMADD_SIZE_8BIT,&value,1U,dev->config.timeout_ms) ;
}

static void IST8310_DecodeRaw(const uint8_t bytes[6], IST8310_RawData_t *raw)
{
    raw->x = (int16_t)(((uint16_t)bytes[1] << 8U) | bytes[0]);
    raw->y = (int16_t)(((uint16_t)bytes[3] << 8U) | bytes[2]);
    raw->z = (int16_t)(((uint16_t)bytes[5] << 8U) | bytes[4]);
}

static bool IST8310_SelfTestAxisPassed(int16_t normal, int16_t reversed)
{
    int32_t normal_value = normal;
    int32_t reversed_value = reversed;

    if (normal_value < 0)
        normal_value = -normal_value;
    if (reversed_value < 0)
        reversed_value = -reversed_value;

    return normal_value == reversed_value;
}

uint8_t IST8310_WaitAndReadRaw(IST8310_Handle_t *dev, IST8310_RawData_t *raw)
{
    uint8_t stat1 = 0;
    uint8_t bytes[6] = {0};
    uint32_t start_tick;

    dev->irq_data_ready = false;
    IST8310_WriteRegister(dev,0x0A,0x01);

    start_tick = HAL_GetTick();
    do
    {
       IST8310_ReadRegister(dev, 0x02, &stat1);
        if ((stat1 & 0x01) != 0U)
        {
            IST8310_ReadRegisters(dev,0x03,bytes,sizeof(bytes));
            IST8310_DecodeRaw(bytes, raw);
            return 1;
        }
        HAL_Delay(1U);

    } while ((uint32_t)(HAL_GetTick() - start_tick) < dev->config.timeout_ms);
    return 0;
}

void IST8310_Init(IST8310_Handle_t *dev, const IST8310_Config_t *config)
{
    memset(dev, 0, sizeof(*dev));
    dev->config = *config;

    IST8310_HardwareReset(dev);
    IST8310_WriteRegister(dev,0x0B,0x08);//0000 1000
    IST8310_WriteRegister(dev,0x41,0x24);
    IST8310_WriteRegister(dev,0x42,0xC0);

    dev->initialized = true;
    IST8310_WriteRegister(dev,0x0A,0x01);
}

void IST8310_HardwareReset(IST8310_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->config.reset_port,dev->config.reset_pin,GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(dev->config.reset_port,dev->config.reset_pin,GPIO_PIN_SET);
    HAL_Delay(50);

    dev->initialized = false;
    dev->irq_data_ready = false;
}

void IST8310_Read(IST8310_Handle_t *dev,IST8310_Data_t *data)
{
    uint8_t bytes[6];
    IST8310_Data_t sample = {0};

    IST8310_ReadRegisters(dev,0x03,bytes,sizeof(bytes));
    IST8310_DecodeRaw(bytes, &sample.raw);
    sample.x_uT = (float)sample.raw.x * 0.3;
    sample.y_uT = (float)sample.raw.y * 0.3;
    sample.z_uT = (float)sample.raw.z * 0.3;

    *data = sample;

    dev->irq_data_ready = false;
    IST8310_WriteRegister(dev,0x0A,0x01);
}

uint8_t IST8310_SelfTest(IST8310_Handle_t *dev,IST8310_SelfTestResult_t *result)
{
    uint8_t normal_ready;
    uint8_t reversed_ready;
    memset(result, 0, sizeof(*result));

    IST8310_WriteRegister(dev, 0x0C, 0x00);
    normal_ready = IST8310_WaitAndReadRaw(dev, &result->normal);

    IST8310_WriteRegister(dev,0x0C,0x40);
    reversed_ready = IST8310_WaitAndReadRaw(dev, &result->reversed);

    IST8310_WriteRegister(dev,0x0C,0x00);

    dev->irq_data_ready = false;
    IST8310_WriteRegister(dev,0x0A,0x01);
    if ((normal_ready == 0U) || (reversed_ready == 0U))
        return 0;

    result->x_passed = IST8310_SelfTestAxisPassed(result->normal.x,result->reversed.x);
    result->y_passed = IST8310_SelfTestAxisPassed(result->normal.y,result->reversed.y);
    result->z_passed = IST8310_SelfTestAxisPassed(result->normal.z,result->reversed.z);

    if (!result->x_passed || !result->y_passed || !result->z_passed)
        return 0;

    return 1;
}

void IST8310_BoardInit(void)
{
    const IST8310_Config_t config = {
        .hi2c = &hi2c3,
        .reset_port = GPIOG,
        .reset_pin = GPIO_PIN_6,
        .i2c_address = 0x1C,
        .timeout_ms = 20U
    };

    memset(&g_ist8310_data, 0, sizeof(g_ist8310_data));
    g_ist8310_sample_count = 0U;
    g_ist8310_last_update_ms = 0U;
    IST8310_Init(&g_ist8310, &config);
}

void IST8310_BoardIRQHandler(uint16_t gpio_pin)
{
    if (gpio_pin == GPIO_PIN_3)
    {
        g_ist8310.irq_data_ready = true;
    }
}

void IST8310_BoardUpdate(void)
{
    if (!g_ist8310.irq_data_ready)
        return;

    IST8310_Read(&g_ist8310, &g_ist8310_data);
        ++g_ist8310_sample_count;
        g_ist8310_last_update_ms = HAL_GetTick();
}

void IST8310_AppStart(void)
{
    IST8310_BoardInit();
    IST8310_SelfTest(&g_ist8310, &g_ist8310_self_test_result);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    IST8310_BoardIRQHandler(GPIO_Pin);
}
