#ifndef DRV_MAG_H
#define DRV_MAG_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

//浮动默认0x0E，低位在前，高位在后
#define IST8310_I2C_ADDRESS_7BIT       0x0EU
#define IST8310_I2C_ADDRESS_HAL        (IST8310_I2C_ADDRESS_7BIT << 1U)//0x1C
#define IST8310_DEVICE_ID              0x10U
#define IST8310_SENSITIVITY_UT_PER_LSB 0.3f
#define IST8310_MIN_SAMPLE_INTERVAL_MS 6U
#define IST8310_DEFAULT_TIMEOUT_MS     20U


typedef struct
{
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    uint16_t i2c_address;
    uint32_t timeout_ms;
} IST8310_Config_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} IST8310_RawData_t;

typedef struct
{
    IST8310_RawData_t raw;

    float x_uT;
    float y_uT;
    float z_uT;
    float norm_uT;

    uint8_t stat1;
    uint8_t stat2;

    bool overrun;
    bool field_interrupt;
} IST8310_Data_t;

typedef struct
{
    IST8310_RawData_t normal;
    IST8310_RawData_t reversed;

    bool x_passed;
    bool y_passed;
    bool z_passed;
} IST8310_SelfTestResult_t;

typedef struct
{
    IST8310_Config_t config;
    volatile bool irq_data_ready;
    uint8_t device_id;
    bool initialized;
} IST8310_Handle_t;

/* DJI C-board onboard IST8310 singleton data. */
extern IST8310_Handle_t g_ist8310;
extern IST8310_Data_t g_ist8310_data;
extern volatile uint32_t g_ist8310_sample_count;
extern volatile uint32_t g_ist8310_last_update_ms;

extern void IST8310_Init(IST8310_Handle_t *dev,const IST8310_Config_t *config);
extern void IST8310_HardwareReset(IST8310_Handle_t *dev);
extern void IST8310_SoftReset(IST8310_Handle_t *dev);
extern void IST8310_Read(IST8310_Handle_t *dev, IST8310_Data_t *data);
extern uint8_t IST8310_SelfTest(IST8310_Handle_t *dev,IST8310_SelfTestResult_t *result);
extern void IST8310_BoardInit(void);
extern void IST8310_BoardIRQHandler(uint16_t gpio_pin);
extern void IST8310_BoardUpdate(void);
extern void IST8310_AppStart(void);

#endif

