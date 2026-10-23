/** @brief IST8310 register driver: no HAL, board pins, singleton or scheduler. */
#ifndef DRV_MAG_H
#define DRV_MAG_H
#include <stdbool.h>
#include <stdint.h>

#define IST8310_I2C_ADDRESS_7BIT        0x0EU
#define IST8310_DEVICE_ID               0x10U
#define IST8310_SENSITIVITY_UT_PER_LSB   0.3f
#define IST8310_MIN_SAMPLE_INTERVAL_MS  6U
#define IST8310_DEFAULT_TIMEOUT_MS      20U

typedef enum {
    IST8310_OK = 0, IST8310_NOT_READY, IST8310_ERR_ARGUMENT,
    IST8310_ERR_NOT_INITIALIZED, IST8310_ERR_BUSY, IST8310_ERR_IO,
    IST8310_ERR_TIMEOUT, IST8310_ERR_DEVICE_ID, IST8310_ERR_VERIFY,
    IST8310_ERR_SELF_TEST, IST8310_SELF_TEST_INCONCLUSIVE
} IST8310_Status_t;

/* read/write operate on this device; addressing belongs to the port.
 * delay_ms is optional except for the explicit blocking diagnostic SelfTest.
 */
typedef struct {
    void *context;
    IST8310_Status_t (*read)(void *context, uint8_t reg, uint8_t *data, uint16_t length);
    IST8310_Status_t (*write)(void *context, uint8_t reg, const uint8_t *data, uint16_t length);
    void (*delay_ms)(void *context, uint32_t ms);
} IST8310_Config_t;

typedef struct { int16_t x, y, z; } IST8310_RawData_t;
typedef struct {
    IST8310_RawData_t raw;
    /* Uncalibrated sensor frame. Magnetic fields, not Euler angles. */
    float x_uT, y_uT, z_uT, norm_uT;
    uint8_t stat1, stat2;
    bool overrun, field_interrupt;
} IST8310_Data_t;
typedef struct {
    IST8310_RawData_t normal, reversed;
    bool x_passed, y_passed, z_passed;
} IST8310_SelfTestResult_t;
typedef struct {
    IST8310_Config_t config;
    uint8_t device_id;
    bool initialized;
} IST8310_Handle_t;

/* Call after hardware reset and settling. Does not start a conversion. */
IST8310_Status_t IST8310_Init(IST8310_Handle_t *dev, const IST8310_Config_t *config);
IST8310_Status_t IST8310_StartMeasurement(IST8310_Handle_t *dev);
/* Caller schedules >=6 ms conversion time and a timeout. Output changes only on OK. */
IST8310_Status_t IST8310_Read(IST8310_Handle_t *dev, IST8310_Data_t *data);
/* Decode STAT1 plus the seven-byte DATA..STAT2 burst into one sample. */
IST8310_Status_t IST8310_DecodeSample(uint8_t stat1, const uint8_t burst[7],
                                      IST8310_Data_t *data);
/* Invalidates initialization; caller waits, then calls Init again. */
IST8310_Status_t IST8310_SoftReset(IST8310_Handle_t *dev);
/* Standalone blocking diagnostic: pause sampling first; never call from an ISR.
 * A zero axis is inconclusive. No tolerance is invented for the polarity test.
 */
IST8310_Status_t IST8310_SelfTest(IST8310_Handle_t *dev, IST8310_SelfTestResult_t *result);
#endif
