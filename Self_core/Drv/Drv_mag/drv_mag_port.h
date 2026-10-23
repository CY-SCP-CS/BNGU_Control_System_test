/** @brief DJI C-board IST8310 sampling, recovery and snapshots. */
#ifndef DRV_MAG_PORT_H
#define DRV_MAG_PORT_H
#include "drv_mag.h"

typedef enum {
    DRV_MAG_DISABLED = 0,
    DRV_MAG_RESET_LOW,
    DRV_MAG_RESET_SETTLE,
    DRV_MAG_CONFIGURE,
    DRV_MAG_START,
    DRV_MAG_WAIT,
    DRV_MAG_READ_STATUS,
    DRV_MAG_READ_DATA,
    DRV_MAG_RECOVERING
} drv_mag_state_t;

typedef struct {
    IST8310_Data_t sample;
    uint32_t sample_count, last_update_ms, error_count, recovery_count;
    IST8310_Status_t last_error;
    drv_mag_state_t state;
    uint8_t device_id;
    bool valid;  /* A complete sample has been acquired at least once. */
    bool online; /* Sampling healthy, with a sample younger than 100 ms. */
} drv_mag_snapshot_t;

/* Board singleton is private to DRV. APP initializes and advances the state
 * machine from the 1 kHz control tick; I2C transfers are interrupt-driven.
 */
IST8310_Status_t drv_mag_port_init(void);
void drv_mag_port_process(void);
bool drv_mag_port_read_snapshot(drv_mag_snapshot_t *out);
#endif
