/* Hardware abstraction: the drivers and control logic only see these function tables,
 * so the same code runs against the STM32G4 peripherals, the host unit-test fakes and
 * the closed-loop flight simulator. No dynamic memory anywhere in fw/. */
#ifndef STRATONODE_HAL_H
#define STRATONODE_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_NACK = -1, HAL_TIMEOUT = -2, HAL_BUS_STUCK = -3, HAL_BAD_DATA = -4 } hal_status_t;

/* I2C master: write ntx bytes then (repeated start) read nrx bytes. */
typedef struct {
    hal_status_t (*xfer)(void *ctx, uint8_t addr7, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx);
    void *ctx;
} hal_i2c_t;

/* Pins of the same I2C bus driven as GPIO, used only for bus recovery. */
typedef struct {
    void (*scl_write)(void *ctx, bool high);
    bool (*sda_read)(void *ctx);
    void (*sda_write)(void *ctx, bool high);
    void (*delay_us)(void *ctx, uint32_t us);
    void *ctx;
} hal_i2c_pins_t;

/* SPI master, full duplex, chip select handled by the driver. */
typedef struct {
    hal_status_t (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n);
    void (*cs)(void *ctx, bool active);
    void (*delay_ms)(void *ctx, uint32_t ms);
    void *ctx;
} hal_spi_t;

#endif
