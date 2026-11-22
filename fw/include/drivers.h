/* Sensor drivers: INA226 (power monitor), TMP117 (board temperature), MS5611 (barometer),
 * and I2C bus recovery. All register-level, all returning hal_status_t. */
#ifndef STRATONODE_DRIVERS_H
#define STRATONODE_DRIVERS_H

#include "hal.h"

/* ---------------- INA226 ---------------- */
#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT 0x01
#define INA226_REG_BUS 0x02
#define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CAL 0x05
#define INA226_REG_MASK 0x06
#define INA226_REG_ALERT 0x07
#define INA226_REG_MFG_ID 0xFE
#define INA226_REG_DIE_ID 0xFF
#define INA226_MASK_BUL (1u << 12) /* bus under-voltage alert enable */

typedef struct {
    hal_i2c_t bus;
    uint8_t addr;
    float r_shunt_ohm;
    float current_lsb_a;
} ina226_t;

hal_status_t ina226_init(ina226_t *d, hal_i2c_t bus, uint8_t addr, float r_shunt_ohm, float max_current_a);
hal_status_t ina226_read(ina226_t *d, float *bus_v, float *current_a);
/* Arm the ALERT pin to assert (active low) when the bus voltage falls below limit_v. */
hal_status_t ina226_set_undervoltage_alert(ina226_t *d, float limit_v);
uint16_t ina226_calibration(float r_shunt_ohm, float current_lsb_a);

/* ---------------- TMP117 ---------------- */
#define TMP117_REG_TEMP 0x00
#define TMP117_REG_DEVICE_ID 0x0F
hal_status_t tmp117_probe(hal_i2c_t bus, uint8_t addr);
hal_status_t tmp117_read_c(hal_i2c_t bus, uint8_t addr, float *temp_c);

/* ---------------- MS5611 ---------------- */
typedef struct {
    hal_spi_t spi;
    uint16_t c[8]; /* PROM: c[0] factory, c[1..6] coefficients, c[7] serial + CRC4 */
} ms5611_t;

hal_status_t ms5611_init(ms5611_t *d, hal_spi_t spi);
hal_status_t ms5611_read(ms5611_t *d, int32_t *temp_centi_c, int32_t *pressure_pa);
/* Pure maths, exposed for tests: first- and second-order compensation from the datasheet. */
void ms5611_compensate(const uint16_t c[8], uint32_t d1, uint32_t d2, int32_t *temp_centi_c, int32_t *pressure_pa);
uint8_t ms5611_crc4(const uint16_t prom[8]);

/* ---------------- I2C bus recovery ---------------- */
/* Clock SCL up to 9 times until a slave stuck mid-byte releases SDA, then issue a STOP.
 * Returns HAL_OK if SDA is high afterwards, HAL_BUS_STUCK otherwise. */
hal_status_t i2c_bus_recover(const hal_i2c_pins_t *p, int *pulses_used);

#endif
