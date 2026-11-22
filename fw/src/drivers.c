#include "drivers.h"

#include <math.h>

/* ======================================================================= INA226 */
static hal_status_t ina_wr(ina226_t *d, uint8_t reg, uint16_t v)
{
    uint8_t b[3] = {reg, (uint8_t)(v >> 8), (uint8_t)v};
    return d->bus.xfer(d->bus.ctx, d->addr, b, 3, NULL, 0);
}

static hal_status_t ina_rd(ina226_t *d, uint8_t reg, uint16_t *v)
{
    uint8_t r[2];
    hal_status_t s = d->bus.xfer(d->bus.ctx, d->addr, &reg, 1, r, 2);
    if (s == HAL_OK)
        *v = (uint16_t)((r[0] << 8) | r[1]);
    return s;
}

uint16_t ina226_calibration(float r_shunt_ohm, float current_lsb_a)
{
    /* Datasheet eq. 1: CAL = 0.00512 / (Current_LSB * R_shunt) */
    return (uint16_t)lroundf(0.00512f / (current_lsb_a * r_shunt_ohm));
}

hal_status_t ina226_init(ina226_t *d, hal_i2c_t bus, uint8_t addr, float r_shunt_ohm, float max_current_a)
{
    d->bus = bus;
    d->addr = addr;
    d->r_shunt_ohm = r_shunt_ohm;
    d->current_lsb_a = max_current_a / 32768.0f;
    uint16_t mfg, die;
    hal_status_t s = ina_rd(d, INA226_REG_MFG_ID, &mfg);
    if (s != HAL_OK)
        return s;
    if ((s = ina_rd(d, INA226_REG_DIE_ID, &die)) != HAL_OK)
        return s;
    if (mfg != 0x5449 || (die & 0xFFF0) != 0x2260) /* "TI", die 0x226x */
        return HAL_BAD_DATA;
    /* AVG=1, VBUSCT=VSHCT=140 us, MODE=shunt+bus continuous: the bus-voltage ALERT re-evaluates
     * every 0.28 ms. Averaging would slow it to ~5 ms, too late to shed load in a dropout, so
     * smoothing is done in firmware instead. This ALERT drives the power-fail interrupt (EXTI4);
     * the ADC analog watchdog on VIN_SENSE is a planned faster second path. */
    uint16_t cfg = (uint16_t)((0u << 9) | (0u << 6) | (0u << 3) | 7u);
    if ((s = ina_wr(d, INA226_REG_CONFIG, cfg)) != HAL_OK)
        return s;
    return ina_wr(d, INA226_REG_CAL, ina226_calibration(r_shunt_ohm, d->current_lsb_a));
}

hal_status_t ina226_read(ina226_t *d, float *bus_v, float *current_a)
{
    uint16_t vb, ic;
    hal_status_t s = ina_rd(d, INA226_REG_BUS, &vb);
    if (s != HAL_OK)
        return s;
    if ((s = ina_rd(d, INA226_REG_CURRENT, &ic)) != HAL_OK)
        return s;
    *bus_v = (float)vb * 1.25e-3f;
    *current_a = (float)(int16_t)ic * d->current_lsb_a;
    return HAL_OK;
}

hal_status_t ina226_set_undervoltage_alert(ina226_t *d, float limit_v)
{
    hal_status_t s = ina_wr(d, INA226_REG_ALERT, (uint16_t)lroundf(limit_v / 1.25e-3f));
    if (s != HAL_OK)
        return s;
    return ina_wr(d, INA226_REG_MASK, INA226_MASK_BUL); /* transparent (non-latched), active low */
}

/* ======================================================================= TMP117 */
hal_status_t tmp117_probe(hal_i2c_t bus, uint8_t addr)
{
    uint8_t reg = TMP117_REG_DEVICE_ID, r[2];
    hal_status_t s = bus.xfer(bus.ctx, addr, &reg, 1, r, 2);
    if (s != HAL_OK)
        return s;
    return ((((r[0] << 8) | r[1]) & 0x0FFF) == 0x0117) ? HAL_OK : HAL_BAD_DATA;
}

hal_status_t tmp117_read_c(hal_i2c_t bus, uint8_t addr, float *temp_c)
{
    uint8_t reg = TMP117_REG_TEMP, r[2];
    hal_status_t s = bus.xfer(bus.ctx, addr, &reg, 1, r, 2);
    if (s != HAL_OK)
        return s;
    int16_t raw = (int16_t)((r[0] << 8) | r[1]);
    if (raw == (int16_t)0x8000) /* reset value: no conversion yet */
        return HAL_BAD_DATA;
    *temp_c = (float)raw * 7.8125e-3f;
    return HAL_OK;
}

/* ======================================================================= MS5611 */
#define MS_RESET 0x1E
#define MS_CONV_D1_4096 0x48
#define MS_CONV_D2_4096 0x58
#define MS_ADC_READ 0x00
#define MS_PROM 0xA0

static hal_status_t ms_cmd(ms5611_t *d, uint8_t cmd, uint8_t *rx, size_t n)
{
    uint8_t tx[4] = {cmd, 0, 0, 0}, buf[4];
    d->spi.cs(d->spi.ctx, true);
    hal_status_t s = d->spi.xfer(d->spi.ctx, tx, buf, 1 + n);
    d->spi.cs(d->spi.ctx, false);
    for (size_t i = 0; i < n; i++)
        rx[i] = buf[1 + i];
    return s;
}

uint8_t ms5611_crc4(const uint16_t prom[8])
{
    /* TE Connectivity AN520: CRC-4 over the 8 PROM words with the CRC nibble zeroed. */
    uint16_t n[8];
    for (int i = 0; i < 8; i++)
        n[i] = prom[i];
    n[7] &= 0xFF00;
    uint32_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1)
            rem ^= (uint16_t)(n[cnt >> 1] & 0x00FF);
        else
            rem ^= (uint16_t)(n[cnt >> 1] >> 8);
        for (int b = 8; b > 0; b--) {
            if (rem & 0x8000)
                rem = (rem << 1) ^ 0x3000;
            else
                rem <<= 1;
        }
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}

hal_status_t ms5611_init(ms5611_t *d, hal_spi_t spi)
{
    d->spi = spi;
    hal_status_t s = ms_cmd(d, MS_RESET, NULL, 0);
    if (s != HAL_OK)
        return s;
    spi.delay_ms(spi.ctx, 3); /* PROM reload after reset: 2.8 ms */
    for (int i = 0; i < 8; i++) {
        uint8_t r[2];
        if ((s = ms_cmd(d, (uint8_t)(MS_PROM + 2 * i), r, 2)) != HAL_OK)
            return s;
        d->c[i] = (uint16_t)((r[0] << 8) | r[1]);
    }
    if (ms5611_crc4(d->c) != (d->c[7] & 0x0F))
        return HAL_BAD_DATA; /* corrupted PROM read: refuse to fly on bad coefficients */
    return HAL_OK;
}

void ms5611_compensate(const uint16_t c[8], uint32_t d1, uint32_t d2, int32_t *temp_centi_c, int32_t *pressure_pa)
{
    int64_t dT = (int64_t)d2 - ((int64_t)c[5] << 8);
    int64_t temp = 2000 + ((dT * c[6]) >> 23);
    int64_t off = ((int64_t)c[2] << 16) + (((int64_t)c[4] * dT) >> 7);
    int64_t sens = ((int64_t)c[1] << 15) + (((int64_t)c[3] * dT) >> 8);
    /* Second-order compensation: matters here because the sensor sees -25 C in the bay. */
    if (temp < 2000) {
        int64_t t2 = (dT * dT) >> 31;
        int64_t d = temp - 2000;
        int64_t off2 = 5 * d * d / 2;
        int64_t sens2 = 5 * d * d / 4;
        if (temp < -1500) {
            int64_t e = temp + 1500;
            off2 += 7 * e * e;
            sens2 += 11 * e * e / 2;
        }
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }
    *temp_centi_c = (int32_t)temp;
    *pressure_pa = (int32_t)((((int64_t)d1 * sens >> 21) - off) >> 15);
}

hal_status_t ms5611_read(ms5611_t *d, int32_t *temp_centi_c, int32_t *pressure_pa)
{
    uint8_t r[3];
    hal_status_t s;
    if ((s = ms_cmd(d, MS_CONV_D1_4096, NULL, 0)) != HAL_OK)
        return s;
    d->spi.delay_ms(d->spi.ctx, 10); /* OSR 4096: 9.04 ms max */
    if ((s = ms_cmd(d, MS_ADC_READ, r, 3)) != HAL_OK)
        return s;
    uint32_t d1 = ((uint32_t)r[0] << 16) | ((uint32_t)r[1] << 8) | r[2];
    if ((s = ms_cmd(d, MS_CONV_D2_4096, NULL, 0)) != HAL_OK)
        return s;
    d->spi.delay_ms(d->spi.ctx, 10);
    if ((s = ms_cmd(d, MS_ADC_READ, r, 3)) != HAL_OK)
        return s;
    uint32_t d2 = ((uint32_t)r[0] << 16) | ((uint32_t)r[1] << 8) | r[2];
    if (d1 == 0 || d2 == 0)
        return HAL_BAD_DATA; /* conversion was interrupted (read too early) */
    ms5611_compensate(d->c, d1, d2, temp_centi_c, pressure_pa);
    return HAL_OK;
}

/* ======================================================================= I2C recovery */
hal_status_t i2c_bus_recover(const hal_i2c_pins_t *p, int *pulses_used)
{
    int n = 0;
    p->sda_write(p->ctx, true);
    while (!p->sda_read(p->ctx) && n < 9) {
        p->scl_write(p->ctx, false);
        p->delay_us(p->ctx, 5);
        p->scl_write(p->ctx, true);
        p->delay_us(p->ctx, 5);
        n++;
    }
    /* STOP: SDA low -> high while SCL is high */
    p->scl_write(p->ctx, false);
    p->sda_write(p->ctx, false);
    p->delay_us(p->ctx, 5);
    p->scl_write(p->ctx, true);
    p->delay_us(p->ctx, 5);
    p->sda_write(p->ctx, true);
    p->delay_us(p->ctx, 5);
    if (pulses_used)
        *pulses_used = n;
    return p->sda_read(p->ctx) ? HAL_OK : HAL_BUS_STUCK;
}
