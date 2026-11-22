/* Host unit tests for StratoNode firmware. Plain C, no framework: CHECK() counts failures. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drivers.h"
#include "node.h"

static int g_fail, g_checks;
#define CHECK(c)                                                                  \
    do {                                                                          \
        g_checks++;                                                               \
        if (!(c)) {                                                               \
            g_fail++;                                                             \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                 \
        }                                                                         \
    } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

/* ------------------------------------------------------------------ fake I2C bus with devices */
typedef struct {
    uint16_t ina[2][256]; /* register files for INA226 at 0x40 and 0x41 */
    uint16_t tmp_temp, tmp_id;
    int nack_addr;        /* address that NACKs (absent / hung device) */
    int writes;
} fake_i2c_t;

static hal_status_t fake_xfer(void *ctx, uint8_t addr, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx)
{
    fake_i2c_t *b = ctx;
    if (addr == b->nack_addr) return HAL_NACK;
    if (addr == 0x40 || addr == 0x41) {
        uint16_t *r = b->ina[addr - 0x40];
        if (ntx == 3) { r[tx[0]] = (uint16_t)((tx[1] << 8) | tx[2]); b->writes++; }
        if (nrx == 2) { rx[0] = (uint8_t)(r[tx[0]] >> 8); rx[1] = (uint8_t)r[tx[0]]; }
        return HAL_OK;
    }
    if (addr == 0x48) {
        uint16_t v = tx[0] == TMP117_REG_TEMP ? b->tmp_temp : (tx[0] == TMP117_REG_DEVICE_ID ? b->tmp_id : 0);
        if (nrx == 2) { rx[0] = (uint8_t)(v >> 8); rx[1] = (uint8_t)v; }
        return HAL_OK;
    }
    return HAL_NACK;
}

static void test_ina226(void)
{
    puts("INA226");
    fake_i2c_t b = {0};
    b.nack_addr = -1;
    b.ina[0][INA226_REG_MFG_ID] = 0x5449;
    b.ina[0][INA226_REG_DIE_ID] = 0x2260;
    hal_i2c_t bus = {fake_xfer, &b};
    ina226_t d;
    CHECK(ina226_calibration(0.010f, 5.0f / 32768.0f) == 3355); /* 0.00512 / (152.6 uA x 10 mOhm) */
    CHECK(ina226_init(&d, bus, 0x40, 0.010f, 5.0f) == HAL_OK);
    CHECK(b.ina[0][INA226_REG_CAL] == 3355);
    CHECK(b.ina[0][INA226_REG_CONFIG] == 0x0007);
    b.ina[0][INA226_REG_BUS] = 9600;                       /* 12.000 V */
    b.ina[0][INA226_REG_CURRENT] = (uint16_t)(int16_t)-6554; /* -1.000 A (reverse) */
    float v, i;
    CHECK(ina226_read(&d, &v, &i) == HAL_OK);
    NEAR(v, 12.0, 1e-4);
    NEAR(i, -1.0, 1e-3);
    CHECK(ina226_set_undervoltage_alert(&d, 10.0f) == HAL_OK);
    CHECK(b.ina[0][INA226_REG_ALERT] == 8000);
    CHECK(b.ina[0][INA226_REG_MASK] == INA226_MASK_BUL);
    b.ina[0][INA226_REG_DIE_ID] = 0x1234; /* wrong part on the bus */
    CHECK(ina226_init(&d, bus, 0x40, 0.010f, 5.0f) == HAL_BAD_DATA);
    b.nack_addr = 0x40;
    CHECK(ina226_init(&d, bus, 0x40, 0.010f, 5.0f) == HAL_NACK);
}

static void test_tmp117(void)
{
    puts("TMP117");
    fake_i2c_t b = {0};
    b.nack_addr = -1;
    hal_i2c_t bus = {fake_xfer, &b};
    b.tmp_id = 0x1117; /* revision bits in the top nibble */
    CHECK(tmp117_probe(bus, 0x48) == HAL_OK);
    b.tmp_id = 0x0118;
    CHECK(tmp117_probe(bus, 0x48) == HAL_BAD_DATA);
    float t;
    b.tmp_temp = (uint16_t)(int16_t)-3200; /* -25.000 C */
    CHECK(tmp117_read_c(bus, 0x48, &t) == HAL_OK);
    NEAR(t, -25.0, 1e-6);
    b.tmp_temp = 0x8000;
    CHECK(tmp117_read_c(bus, 0x48, &t) == HAL_BAD_DATA);
}

/* ------------------------------------------------------------------ fake MS5611 on SPI */
typedef struct {
    uint16_t prom[8];
    uint32_t d1, d2, adc;
    int conv;
} fake_ms_t;

static hal_status_t ms_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    fake_ms_t *m = ctx;
    memset(rx, 0, n);
    uint8_t c = tx[0];
    if (c >= 0xA0 && c <= 0xAE) { uint16_t w = m->prom[(c - 0xA0) / 2]; rx[1] = (uint8_t)(w >> 8); rx[2] = (uint8_t)w; }
    else if (c == 0x48) m->adc = m->d1;
    else if (c == 0x58) m->adc = m->d2;
    else if (c == 0x00) { rx[1] = (uint8_t)(m->adc >> 16); rx[2] = (uint8_t)(m->adc >> 8); rx[3] = (uint8_t)m->adc; m->adc = 0; }
    return HAL_OK;
}
static void ms_cs(void *ctx, bool a) { (void)ctx; (void)a; }
static void ms_delay(void *ctx, uint32_t ms) { (void)ctx; (void)ms; }

static void test_ms5611(void)
{
    puts("MS5611");
    /* AN520 example PROM: CRC-4 = 0xB */
    uint16_t an520[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x4500};
    CHECK(ms5611_crc4(an520) == 0x0B);
    /* Datasheet worked example: C1..C6, D1, D2 -> TEMP 20.07 C, P 1000.09 mbar */
    fake_ms_t m = {{0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x0000}, 9085466, 8569150, 0, 0};
    m.prom[7] = 0x1230; /* serial number bits; CRC goes in the low nibble */
    m.prom[7] = (uint16_t)(m.prom[7] | ms5611_crc4(m.prom));
    int32_t t, p;
    ms5611_compensate(m.prom, m.d1, m.d2, &t, &p);
    CHECK(t == 2007);
    CHECK(p == 100009);
    hal_spi_t spi = {ms_xfer, ms_cs, ms_delay, &m};
    ms5611_t d;
    CHECK(ms5611_init(&d, spi) == HAL_OK);
    CHECK(ms5611_read(&d, &t, &p) == HAL_OK && t == 2007 && p == 100009);
    m.prom[3] ^= 0x0004; /* one flipped bit in a coefficient */
    CHECK(ms5611_init(&d, spi) == HAL_BAD_DATA);
    /* Cold: lower D2 -> below -15 C, both second-order branches active, result stays monotonic */
    int32_t t_prev = 99999, p_prev = 0;
    for (uint32_t d2 = 8569150; d2 > 7400000; d2 -= 40000) {
        ms5611_compensate(m.prom, m.d1, d2, &t, &p);
        CHECK(t < t_prev);
        t_prev = t;
        p_prev = p;
    }
    CHECK(t_prev < -1500);
    CHECK(p_prev > 0);
}

/* ------------------------------------------------------------------ I2C bus recovery */
typedef struct { int stuck_for; int clocks; bool scl, sda_drv; } fake_pins_t;
static void p_scl(void *c, bool h) { fake_pins_t *f = c; if (!f->scl && h) f->clocks++; f->scl = h; }
static bool p_sda_rd(void *c) { fake_pins_t *f = c; return f->sda_drv && !(f->stuck_for < 0 || f->clocks < f->stuck_for); }
static void p_sda_wr(void *c, bool h) { ((fake_pins_t *)c)->sda_drv = h; }
static void p_delay(void *c, uint32_t us) { (void)c; (void)us; }

static void test_i2c_recovery(void)
{
    puts("I2C bus recovery");
    for (int k = 0; k <= 8; k++) {
        fake_pins_t f = {k, 0, true, true};
        hal_i2c_pins_t p = {p_scl, p_sda_rd, p_sda_wr, p_delay, &f};
        int used = -1;
        CHECK(i2c_bus_recover(&p, &used) == HAL_OK);
        CHECK(used == k);
    }
    fake_pins_t f = {-1, 0, true, true}; /* shorted to ground: unrecoverable */
    hal_i2c_pins_t p = {p_scl, p_sda_rd, p_sda_wr, p_delay, &f};
    int used;
    CHECK(i2c_bus_recover(&p, &used) == HAL_BUS_STUCK);
    CHECK(used == 9);
}

/* ------------------------------------------------------------------ power manager */
static const pm_config_t PMC = {10.0f, 11.0f, 13.2f, 12.4f, 200, 100, 50, 10000};

static void run(power_mgr_t *pm, float v, int ms) { for (int i = 0; i < ms; i++) pm_step(pm, v, 1); }

static void test_power_manager(void)
{
    puts("power manager");
    power_mgr_t pm;
    pm_init(&pm, &PMC);
    run(&pm, 14.8f, 100);
    CHECK(pm.state == PM_NORMAL && pm.radio_on && pm.payload_on);
    pm_on_power_fail_irq(&pm); /* dropout interrupt: loads shed synchronously */
    CHECK(pm.state == PM_HOLDUP && !pm.radio_on && !pm.payload_on && !pm.heaters_on);
    run(&pm, 7.0f, 30);
    CHECK(pm.state == PM_HOLDUP);
    run(&pm, 14.8f, 99);
    CHECK(pm.state == PM_RECOVER && !pm.radio_on);
    run(&pm, 14.8f, 2);
    CHECK(pm.radio_on && !pm.payload_on); /* radio first, after 100 ms of stable input */
    run(&pm, 14.8f, 50);
    CHECK(pm.state == PM_NORMAL && pm.payload_on && pm.heaters_on && pm.n_recoveries == 1);
    /* bouncing input restarts the stability timer */
    pm_step(&pm, 8.0f, 1);
    run(&pm, 14.8f, 60);
    pm_step(&pm, 10.5f, 1);
    run(&pm, 14.8f, 60);
    CHECK(!pm.radio_on);
    run(&pm, 14.8f, 100);
    CHECK(pm.state == PM_NORMAL);
    /* long loss -> POWER_LOST, then recovery when input returns */
    pm_step(&pm, 0.0f, 1);
    run(&pm, 0.0f, 250);
    CHECK(pm.state == PM_POWER_LOST && pm.n_power_lost == 1);
    run(&pm, 14.8f, 200);
    CHECK(pm.state == PM_NORMAL);
    /* low battery needs persistence: a 5 s sag is ignored, 10 s latches */
    run(&pm, 13.0f, 5000);
    run(&pm, 14.0f, 10);
    CHECK(pm.state == PM_NORMAL);
    run(&pm, 13.0f, 10000);
    CHECK(pm.state == PM_LOW_BATT && !pm.payload_on && pm.radio_on);
    run(&pm, 14.0f, 20000);
    CHECK(pm.state == PM_LOW_BATT); /* latched */
    run(&pm, 12.2f, 10000);
    CHECK(pm.state == PM_CRITICAL && !pm.bay_heater_allowed && pm.radio_on);
    /* a dropout while low: on recovery the payload stays off */
    pm_on_power_fail_irq(&pm);
    run(&pm, 14.0f, 200);
    CHECK(pm.state == PM_LOW_BATT && !pm.payload_on && pm.radio_on);
}

/* ------------------------------------------------------------------ heater */
static void test_heater(void)
{
    puts("heater");
    heater_config_t hc = {5.0f, 0.15f, 0.002f, -60.0f, 80.0f, 16.4f, 0.55f, 1.0f};
    heater_t h;
    heater_init(&h, &hc);
    /* first-order plant: C dT/dt = P_max*duty - G (T - Ta) */
    double T = -30, Ta = -70, C = 1014, G = 0.084, Pmax = 12.6;
    double max_after = -1e9;
    for (int s = 0; s < 6 * 3600; s++) {
        float u = heater_step(&h, (float)T, true, 1.0f);
        T += (Pmax * u - G * (T - Ta)) / C;
        if (s > 3 * 3600 && T > max_after) max_after = T;
    }
    NEAR(T, 5.0, 0.05);           /* zero steady-state error (integral action) */
    CHECK(max_after < 5.6);       /* bounded overshoot after the long saturated warm-up */
    NEAR(h.duty, G * (5 - Ta) / Pmax, 0.01);
    /* sensor lost: fallback duty, fault flagged, recovers when the sensor returns */
    CHECK(fabsf(heater_step(&h, NAN, true, 1.0f) - 0.55f) < 1e-6f);
    CHECK(h.faults & HTR_SENSOR_FAULT);
    heater_step(&h, -150.0f, true, 1.0f); /* open NTC reads as absurd cold */
    CHECK(h.n_sensor_faults == 1);
    heater_step(&h, 5.0f, true, 1.0f);
    CHECK(!(h.faults & HTR_SENSOR_FAULT));
    /* element checks */
    heater_check_element(&h, 14.4f, 0.44f, 0.5f);
    CHECK(h.faults == HTR_OK);
    heater_check_element(&h, 14.4f, 0.02f, 0.5f);
    CHECK(h.faults & HTR_ELEMENT_OPEN);
    heater_check_element(&h, 14.4f, 2.0f, 0.5f);
    CHECK(h.faults & HTR_ELEMENT_SHORT);
}

/* ------------------------------------------------------------------ NMEA */
static int feed(nmea_t *n, const char *body)
{
    char line[120];
    uint8_t sum = 0;
    for (const char *p = body; *p; p++) sum ^= (uint8_t)*p;
    snprintf(line, sizeof line, "$%s*%02X\r\n", body, sum);
    int ok = 0;
    for (char *p = line; *p; p++) ok += nmea_feed(n, *p);
    return ok;
}

static void test_nmea(void)
{
    puts("NMEA (GPS + NavIC)");
    nmea_t n;
    nmea_init(&n);
    CHECK(feed(&n, "GNGGA,062735.00,3517.4700,N,07710.9480,E,1,14,0.8,15502.3,M,-48.1,M,,") == 1);
    CHECK(n.fix.valid && n.fix.sats_used == 14);
    NEAR(n.fix.lat_deg, 35.0 + 17.47 / 60.0, 1e-9);
    NEAR(n.fix.lon_deg, 77.0 + 10.948 / 60.0, 1e-9);
    NEAR(n.fix.alt_m, 15502.3, 0.01);
    CHECK(feed(&n, "GIGSV,1,1,07,02,45,120,38,03,60,200,40,05,30,80,35,06,25,300,33,0") == 1);
    CHECK(feed(&n, "GPGSV,3,1,11,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45") == 1);
    CHECK(n.fix.in_view[GNSS_NAVIC] == 7 && n.fix.in_view[GNSS_GPS] == 11);
    /* corrupted checksum, garbage, overflow, no-fix sentence */
    const char *bad = "$GNGGA,062735.00,3517.4700,N,07710.9480,E,1,14,0.8,15502.3,M,-48.1,M,,*00\r\n";
    for (const char *p = bad; *p; p++) nmea_feed(&n, *p);
    CHECK(n.n_bad_checksum == 1);
    for (int i = 0; i < 200; i++) nmea_feed(&n, i == 0 ? '$' : 'A');
    CHECK(n.n_overflow == 1);
    /* checksum-valid but physically impossible: +900 m in one second */
    CHECK(feed(&n, "GNGGA,062736.00,3517.4700,N,07710.9480,E,1,14,0.8,16402.3,M,-48.1,M,,") == 1);
    CHECK(n.n_implausible == 1);
    NEAR(n.fix.alt_m, 15502.3, 0.01);
    CHECK(feed(&n, "GNGGA,062737.00,3517.4710,N,07710.9480,E,1,14,0.8,15504.0,M,-48.1,M,,") == 1);
    NEAR(n.fix.alt_m, 15504.0, 0.01); /* normal motion accepted */
    CHECK(feed(&n, "GNGGA,062737.00,3517.4710,N,07710.9480,E,1,14,0.8,15504.5,M,-48.1,M,,") == 1);
    CHECK(n.n_implausible == 2);       /* repeated time stamp = stale */
    /* a single wild fix is never accepted on its own; a jump confirmed by the next fix is */
    CHECK(feed(&n, "GNGGA,062739.00,3517.4710,N,07710.9480,E,1,14,0.8,25504.0,M,-48.1,M,,") == 1);
    NEAR(n.fix.alt_m, 15504.0, 0.01);
    CHECK(feed(&n, "GNGGA,062740.00,3517.4710,N,07710.9480,E,1,14,0.8,15505.0,M,-48.1,M,,") == 1);
    NEAR(n.fix.alt_m, 15505.0, 0.01); /* the wild one was a glitch: back on track, no lock-out */
    CHECK(feed(&n, "GNGGA,062752.00,3517.6000,N,07710.9480,E,1,14,0.8,16000.0,M,-48.1,M,,") == 1);
    NEAR(n.fix.alt_m, 15505.0, 0.01); /* after a 12 s gap: candidate only */
    CHECK(feed(&n, "GNGGA,062753.00,3517.6010,N,07710.9480,E,1,14,0.8,16001.0,M,-48.1,M,,") == 1);
    NEAR(n.fix.alt_m, 16001.0, 0.01); /* confirmed by the next fix: re-acquired */
    CHECK(feed(&n, "GNGGA,062738.00,,,,,0,00,99.99,,,,,,") == 1);
    CHECK(!n.fix.valid);
    NEAR(n.fix.alt_m, 16001.0, 0.01); /* last good altitude kept, flagged invalid */
}

/* ------------------------------------------------------------------ CAN telemetry */
static void test_can(void)
{
    puts("CAN-FD telemetry");
    uint8_t chk[] = "123456789";
    CHECK(crc8_sae_j1850(chk, 9) == 0x4B); /* CRC-8/SAE-J1850 check value */
    tlm_status_t s = {14812, -1250, 512, -2475, 120, 37, PM_HOLDUP, HTR_SENSOR_FAULT}, r;
    can_frame_t f;
    uint8_t last = 15;
    bool lost;
    tlm_pack_status(&s, 0, &f);
    CHECK(tlm_unpack_status(&f, &r, &last, &lost) && !lost);
    CHECK(r.v_in_mv == 14812 && r.i_in_ma == -1250 && r.t_batt_cc == 512 && r.t_bay_cc == -2475);
    CHECK(r.htr_batt_duty == 120 && r.htr_bay_duty == 37 && r.pm_state == PM_HOLDUP && r.faults == HTR_SENSOR_FAULT);
    CHECK(!tlm_unpack_status(&f, &r, &last, &lost)); /* same counter again = stale */
    int missed = 0;
    for (int bit = 0; bit < 15 * 8; bit++) { /* every single-bit error in the payload is caught */
        can_frame_t g = f;
        g.data[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        uint8_t l2 = 15;
        missed += tlm_unpack_status(&g, &r, &l2, &lost);
    }
    CHECK(missed == 0);
    tlm_pack_status(&s, 3, &f);
    CHECK(tlm_unpack_status(&f, &r, &last, &lost) && lost); /* counters 1, 2 skipped */
    f.id = 0x102;
    CHECK(!tlm_unpack_status(&f, &r, &last, &lost)); /* wrong ID */
}

int main(void)
{
    test_ina226();
    test_tmp117();
    test_ms5611();
    test_i2c_recovery();
    test_power_manager();
    test_heater();
    test_nmea();
    test_can();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
