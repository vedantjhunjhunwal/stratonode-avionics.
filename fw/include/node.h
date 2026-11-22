/* Flight logic: power manager (brown-out / load shedding), heater control, GNSS NMEA parser
 * (GPS + NavIC), CAN telemetry packing with end-to-end protection. */
#ifndef STRATONODE_NODE_H
#define STRATONODE_NODE_H

#include <stdbool.h>
#include <stdint.h>

/* ================================================================ power manager */
typedef enum { PM_NORMAL, PM_HOLDUP, PM_RECOVER, PM_LOW_BATT, PM_CRITICAL, PM_POWER_LOST } pm_state_t;

typedef struct {
    float v_fail;        /* input below this = dropout (fast path threshold), V */
    float v_restore;     /* input back above this = recovered, V */
    float v_low;         /* sustained below this (loaded) = low battery, V */
    float v_crit;        /* sustained below this = critical, V */
    uint32_t holdup_max_ms;   /* dropout longer than this = real power loss */
    uint32_t restore_ms;      /* input must be stable this long before re-enabling loads */
    uint32_t stagger_ms;      /* delay between re-enabling radio and payload (limits inrush) */
    uint32_t low_ms;          /* persistence before entering LOW / CRITICAL */
} pm_config_t;

typedef struct {
    pm_config_t cfg;
    pm_state_t state;
    uint32_t t_state_ms;
    uint32_t t_below_low_ms, t_below_crit_ms, t_stable_ms;
    bool radio_on, payload_on, heaters_on, bay_heater_allowed;
    bool battery_low_latched;
    uint32_t n_dropouts, n_recoveries, n_power_lost;
    uint32_t longest_dropout_ms;
} power_mgr_t;

void pm_init(power_mgr_t *pm, const pm_config_t *cfg);
/* Called from the ADC-watchdog / INA226-ALERT interrupt: sheds radio and payload immediately. */
void pm_on_power_fail_irq(power_mgr_t *pm);
/* Called every 1 ms from the main loop with the filtered input voltage. */
void pm_step(power_mgr_t *pm, float v_in, uint32_t dt_ms);
const char *pm_state_name(pm_state_t s);

/* ================================================================ heater control */
typedef enum { HTR_OK = 0, HTR_SENSOR_FAULT = 1, HTR_ELEMENT_OPEN = 2, HTR_ELEMENT_SHORT = 4 } heater_fault_t;

typedef struct {
    float setpoint_c, kp, ki;         /* PI gains: duty per K, duty per K.s */
    float t_valid_min, t_valid_max;   /* plausible sensor range */
    float r_element_ohm;              /* nominal heater resistance */
    float fallback_duty;              /* open-loop duty used when the sensor is lost */
    float max_duty;                   /* power cap (may be reduced by the power manager) */
} heater_config_t;

typedef struct {
    heater_config_t cfg;
    float integ;
    float duty;
    uint8_t faults;
    uint32_t n_sensor_faults;
} heater_t;

void heater_init(heater_t *h, const heater_config_t *cfg);
/* temp_ok=false when the sensor read failed. Returns the PWM duty 0..1 for the next period. */
float heater_step(heater_t *h, float temp_c, bool temp_ok, float dt_s);
/* Element check: compare the measured heater current with V/R x duty. */
void heater_check_element(heater_t *h, float v_supply, float i_measured, float duty_applied);

/* ================================================================ NMEA (GPS + NavIC) */
enum { GNSS_GPS, GNSS_GLONASS, GNSS_GALILEO, GNSS_BEIDOU, GNSS_NAVIC, GNSS_QZSS, GNSS_N };

typedef struct {
    double lat_deg, lon_deg;
    float alt_m, hdop;
    uint8_t fix_quality, sats_used;
    uint8_t in_view[GNSS_N];
    bool valid;
    uint32_t n_gga;
} gnss_fix_t;

typedef struct {
    char buf[96];
    uint8_t len;
    bool in_sentence;
    gnss_fix_t fix;
    uint32_t n_ok, n_bad_checksum, n_overflow, n_unknown, n_implausible, n_fix_updates;
    /* A fix inconsistent with the current one becomes a candidate; it is accepted only when the next
     * fix confirms it. One corrupted sentence can never be accepted, yet a real jump (or re-acquisition
     * after an outage) costs just one extra second. */
    bool cand_valid;
    double cand_lat, cand_lon, cand_utc;
    float cand_alt;
    double last_utc_s;     /* time of the last accepted fix, for the plausibility gate */
} nmea_t;

/* The NMEA checksum is an 8-bit XOR: two flips of the same bit cancel. Fixes are therefore also
 * gated on physics: a stratospheric UAV cannot move more than these per second. */
#define NMEA_MAX_VERT_MPS 100.0
#define NMEA_MAX_HORIZ_MPS 400.0

void nmea_init(nmea_t *n);
/* Feed one byte from the UART. Returns true when a sentence was accepted. */
bool nmea_feed(nmea_t *n, char c);

/* ================================================================ CAN telemetry */
typedef struct {
    uint16_t v_in_mv;
    int16_t i_in_ma;
    int16_t t_batt_cc, t_bay_cc;      /* centi-degC */
    uint8_t htr_batt_duty, htr_bay_duty; /* 0..200 = 0..100% */
    uint8_t pm_state;
    uint8_t faults;
} tlm_status_t;

#define TLM_ID_STATUS 0x101u
typedef struct {
    uint32_t id;
    uint8_t len;      /* CAN-FD: 16 bytes (DLC 10) */
    uint8_t data[16];
} can_frame_t;

uint8_t crc8_sae_j1850(const uint8_t *d, uint32_t n);
/* Packs a 16-byte CAN-FD frame: payload, 4-bit alive counter, CRC-8 over (ID, payload, counter). */
void tlm_pack_status(const tlm_status_t *s, uint8_t counter, can_frame_t *f);
/* Returns false on CRC mismatch or a repeated/skipped counter (stale or lost frame). */
bool tlm_unpack_status(const can_frame_t *f, tlm_status_t *s, uint8_t *last_counter, bool *lost);

#endif
