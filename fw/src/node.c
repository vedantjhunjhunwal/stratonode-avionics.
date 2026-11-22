#include "node.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ======================================================================= power manager */
const char *pm_state_name(pm_state_t s)
{
    static const char *n[] = {"NORMAL", "HOLDUP", "RECOVER", "LOW_BATT", "CRITICAL", "POWER_LOST"};
    return (unsigned)s < sizeof n / sizeof n[0] ? n[s] : "?";
}

static void pm_enter(power_mgr_t *pm, pm_state_t s)
{
    pm->state = s;
    pm->t_state_ms = 0;
    pm->t_stable_ms = 0;
}

void pm_init(power_mgr_t *pm, const pm_config_t *cfg)
{
    memset(pm, 0, sizeof *pm);
    pm->cfg = *cfg;
    pm->radio_on = pm->payload_on = pm->heaters_on = pm->bay_heater_allowed = true;
    pm_enter(pm, PM_NORMAL);
}

void pm_on_power_fail_irq(power_mgr_t *pm)
{
    /* Interrupt context: two GPIO writes in the real build (load-switch ON pins low). */
    if (pm->state == PM_HOLDUP || pm->state == PM_POWER_LOST)
        return;
    pm->radio_on = false;
    pm->payload_on = false;
    pm->heaters_on = false; /* heaters hang off VBAT: 17 W would drain the hold-up caps in ~3 ms */
    pm->n_dropouts++;
    pm_enter(pm, PM_HOLDUP);
}

void pm_step(power_mgr_t *pm, float v_in, uint32_t dt_ms)
{
    const pm_config_t *c = &pm->cfg;
    pm->t_state_ms += dt_ms;
    if (v_in < c->v_fail && pm->state != PM_HOLDUP && pm->state != PM_POWER_LOST)
        pm_on_power_fail_irq(pm); /* polled backup to the interrupt */

    switch (pm->state) {
    case PM_HOLDUP:
        if (pm->t_state_ms > pm->longest_dropout_ms)
            pm->longest_dropout_ms = pm->t_state_ms;
        if (v_in >= c->v_restore) {
            pm_enter(pm, PM_RECOVER);
        } else if (pm->t_state_ms >= c->holdup_max_ms) {
            /* Real loss of input: the hold-up caps cannot carry us; write the last-gasp log. */
            pm->n_power_lost++;
            pm_enter(pm, PM_POWER_LOST);
        }
        break;
    case PM_RECOVER:
        if (v_in < c->v_restore) {
            pm->t_stable_ms = 0;
            break;
        }
        pm->t_stable_ms += dt_ms;
        if (pm->t_stable_ms >= c->restore_ms && !pm->radio_on)
            pm->radio_on = true; /* radio first: telemetry is the essential load */
        if (pm->t_stable_ms >= c->restore_ms + c->stagger_ms) {
            pm->payload_on = !pm->battery_low_latched;
            pm->heaters_on = true;
            pm->n_recoveries++;
            pm_enter(pm, pm->battery_low_latched ? PM_LOW_BATT : PM_NORMAL);
        }
        break;
    case PM_POWER_LOST:
        if (v_in >= c->v_restore)
            pm_enter(pm, PM_RECOVER);
        break;
    case PM_NORMAL:
    case PM_LOW_BATT:
    case PM_CRITICAL:
        pm->t_below_low_ms = (v_in < c->v_low) ? pm->t_below_low_ms + dt_ms : 0;
        pm->t_below_crit_ms = (v_in < c->v_crit) ? pm->t_below_crit_ms + dt_ms : 0;
        if (pm->state != PM_CRITICAL && pm->t_below_crit_ms >= c->low_ms) {
            /* Keep GNSS, CAN and the battery heater; drop payload and the bay heater. */
            pm->payload_on = false;
            pm->bay_heater_allowed = false;
            pm->battery_low_latched = true;
            pm_enter(pm, PM_CRITICAL);
        } else if (pm->state == PM_NORMAL && pm->t_below_low_ms >= c->low_ms) {
            pm->payload_on = false;
            pm->battery_low_latched = true; /* no un-latching on a rested-battery voltage bounce */
            pm_enter(pm, PM_LOW_BATT);
        }
        break;
    }
}

/* ======================================================================= heater */
void heater_init(heater_t *h, const heater_config_t *cfg)
{
    memset(h, 0, sizeof *h);
    h->cfg = *cfg;
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

float heater_step(heater_t *h, float temp_c, bool temp_ok, float dt_s)
{
    const heater_config_t *c = &h->cfg;
    bool plausible = temp_ok && temp_c > c->t_valid_min && temp_c < c->t_valid_max && !isnan(temp_c);
    if (!plausible) {
        /* Lost or implausible sensor (open/shorted NTC): fall back to open-loop duty rather than
         * either cooking the cells (stuck high) or letting them freeze (stuck low). */
        if (!(h->faults & HTR_SENSOR_FAULT))
            h->n_sensor_faults++;
        h->faults |= HTR_SENSOR_FAULT;
        h->duty = clampf(c->fallback_duty, 0.0f, c->max_duty);
        return h->duty;
    }
    h->faults &= (uint8_t)~HTR_SENSOR_FAULT;
    float e = c->setpoint_c - temp_c;
    float u_p = c->kp * e;
    float u = u_p + h->integ;
    /* Conditional integration (anti-windup): only integrate while not saturated in the same direction. */
    if (!((u >= c->max_duty && e > 0) || (u <= 0.0f && e < 0)))
        h->integ = clampf(h->integ + c->ki * e * dt_s, 0.0f, c->max_duty);
    h->duty = clampf(u_p + h->integ, 0.0f, c->max_duty);
    return h->duty;
}

void heater_check_element(heater_t *h, float v_supply, float i_measured, float duty_applied)
{
    if (duty_applied < 0.2f)
        return; /* too little current to judge */
    float expected = v_supply / h->cfg.r_element_ohm * duty_applied;
    h->faults &= (uint8_t)~(HTR_ELEMENT_OPEN | HTR_ELEMENT_SHORT);
    if (i_measured < 0.5f * expected)
        h->faults |= HTR_ELEMENT_OPEN;
    else if (i_measured > 1.6f * expected)
        h->faults |= HTR_ELEMENT_SHORT;
}

/* ======================================================================= NMEA */
void nmea_init(nmea_t *n) { memset(n, 0, sizeof *n); }

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Split in place on commas; returns field count. Empty fields stay as "". */
static int split(char *s, char **f, int max)
{
    int n = 0;
    f[n++] = s;
    for (; *s && n < max; s++)
        if (*s == ',') {
            *s = '\0';
            f[n++] = s + 1;
        }
    return n;
}

static double nmea_deg(const char *v, const char *hemi)
{
    if (!*v) return NAN;
    double x = atof(v);
    int deg = (int)(x / 100.0);
    double d = deg + (x - deg * 100.0) / 60.0;
    return (*hemi == 'S' || *hemi == 'W') ? -d : d;
}

static int constellation(const char *talker)
{
    if (!strncmp(talker, "GP", 2)) return GNSS_GPS;
    if (!strncmp(talker, "GL", 2)) return GNSS_GLONASS;
    if (!strncmp(talker, "GA", 2)) return GNSS_GALILEO;
    if (!strncmp(talker, "GB", 2) || !strncmp(talker, "BD", 2)) return GNSS_BEIDOU;
    if (!strncmp(talker, "GI", 2)) return GNSS_NAVIC; /* NMEA 4.11 talker ID for NavIC/IRNSS */
    if (!strncmp(talker, "GQ", 2)) return GNSS_QZSS;
    return -1;
}

/* Could a vehicle get from fix A to fix B? Requires B strictly newer (<= 10 s) and within the
 * vertical / horizontal speed limits. */
static bool consistent(double lat0, double lon0, float alt0, double utc0, double lat, double lon, float alt, double utc)
{
    if (fabs(lat) > 90.0 || fabs(lon) > 180.0)
        return false;
    double dt = utc - utc0;
    if (dt < -43200.0)
        dt += 86400.0; /* midnight roll-over */
    if (dt <= 0.0 || dt > 10.0)
        return false;
    double dn = (lat - lat0) * 111320.0, de = (lon - lon0) * 111320.0 * cos(lat * M_PI / 180.0);
    return fabs(alt - alt0) <= NMEA_MAX_VERT_MPS * dt && sqrt(dn * dn + de * de) <= NMEA_MAX_HORIZ_MPS * dt;
}

static bool nmea_sentence(nmea_t *n)
{
    char *s = n->buf; /* without '$', '*' replaced by NUL */
    char *f[24];
    int nf = split(s, f, 24);
    const char *type = f[0] + 2;
    if (strlen(f[0]) != 5) return false;
    if (!strcmp(type, "GGA") && nf >= 10) {
        gnss_fix_t *x = &n->fix;
        x->fix_quality = (uint8_t)atoi(f[6]);
        x->sats_used = (uint8_t)atoi(f[7]);
        x->hdop = *f[8] ? (float)atof(f[8]) : 99.9f;
        bool has_fix = x->fix_quality > 0 && *f[2] && *f[4] && strlen(f[1]) >= 6;
        if (has_fix) {
            double lat = nmea_deg(f[2], f[3]), lon = nmea_deg(f[4], f[5]);
            float alt = (float)atof(f[9]);
            const char *u = f[1];
            double utc = ((u[0] - '0') * 10 + (u[1] - '0')) * 3600.0 + ((u[2] - '0') * 10 + (u[3] - '0')) * 60.0 + atof(u + 4);
            if (x->valid && !consistent(x->lat_deg, x->lon_deg, x->alt_m, n->last_utc_s, lat, lon, alt, utc)) {
                bool confirmed = n->cand_valid && consistent(n->cand_lat, n->cand_lon, n->cand_alt, n->cand_utc, lat, lon, alt, utc);
                if (!confirmed) {
                    n->n_implausible++; /* keep the previous fix; remember this one as a candidate */
                    n->cand_valid = fabs(lat) <= 90.0 && fabs(lon) <= 180.0;
                    n->cand_lat = lat;
                    n->cand_lon = lon;
                    n->cand_alt = alt;
                    n->cand_utc = utc;
                    x->n_gga++;
                    return true;
                }
            }
            n->cand_valid = false;
            n->n_fix_updates++;
            x->lat_deg = lat;
            x->lon_deg = lon;
            x->alt_m = alt;
            n->last_utc_s = utc;
            x->valid = true;
        } else {
            x->valid = false;
        }
        x->n_gga++;
        return true;
    }
    if (!strcmp(type, "GSV") && nf >= 4) {
        int c = constellation(f[0]);
        if (c >= 0)
            n->fix.in_view[c] = (uint8_t)atoi(f[3]);
        return true;
    }
    n->n_unknown++;
    return true;
}

bool nmea_feed(nmea_t *n, char c)
{
    if (c == '$') {
        n->in_sentence = true;
        n->len = 0;
        return false;
    }
    if (!n->in_sentence) return false;
    if (c == '\r' || c == '\n') {
        n->in_sentence = false;
        n->buf[n->len] = '\0';
        char *star = strrchr(n->buf, '*');
        if (!star || star[1] == '\0' || star[2] == '\0') {
            n->n_bad_checksum++;
            return false;
        }
        int hi = hexval(star[1]), lo = hexval(star[2]);
        uint8_t sum = 0;
        for (char *p = n->buf; p < star; p++) sum ^= (uint8_t)*p;
        if (hi < 0 || lo < 0 || sum != (uint8_t)(hi * 16 + lo)) {
            n->n_bad_checksum++;
            return false;
        }
        *star = '\0';
        if (nmea_sentence(n)) {
            n->n_ok++;
            return true;
        }
        n->n_bad_checksum++;
        return false;
    }
    if (n->len >= sizeof n->buf - 1) { /* NMEA max is 82 chars: anything longer is garbage */
        n->in_sentence = false;
        n->n_overflow++;
        return false;
    }
    n->buf[n->len++] = c;
    return false;
}

/* ======================================================================= CAN telemetry */
uint8_t crc8_sae_j1850(const uint8_t *d, uint32_t n)
{
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0x1D) : (crc << 1));
    }
    return (uint8_t)(crc ^ 0xFF);
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static uint8_t frame_crc(const can_frame_t *f)
{
    uint8_t b[2 + 15];
    b[0] = (uint8_t)(f->id >> 8);
    b[1] = (uint8_t)f->id; /* data ID in the CRC catches frames delivered under the wrong ID */
    memcpy(b + 2, f->data, 15);
    return crc8_sae_j1850(b, sizeof b);
}

void tlm_pack_status(const tlm_status_t *s, uint8_t counter, can_frame_t *f)
{
    memset(f, 0, sizeof *f);
    f->id = TLM_ID_STATUS;
    f->len = 16;
    put16(f->data + 0, s->v_in_mv);
    put16(f->data + 2, (uint16_t)s->i_in_ma);
    put16(f->data + 4, (uint16_t)s->t_batt_cc);
    put16(f->data + 6, (uint16_t)s->t_bay_cc);
    f->data[8] = s->htr_batt_duty;
    f->data[9] = s->htr_bay_duty;
    f->data[10] = (uint8_t)((s->pm_state << 4) | (s->faults & 0x0F));
    f->data[11] = counter & 0x0F;
    f->data[15] = frame_crc(f);
}

bool tlm_unpack_status(const can_frame_t *f, tlm_status_t *s, uint8_t *last_counter, bool *lost)
{
    if (f->id != TLM_ID_STATUS || f->len != 16 || frame_crc(f) != f->data[15])
        return false;
    uint8_t ctr = f->data[11] & 0x0F;
    uint8_t expect = (uint8_t)((*last_counter + 1) & 0x0F);
    if (ctr == *last_counter)
        return false; /* repeated frame: stale sender */
    *lost = (ctr != expect);
    *last_counter = ctr;
    s->v_in_mv = get16(f->data + 0);
    s->i_in_ma = (int16_t)get16(f->data + 2);
    s->t_batt_cc = (int16_t)get16(f->data + 4);
    s->t_bay_cc = (int16_t)get16(f->data + 6);
    s->htr_batt_duty = f->data[8];
    s->htr_bay_duty = f->data[9];
    s->pm_state = f->data[10] >> 4;
    s->faults = f->data[10] & 0x0F;
    return true;
}
