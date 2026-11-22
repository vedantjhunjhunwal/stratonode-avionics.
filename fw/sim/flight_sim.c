/* Closed-loop flight simulation of StratoNode over the 10.5 h design mission.
 *
 * The firmware application layer (fw/src/app.c and everything under it) runs unmodified at
 * 1 kHz (10 kHz while a dropout is in progress) against plant models of:
 *   - the ISA-13.5 K atmosphere (-70 C at 15.5 km) and the mission profile,
 *   - the battery box and avionics bay (same thermal model as analysis/thermal.py),
 *   - a 4S5P Li-ion pack (OCV(SOC), Arrhenius internal resistance, I^2R self-heating),
 *   - the 5 V buck / 3V3 LDO / loads, and the hold-up capacitor bank behind the ideal diode.
 * Injected faults: connector micro-dropouts (2-50 ms), I2C bus lock-ups on the temperature
 * sensor (recovered with i2c_bus_recover), a 20-minute battery NTC open circuit, GNSS UART
 * byte corruption, CAN bit errors.
 *
 *   flight_sim [--seed N] [--no-shed] [--csv out.csv] [--json out.json]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "drivers.h"

/* ------------------------------------------------------------------ RNG (xorshift, reproducible) */
static uint64_t rng = 88172645463325252ull;
static double urand(void)
{
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    return (double)(rng >> 11) * (1.0 / 9007199254740992.0);
}
static double nrand(void) { return sqrt(-2 * log(urand() + 1e-300)) * cos(2 * M_PI * urand()); }

/* ------------------------------------------------------------------ atmosphere and mission */
static void isa(double h, double dT, double *T_c, double *p)
{
    double T, P;
    if (h <= 11000) { T = 288.15 - 0.0065 * h; P = 101325 * pow(T / 288.15, 5.255877); }
    else if (h <= 20000) { T = 216.65; P = 22632.06 * exp(-9.80665 * (h - 11000) / (287.053 * 216.65)); }
    else { T = 216.65 + 0.001 * (h - 20000); P = 5474.889 * pow(T / 216.65, -34.1632); }
    *T_c = T + dT - 273.15;
    *p = P;
}

#define H_GROUND 5400.0
#define H_CRUISE 15500.0
static const double PH_END[] = {1.0 * 3600, 2.5 * 3600, 10.0 * 3600, 10.5 * 3600};

static void mission(double t, double *alt, double *T_c, double *p, int *phase)
{
    if (t < PH_END[0]) { *phase = 0; *alt = H_GROUND; }
    else if (t < PH_END[1]) { *phase = 1; *alt = H_GROUND + (H_CRUISE - H_GROUND) * (t - PH_END[0]) / (PH_END[1] - PH_END[0]); }
    else if (t < PH_END[2]) { *phase = 2; *alt = H_CRUISE; }
    else { *phase = 3; *alt = H_CRUISE - (H_CRUISE - H_GROUND) * (t - PH_END[2]) / (PH_END[3] - PH_END[2]); }
    isa(*alt, -13.5, T_c, p);
    if (*phase == 0) *T_c = -30.0; /* cold-soaked launch site */
}

/* ------------------------------------------------------------------ thermal (as analysis/thermal.py) */
typedef struct { double l, w, h, ins, k, eps; } box_t;
static const box_t BATT_BOX = {0.140, 0.080, 0.075, 0.020, 0.028, 0.85};
static const box_t BAY_BOX = {0.110, 0.080, 0.040, 0.015, 0.028, 0.85};

static double box_loss(const box_t *b, double t_in, double t_amb, double p)
{
    double a_in = 2 * (b->l * b->w + b->l * b->h + b->w * b->h);
    double L = b->l + 2 * b->ins, W = b->w + 2 * b->ins, H = b->h + 2 * b->ins;
    double a_out = 2 * (L * W + L * H + W * H);
    double g_wall = b->k * sqrt(a_in * a_out) / b->ins;
    double ts = 0.5 * (t_in + t_amb), q = 0;
    for (int i = 0; i < 30; i++) {
        double tm = 0.5 * (ts + t_amb) + 273.15;
        double g_out = (4.0 * sqrt(p / 101325.0) + 4 * b->eps * 5.670374e-8 * tm * tm * tm) * a_out;
        q = (t_in - t_amb) / (1 / g_wall + 1 / g_out);
        ts = t_amb + q / g_out;
    }
    return q;
}

/* ------------------------------------------------------------------ battery 4S5P */
static const double OCV_SOC[][2] = {{0, 3.00}, {.05, 3.30}, {.1, 3.42}, {.2, 3.52}, {.3, 3.58}, {.4, 3.63},
                                     {.5, 3.68}, {.6, 3.75}, {.7, 3.84}, {.8, 3.94}, {.9, 4.05}, {1, 4.17}};
static double ocv(double soc)
{
    if (soc <= 0) return 3.0;
    for (int i = 1; i < 12; i++)
        if (soc <= OCV_SOC[i][0])
            return OCV_SOC[i - 1][1] + (OCV_SOC[i][1] - OCV_SOC[i - 1][1]) * (soc - OCV_SOC[i - 1][0]) / (OCV_SOC[i][0] - OCV_SOC[i - 1][0]);
    return 4.17;
}
static double r_cell(double t_c) { return 0.035 * exp(20000.0 / 8.314 * (1 / (t_c + 273.15) - 1 / 298.15)); }
static double cap_frac(double t_c)
{
    static const double T[][2] = {{-30, .55}, {-20, .72}, {-10, .85}, {0, .93}, {10, .97}, {25, 1}};
    if (t_c <= -30) return .55;
    for (int i = 1; i < 6; i++)
        if (t_c <= T[i][0]) return T[i - 1][1] + (T[i][1] - T[i - 1][1]) * (t_c - T[i - 1][0]) / (T[i][0] - T[i - 1][0]);
    return 1;
}

/* ------------------------------------------------------------------ board electrical (as analysis/power.py) */
static double buck_loss(double i, double vin) { return 0.03 + 0.30 * i * (vin / 12.0) + 0.12 * i * i; }

/* returns battery-side power of the board (not heaters) and its on-board dissipation */
static double board_power(bool radio, bool radio_tx, bool payload, double vin, double *dissipation)
{
    double p33 = 0.150 + 0.006 + 0.007;
    double p5 = 0.085 + 0.350 + (radio ? (radio_tx ? 4.0 : 0.3) : 0.0) + (payload ? 3.0 : 0.0);
    double i33 = p33 / 3.3, ldo = (5.0 - 3.3) * i33 + 5.0 * 55e-6;
    double p5t = p5 + p33 + ldo, i5 = p5t / 5.0;
    double bl = buck_loss(i5, vin < 6 ? 6 : vin);
    *dissipation = p33 + ldo + bl;
    return p5t + bl;
}

/* ------------------------------------------------------------------ fake I2C pins for a hung TMP117 */
typedef struct { int stuck_for, clocks; bool scl, sda; } pins_t;
static void ps(void *c, bool h) { pins_t *f = c; if (!f->scl && h) f->clocks++; f->scl = h; }
static bool pr(void *c) { pins_t *f = c; return f->sda && f->clocks >= f->stuck_for; }
static void pw(void *c, bool h) { ((pins_t *)c)->sda = h; }
static void pd(void *c, uint32_t us) { (void)c; (void)us; }

static void nmea_line(char *out, size_t n, const char *body)
{
    uint8_t s = 0;
    for (const char *p = body; *p; p++) s ^= (uint8_t)*p;
    snprintf(out, n, "$%s*%02X\r\n", body, s);
}

int main(int argc, char **argv)
{
    int seed = 1;
    bool shed = true;
    const char *csv_path = NULL, *json_path = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--no-shed")) shed = false;
        else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else if (!strcmp(argv[i], "--json") && i + 1 < argc) json_path = argv[++i];
    }
    rng ^= (uint64_t)seed * 0x9E3779B97F4A7C15ull;
    FILE *csv = csv_path ? fopen(csv_path, "w") : NULL;
    if (csv) fprintf(csv, "t_h,alt_m,t_amb_c,t_batt_c,t_bay_c,soc,v_pack,duty_batt,duty_bay,p_total_w,pm_state\n");

    app_t app;
    app_init(&app);
    nmea_t gps;
    nmea_init(&gps);

    const int NP = 5, NS = 4;
    const double C_BATT = NP * NS * 0.048 * 900.0 + 150.0, C_BAY = 400.0;
    const double CAP_AH = 3.35 * NP, C_HOLD = 3 * 470e-6 * 0.95;
    double T_batt = -30.0, T_bay = -30.0, soc = 1.0, ah_used = 0, wh_used = 0;
    double v_cap = 4 * ocv(1.0), v_pack = v_cap;
    double t_ntc_fault = 7.0 * 3600;
    double next_drop = 600 + urand() * 1200, drop_end = -1;
    double next_i2c_hang = 1800 + urand() * 3600;
    int i2c_hang_s = 0;
    double sum_heat_wh = 0;

    uint32_t brownouts = 0, i2c_recoveries = 0, i2c_fail = 0, gnss_corrupted = 0, gnss_wrong_accepted = 0;
    double gnss_worst_err = 0;
    uint32_t can_frames = 0, can_err_injected = 0, can_err_caught = 0, drops = 0;
    double min_v_cap = 99, min_tb_flight = 99, max_tb = -99, min_ty_flight = 99, max_ty = -99, drop_total_ms = 0;
    double longest_drop = 0;
    int ntc_fault_seen = 0;
    uint8_t can_last = 15;

    app_inputs_t in = {0};
    double t = 0;
    double next_slow = 0;
    const double T_END = PH_END[3];
    double heat_dissipation = 0;
    bool radio_tx = false;
    int phase = 0;
    double alt = 0, T_amb = 0, p_amb = 0;
    while (t < T_END) {
        bool slow = t + 1e-9 >= next_slow;
        if (slow) next_slow += 1.0;
        if (slow) {
            mission(t, &alt, &T_amb, &p_amb, &phase);
            radio_tx = urand() < 0.25;
        }
        /* -------- input dropouts */
        bool dropped = t < drop_end;
        if (!dropped && t >= next_drop) {
            double len = 0.002 + urand() * 0.048;
            drop_end = t + len;
            next_drop = t + 600 + urand() * 1200;
            drops++;
            drop_total_ms += len * 1000;
            if (len > longest_drop) longest_drop = len;
            dropped = true;
        }
        double sub = dropped || t < drop_end + 0.3 ? 1e-4 : 1e-3; /* fine steps around events */

        /* -------- electrical: pack -> shunt -> ideal diode -> VBAT (hold-up caps) */
        /* --no-shed baseline: loads stay exactly as they were when the input dropped */
        static app_outputs_t frozen;
        static bool was_dropped = false;
        if (dropped && !was_dropped) frozen = app.out;
        was_dropped = dropped;
        const app_outputs_t *o = (!shed && dropped) ? &frozen : &app.out;
        double diss;
        double p_board = board_power(o->radio_en, radio_tx, o->payload_en, v_cap, &diss);
        double p_htr = v_cap * v_cap * (o->duty_batt / 16.4 + o->duty_bay / 47.0);
        double r_pack = r_cell(T_batt) * NS / NP + 0.024;
        double v_ocv = NS * ocv(soc);
        double i_pack = 0, v_in;
        if (!dropped) {
            /* pack supplies loads and recharges the caps instantly (stiff source vs 1.3 mF) */
            double p = p_board + p_htr;
            i_pack = (v_ocv - sqrt(fmax(v_ocv * v_ocv - 4 * r_pack * p, 0))) / (2 * r_pack);
            v_pack = v_ocv - i_pack * r_pack;
            v_cap = v_pack - 0.01;
            v_in = v_pack;
        } else {
            v_in = 0.0; /* VIN collapses: the ideal diode isolates the caps */
            double p = (v_cap > 5.6 ? p_board : 0) + p_htr;
            v_cap = sqrt(fmax(v_cap * v_cap - 2 * p * sub / C_HOLD, 0));
            if (v_cap < min_v_cap) min_v_cap = v_cap;
        }
        if (v_cap < 5.6 && !dropped) v_cap = v_pack;
        static bool in_brownout = false;
        if (v_cap < 5.6 && !in_brownout) { brownouts++; in_brownout = true; app_init(&app); }
        if (v_cap >= 5.6) in_brownout = false;

        /* -------- firmware tick. Power-fail interrupt: INA226 ALERT (0.28 ms conversion cycle) -> EXTI4, ~0.35 ms */
        static double t_irq_due = -1;
        if (dropped && t_irq_due < 0) t_irq_due = t + 3.5e-4;
        in.power_fail_irq = shed && t_irq_due >= 0 && t >= t_irq_due;
        if (!dropped) t_irq_due = -1;
        in.v_in = (float)v_in;
        in.payload_requested = (phase == 2);
        in.new_slow_sample = slow;
        if (slow) {
            /* battery NTC: open circuit for 20 minutes reads as absurd cold */
            bool ntc_open = t >= t_ntc_fault && t < t_ntc_fault + 1200;
            in.t_batt_c = ntc_open ? -150.0f : (float)(T_batt + 0.05 * nrand());
            in.t_batt_ok = true;
            /* TMP117 on I2C1: occasional hang, recovered by clocking out the stuck slave */
            if (t >= next_i2c_hang) { i2c_hang_s = 1 + (int)(urand() * 3); next_i2c_hang = t + 1800 + urand() * 3600; }
            if (i2c_hang_s > 0) {
                pins_t f = {1 + (int)(urand() * 8), 0, true, true};
                hal_i2c_pins_t pins = {ps, pr, pw, pd, &f};
                int used;
                if (i2c_bus_recover(&pins, &used) == HAL_OK) i2c_recoveries++; else i2c_fail++;
                in.t_bay_ok = false; /* this second's read is lost */
                i2c_hang_s--;
            } else {
                in.t_bay_ok = true;
                in.t_bay_c = (float)(T_bay + 0.01 * nrand());
            }
            in.v_htr = (float)v_cap;
            in.i_htr = (float)(v_cap * (o->duty_batt / 16.4 + o->duty_bay / 47.0));
            if (app.h_batt.faults & HTR_SENSOR_FAULT) ntc_fault_seen = 1;
        }
        static double next_tick = 0;
        if (in.power_fail_irq) {
            app_power_fail_isr(&app); /* interrupt: acts now, not at the next 1 ms tick */
            t_irq_due = 1e18;         /* fire once per event */
            in.power_fail_irq = false;
        }
        if (t + 1e-9 >= next_tick) {
            app_tick_1ms(&app, &in);
            next_tick += 1e-3;
        }

        /* -------- plant integration */
        double i_cell = i_pack / NP;
        double self_heat = i_pack * i_pack * (r_pack - 0.024);
        double q_hb = v_cap * v_cap * o->duty_batt / 16.4, q_hy = v_cap * v_cap * o->duty_bay / 47.0;
        T_batt += (q_hb + self_heat - box_loss(&BATT_BOX, T_batt, T_amb, p_amb)) * sub / C_BATT;
        T_bay += (q_hy + diss - box_loss(&BAY_BOX, T_bay, T_amb, p_amb)) * sub / C_BAY;
        heat_dissipation = diss;
        ah_used += i_pack * sub / 3600.0;
        wh_used += i_pack * v_pack * sub / 3600.0;
        sum_heat_wh += (q_hb + q_hy) * sub / 3600.0;
        soc = 1.0 - ah_used / (CAP_AH * cap_frac(T_batt) * 0.8 + 1e-9) * 0 - ah_used / (CAP_AH);
        (void)i_cell;

        if (phase >= 1 && phase <= 2 && t > PH_END[0] + 600) {
            if (T_batt < min_tb_flight) min_tb_flight = T_batt;
            if (T_bay < min_ty_flight) min_ty_flight = T_bay;
        }
        if (T_batt > max_tb) max_tb = T_batt;
        if (T_bay > max_ty) max_ty = T_bay;

        if (slow) {
            /* GNSS: one GGA + one NavIC GSV per second, 0.3% of bytes corrupted on the wire */
            char body[128], line[160];
            int ts = (int)fmod(t + 6 * 3600, 86400); /* UTC hhmmss, launch at 06:00 */
            snprintf(body, sizeof body, "GNGGA,%02d%02d%02d.00,3517.4700,N,07710.9480,E,1,14,0.8,%.1f,M,-48.1,M,,",
                     ts / 3600, (ts / 60) % 60, ts % 60, alt);
            nmea_line(line, sizeof line, body);
            bool corrupt = false;
            for (char *c = line + 1; *c && *c != '\r'; c++)
                if (urand() < 0.003) { *c ^= (char)(1 << (int)(urand() * 7)); corrupt = true; }
            uint32_t upd_before = gps.n_fix_updates;
            for (char *c = line; *c; c++) nmea_feed(&gps, *c);
            if (corrupt) gnss_corrupted++;
            double err = fabs(gps.fix.alt_m - alt);
            if (gps.n_fix_updates > upd_before && err > 0.11) { gnss_wrong_accepted++; if (err > gnss_worst_err) gnss_worst_err = err; }
            nmea_line(line, sizeof line, "GIGSV,1,1,07,02,45,120,38,03,60,200,40,05,30,80,35,06,25,300,33,0");
            for (char *c = line; *c; c++) nmea_feed(&gps, *c);
        }
        static double next_can = 0;
        if (t + 1e-9 >= next_can) {
            next_can += 0.1;
            /* CAN-FD status frame every 100 ms; 1 in 500 frames gets a random bit error */
            tlm_status_t st, rx;
            app_status(&app, &in, (float)i_pack, &st);
            can_frame_t f;
            tlm_pack_status(&st, app.tlm_counter++, &f);
            bool err = urand() < 0.002;
            if (err) { int b = (int)(urand() * 120); f.data[b / 8] ^= (uint8_t)(1u << (b % 8)); can_err_injected++; }
            bool lost;
            bool ok = tlm_unpack_status(&f, &rx, &can_last, &lost);
            if (err && !ok) can_err_caught++;
            if (!ok) can_last = (uint8_t)((can_last + 1) & 0x0F); /* receiver resyncs on next frame */
            can_frames++;
        }
        static double next_csv = 0;
        if (csv && t + 1e-9 >= next_csv && (next_csv += 10.0) > 0)
            fprintf(csv, "%.4f,%.0f,%.2f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%d\n", t / 3600, alt, T_amb, T_batt, T_bay,
                    soc, v_pack, o->duty_batt, o->duty_bay, p_board + p_htr, app.pm.state);
        t += sub;
    }
    (void)heat_dissipation;
    if (csv) fclose(csv);

    char js[3072];
    snprintf(js, sizeof js,
             "{\n  \"seed\": %d, \"load_shedding\": %s,\n  \"hours\": %.1f, \"energy_Wh\": %.1f, \"heater_energy_Wh\": %.1f,"
             " \"final_soc\": %.3f,\n  \"battery_min_in_flight_C\": %.2f, \"battery_max_C\": %.2f,"
             " \"bay_min_in_flight_C\": %.2f, \"bay_max_C\": %.2f,\n"
             "  \"dropouts\": %u, \"dropout_longest_ms\": %.1f, \"holdup_min_V\": %.2f, \"brownout_resets\": %u,"
             " \"pm_recoveries\": %u, \"pm_power_lost\": %u,\n"
             "  \"i2c_lockups_recovered\": %u, \"i2c_unrecovered\": %u, \"ntc_fault_detected\": %s,"
             " \"heater_sensor_faults\": %u,\n"
             "  \"gnss_sentences_ok\": %u, \"gnss_corrupted_sent\": %u, \"gnss_bad_checksum\": %u,"
             " \"gnss_implausible_rejected\": %u, \"gnss_corrupt_accepted_with_wrong_data\": %u, \"gnss_worst_accepted_alt_error_m\": %.1f,\n"
             "  \"can_frames\": %u, \"can_errors_injected\": %u, \"can_errors_caught\": %u\n}\n",
             seed, shed ? "true" : "false", T_END / 3600, wh_used, sum_heat_wh, soc, min_tb_flight, max_tb,
             min_ty_flight, max_ty, drops, longest_drop * 1000, min_v_cap, brownouts, app.pm.n_recoveries,
             app.pm.n_power_lost, i2c_recoveries, i2c_fail, ntc_fault_seen ? "true" : "false",
             app.h_batt.n_sensor_faults, gps.n_ok, gnss_corrupted, gps.n_bad_checksum, gps.n_implausible, gnss_wrong_accepted, gnss_worst_err,
             can_frames, can_err_injected, can_err_caught);
    fputs(js, stdout);
    if (json_path) {
        FILE *j = fopen(json_path, "w");
        if (j) { fputs(js, j); fclose(j); }
    }
    (void)drop_total_ms;
    return 0;
}
