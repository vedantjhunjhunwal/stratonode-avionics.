#include "app.h"

#include <string.h>

/* Thresholds for a 4S Li-ion pack. VIN is measured before the ideal diode, so it collapses
 * within microseconds of a connector opening while VBAT is still held up by the capacitors. */
static const pm_config_t PM_CFG = {
    .v_fail = 10.0f, .v_restore = 11.0f, .v_low = 13.2f, .v_crit = 12.4f,
    .holdup_max_ms = 200, .restore_ms = 100, .stagger_ms = 50, .low_ms = 10000,
};
/* Battery box: 16.4 ohm element (12.6 W at 14.4 V). Bay: 47 ohm (4.4 W). Gains tuned on the
 * analysis thermal model (tests/test_main.c, sim/flight_sim.c). */
static const heater_config_t HB_CFG = {5.0f, 0.15f, 0.002f, -60.0f, 80.0f, 16.4f, 0.55f, 1.0f};
static const heater_config_t HY_CFG = {-25.0f, 0.10f, 0.002f, -80.0f, 90.0f, 47.0f, 0.50f, 1.0f};

void app_init(app_t *a)
{
    memset(a, 0, sizeof *a);
    pm_init(&a->pm, &PM_CFG);
    heater_init(&a->h_batt, &HB_CFG);
    heater_init(&a->h_bay, &HY_CFG);
    a->last_t_batt = a->last_t_bay = -100.0f;
    a->bad_batt_s = a->bad_bay_s = APP_SENSOR_HOLD_S; /* no valid reading yet */
}

/* Hold the last good reading through short outages (an I2C recovery, one noisy sample);
 * only after APP_SENSOR_HOLD_S bad seconds is the heater put into its fallback mode. */
static bool hold(float *last, int *bad, float t, bool ok, float *out)
{
    if (ok) {
        *last = t;
        *bad = 0;
    } else if (*bad < APP_SENSOR_HOLD_S) {
        (*bad)++;
    }
    *out = ok ? t : *last;
    return ok || *bad < APP_SENSOR_HOLD_S;
}

void app_power_fail_isr(app_t *a)
{
    pm_on_power_fail_irq(&a->pm);
    a->out.radio_en = a->out.payload_en = false; /* real build: GPIO BSRR writes to the load switches */
    a->out.duty_batt = a->out.duty_bay = 0.0f;   /* and TIM3 CCR3/CCR4 = 0 */
}

const app_outputs_t *app_tick_1ms(app_t *a, const app_inputs_t *in)
{
    a->ms++;
    if (in->power_fail_irq)
        pm_on_power_fail_irq(&a->pm);
    pm_step(&a->pm, in->v_in, 1);

    if (in->new_slow_sample) {
        float tb, ty;
        bool okb = hold(&a->last_t_batt, &a->bad_batt_s, in->t_batt_c, in->t_batt_ok, &tb);
        bool oky = hold(&a->last_t_bay, &a->bad_bay_s, in->t_bay_c, in->t_bay_ok, &ty);
        a->h_bay.cfg.max_duty = a->pm.bay_heater_allowed ? 1.0f : 0.0f;
        heater_step(&a->h_batt, tb, okb, 1.0f);
        heater_step(&a->h_bay, ty, oky, 1.0f);
        if (a->pm.heaters_on)
            heater_check_element(&a->h_batt, in->v_htr, in->i_htr, a->h_batt.duty + a->h_bay.duty * 16.4f / 47.0f);
    }
    a->out.radio_en = a->pm.radio_on;
    a->out.payload_en = a->pm.payload_on && in->payload_requested;
    a->out.duty_batt = a->pm.heaters_on ? a->h_batt.duty : 0.0f;
    a->out.duty_bay = a->pm.heaters_on ? a->h_bay.duty : 0.0f;
    return &a->out;
}

void app_status(const app_t *a, const app_inputs_t *in, float i_in_a, tlm_status_t *s)
{
    s->v_in_mv = (uint16_t)(in->v_in * 1000.0f);
    s->i_in_ma = (int16_t)(i_in_a * 1000.0f);
    s->t_batt_cc = (int16_t)(a->last_t_batt * 100.0f);
    s->t_bay_cc = (int16_t)(a->last_t_bay * 100.0f);
    s->htr_batt_duty = (uint8_t)(a->out.duty_batt * 200.0f);
    s->htr_bay_duty = (uint8_t)(a->out.duty_bay * 200.0f);
    s->pm_state = (uint8_t)a->pm.state;
    s->faults = (uint8_t)(a->h_batt.faults | a->h_bay.faults);
}
