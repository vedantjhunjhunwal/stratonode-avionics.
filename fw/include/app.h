/* Application layer: one 1 ms tick that ties the power manager, both heater loops, sensor
 * fault handling and telemetry together. The STM32 main loop and the host flight simulator
 * both call exactly this code. */
#ifndef STRATONODE_APP_H
#define STRATONODE_APP_H

#include "node.h"

#define APP_SENSOR_HOLD_S 5 /* tolerate this many consecutive failed reads before declaring a fault */

typedef struct {
    float v_in;            /* filtered input voltage (ADC), every tick */
    bool power_fail_irq;   /* ADC watchdog / INA226 ALERT fired since the last tick */
    /* once per second (valid when new_slow_sample): */
    bool new_slow_sample;
    float t_batt_c, t_bay_c;
    bool t_batt_ok, t_bay_ok;
    float v_htr, i_htr;    /* heater supply and measured heater current (INA226 #2) */
    bool payload_requested; /* mission phase wants the payload on */
} app_inputs_t;

typedef struct {
    bool radio_en, payload_en;
    float duty_batt, duty_bay;
} app_outputs_t;

typedef struct {
    power_mgr_t pm;
    heater_t h_batt, h_bay;
    float last_t_batt, last_t_bay;
    int bad_batt_s, bad_bay_s;
    uint8_t tlm_counter;
    uint32_t ms;
    app_outputs_t out;
} app_t;

void app_init(app_t *a);
const app_outputs_t *app_tick_1ms(app_t *a, const app_inputs_t *in);
/* Interrupt entry (ADC analog watchdog on VIN_SENSE, or INA226 ALERT): sheds radio, payload and
 * both heaters before returning. */
void app_power_fail_isr(app_t *a);
void app_status(const app_t *a, const app_inputs_t *in, float i_in_a, tlm_status_t *s);

#endif
