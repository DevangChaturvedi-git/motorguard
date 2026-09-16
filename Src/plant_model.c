#include "plant_model.h"

/*
 * Wokwi baseline plant
 * -------------------
 * Wokwi has no DC-motor electrical/mechanical model.  The earlier detailed
 * Q16.16 plant was useful for offline tuning, but its chained 64-bit math did
 * not behave repeatably in this simulator.  This small model is intentional:
 * it makes the two existing pots visible and deterministic while preserving
 * all of the production-facing interfaces used by the control/protection code.
 *
 * - PA0 setpoint reaches PWM through the supervisor.
 * - PA1 load is the simulated current sensor.
 * - the red buttons inject their named conditions.
 *
 * It is a Wokwi demonstrator, not a motor calibration model.
 */
static q16_t s_speed_q16;
static q16_t s_current_q16;
static q16_t s_temp_q16;

static bool s_inject_stall;
static bool s_inject_overload;
static bool s_inject_sensor_fault;

void plant_model_init(void)
{
    s_speed_q16 = 0;
    s_current_q16 = 0;
    s_temp_q16 = 0;
    s_inject_stall = false;
    s_inject_overload = false;
    s_inject_sensor_fault = false;
}

void plant_model_step(q16_t commanded_duty, q16_t load_disturbance)
{
    if (commanded_duty < 0) commanded_duty = 0;
    if (load_disturbance < 0) load_disturbance = 0;

    /* A direct, stable speed proxy.  The stall injector is the only case that
     * reports a commanded-but-not-turning condition. */
    s_speed_q16 = s_inject_stall ? 0 : commanded_duty;

    /* The second pot is the demonstrable load/current sensor.  Averaging it
     * with duty keeps the normal 0..100% operating range below trip limits. */
    s_current_q16 = (commanded_duty + load_disturbance) / 2;
    if (s_inject_overload) s_current_q16 += Q16_FROM_PCT(180);

    /* A bounded temperature proxy for diagnostics; it intentionally avoids
     * dynamic thermal integration in the Wokwi-only baseline. */
    s_temp_q16 = s_current_q16 / 3;
}

q16_t plant_model_read_speed(void)
{
    return s_speed_q16;
}

q16_t plant_model_read_current(void)
{
    return s_inject_sensor_fault ? 0 : s_current_q16;
}

q16_t plant_model_read_temperature(void)
{
    return s_temp_q16;
}

q16_t plant_model_max_continuous_duty(q16_t continuous_current_limit)
{
    /* Keep the existing approval threshold (~57% at a 115% current limit)
     * without the old resistance/back-EMF approximation. */
    return continuous_current_limit / 2;
}

void plant_model_inject_stall(bool active)           { s_inject_stall = active; }
void plant_model_inject_overload(bool active)        { s_inject_overload = active; }
void plant_model_inject_sensor_fault(bool active)    { s_inject_sensor_fault = active; }
