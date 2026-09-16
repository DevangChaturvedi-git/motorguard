/*
 * plant_model.h
 *
 * Wokwi doesn't model DC motor electrical/mechanical physics the way it
 * does for, say, a servo - there's no back-EMF, no inertia, no real
 * torque-speed curve on anything in the part library. So instead of
 * driving a "real" simulated motor, this is a small first-order model
 * living entirely in firmware: given the commanded PWM duty and a load
 * disturbance (from a pot), it estimates what speed/current/temperature
 * a real small DC motor would settle at.
 *
 * This is a completely standard technique (plant modelling for control
 * validation, HIL testing, digital twins are all the same idea at
 * different scales) - it's not a shortcut being passed off as real
 * hardware, it's how you validate a control loop before it ever touches
 * a physical motor. Documented here plainly for exactly that reason.
 *
 * Everything the rest of the firmware sees is just three q16_t reads
 * (speed, current, temperature) - it has no idea whether those numbers
 * came from this model or a real ADC + tachometer. That's deliberate;
 * swapping this file out for real sensor code later shouldn't touch
 * anything else.
 */

#ifndef PLANT_MODEL_H
#define PLANT_MODEL_H

#include "fixed_point.h"
#include <stdbool.h>

void plant_model_init(void);

/* advance the model by one control tick, given the duty cycle actually
 * commanded to the (virtual) motor this tick, 0..Q16_ONE */
void plant_model_step(q16_t commanded_duty, q16_t load_disturbance);

q16_t plant_model_read_speed(void);
q16_t plant_model_read_current(void);
q16_t plant_model_read_temperature(void);

/* Layer 3 needs to turn "the continuous-safe current limit" into "the
 * duty cycle that guarantees we stay under it", but the actual
 * relationship between duty and current is plant-specific (depends on
 * winding resistance, back-EMF constant) - so that one piece of
 * plant-specific math lives here instead of leaking into limiter.c.
 * Uses the worst case (locked-rotor, zero speed, zero back-EMF) so the
 * derived ceiling is safe at every speed, not just one operating
 * point - genuine running current at any nonzero speed will always be
 * lower than this for the same duty, never higher. */
q16_t plant_model_max_continuous_duty(q16_t continuous_current_limit);

/* fault injection, driven by the pushbuttons in the Wokwi wiring - lets
 * the demo force each protection path without needing real hardware
 * that can actually be jammed or shorted */
void plant_model_inject_stall(bool active);
void plant_model_inject_overload(bool active);
void plant_model_inject_sensor_fault(bool active);

#endif /* PLANT_MODEL_H */
