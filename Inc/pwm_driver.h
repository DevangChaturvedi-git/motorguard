/*
 * pwm_driver.h
 *
 * TIM1_CH1 drives the (virtual) motor. BDTR.BKE wires TIM1's own break
 * circuit to the BKIN pin - when it goes high, the timer forces CH1
 * off in hardware, in silicon, with zero firmware involvement, even if
 * the CPU is completely wedged. BDTR.AOE is deliberately left at 0:
 * once a break clears MOE, it stays cleared until firmware explicitly
 * sets MOE again - the timer itself will not silently resume output.
 * That's not a limitation, it's the same "no auto-recovery without an
 * explicit act" principle applied at the silicon level.
 *
 * In a real build, BKIN is driven by a discrete comparator watching
 * the current-sense shunt. Wokwi has no comparator/op-amp part to wire
 * one up with, so in the simulation this pin is driven directly by a
 * pushbutton instead - electrically identical at the pin, see
 * pin_map.h for the longer version of this note.
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "fixed_point.h"
#include <stdbool.h>

void pwm_driver_init(void);

/* duty is 0..Q16_ONE. Written through CCR1's shadow register (OC1PE),
 * so a change made mid-cycle takes effect at the next update event
 * instead of glitching the current pulse. */
void pwm_set_duty(q16_t duty);

/* re-arms the output after a break event - this is the "software
 * explicitly re-authorizes power" step, it does not clear whatever
 * caused the break in the first place, that's the supervisor's job */
void pwm_reenable_output(void);

/* true exactly once if a break event has fired since the last call -
 * this is software finding out what hardware already did on its own,
 * not what caused the shutoff */
bool pwm_break_event_pending(void);

#endif /* PWM_DRIVER_H */
