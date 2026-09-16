/*
 * control.h
 *
 * Layer 1 - Maintain Operation. Its only job is hitting the setpoint.
 * It does not know about limits (that's Layer 3) or faults (Layer 2) -
 * it just tries its best, and expects to be told from outside when its
 * output got clamped so it can stop winding up its integrator.
 *
 * Runs on a fixed 5ms tick (see main.c's SysTick), not off the raw
 * input-capture interrupt - tying the control math to a variable
 * update rate would mean the effective loop gain drifts with speed
 * (pulses arrive far apart at low speed), which is a real way to
 * destabilise a loop that was only ever tuned at one operating point.
 * The tick just reads whatever speed value is current at that moment.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "fixed_point.h"
#include <stdbool.h>

typedef struct {
    q16_t kp;
    q16_t ki;
    q16_t integral;
    q16_t prev_feedback;        /* for the rate-of-change plausibility clamp */
    bool  have_prev_feedback;
} control_state_t;

void control_init(control_state_t *s, q16_t kp, q16_t ki);

/* one control tick. feedback_raw is whatever the speed channel just
 * read - may be lightly sanity-clamped before use (see control.c).
 * feedforward_duty is an optional head start (0 if not used).
 * saturated tells the loop the LAST tick's output got clamped by
 * Layer 3 or pinned at 0/100%, so this tick should not integrate -
 * this is the conditional-integration anti-windup rule: freeze on ANY
 * cause of saturation, not just a fault latch. Returns requested duty,
 * 0..Q16_ONE, BEFORE Layer 3 has had a chance to clamp it. */
q16_t control_tick(control_state_t *s, q16_t setpoint, q16_t feedback_raw,
                    q16_t feedforward_duty, bool saturated);

#endif /* CONTROL_H */
