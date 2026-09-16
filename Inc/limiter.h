/*
 * limiter.h
 *
 * Layer 3 - the proactive limiter. Sits between whatever Layer 1
 * computed and the actual PWM register, and makes sure the commanded
 * duty can never ask for more than the continuous-safe rating - not
 * just during a fault, all the time, every tick. This is what lets the
 * fan example hold "jogging pace" forever instead of running flat out
 * and hoping Layer 2's fault engine catches it if it overdoes it.
 *
 * Reads continuous_limit straight out of the same channel descriptor
 * table Layer 2 uses for its I^2t math (see channels.h) - one shared
 * number, not two copies that could quietly drift apart.
 */

#ifndef LIMITER_H
#define LIMITER_H

#include "fixed_point.h"
#include <stdbool.h>

typedef struct {
    q16_t clamped_duty;
    bool  was_clamped;     /* feed this back into control_tick's saturated flag */
} limiter_result_t;

limiter_result_t limiter_apply(q16_t requested_duty);

#endif /* LIMITER_H */
