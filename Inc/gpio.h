#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>

typedef enum {
    LED_RUN = 0,
    LED_WARN,
    LED_FAULT,
    LED_LOCKOUT,
} led_t;

typedef enum {
    SW_APPROVE = 0,
    SW_INJECT_STALL,
    SW_INJECT_OVERLOAD,
    SW_INJECT_SENSOR,
    SW_COUNT
} switch_t;

void gpio_init(void);
void led_set(led_t led, bool on);
void heartbeat_toggle(void);

/* edge-detected: true exactly once per press, not held-down spam.
 * called from the fixed 5ms tick so the debounce window is implicitly
 * in ticks, not a separate timer */
bool switch_pressed_edge(switch_t sw);

/* raw level, no debounce/edge-detection - used for the fault-injection
 * switches, where "held down" is meant to mean "condition is active
 * right now", not a one-shot action like the approve button */
bool switch_is_pressed(switch_t sw);

/* The diagram drives PB12 high for a simulated comparator/BKIN trip.  TIM1
 * remains the primary hardware path; this readback is a simulator fallback so
 * the same button still demonstrates a latched fault when BKIN is not modelled. */
bool gpio_bkin_asserted(void);

#endif /* GPIO_H */
