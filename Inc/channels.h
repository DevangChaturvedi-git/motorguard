/*
 * channels.h
 *
 * This is the one table everything else in the project reads from.
 * Layer 2 (protection.c) uses it for hysteresis/I^2t/plausibility.
 * Layer 3 (limiter.c) uses the same continuous_limit field for the
 * proactive clamp. The point of putting it in one place is that the
 * two layers can never quietly disagree about what "safe" means for
 * a given channel - there's nowhere for a second, drifted copy of a
 * threshold to live.
 *
 * Deliberately NOT motor-specific. Add a row and a case in
 * channel_read_value() (channels.c), and the fusion engine in
 * protection.c doesn't care what it's protecting.
 *
 * Values are obtained through channel_read_value(id) - a plain
 * switch-dispatch, not a function pointer stored per-channel. That's
 * not the original design (see git history / README) - it was a
 * function-pointer table until testing showed this simulator's core
 * dies on the very first indirect call through a pointer, with every
 * direct call before it working fine. Rather than fight that, the
 * indirection was removed entirely.
 */

#ifndef CHANNELS_H
#define CHANNELS_H

#include <stdint.h>
#include <stdbool.h>
#include "fixed_point.h"

typedef enum {
    CH_CURRENT = 0,
    CH_TEMPERATURE,
    CH_SPEED,           /* used for the overspeed check, not the PI feedback path */
    CH_COUNT
} channel_id_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_TRIP,          /* threshold/I^2t exceeded - can clear on approval */
    FAULT_SENSOR,        /* reading is implausible, not just out of range - needs a human to look at the sensor, approval alone can't fix it */
    FAULT_STALL,         /* handled separately in protection.c, not part of the generic table, but shares the same fault_event_t shape */
} fault_kind_t;

/* one row per protected quantity */
typedef struct {
    channel_id_t id;
    const char  *name;              /* for the UART diagnostic line, not used in logic */

    q16_t continuous_limit;         /* tier 1 - Layer 3 clamps to this, forever-safe */
    q16_t absolute_limit;           /* tier 3 - instant fault, no accumulation, checked here as a software backstop behind BKIN */
    q16_t prealarm_fraction;        /* e.g. Q16_FROM_PCT(80) - warn before tripping */

    q16_t hysteresis_band;          /* upper/lower trip points are continuous_limit +/- this, see protection.c */
    uint8_t confidence_required;    /* consecutive over-threshold samples needed before it counts as real, not noise */

    /* I^2t (inverse-time) accumulator state - lives here because it's
     * per-channel, not because the table needs to know the math.
     * accumulator/ceiling/decay are all in the same raw "excess^2,
     * right-shifted by I2T_SHIFT" units - see protection.c. Not a
     * physical unit, just a scale picked so trip timing lands on a
     * human-watchable timescale for this demo. */
    int64_t i2t_accumulator;
    int64_t i2t_trip_ceiling;
    int64_t i2t_decay_per_tick;      /* how much the accumulator relaxes per tick when back under the continuous limit - this is the "cooling off" behaviour, without it a string of harmless brief spikes could stack up over time */
    bool    i2t_was_accumulating;    /* hysteresis state - see protection.c */

    /* sensor plausibility - not "is it over the limit", but "could this
     * reading possibly be real" (pinned at a rail, etc) */
    q16_t plausibility_min;
    q16_t plausibility_max;
    uint8_t implausible_count;
    uint8_t implausible_confidence_required;

    /* runtime state, not configuration - separated below for clarity
     * but kept in the same struct since there's only one of each */
    uint8_t  over_threshold_count;
    uint8_t  prealarm_latched;
} channel_t;

extern channel_t g_channels[CH_COUNT];

void channels_init(void);
channel_t *channels_get(channel_id_t id);

/* direct-dispatch replacement for the old per-channel function
 * pointer - see the note above the struct definition */
q16_t channel_read_value(channel_id_t id);

#endif /* CHANNELS_H */
