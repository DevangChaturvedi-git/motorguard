/*
 * protection.h
 *
 * Layer 2 - Reactive Protection. Runs every tick, in every supervisor
 * state, independent of what Layer 1/3 are doing. Its only job is to
 * notice when something has actually gone wrong and say so - it does
 * not itself decide what the supervisor does about it (see supervisor.c
 * for that split).
 *
 * Three trip classes, deliberately not collapsed into one flat "fault":
 *   FAULT_TRIP   - threshold/I^2t exceeded, can clear once approved
 *   FAULT_SENSOR - reading is implausible, approval alone can't fix a
 *                  dead sensor, needs a human to actually look at it
 *   FAULT_STALL  - commanded to run, isn't turning; handled separately
 *                  from the generic channel table since it's a
 *                  correlation (duty > 0 AND speed ~= 0), not a single
 *                  threshold crossing
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#include "channels.h"
#include <stdint.h>
#include <stdbool.h>

#define FAULT_HISTORY_DEPTH   8
#define LOCKOUT_TRIP_COUNT    3      /* same fault kind, this many times... */
#define LOCKOUT_WINDOW_TICKS  (120000)  /* ...within this many ticks (10 min at 5ms/tick) escalates to LOCKOUT */

#define STALL_DUTY_THRESHOLD  Q16_FROM_PCT(15)   /* below this, "not turning" is just idle, not a stall */
#define STALL_SPEED_THRESHOLD Q16_FROM_PCT(3)
#define STALL_CONFIRM_TICKS   40                 /* 200ms at 5ms/tick before a stall reading counts as real, matches the same confidence-counter philosophy as the channel table */

typedef struct {
    uint32_t     tick_timestamp;
    fault_kind_t kind;
    channel_id_t channel;         /* only meaningful for FAULT_TRIP / FAULT_SENSOR */
    q16_t        snapshot[CH_COUNT];
    int64_t      i2t_value;
    const char  *description;
} fault_event_t;

void protection_init(void);

/* call once per control tick. commanded_duty is needed only for the
 * stall correlation check. writes the event out and returns true if
 * something newly tripped this tick (prealarms are reported separately,
 * see protection_prealarm_active) */
bool protection_update(q16_t commanded_duty, uint32_t tick, fault_event_t *out_event);

bool protection_prealarm_active(void);

/* fault history / lockout */
bool protection_should_lockout(fault_kind_t kind, channel_id_t channel, uint32_t tick);
void protection_record_trip(fault_kind_t kind, channel_id_t channel, uint32_t tick);
void protection_clear_channel_latches(void); /* called on recovery approval, not on every tick */

#endif /* PROTECTION_H */
