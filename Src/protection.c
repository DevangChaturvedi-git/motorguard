#include "protection.h"
#include <string.h>

/* raw excess^2 gets right-shifted by this before accumulating - brings
 * a ~100% overcurrent event down from a ~4.3e9 single-tick number to
 * something that takes on the order of a few hundred ticks to reach a
 * ~4e6 ceiling. See the native test harness (test/i2t_tune.c) for the
 * numbers this was actually checked against rather than guessed. */
#define I2T_SHIFT             18
#define ABSOLUTE_CONFIRM_TICKS 3

static uint8_t  s_absolute_over_count[CH_COUNT];
static bool     s_prealarm_active;

static uint32_t s_stall_confirm_count;

static struct {
    bool         used;
    fault_kind_t kind;
    channel_id_t channel;
    uint32_t     tick;
} s_history[FAULT_HISTORY_DEPTH];
static uint8_t s_history_next;

void protection_init(void)
{
    memset(s_absolute_over_count, 0, sizeof(s_absolute_over_count));
    s_prealarm_active = false;
    s_stall_confirm_count = 0;
    memset(s_history, 0, sizeof(s_history));
    s_history_next = 0;
}

static bool check_plausibility(channel_t *ch, q16_t value, fault_event_t *out)
{
    bool ok = (value >= ch->plausibility_min) && (value <= ch->plausibility_max);
    if (ok) {
        ch->implausible_count = 0;
        return false;
    }
    if (ch->implausible_count < 255) ch->implausible_count++;
    if (ch->implausible_count < ch->implausible_confidence_required) {
        return false; /* not yet confirmed - could still be a one-off ADC glitch */
    }
    out->kind = FAULT_SENSOR;
    out->channel = ch->id;
    out->description = "implausible reading, sensor suspect";
    return true;
}

static bool check_absolute(channel_t *ch, q16_t value, fault_event_t *out)
{
    if (value <= ch->absolute_limit) {
        s_absolute_over_count[ch->id] = 0;
        return false;
    }
    if (s_absolute_over_count[ch->id] < 255) s_absolute_over_count[ch->id]++;
    if (s_absolute_over_count[ch->id] < ABSOLUTE_CONFIRM_TICKS) {
        return false;
    }
    out->kind = FAULT_TRIP;
    out->channel = ch->id;
    out->description = "absolute limit exceeded (software backstop)";
    return true;
}

/* the I^2t stage - returns true if this channel just crossed its trip
 * ceiling. Sets *prealarm if it's past the warn line but hasn't
 * tripped yet. */
static bool check_i2t(channel_t *ch, q16_t value, bool *prealarm)
{
    *prealarm = false;

    /* hysteresis around continuous_limit so a reading sitting right on
     * the boundary doesn't flicker the accumulator direction every
     * tick - once accumulating, stay accumulating until it drops
     * clearly below the limit, and vice versa */
    q16_t upper = ch->continuous_limit + ch->hysteresis_band;
    q16_t lower = ch->continuous_limit - ch->hysteresis_band;

    if (value > upper) {
        ch->i2t_was_accumulating = true;
    } else if (value < lower) {
        ch->i2t_was_accumulating = false;
    }
    /* else: leave i2t_was_accumulating exactly as it was */

    if (ch->i2t_was_accumulating) {
        q16_t excess = value - ch->continuous_limit;
        if (excess < 0) excess = 0; /* possible inside the hysteresis band */
        int64_t excess_sq = (int64_t)excess * (int64_t)excess;
        ch->i2t_accumulator += (excess_sq >> I2T_SHIFT);
    } else {
        ch->i2t_accumulator -= ch->i2t_decay_per_tick;
        if (ch->i2t_accumulator < 0) ch->i2t_accumulator = 0;
    }

    int64_t warn_line = (ch->i2t_trip_ceiling * (int64_t)ch->prealarm_fraction) >> Q16_SHIFT;
    /* A warning only has meaning while this channel is actively carrying an
     * overload. This also prevents an idle Wokwi run from showing a stale
     * pre-alarm indication. */
    if (ch->i2t_was_accumulating && ch->i2t_accumulator >= warn_line) {
        *prealarm = true;
    }

    return ch->i2t_accumulator >= ch->i2t_trip_ceiling;
}

static bool check_stall(q16_t commanded_duty, q16_t speed, fault_event_t *out)
{
    bool commanding_run = commanded_duty > STALL_DUTY_THRESHOLD;
    bool not_turning     = speed < STALL_SPEED_THRESHOLD;

    if (!(commanding_run && not_turning)) {
        s_stall_confirm_count = 0;
        return false;
    }

    s_stall_confirm_count++;
    if (s_stall_confirm_count < STALL_CONFIRM_TICKS) {
        return false;
    }

    out->kind = FAULT_STALL;
    out->channel = CH_SPEED;
    out->description = "commanded to run, not turning";
    return true;
}

bool protection_update(q16_t commanded_duty, uint32_t tick, fault_event_t *out_event)
{
    s_prealarm_active = false;
    bool any_prealarm = false;

    for (int i = 0; i < CH_COUNT; i++) {
        channel_t *ch = &g_channels[i];
        q16_t value = channel_read_value(ch->id);

        fault_event_t candidate;
        if (check_plausibility(ch, value, &candidate)) {
            candidate.tick_timestamp = tick;
            for (int j = 0; j < CH_COUNT; j++) candidate.snapshot[j] = channel_read_value((channel_id_t)j);
            candidate.i2t_value = ch->i2t_accumulator;
            *out_event = candidate;
            return true;
        }

        if (check_absolute(ch, value, &candidate)) {
            candidate.tick_timestamp = tick;
            for (int j = 0; j < CH_COUNT; j++) candidate.snapshot[j] = channel_read_value((channel_id_t)j);
            candidate.i2t_value = ch->i2t_accumulator;
            *out_event = candidate;
            return true;
        }

        bool prealarm = false;
        bool tripped = check_i2t(ch, value, &prealarm);
        if (prealarm) any_prealarm = true;

        if (tripped) {
            out_event->tick_timestamp = tick;
            out_event->kind = FAULT_TRIP;
            out_event->channel = ch->id;
            out_event->description = "I2t ceiling exceeded (sustained overload)";
            for (int j = 0; j < CH_COUNT; j++) out_event->snapshot[j] = channel_read_value((channel_id_t)j);
            out_event->i2t_value = ch->i2t_accumulator;
            return true;
        }
    }

    fault_event_t stall_candidate;
    if (check_stall(commanded_duty, channel_read_value(CH_SPEED), &stall_candidate)) {
        stall_candidate.tick_timestamp = tick;
        for (int j = 0; j < CH_COUNT; j++) stall_candidate.snapshot[j] = channel_read_value((channel_id_t)j);
        stall_candidate.i2t_value = 0;
        *out_event = stall_candidate;
        return true;
    }

    s_prealarm_active = any_prealarm;
    return false;
}

bool protection_prealarm_active(void)
{
    return s_prealarm_active;
}

bool protection_should_lockout(fault_kind_t kind, channel_id_t channel, uint32_t tick)
{
    uint8_t count = 0;
    for (int i = 0; i < FAULT_HISTORY_DEPTH; i++) {
        if (!s_history[i].used) continue;
        if (s_history[i].kind != kind || s_history[i].channel != channel) continue;
        if ((tick - s_history[i].tick) > LOCKOUT_WINDOW_TICKS) continue;
        count++;
    }
    return count >= (LOCKOUT_TRIP_COUNT - 1); /* -1: the trip about to be recorded counts as the Nth */
}

void protection_record_trip(fault_kind_t kind, channel_id_t channel, uint32_t tick)
{
    s_history[s_history_next].used = true;
    s_history[s_history_next].kind = kind;
    s_history[s_history_next].channel = channel;
    s_history[s_history_next].tick = tick;
    s_history_next = (s_history_next + 1) % FAULT_HISTORY_DEPTH;
}

void protection_clear_channel_latches(void)
{
    for (int i = 0; i < CH_COUNT; i++) {
        g_channels[i].i2t_accumulator = 0;
        g_channels[i].i2t_was_accumulating = false;
        g_channels[i].over_threshold_count = 0;
        g_channels[i].implausible_count = 0;
    }
    s_stall_confirm_count = 0;
    for (int i = 0; i < CH_COUNT; i++) s_absolute_over_count[i] = 0;
}
