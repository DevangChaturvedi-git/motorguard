#include "channels.h"
#include "plant_model.h"

channel_t g_channels[CH_COUNT];

/* I^2t ceilings and decay rates were picked so that a severe excess
 * (stall-level current, ~2x continuous limit) trips within roughly
 * half a second of ticks, while something briefly grazing the limit
 * during a normal setpoint change self-clears instead of accumulating
 * toward a trip - same shape as the classic inverse-time curve, just
 * tuned for this demo's timescale rather than a real motor's thermal
 * time constant. */

void channels_init(void)
{
    g_channels[CH_CURRENT] = (channel_t){
        .id = CH_CURRENT,
        .name = "CURRENT",
        .continuous_limit = Q16_FROM_PCT(115),
        .absolute_limit   = Q16_FROM_PCT(220),
        .prealarm_fraction = Q16_FROM_PCT(80),
        .hysteresis_band  = Q16_FROM_PCT(5),
        .confidence_required = 3,
        .i2t_accumulator = 0,
        .i2t_trip_ceiling = 4000000, /* tuned against the plant model, see protection.c comments */
        .i2t_decay_per_tick = 3000,
        .plausibility_min = Q16_FROM_PCT(-5),  /* small negative slack, real ADC noise dips slightly below 0 */
        .plausibility_max = Q16_FROM_PCT(300),
        .implausible_confidence_required = 10,
    };

    g_channels[CH_TEMPERATURE] = (channel_t){
        .id = CH_TEMPERATURE,
        .name = "TEMP",
        .continuous_limit = Q16_FROM_PCT(100),
        .absolute_limit   = Q16_FROM_PCT(180),
        .prealarm_fraction = Q16_FROM_PCT(80),
        .hysteresis_band  = Q16_FROM_PCT(5),
        .confidence_required = 5,   /* temperature is slow and shouldn't be noisy - a longer confidence window is fine here and avoids nuisance trips */
        .i2t_accumulator = 0,
        .i2t_trip_ceiling = 6000000,
        .i2t_decay_per_tick = 2000,
        .plausibility_min = Q16_FROM_PCT(-5),
        .plausibility_max = Q16_FROM_PCT(500),
        .implausible_confidence_required = 10,
    };

    g_channels[CH_SPEED] = (channel_t){
        .id = CH_SPEED,
        .name = "SPEED",
        .continuous_limit = Q16_FROM_PCT(105),   /* overspeed check, not the PI feedback path */
        .absolute_limit   = Q16_FROM_PCT(140),
        .prealarm_fraction = Q16_FROM_PCT(90),
        .hysteresis_band  = Q16_FROM_PCT(3),
        .confidence_required = 3,
        .i2t_accumulator = 0,
        .i2t_trip_ceiling = 3000000,
        .i2t_decay_per_tick = 4000,
        .plausibility_min = 0,
        .plausibility_max = Q16_FROM_PCT(200),
        .implausible_confidence_required = 10,
    };
}

channel_t *channels_get(channel_id_t id)
{
    if (id >= CH_COUNT) return 0;
    return &g_channels[id];
}

q16_t channel_read_value(channel_id_t id)
{
    /* plain switch-dispatch, direct calls only - see the note in
     * channels.h for why this isn't a function pointer table */
    switch (id) {
    case CH_CURRENT:     return plant_model_read_current();
    case CH_TEMPERATURE: return plant_model_read_temperature();
    case CH_SPEED:        return plant_model_read_speed();
    default:               return 0;
    }
}
