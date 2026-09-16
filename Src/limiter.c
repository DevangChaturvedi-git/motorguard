#include "limiter.h"
#include "channels.h"
#include "plant_model.h"

limiter_result_t limiter_apply(q16_t requested_duty)
{
    channel_t *current_ch = channels_get(CH_CURRENT);
    q16_t ceiling = plant_model_max_continuous_duty(current_ch->continuous_limit);

    limiter_result_t r;
    if (requested_duty > ceiling) {
        r.clamped_duty = ceiling;
        r.was_clamped = true;
    } else {
        r.clamped_duty = requested_duty;
        r.was_clamped = false;
    }
    return r;
}
