#include "control.h"

/* Layer 2's own plausibility check on the speed channel needs several
 * samples before it'll call something a sensor fault - correctly so,
 * that's what keeps it from false-tripping on ordinary noise. But that
 * leaves a short window where a single glitchy reading could still hit
 * the PI math before Layer 2 has made up its mind. This isn't trying
 * to detect a bad sensor - it's just refusing to let one physically
 * impossible sample fling the control loop around while the slower,
 * more careful check downstream is still deciding whether it's real. */
#define MAX_FEEDBACK_STEP_Q16   Q16_FROM_PCT(25)

void control_init(control_state_t *s, q16_t kp, q16_t ki)
{
    s->kp = kp;
    s->ki = ki;
    s->integral = 0;
    s->prev_feedback = 0;
    s->have_prev_feedback = false;
}

static q16_t sanity_clamp_feedback(control_state_t *s, q16_t raw)
{
    if (!s->have_prev_feedback) {
        s->have_prev_feedback = true;
        s->prev_feedback = raw;
        return raw;
    }
    q16_t lo = s->prev_feedback - MAX_FEEDBACK_STEP_Q16;
    q16_t hi = s->prev_feedback + MAX_FEEDBACK_STEP_Q16;
    q16_t clamped = q_clamp(raw, lo, hi);
    s->prev_feedback = clamped;
    return clamped;
}

q16_t control_tick(control_state_t *s, q16_t setpoint, q16_t feedback_raw,
                    q16_t feedforward_duty, bool saturated)
{
    q16_t feedback = sanity_clamp_feedback(s, feedback_raw);
    q16_t error = setpoint - feedback;

    /* conditional integration - do not accumulate while the last
     * output was saturated for any reason. this is the general rule
     * the fault-latch freeze and the Layer 3 clamp freeze are both
     * special cases of - if we didn't do this, the integral term would
     * keep winding up the whole time it's held back, then dump all of
     * that pent-up correction the instant it's finally free */
    if (!saturated) {
        s->integral += error;
    }

    q16_t p_term = q_mul(s->kp, error);
    q16_t i_term = q_mul(s->ki, s->integral);

    q16_t output = feedforward_duty + p_term + i_term;
    return q_clamp(output, 0, Q16_ONE);
}
