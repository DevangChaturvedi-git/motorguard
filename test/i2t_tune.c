/* Native (x86) test harness - NOT part of the firmware build, this
 * never ships to the STM32. Only exists to check the I2t tuning and
 * plant model behaviour against real numbers before committing the
 * constants in channels.c, since guessing them would be worse than
 * actually running the math. */
#include <stdio.h>
#include "../Inc/fixed_point.h"
#include "../Inc/channels.h"
#include "../Inc/protection.h"
#include "../Inc/plant_model.h"

static void run_scenario(const char *name, q16_t duty, bool stall, bool overload, int max_ticks)
{
    printf("\n=== %s ===\n", name);
    plant_model_init();
    channels_init();
    protection_init();
    plant_model_inject_stall(stall);
    plant_model_inject_overload(overload);

    for (int t = 0; t < max_ticks; t++) {
        plant_model_step(duty, 0);
        fault_event_t ev;
        bool tripped = protection_update(duty, (uint32_t)t, &ev);

        if (t % 20 == 0 || tripped) {
            printf("tick=%4d speed=%.3f current=%.3f temp=%.3f i2t[I]=%lld i2t[T]=%lld prealarm=%d\n",
                t,
                Q16_TO_INT(plant_model_read_speed()*1000)/1000.0,
                (double)plant_model_read_current() / Q16_ONE,
                (double)plant_model_read_temperature() / Q16_ONE,
                (long long)g_channels[CH_CURRENT].i2t_accumulator,
                (long long)g_channels[CH_TEMPERATURE].i2t_accumulator,
                protection_prealarm_active());
        }
        if (tripped) {
            printf(">>> TRIPPED at tick=%d kind=%d channel=%d desc=%s\n", t, ev.kind, ev.channel, ev.description);
            return;
        }
    }
    printf("did not trip within %d ticks\n", max_ticks);
}

int main(void)
{
    /* 1: normal running at 60% duty, no faults - should NEVER trip, and current should settle comfortably under the continuous limit */
    run_scenario("normal run 60% duty", Q16_FROM_PCT(60), false, false, 400);

    /* 2: normal running at 100% duty, no faults - current should settle low too since back-EMF cancels it out at full speed */
    run_scenario("normal run 100% duty", Q16_FROM_PCT(100), false, false, 400);

    /* 3: genuine stall at 100% duty - should trip via I2t on CURRENT within a demo-friendly timescale, and via FAULT_STALL correlation check */
    run_scenario("stall at 100% duty", Q16_FROM_PCT(100), true, false, 600);

    /* 4: injected overload (not a full stall) at 60% duty - should trip fast via absolute limit since the injected current is large */
    run_scenario("overload inject at 60% duty", Q16_FROM_PCT(60), false, true, 600);

    /* 5: brief graze - duty stepped to 100% just for a moment then back to 50%, should NOT trip, accumulator should decay back down */
    printf("\n=== brief graze, then back to normal ===\n");
    plant_model_init(); channels_init(); protection_init();
    for (int t = 0; t < 600; t++) {
        q16_t duty = (t < 30) ? Q16_FROM_PCT(100) : Q16_FROM_PCT(50);
        plant_model_step(duty, 0);
        fault_event_t ev;
        bool tripped = protection_update(duty, (uint32_t)t, &ev);
        if (t % 40 == 0 || tripped)
            printf("tick=%4d current=%.3f i2t[I]=%lld tripped=%d\n", t, (double)plant_model_read_current()/Q16_ONE, (long long)g_channels[CH_CURRENT].i2t_accumulator, tripped);
        if (tripped) { printf(">>> unexpected trip on a brief graze! bad.\n"); break; }
    }

    /* 6: heavy but non-stalling load - speed stays above the stall
     * threshold (so the dedicated stall detector never fires) but
     * current stays moderately elevated for a long time, never
     * touching the absolute limit either. This is the scenario I2t
     * actually exists for - the other two paths shouldn't be able to
     * catch this one. */
    printf("\n=== heavy sustained load, not a stall, not absolute ===\n");
    plant_model_init(); channels_init(); protection_init();
    for (int t = 0; t < 1200; t++) {
        plant_model_step(Q16_FROM_PCT(100), Q16_FROM_PCT(70)); /* heavy constant load */
        fault_event_t ev;
        bool tripped = protection_update(Q16_FROM_PCT(100), (uint32_t)t, &ev);
        if (t % 60 == 0 || tripped)
            printf("tick=%4d speed=%.3f current=%.3f i2t[I]=%lld tripped=%d\n",
                t, (double)plant_model_read_speed()/Q16_ONE, (double)plant_model_read_current()/Q16_ONE,
                (long long)g_channels[CH_CURRENT].i2t_accumulator, tripped);
        if (tripped) { printf(">>> trip via kind=%d channel=%d (%s)\n", ev.kind, ev.channel, ev.description); break; }
    }

    return 0;
}
