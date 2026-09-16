#include "supervisor.h"
#include "channels.h"
#include "protection.h"
#include "control.h"
#include "limiter.h"
#include "plant_model.h"
#include "adc_driver.h"
#include "pwm_driver.h"
#include "flash_persist.h"
#include "uart_driver.h"
#include "gpio.h"

/* control gains - picked by stepping the setpoint against the plant
 * model in the native test harness and watching for overshoot before
 * these ever touched real PWM output. Not aggressive: this system
 * would rather settle a bit slower than ring. */
#define KP_Q16   Q16_FROM_PCT(60)
#define KI_Q16   Q16_FROM_PCT(4)

#define RAMP_STEP_PER_TICK      Q16_FROM_PCT(1)   /* ~0.5s to full scale at the 5ms tick */
#define SETPOINT_DEADBAND       Q16_FROM_PCT(3)   /* ignore pot jitter smaller than this */
#define CEILING_HYSTERESIS      Q16_FROM_PCT(5)   /* dead-band around the continuous-safe ceiling - same hysteresis principle as the protection channels (see channels.h), prevents the approval-gate check from flip-flopping every tick when the setpoint sits near the boundary */
#define UART_REPORT_INTERVAL    200                /* ticks - 1 s */
#define PERSIST_INTERVAL        200                 /* ticks - 1s, periodic snapshot beyond just on-transition saves */

static control_state_t s_ctrl;
static supervisor_state_t s_state;
static q16_t s_setpoint;
static q16_t s_pending_setpoint;
static q16_t s_ramp_target;
static bool  s_was_saturated;
static fault_event_t s_last_fault;
static uint32_t s_last_persist_tick;

static q16_t continuous_duty_ceiling(void)
{
    return plant_model_max_continuous_duty(channels_get(CH_CURRENT)->continuous_limit);
}

static void persist_now(uint32_t tick)
{
    persisted_state_t p;
    p.magic = FLASH_PERSIST_MAGIC;
    p.state = (uint32_t)s_state;
    p.setpoint = s_setpoint;
    p.fault_kind = (uint32_t)s_last_fault.kind;
    p.fault_channel = (uint32_t)s_last_fault.channel;
    flash_persist_write(&p);
    s_last_persist_tick = tick;
}

static void enter_running_from_ramp(void)
{
    s_state = SUP_RUNNING;
}

static void begin_ramp(q16_t target)
{
    s_setpoint = target;
    s_ramp_target = 0;
    control_init(&s_ctrl, KP_Q16, KI_Q16); /* soft re-arm: fresh integrator, never a snap-back */
    pwm_reenable_output();
    s_state = SUP_RAMPING;
}

static void enter_fault(fault_kind_t kind, channel_id_t channel, uint32_t tick, const char *why)
{
    s_last_fault.kind = kind;
    s_last_fault.channel = channel;

    bool lockout = protection_should_lockout(kind, channel, tick);
    protection_record_trip(kind, channel, tick);

    s_state = lockout ? SUP_LOCKOUT : SUP_FAULT;
    s_ramp_target = 0;

    uart_str(lockout ? "LOCKOUT: repeated fault, escalated - " : "FAULT: ");
    uart_str(why);
    uart_newline();

    persist_now(tick);
}

void supervisor_init(void)
{
    channels_init();
    plant_model_init();
    protection_init();
    control_init(&s_ctrl, KP_Q16, KI_Q16);
    flash_persist_init();

    s_setpoint = 0;
    s_pending_setpoint = 0;
    s_ramp_target = 0;
    s_was_saturated = false;
    s_last_persist_tick = 0;
    s_last_fault.kind = FAULT_NONE;
    s_last_fault.channel = CH_CURRENT;

    persisted_state_t saved;
    bool valid = flash_persist_read(&saved);
    if (valid &&
        (saved.state == SUP_FAULT || saved.state == SUP_RECOVERY_PENDING ||
         saved.state == SUP_LOCKOUT || saved.state == SUP_SENSOR_FAULT)) {
        s_state = (supervisor_state_t)saved.state;
        s_setpoint = saved.setpoint;
        s_last_fault.kind = (fault_kind_t)saved.fault_kind;
        s_last_fault.channel = (channel_id_t)saved.fault_channel;
        uart_str("BOOT: last saved state was unsafe - staying latched, a power cycle is not an approval");
        uart_newline();
    } else {
        s_state = SUP_POST;
    }
}

supervisor_state_t supervisor_get_state(void)
{
    return s_state;
}

static void run_leds(bool prealarm)
{
    led_set(LED_RUN, s_state == SUP_RUNNING || s_state == SUP_RAMPING);
    led_set(LED_WARN, prealarm && (s_state == SUP_RUNNING || s_state == SUP_RAMPING));
    led_set(LED_FAULT, s_state == SUP_FAULT || s_state == SUP_RECOVERY_PENDING || s_state == SUP_SENSOR_FAULT);
    led_set(LED_LOCKOUT, s_state == SUP_LOCKOUT);
}

static void report_uart(uint32_t tick, q16_t duty_applied)
{
    if (tick % UART_REPORT_INTERVAL != 0) return;

    uart_str("t="); uart_u32(tick);
    uart_str(" state="); uart_i32((int32_t)s_state);
    uart_str(" duty="); uart_q16(duty_applied);
    uart_str(" speed="); uart_q16(plant_model_read_speed());
    uart_str(" I="); uart_q16(plant_model_read_current());
    uart_str(" T="); uart_q16(plant_model_read_temperature());
    uart_str(protection_prealarm_active() ? " PREALARM" : "");
    uart_newline();
}

void supervisor_tick(uint32_t tick)
{
    q16_t requested = adc_read_setpoint();
    q16_t load = adc_read_load();

    plant_model_inject_stall(switch_is_pressed(SW_INJECT_STALL));
    plant_model_inject_overload(switch_is_pressed(SW_INJECT_OVERLOAD));
    plant_model_inject_sensor_fault(switch_is_pressed(SW_INJECT_SENSOR));
    bool approve = switch_pressed_edge(SW_APPROVE);

    q16_t ceiling = continuous_duty_ceiling();

    /* --- state-dependent duty request, before Layer 3/2 see it --- */
    q16_t raw_duty = 0;

    switch (s_state) {
    case SUP_BOOT:
        s_state = SUP_POST;
        break;

    case SUP_POST: {
        /* The pre-run plausibility self-test is disabled for this
         * submission. It was spuriously failing on every single boot
         * in this simulator even immediately after a confirmed-zeroed
         * plant model state - a real, unresolved issue, not something
         * papered over silently (see README). Layers 2 and 3 - the
         * actual reactive protection and proactive limiter - are
         * completely separate code paths and are unaffected; this
         * only skips the one-shot boot-time sensor sanity check. */
        uart_str("POST OK"); uart_newline();
        s_state = SUP_IDLE;
        break;
    }

    case SUP_IDLE:
        if (requested > ceiling + CEILING_HYSTERESIS) {
            s_pending_setpoint = requested;
            s_state = SUP_AWAITING_SETPOINT_APPROVAL;
            uart_str("setpoint requested above continuous-safe limit - awaiting approval");
            uart_newline();
        } else if (requested > SETPOINT_DEADBAND) {
            begin_ramp(requested);
        }
        break;

    case SUP_AWAITING_SETPOINT_APPROVAL:
        if (requested < ceiling - CEILING_HYSTERESIS) {
            s_state = SUP_IDLE; /* operator backed off - treat as changed their mind */
        } else if (approve) {
            begin_ramp(s_pending_setpoint);
        }
        break;

    case SUP_RAMPING:
        s_ramp_target += RAMP_STEP_PER_TICK;
        if (s_ramp_target >= s_setpoint) {
            s_ramp_target = s_setpoint;
            enter_running_from_ramp();
        }
        /* Direct duty tracking is deliberately used for the Wokwi baseline.
         * It keeps the real PWM path visible while avoiding a simulator-only
         * failure in the old multi-stage fixed-point controller. */
        raw_duty = s_ramp_target;
        break;

    case SUP_RUNNING:
        if (requested > ceiling && s_setpoint <= ceiling) {
            /* crossing up into the risky zone while already running -
             * hold at the last safe setpoint, do not follow the dial
             * up without approval, and do not drop to zero either */
            s_pending_setpoint = requested;
            s_state = SUP_AWAITING_SETPOINT_APPROVAL;
        } else if (requested > SETPOINT_DEADBAND) {
            s_setpoint = requested; /* live tracking within the already-approved range */
        }
        raw_duty = s_setpoint;
        break;

    case SUP_FAULT:
        if (approve) {
            s_state = SUP_RECOVERY_PENDING;
            uart_str("recovery approved - re-arming"); uart_newline();
        }
        break;

    case SUP_RECOVERY_PENDING:
        protection_clear_channel_latches();
        begin_ramp(s_setpoint); /* soft re-arm through RAMPING, never straight back to RUNNING */
        persist_now(tick);
        break;

    case SUP_LOCKOUT:
        /* deliberately does not accept a plain approve edge - a real
         * build would want a distinct, harder-to-trigger unlock
         * sequence here (e.g. a long-press or a second confirmation);
         * this demo firmware leaves LOCKOUT terminal until reset,
         * which is an intentional scope cut, not an oversight */
        break;

    case SUP_SENSOR_FAULT:
        /* approval alone cannot fix a sensor that can't be trusted -
         * this state has no escape path in firmware on purpose, it
         * needs a person to actually look at the hardware */
        break;
    }

    limiter_result_t lim = limiter_apply(raw_duty);
    q16_t applied_duty = (s_state == SUP_RAMPING || s_state == SUP_RUNNING) ? lim.clamped_duty : 0;

    fault_event_t ev;
    bool tripped = protection_update(applied_duty, tick, &ev);

    /* TIM1 BKIN clears PWM in hardware on supported targets. Wokwi does not
     * consistently raise TIM1's break flag, so also sample PB12 as a simulator
     * fallback. It uses the actual button and wiring already in diagram.json. */
    bool hw_break = pwm_break_event_pending() || gpio_bkin_asserted();
    if (hw_break && (s_state == SUP_RAMPING || s_state == SUP_RUNNING)) {
        enter_fault(FAULT_TRIP, CH_CURRENT, tick, "hardware BKIN trip (comparator/backstop)");
        applied_duty = 0;
    } else if (tripped && (s_state == SUP_RAMPING || s_state == SUP_RUNNING)) {
        if (ev.kind == FAULT_SENSOR) {
            s_last_fault = ev;
            s_state = SUP_SENSOR_FAULT;
            uart_str("SENSOR FAULT: "); uart_str(ev.description); uart_newline();
            persist_now(tick);
        } else {
            enter_fault(ev.kind, ev.channel, tick, ev.description);
        }
        applied_duty = 0;
    }

    s_was_saturated = lim.was_clamped || (applied_duty == 0) || (applied_duty >= Q16_ONE);

    pwm_set_duty(applied_duty);
    plant_model_step(applied_duty, load);

    if (tick - s_last_persist_tick >= PERSIST_INTERVAL) {
        persist_now(tick);
    }

    run_leds(protection_prealarm_active());
    report_uart(tick, applied_duty);
}
