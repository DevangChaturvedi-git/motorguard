/*
 * supervisor.h
 *
 * The orchestrator. Owns reading the pots/buttons, owns the state
 * machine, and calls into control.c/limiter.c/protection.c/plant_model.c
 * in the right order each tick. Nothing else in the project decides
 * what state the system is in - everything else just answers
 * questions ("is this safe", "what's the model's current reading")
 * when the supervisor asks.
 *
 *   POST -> IDLE -> [AWAITING_SETPOINT_APPROVAL ->] RAMPING -> RUNNING
 *                                                       |
 *                                                       v
 *                                                     FAULT -> RECOVERY_PENDING -> RAMPING (soft re-arm, never a snap-back)
 *                                                       |
 *                                                       v (repeated trips)
 *                                                    LOCKOUT
 *
 * Boot-time behaviour: if flash shows the last saved state was FAULT,
 * RECOVERY_PENDING, LOCKOUT, or SENSOR_FAULT, the supervisor comes up
 * latched in that same state rather than IDLE - a power cycle is not
 * an approval.
 */

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>

typedef enum {
    SUP_BOOT = 0,
    SUP_POST,
    SUP_IDLE,
    SUP_AWAITING_SETPOINT_APPROVAL,
    SUP_RAMPING,
    SUP_RUNNING,
    SUP_FAULT,
    SUP_RECOVERY_PENDING,
    SUP_LOCKOUT,
    SUP_SENSOR_FAULT,
} supervisor_state_t;

void supervisor_init(void);
void supervisor_tick(uint32_t tick);

supervisor_state_t supervisor_get_state(void);

#endif /* SUPERVISOR_H */
