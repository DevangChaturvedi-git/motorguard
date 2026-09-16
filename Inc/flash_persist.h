/*
 * flash_persist.h
 *
 * The whole point of this file: a power cycle must never look like an
 * approval. If the supervisor was in FAULT, RECOVERY_PENDING, or
 * LOCKOUT when power was lost, it needs to come back up in that exact
 * state, not a fresh IDLE - otherwise unplugging and replugging the
 * device would silently defeat the entire human-approval requirement.
 *
 * Deliberately simple compared to AURA's wear-leveled EEPROM
 * telemetry (RAM-accumulation + periodic checkpoint, ~50x fewer
 * writes): this erases and rewrites the whole page on every state
 * transition, which is fine for a single evening's demo but would
 * chew through flash endurance in real continuous service. A real
 * deployment should apply AURA's checkpoint strategy here instead -
 * noted as a real limitation, not silently glossed over.
 *
 * Lives in the last 1KB page of flash, kept out of the linker's FLASH
 * region on purpose (see the .ld file) so normal code/data can never
 * accidentally land on top of it.
 */

#ifndef FLASH_PERSIST_H
#define FLASH_PERSIST_H

#include <stdint.h>
#include <stdbool.h>
#include "fixed_point.h"

typedef struct {
    uint32_t magic;          /* distinguishes "real saved state" from erased/blank flash (reads as 0xFFFFFFFF) */
    uint32_t state;          /* supervisor_state_t, stored as plain uint32 so this header doesn't need to depend on supervisor.h */
    q16_t    setpoint;
    uint32_t fault_kind;
    uint32_t fault_channel;
} persisted_state_t;

#define FLASH_PERSIST_MAGIC   0x4D475032u  /* "MGP2" - MotorGuard Persist v2, bumped if the struct layout ever changes */

void flash_persist_init(void);

/* returns false if nothing valid has ever been saved (freshly erased
 * part) - caller should fall back to defaults in that case, not treat
 * it as an error */
bool flash_persist_read(persisted_state_t *out);
void flash_persist_write(const persisted_state_t *in);

#endif /* FLASH_PERSIST_H */
