#ifndef DRIVERS_H
#define DRIVERS_H

#include <stdint.h>

/* HSE(8MHz) * 9 = 72MHz via PLL. AHB=72MHz, APB2=72MHz, APB1=36MHz
 * (its hard ceiling on this part). 2 flash wait states required above
 * 48MHz - see RM0008 table on flash latency vs SYSCLK. */
void clock_init(void);

/* DWT->CYCCNT, used to actually measure fault-path latency instead of
 * just asserting it's fast. Needs DEMCR.TRCENA set first or the DWT
 * block is unpowered and every register read silently returns 0. */
void dwt_init(void);
static inline uint32_t dwt_now(void);

/* stm32f103xb.h defines IRQn_Type and __NVIC_PRIO_BITS before
 * including core_cm3.h itself in the right order - including
 * core_cm3.h directly here without that would break, so pull in the
 * device header instead and let it do the ordering */
#include "stm32f103xb.h"
static inline uint32_t dwt_now(void) { return DWT->CYCCNT; }

#endif /* DRIVERS_H */
