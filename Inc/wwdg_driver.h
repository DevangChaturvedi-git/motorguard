/*
 * wwdg_driver.h
 *
 * IWDG isn't implemented in Wokwi's Blue Pill simulation (confirmed
 * against Wokwi's own docs before committing to this - see the
 * project notes). WWDG is, and it's arguably not a downgrade: WWDG
 * only accepts a refresh inside a specific window - not too early, not
 * too late - so it catches a genuinely hung loop AND a corrupted or
 * runaway loop executing faster than expected, which IWDG can't do on
 * its own.
 *
 * Configured for a ~29ms timeout with a valid refresh window
 * opening around 21ms after reload, at APB1=36MHz. The control loop
 * refreshes it once every 5 ticks (25ms at the fixed 5ms tick), which
 * lands inside that window - see main.c. Timing worth re-checking
 * against a scope/logic analyzer once this is actually flashed; the
 * math here is computed from the RM0008 formula, not bench-verified
 * on real silicon.
 */

#ifndef WWDG_DRIVER_H
#define WWDG_DRIVER_H

void wwdg_driver_init(void);
void wwdg_refresh(void);

#endif /* WWDG_DRIVER_H */
