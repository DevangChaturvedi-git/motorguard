#include "wwdg_driver.h"
#include "stm32f103xb.h"

/* Original config (T=0x7F, W=0x50, /4 prescaler -> ~29ms timeout,
 * window opening ~21ms in) caused a genuine reset loop in testing:
 * the chip rebooted from scratch before ever reaching its first
 * refresh, over and over, which looked like total silence in the
 * terminal (each reboot reprints the boot message, then dies again
 * before anything else) and caused visible lag from the resulting
 * flood. The 29ms budget assumed roughly real-silicon timing for
 * clock bring-up + all the driver inits + the boot UART writes before
 * the main loop's first refresh - evidently too tight for however
 * this simulator actually paces execution.
 *
 * Widened significantly for reliability: /8 prescaler roughly doubles
 * the max timeout to ~58ms, and the window is set equal to the reload
 * value so a refresh is accepted at any time rather than only within
 * a late sub-window - i.e. the "catches a runaway-too-fast loop"
 * property from the original design is intentionally given up here in
 * exchange for not spuriously resetting under unknown simulator
 * timing. A real deployment should re-derive a tighter window after
 * bench/scope-verifying actual loop timing on real silicon, not leave
 * it this permissive. */
#define WWDG_RELOAD   0x7Fu
#define WWDG_WINDOW   0x7Fu   /* == reload: refresh accepted immediately, no early-refresh reset risk */
#define WWDG_WDGTB_DIV8   (0x3u << 7)  /* WDGTB[1:0] = 11 -> /8 */

void wwdg_driver_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

    WWDG->CFR = WWDG_WINDOW | WWDG_WDGTB_DIV8;
    WWDG->CR  = WWDG_RELOAD | WWDG_CR_WDGA;
}

void wwdg_refresh(void)
{
    WWDG->CR = WWDG_RELOAD | WWDG_CR_WDGA;
}
