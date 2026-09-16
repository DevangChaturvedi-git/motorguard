#include "drivers.h"
#include "stm32f103xb.h"

extern uint32_t SystemCoreClock;

void clock_init(void)
{
    /* HSE on, wait for it - if this hangs, there's no crystal (or
     * Wokwi hasn't modelled HSE and expects HSI - worth checking first
     * if this driver ever gets stuck at boot) */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }

    /* flash needs 2 wait states above 48MHz, and the prefetch buffer
     * should be on before we're actually running at 72MHz, not after -
     * RM0008 9.3.3 flags a specific latency table not just "more is
     * always safe" */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1;
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* AHB = SYSCLK (no divide), APB2 = AHB (no divide, 72MHz is within
     * its limit), APB1 = AHB/2 (36MHz is APB1's hard ceiling on this
     * part - RM0008 7.3.2) */
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    RCC->CFGR &= ~RCC_CFGR_PPRE2;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV2;

    /* PLL: HSE (not pre-divided) x9 = 72MHz */
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;
    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) | RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = 72000000UL;
}

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
