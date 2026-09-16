#include "pwm_driver.h"
#include "stm32f103xb.h"

/* 72MHz / 3600 = 20kHz - a fairly ordinary PWM rate for a small DC
 * motor, well above audible range, low enough that a 20-bit-ish
 * effective duty resolution isn't needed */
#define PWM_ARR   3599u

static volatile bool s_break_pending;

void pwm_driver_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 0;
    TIM1->ARR = PWM_ARR;
    TIM1->CCR1 = 0;

    /* PWM mode 1, preload enabled - CCR1 writes land in the shadow
     * register and only take effect at the next update event, which
     * is the whole point: no torn/glitched pulse mid-cycle when the
     * control loop changes duty */
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | (0x6u << TIM_CCMR1_OC1M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CR1   |= TIM_CR1_ARPE;

    TIM1->CCER |= TIM_CCER_CC1E;
    /* CC1P left at 0 - active high output */

    /* break input: active high (button drives BKIN high = break),
     * AOE=0 so a break event requires an explicit pwm_reenable_output()
     * call afterwards, never a silent auto-resume */
    TIM1->BDTR = (TIM1->BDTR & ~(TIM_BDTR_AOE))
               | TIM_BDTR_BKE | TIM_BDTR_BKP | TIM_BDTR_MOE;

    TIM1->DIER |= TIM_DIER_BIE;
    NVIC_EnableIRQ(TIM1_BRK_IRQn);

    TIM1->EGR |= 0x1u; /* UG - force an update event so the preloaded registers actually latch before the first period */
    TIM1->CR1 |= TIM_CR1_CEN;

    s_break_pending = false;
}

void pwm_set_duty(q16_t duty)
{
    uint32_t ccr = (uint32_t)(((int64_t)duty * (PWM_ARR + 1)) >> Q16_SHIFT);
    if (ccr > PWM_ARR) ccr = PWM_ARR;
    TIM1->CCR1 = ccr;
}

void pwm_reenable_output(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

bool pwm_break_event_pending(void)
{
    bool p = s_break_pending;
    s_break_pending = false;
    return p;
}

void TIM1_BRK_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_BIF) {
        TIM1->SR = (uint16_t)~TIM_SR_BIF; /* clear by writing 0 to the bit, per RM0008 */
        s_break_pending = true;
    }
}
