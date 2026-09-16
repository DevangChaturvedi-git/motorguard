#include "adc_driver.h"
#include "pin_map.h"
#include "stm32f103xb.h"

#define NUM_ADC_CHANNELS  2

static const uint8_t s_channel_map[NUM_ADC_CHANNELS] = { ADC_SETPOINT_CH, ADC_LOAD_CH };
static volatile uint16_t s_raw[NUM_ADC_CHANNELS];
static volatile uint8_t  s_rr_index;

static void start_conversion(uint8_t index)
{
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (s_channel_map[index] << ADC_SQR3_SQ1_Pos);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void adc_driver_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* ADC clock must stay under 14MHz per the datasheet - APB2 is
     * 72MHz here, so /6 gives 12MHz */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL; /* software trigger (EXTSEL=111) */
    ADC1->SQR1 = 0; /* one conversion in the regular sequence */

    /* the calibration sequence RM0008 9.3.11 actually specifies -
     * power up, wait, reset calibration registers, calibrate, wait for
     * CAL to clear on its own. skipping the settle delay after ADON
     * is a documented way to get a garbage first calibration. */
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++) { } /* tADCVREG startup, see datasheet */

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }

    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC1_2_IRQn);

    s_rr_index = 0;
}

void adc_driver_start_next(void)
{
    start_conversion(s_rr_index);
}

void ADC1_2_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_EOC) {
        s_raw[s_rr_index] = (uint16_t)ADC1->DR; /* reading DR clears EOC */
        s_rr_index = (uint8_t)((s_rr_index + 1) % NUM_ADC_CHANNELS);
        start_conversion(s_rr_index);
    }
}

/* 12-bit ADC (0..4095) -> Q16.16, 0..Q16_ONE */
static q16_t raw_to_q16(uint16_t raw)
{
    return (q16_t)(((int64_t)raw << Q16_SHIFT) / 4095);
}

q16_t adc_read_setpoint(void) { return raw_to_q16(s_raw[0]); }
q16_t adc_read_load(void)     { return raw_to_q16(s_raw[1]); }
