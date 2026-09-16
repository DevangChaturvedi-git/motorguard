/*
 * adc_driver.h
 *
 * Wokwi's Blue Pill simulation doesn't implement DMA, which was the
 * original plan for feeding this. Falling back to EOC-interrupt-driven
 * conversion instead: round-robins between the two channels, one
 * conversion at a time, latching each result into a small array the
 * rest of the firmware reads from. Still fully non-blocking - nothing
 * spins waiting on ADC hardware - just ISR-fed instead of DMA-fed.
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "fixed_point.h"

void adc_driver_init(void);
void adc_driver_start_next(void); /* kick off the next conversion in the round-robin */

q16_t adc_read_setpoint(void);
q16_t adc_read_load(void);

#endif /* ADC_DRIVER_H */
