/*
 * uart_driver.h
 *
 * Interrupt-driven TX into a small ring buffer - same reasoning as the
 * ADC driver, no DMA in Wokwi's Blue Pill sim so this is ISR-fed
 * instead. Deliberately not pulling in libc's printf/snprintf here:
 * this project never needs float formatting (everything's fixed-point
 * already), and the float-capable formatter in newlib is not a small
 * amount of flash on a part with 64KB total. A handful of small
 * integer/string helpers cover every diagnostic line this needs.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include "fixed_point.h"

void uart_driver_init(void);

void uart_str(const char *s);
void uart_u32(uint32_t v);
void uart_i32(int32_t v);
/* prints a Q16.16 value as a plain decimal with 2 fractional digits,
 * e.g. 1.15 - good enough for a diagnostic line, not trying to be a
 * general-purpose formatter */
void uart_q16(q16_t v);
void uart_newline(void);

#endif /* UART_DRIVER_H */
