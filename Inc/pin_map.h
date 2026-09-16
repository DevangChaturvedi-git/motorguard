/*
 * pin_map.h - Blue Pill pin assignments.
 *
 * Deliberately keeping clear of PA13/PA14/PA15/PB3/PB4 (JTAG) and
 * PA9/PA10 collisions with the debug UART - burned an afternoon on a
 * past project forgetting that once, not doing it again.
 */

#ifndef PIN_MAP_H
#define PIN_MAP_H

/* PWM output to the (virtual) motor driver stage */
#define PWM_PORT        GPIOA
#define PWM_PIN         8      /* TIM1_CH1 */

/* TIM1 break input - in a real build this is driven by a discrete
 * comparator watching the current-sense shunt, tripping the amplifier
 * in hardware with no firmware involvement at all. Wokwi's part
 * library has no op-amp/comparator primitive to actually wire one up,
 * so for the simulation this pin is instead driven directly by a
 * pushbutton standing in for "the comparator just fired" - the
 * electrical behaviour at this specific pin is identical either way,
 * that's the whole point of testing it this way. */
#define BKIN_PORT       GPIOB
#define BKIN_PIN        12

#define ADC_SETPOINT_PORT   GPIOA
#define ADC_SETPOINT_PIN    0
#define ADC_SETPOINT_CH     0

#define ADC_LOAD_PORT       GPIOA
#define ADC_LOAD_PIN        1
#define ADC_LOAD_CH         1

#define UART_TX_PORT    GPIOA
#define UART_TX_PIN     9
#define UART_RX_PORT    GPIOA
#define UART_RX_PIN     10

#define SW_APPROVE_PORT           GPIOB
#define SW_APPROVE_PIN            5
#define SW_INJECT_STALL_PORT      GPIOB
#define SW_INJECT_STALL_PIN       6
#define SW_INJECT_OVERLOAD_PORT   GPIOB
#define SW_INJECT_OVERLOAD_PIN    7
#define SW_INJECT_SENSOR_PORT     GPIOB
#define SW_INJECT_SENSOR_PIN      8

#define LED_RUN_PORT      GPIOB
#define LED_RUN_PIN       13
#define LED_WARN_PORT     GPIOB
#define LED_WARN_PIN      14
#define LED_FAULT_PORT    GPIOB
#define LED_FAULT_PIN     15
#define LED_LOCKOUT_PORT  GPIOA
#define LED_LOCKOUT_PIN   4

/* onboard LED, used as a plain heartbeat so a hung main loop is
 * visible even before anything touches the UART */
#define HEARTBEAT_PORT    GPIOC
#define HEARTBEAT_PIN     13

#endif /* PIN_MAP_H */
