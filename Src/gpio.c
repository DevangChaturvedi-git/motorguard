#include "gpio.h"
#include "pin_map.h"
#include "stm32f103xb.h"

/* CRL/CRH pack two config bits (CNF) + two mode bits (MODE) per pin,
 * four bits total, sixteen pins split across two 32-bit registers -
 * this just writes those four bits into whichever register/position a
 * given pin actually lives at, so the rest of this file can talk about
 * pins 0-15 without caring about the CRL/CRH split. */
static void gpio_config_pin(GPIO_TypeDef *port, uint8_t pin, uint32_t mode_cnf)
{
    volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
    uint8_t shift = (pin % 8) * 4;
    uint32_t mask = 0xFu << shift;
    *cr = (*cr & ~mask) | (mode_cnf << shift);
}

#define MODE_INPUT             0x0u
#define MODE_OUTPUT_2MHZ       0x2u
#define MODE_OUTPUT_50MHZ      0x3u

#define CNF_IN_ANALOG           (0x0u << 2)
#define CNF_IN_FLOATING         (0x1u << 2)
#define CNF_IN_PULL             (0x2u << 2)
#define CNF_OUT_PUSHPULL        (0x0u << 2)
#define CNF_OUT_AF_PUSHPULL     (0x2u << 2)

typedef struct { GPIO_TypeDef *port; uint8_t pin; } pin_ref_t;

static const pin_ref_t s_leds[] = {
    [LED_RUN]     = { LED_RUN_PORT,     LED_RUN_PIN },
    [LED_WARN]    = { LED_WARN_PORT,    LED_WARN_PIN },
    [LED_FAULT]   = { LED_FAULT_PORT,   LED_FAULT_PIN },
    [LED_LOCKOUT] = { LED_LOCKOUT_PORT, LED_LOCKOUT_PIN },
};

static const pin_ref_t s_switches[] = {
    [SW_APPROVE]          = { SW_APPROVE_PORT,          SW_APPROVE_PIN },
    [SW_INJECT_STALL]     = { SW_INJECT_STALL_PORT,     SW_INJECT_STALL_PIN },
    [SW_INJECT_OVERLOAD]  = { SW_INJECT_OVERLOAD_PORT,  SW_INJECT_OVERLOAD_PIN },
    [SW_INJECT_SENSOR]    = { SW_INJECT_SENSOR_PORT,    SW_INJECT_SENSOR_PIN },
};

/* one bit of debounce state per switch - was it seen pressed last
 * tick? edge-detect off that rather than reacting to every tick a
 * button happens to be held down for */
static bool s_sw_prev_state[SW_COUNT];

void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN
                  | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    /* heartbeat - onboard LED, push-pull output */
    gpio_config_pin(HEARTBEAT_PORT, HEARTBEAT_PIN, MODE_OUTPUT_2MHZ | CNF_OUT_PUSHPULL);

    /* status LEDs */
    for (int i = 0; i < 4; i++) {
        gpio_config_pin(s_leds[i].port, s_leds[i].pin, MODE_OUTPUT_2MHZ | CNF_OUT_PUSHPULL);
    }

    /* ADC inputs - analog mode disables the Schmitt trigger and digital
     * input buffer, which is what actually lets analogRead-style
     * conversion work correctly instead of just floating garbage */
    gpio_config_pin(ADC_SETPOINT_PORT, ADC_SETPOINT_PIN, MODE_INPUT | CNF_IN_ANALOG);
    gpio_config_pin(ADC_LOAD_PORT, ADC_LOAD_PIN, MODE_INPUT | CNF_IN_ANALOG);

    /* PWM output, TIM1_CH1 alternate function */
    gpio_config_pin(PWM_PORT, PWM_PIN, MODE_OUTPUT_50MHZ | CNF_OUT_AF_PUSHPULL);

    /* TIM1 break input - input with pull-down, so it reads a clean
     * LOW (no break) until whatever's wired to it (the button standing
     * in for the comparator) actively drives it HIGH */
    gpio_config_pin(BKIN_PORT, BKIN_PIN, MODE_INPUT | CNF_IN_PULL);
    BKIN_PORT->BSRR = (1u << (BKIN_PIN + 16)); /* ODR=0 selects pull-down, not pull-up */

    /* UART */
    gpio_config_pin(UART_TX_PORT, UART_TX_PIN, MODE_OUTPUT_50MHZ | CNF_OUT_AF_PUSHPULL);
    gpio_config_pin(UART_RX_PORT, UART_RX_PIN, MODE_INPUT | CNF_IN_FLOATING);

    /* buttons - pull-up, pressed = reads LOW */
    for (int i = 0; i < SW_COUNT; i++) {
        gpio_config_pin(s_switches[i].port, s_switches[i].pin, MODE_INPUT | CNF_IN_PULL);
        s_switches[i].port->BSRR = (1u << s_switches[i].pin); /* ODR=1 selects pull-up */
        s_sw_prev_state[i] = false;
    }
}

void led_set(led_t led, bool on)
{
    const pin_ref_t *p = &s_leds[led];
    if (on) p->port->BSRR = (1u << p->pin);
    else    p->port->BSRR = (1u << (p->pin + 16));
}

void heartbeat_toggle(void)
{
    HEARTBEAT_PORT->ODR ^= (1u << HEARTBEAT_PIN);
}

bool switch_pressed_edge(switch_t sw)
{
    const pin_ref_t *p = &s_switches[sw];
    bool pressed_now = (p->port->IDR & (1u << p->pin)) == 0; /* active low */
    bool edge = pressed_now && !s_sw_prev_state[sw];
    s_sw_prev_state[sw] = pressed_now;
    return edge;
}

bool switch_is_pressed(switch_t sw)
{
    const pin_ref_t *p = &s_switches[sw];
    return (p->port->IDR & (1u << p->pin)) == 0;
}

bool gpio_bkin_asserted(void)
{
    return (BKIN_PORT->IDR & (1u << BKIN_PIN)) != 0;
}
