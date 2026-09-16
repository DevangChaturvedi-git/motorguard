#include <stdint.h>

#include "stm32f103xb.h"
#include "drivers.h"
#include "gpio.h"
#include "adc_driver.h"
#include "pwm_driver.h"
#include "uart_driver.h"
#include "supervisor.h"

/* The Wokwi baseline uses SysTick for the supervisor cadence instead of
 * DWT->CYCCNT. DWT timing was unreliable in the simulator, whereas SysTick is
 * part of the normal Cortex-M exception model and gives the control loop a
 * stable 5 ms period. */
static volatile uint32_t s_millis;

void SysTick_Handler(void)
{
    s_millis++;
}

int main(void)
{
    clock_init();
    gpio_init();                 /* outputs are configured before PWM is enabled */
    uart_driver_init();
    pwm_driver_init();
    adc_driver_init();
    supervisor_init();

    /* 1 ms timebase; supervisor_tick() runs every 5 ms below. */
    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) {
        /* A bad reload value means the deterministic schedule is unavailable.
         * Keep PWM at its reset value (zero) and do not start the plant. */
        for (;;) { __WFI(); }
    }

    adc_driver_start_next();
    uart_str("MotorGuard baseline ready (STM32F103 / Wokwi)");
    uart_newline();

    uint32_t last_tick = 0;
    for (;;) {
        uint32_t now = s_millis;
        if ((now - last_tick) >= 5u) {
            last_tick += 5u;
            supervisor_tick(last_tick / 5u);

            /* A simple heartbeat proves that the main scheduling loop is alive. */
            if ((last_tick % 500u) == 0u) {
                heartbeat_toggle();
            }
        } else {
            __WFI();
        }
    }
}
