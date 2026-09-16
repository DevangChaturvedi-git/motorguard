#include "uart_driver.h"
#include "stm32f103xb.h"

#define TX_BUF_SIZE 256

static volatile uint8_t  s_tx_buf[TX_BUF_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;

void uart_driver_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* APB2 = 72MHz, 115200 baud -> BRR = 72000000/115200 = 625 exactly,
     * no fractional remainder to worry about */
    USART1->BRR = 625;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    NVIC_EnableIRQ(USART1_IRQn);

    s_tx_head = 0;
    s_tx_tail = 0;
}

static void tx_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_tx_head + 1) % TX_BUF_SIZE);
    if (next == s_tx_tail) {
        return; /* buffer full - drop rather than block; diagnostics are best-effort, the control loop is not allowed to wait on a UART */
    }
    s_tx_buf[s_tx_head] = byte;
    s_tx_head = next;
    USART1->CR1 |= USART_CR1_TXEIE;
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_TXE) {
        if (s_tx_tail != s_tx_head) {
            USART1->DR = s_tx_buf[s_tx_tail];
            s_tx_tail = (uint16_t)((s_tx_tail + 1) % TX_BUF_SIZE);
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void uart_str(const char *s)
{
    while (*s) { tx_push((uint8_t)*s); s++; }
}

void uart_u32(uint32_t v)
{
    char digits[10];
    int n = 0;
    if (v == 0) { tx_push('0'); return; }
    while (v > 0 && n < 10) { digits[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n > 0) { tx_push((uint8_t)digits[--n]); }
}

void uart_i32(int32_t v)
{
    if (v < 0) { tx_push('-'); uart_u32((uint32_t)(-v)); }
    else       { uart_u32((uint32_t)v); }
}

void uart_q16(q16_t v)
{
    if (v < 0) { tx_push('-'); v = -v; }
    int32_t whole = v >> Q16_SHIFT;
    int32_t frac_raw = v & 0xFFFF;
    int32_t hundredths = (int32_t)(((int64_t)frac_raw * 100) >> Q16_SHIFT);
    uart_u32((uint32_t)whole);
    tx_push('.');
    if (hundredths < 10) tx_push('0');
    uart_u32((uint32_t)hundredths);
}

void uart_newline(void)
{
    tx_push('\r');
    tx_push('\n');
}
