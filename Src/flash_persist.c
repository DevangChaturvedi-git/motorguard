#include "flash_persist.h"
#include "stm32f103xb.h"
#include <string.h>

#define PERSIST_PAGE_ADDR   (FLASH_BASE + 63u * 1024u)  /* last 1K page of a 64K part - see the .ld file */

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_ready(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
}

static void flash_erase_page(uint32_t addr)
{
    flash_wait_ready();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait_ready();
    FLASH->CR &= ~FLASH_CR_PER;
}

static void flash_write_halfword(uint32_t addr, uint16_t data)
{
    flash_wait_ready();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)addr = data;
    flash_wait_ready();
    FLASH->CR &= ~FLASH_CR_PG;
}

void flash_persist_init(void)
{
    /* nothing to do at boot beyond what flash_persist_read already
     * does on its own - kept as a separate function anyway since
     * every other driver in this project has an _init, and a future
     * version that adds real wear-levelling will need somewhere to
     * put its bookkeeping */
}

bool flash_persist_read(persisted_state_t *out)
{
    memcpy(out, (const void *)PERSIST_PAGE_ADDR, sizeof(persisted_state_t));
    return out->magic == FLASH_PERSIST_MAGIC;
}

void flash_persist_write(const persisted_state_t *in)
{
    flash_unlock();
    flash_erase_page(PERSIST_PAGE_ADDR);

    const uint16_t *src = (const uint16_t *)in;
    uint32_t count = (sizeof(persisted_state_t) + 1) / 2;
    for (uint32_t i = 0; i < count; i++) {
        flash_write_halfword(PERSIST_PAGE_ADDR + (i * 2u), src[i]);
    }

    flash_lock();
}
