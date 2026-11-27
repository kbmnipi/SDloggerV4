/**
 * log_ring.c — кольцевой буфер 16 блоков по 4096 байт (Docs/05 §2, Docs/04 §3).
 *
 * ring_buf — в секции .d1ram: DMA2 обязан видеть эти блоки (Docs/01 §5),
 * в CCM их размещать нельзя.
 *
 * Порядок записи одного блока:
 *   байты 0..15    — заголовок (заполняется на этапе SDIO/FatFs, здесь не трогаем)
 *   байты 16..4095 — 680 сэмплов по 6 байт {x, y, z}, little-endian
 */

#include "log_ring.h"

/* 16 * 4096 = 65536 байт D1 SRAM. KEEP в скрипте линковщика обязателен:
   к буферу обращается железо (SDIO DMA) без явных ссылок из кода. */
static uint8_t ring_buf[RING_BLOCK_COUNT][RING_BLOCK_SIZE]
    __attribute__((section(".d1ram"), aligned(4)));

static ring_state_t g_ring;

static void ring_put_i16(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    dst[0] = (uint8_t)(raw & 0xFFu);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFu);
}

void ring_init(void)
{
    g_ring.head = 0u;
    g_ring.tail = 0u;
    g_ring.current_block_samples = 0u;
    g_ring.overflow_count = 0u;
}

bool ring_push_sample(int16_t x, int16_t y, int16_t z)
{
    uint16_t slot = g_ring.current_block_samples;
    uint8_t *dst;

    if (slot >= RING_SAMPLES_PER_BLOCK) {
        /* Защита: счётчик не должен перепрыгнуть конец полезной нагрузки */
        g_ring.overflow_count++;
        g_ring.current_block_samples = 0u;
        return false;
    }

    dst = &ring_buf[g_ring.head][RING_HEADER_SIZE + (uint32_t)slot * RING_SAMPLE_SIZE];
    ring_put_i16(dst, x);
    ring_put_i16(dst + 2u, y);
    ring_put_i16(dst + 4u, z);

    slot++;
    if (slot < RING_SAMPLES_PER_BLOCK) {
        g_ring.current_block_samples = slot;
        return true;
    }

    /* Блок заполнен (680 сэмплов). Двигаем head, но не наступает на tail:
       Docs/05 §2 п.1 — при полном кольце новый блок сбрасывается, датчик
       не останавливается, потеря помечается флагом OVERFLOW (Docs/04 §3). */
    if ((((g_ring.head + 1u) & RING_BLOCK_MASK) == g_ring.tail)) {
        g_ring.overflow_count++;
        g_ring.current_block_samples = 0u;
        return true;
    }

    g_ring.head = (uint16_t)((g_ring.head + 1u) & RING_BLOCK_MASK);
    g_ring.current_block_samples = 0u;

    return true;
}

uint32_t ring_overflow_count(void)
{
    return g_ring.overflow_count;
}
