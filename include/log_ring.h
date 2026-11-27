/**
 * log_ring.h — кольцевой буфер блоков VLOG (Docs/05 §2, Docs/04 §3).
 *
 * 16 блоков по 4096 байт в D1 SRAM (64 КБ = ~6.8 с запаса при задержках SD-карты).
 * Производитель: разбор DMA-буфера (bsp_capture_parse_fifo -> ring_push_sample).
 * Потребитель (f_write по 4096 байт) появляется на этапе SDIO/FatFs.
 *
 * Блок: 16 байт заголовок + 4080 байт сэмплов (680 сэмплов x 6 байт,
 * {int16 x, int16 y, int16 z}, little-endian). Заголовок блока на этом этапе
 * НЕ заполняется — пишется только полезная нагрузка (Docs/04 §3).
 */

#ifndef LOG_RING_H
#define LOG_RING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Число блоков в кольце (Docs/05 §2: ring[16][4096]). */
#define RING_BLOCK_COUNT 16u
/** Маска индекса блока: head/tail всегда 0..15. */
#define RING_BLOCK_MASK 0x0Fu
/** Размер блока = 8 секторов SD (Docs/03 §2). */
#define RING_BLOCK_SIZE 4096u
/** Заголовок блока (Docs/04 §3): полезная нагрузка начинается с 0x10. */
#define RING_HEADER_SIZE 16u
/** Сэмплов в одном блоке (Docs/04 §3): 680 * 6 = 4080 байт. */
#define RING_SAMPLES_PER_BLOCK 680u
/** Байт в сэмпле: {x, y, z} как int16 little-endian (Docs/04 §3). */
#define RING_SAMPLE_SIZE 6u

/**
 * Состояние кольца. head — блок, в который пишем сейчас; tail — блок,
 * ещё не записанный на карту. Переполнение (Docs/05 §2 п.1): если
 * (head + 1) & 15 == tail, недописанный блок сбрасывается и пересобирается
 * на месте, а потеря фиксируется в overflow_count.
 */
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t current_block_samples;
    volatile uint32_t overflow_count;
} ring_state_t;

/** Сброс состояния кольца: head = tail = 0, счётчики обнулены. */
void ring_init(void);

/**
 * Записать один сэмпл в текущий блок (6 байт: x, y, z, little-endian).
 * При 680 сэмплов head = (head + 1) & 15, счётчик сэмплов обнуляется.
 * Если кольцо заполнено (next head == tail), блок сбрасывается и пересобирается
 * на месте, overflow_count увеличивается (кольцо не переполняется молча).
 * true — сэмпл записан; false — защитная проверка состояния кольца.
 */
bool ring_push_sample(int16_t x, int16_t y, int16_t z);

/** Число сброшенных из-за переполнения блоков (флаг OVERFLOW для заголовка). */
uint32_t ring_overflow_count(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_RING_H */
