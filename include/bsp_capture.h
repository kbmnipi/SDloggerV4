/**
 * bsp_capture.h — hardware-захват сырых байтов FIFO датчика BMI160 по SPI1 DMA.
 *
 * Назначение этапа (Docs/05 §3 M0): проверить прерывание INT1 (PB5) и пакетную
 * вычитку FIFO_DATA (0x24) потоками DMA2. Разбор фреймов (0x84 / 0x80 / 0x40)
 * и кольцевой буфер на этом этапе СОЗНАТЕЛЬНО не реализуются.
 *
 * Назначение потоков (Docs/03_sdio_and_dma.md §1, матрица RM0090):
 *   DMA2 Stream 0 / Channel 3 -> SPI1_RX (периферия -> память)
 *   DMA2 Stream 3 / Channel 3 -> SPI1_TX (память -> периферия, dummy-байты)
 *   PB5 -> EXTI9_5 (нарастающий фронт, INT1 датчика Active HIGH)
 *
 * Все буферы, участвующие в DMA, лежат в D1 SRAM (Docs/01 §5).
 */

#ifndef BSP_CAPTURE_H
#define BSP_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Вычитка FIFO по DMA в полёте: true с момента bsp_capture_start_dma()
 * до входа в DMA2_Stream0_IRQHandler (TC или ошибка). Повторный вызов
 * bsp_capture_start_dma() при true молча отклоняется — защита от гонки,
 * когда EXTI успевает поднять следующий FWM раньше разбора пакета.
 */
extern volatile bool g_dma_busy;

/** Размер приёмного буфера spi_rx_buf вызывающего (Docs/01 §5). */
#define BSP_CAPTURE_BUF_SIZE 256u

/** Потолок одной DMA-вычитки: 252 = 36 * 7 байт (Docs/02 §4 п.4). */
#define BSP_CAPTURE_MAX_LEN 252u

/** Байт в regular-фрейме BMI160: 1 заголовок + 6 данных (Docs/02 §4). */
#define BSP_CAPTURE_FRAME_BYTES 7u

/** FIFO почти полон: >= 1015 байт -> SENSOR_OVR (Docs/02 §4 п.3). */
#define BSP_CAPTURE_FIFO_OVR_BYTES 1015u

/**
 * Инициализация DMA2 Stream 0 (SPI1_RX) и Stream 3 (SPI1_TX) + NVIC для
 * Stream 0 (прерывание Transfer Complete). Сбой HAL_DMA_Init() увеличивает
 * bsp_capture_error_count().
 */
void bsp_capture_dma_init(void);

/**
 * Инициализация PB5 как EXTI (нарастающий фронт, INT1 датчика Active HIGH)
 * и включение EXTI9_5_IRQn с приоритетом 6.
 */
void bsp_capture_exti_init(void);

/**
 * Запуск пакетного чтения FIFO_DATA (0x24) в dest_buf (Docs/02 §4):
 * Quiesce потоков -> CS = 0 -> командный байт 0xA4 опросом ->
 * RX DMA (SPI1->DR -> dest_buf) + TX DMA (dummy -> SPI1->DR).
 *
 * Если g_dma_busy (предыдущая вычитка ещё в полёте) — возврат false БЕЗ
 * инкремента ошибок: транзакция не потеряна, просто ещё не завершилась.
 *
 * CS остаётся активным до прерывания завершения приёма
 * (bsp_capture_dma_complete()). Длина len: 1..BSP_CAPTURE_MAX_LEN,
 * dest_buf обязан лежать в D1 SRAM. Сбой увеличивает bsp_capture_error_count().
 *
 * Возврат ОБЯЗАТЕЛЕН к проверке (аудит M1, V1): true — транзакция реально
 * стартовала, и только тогда len соответствует данным в dest_buf. false —
 * буфер не заполнялся, разбирать по этому len нельзя.
 */
bool bsp_capture_start_dma(uint8_t *dest_buf, uint16_t len);

/**
 * Завершение приёма: вызывается из DMA2_Stream0_IRQHandler после снятия
 * флагов потока. Гасит DMA-запросы SPI1 и поднимает CS (конец транзакции
 * BMI160 определяется по фронту CS = 1).
 */
void bsp_capture_dma_complete(void);

/**
 * Разбор принятого пакета FIFO_DATA (Docs/02 §4, заголовки BMI160):
 *   0x84 Regular Accel Frame : 1 + 6 байт {XL,XH,YL,YH,ZL,ZH} -> ring_push_sample
 *   0x80 Empty / Overread    : конец валидных данных — выход из цикла
 *   0x40 Skip Frame          : 2 байта, потеря сэмплов (счётчик skip-фреймов)
 *   0x44 Sensortime Frame    : 4 байта, игнорируем
 *   0x48 FIFO Config Change  : 2 байта, игнорируем
 *   любой другой заголовок   : сбой SPI / потеря синхронизации — выход
 * За предел len чтение не выходит: неполный фрейм в конце пакета отбрасывается.
 */
void bsp_capture_parse_fifo(uint8_t *buf, uint16_t len);

/**
 * Инкремент счётчика ошибок из DMA2_Stream0_IRQHandler (TEIF0/FEIF0/DMEIF0):
 * счётчик g_capture_errors статический в bsp_capture.c, наружу — только счётчик
 * и этот raising-хелпер.
 */
void bsp_capture_error_raise(void);

/**
 * Ручки потоков SPI1-вычитки наружу: DMA2_Stream0_IRQHandler обязан привести
 * State обоих потоков в HAL_DMA_STATE_READY (свой и TX), иначе следующий
 * HAL_DMA_Start_IT() вернёт HAL_ERROR.
 */
DMA_HandleTypeDef *bsp_capture_rx_dma_handle(void);
DMA_HandleTypeDef *bsp_capture_tx_dma_handle(void);

/** Число Skip-фреймов (SENSOR_OVR для заголовка блока, Docs/04 §3). */
uint32_t bsp_capture_skip_frame_count(void);

/** Число сбоев инициализации/запуска вычитки (запрет тихих отказов). */
uint32_t bsp_capture_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAPTURE_H */
