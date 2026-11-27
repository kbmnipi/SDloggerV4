/**
 * bsp_sdio.h — MicroSD на SDIO 4-bit с DMA (Docs/03_sdio_and_dma.md §1-2,
 * Docs/01_hardware_and_build.md §2).
 *
 *   SDIO_CK = PC12, SDIO_CMD = PD2, SDIO_D0..D3 = PC8..PC11 (AF12)
 *   SDIO_CK = SDIOCLK 48 МГц / (ClockDiv 0 + 2) = 24 МГц (Docs/03 §2)
 *   DMA2 Stream 6 / Channel 4, приоритет MEDIUM — строго НИЖЕ SPI1 (Docs/03 §1)
 *
 * Запись на карте — всегда кратно 4096 байт (8 секторов) за один вызов
 * (Docs/03 §2): счётчик SDIO_DCTRL 13-битный, 4096 гарантированно влезает.
 */

#ifndef BSP_SDIO_H
#define BSP_SDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Байт в блоке SD-записи = 8 секторов FatFs (Docs/03 §2). */
#define BSP_SDIO_BLOCK_BYTES 4096u
/** Секторов в одном блоке SD-записи. */
#define BSP_SDIO_BLOCKS_PER_WRITE 8u

/**
 * Инициализация SDIO + DMA2 Stream6/CH4 + NVIC (SDIO_IRQn, DMA2_Stream6_IRQn),
 * инициализация карты (HAL_SD_Init) и переход в 4-битный режим
 * (HAL_SD_ConfigWideBusOperation). Повторный вызов после успеха — ничего не
 * делает и возвращает true.
 * true — карта отвечает, 4-битный режим включён; false — любой сбой SDIO/карты.
 */
bool bsp_sdio_init(void);

/** true если bsp_sdio_init() уже отработал успешно. */
bool bsp_sdio_ready(void);

/** Ручка SDIO для FatFs/diskio.c. */
SD_HandleTypeDef *bsp_sdio_handle(void);

/** Ручка DMA2 Stream6/CH4 (SDIO) для DMA2_Stream6_IRQHandler(). */
DMA_HandleTypeDef *bsp_sdio_dma_handle(void);

/** Перед запуском HAL_SD_ReadBlocks_DMA()/HAL_SD_WriteBlocks_DMA(). */
void bsp_sdio_transfer_begin(void);

/**
 * Ждать завершения DMA-транзакции (колбэки HAL_SD_TxCpltCallback /
 * HAL_SD_RxCpltCallback / HAL_SD_ErrorCallback). При таймауте транзакция
 * прерывается HAL_SD_Abort(), чтобы состояние HAL не осталось BUSY.
 * true — транзакция завершена; false — таймаут или ошибка SDIO/DMA.
 */
bool bsp_sdio_wait_transfer(uint32_t timeout_ms);

/**
 * Срыв транзакции и восстановление DMA-обвязки: HAL_SD_Abort() + повторный
 * HAL_SD_MspInit() (Docs/03 §2). Вызывать после ЛЮБОГО сбоя DMA-транзакции:
 * сам HAL_SD_Abort() состояние DMA-потока не восстанавливает, и без этого
 * следующая транзакция вернёт HAL_ERROR навсегда. Сброс флагов done/error
 * выполняется внутри.
 * true — поток перепривязан; false — HAL_DMA_Init() не удалось (SDIO готово
 * к повторной bsp_sdio_init(), но не к транзакциям).
 */
bool bsp_sdio_abort_recover(void);

/**
 * L1 (Docs/03 §2): полный перезапуск инициализации SDIO/карты после исчерпания
 * повторов записи. Обходит идемпотентный guard bsp_sdio_init(), поэтому
 * вызов bsp_sdio_init() «напрямую» после сбоя ничего не делает.
 */
void bsp_sdio_reinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SDIO_H */
