/**
 * diskio.c — связка FatFs R0.12c <-> SDIO (bsp_sdio.c) для SDloggerV4.
 *
 * Правила (Docs/03_sdio_and_dma.md §2):
 *   - буфер, который уходит в HAL_SD_WriteBlocks_DMA(), обязан лежать в
 *     D1 SRAM (0x20000000..0x20020000) и быть выровнен на 4 байта: DMA2 не
 *     видит CCM (Docs/01 §5), а SDIO-DMA работает словами;
 *   - после конца DMA обязательно опрашивать состояние карты (CMD13) до
 *     HAL_SD_CARD_TRANSFER: карта держит DAT0, пока программирует флеш,
 *     нормальная задержка сборки мусора — до 450 мс на блок.
 *
 * Прототипы берутся из vendored diskio.h (R0.12c): сектор — DWORD, не LBA_t.
 */

#include "diskio.h"

#include "stm32f4xx_hal.h"

#include "bsp_sdio.h"

/** Единственный физический диск проекта: MicroSD на SDIO (Docs/01 §2). */
#define PD_DRV_SDIO 0u

/* D1 SRAM: 0x20000000 + 128 KiB (Docs/01 §5) */
#define D1_SRAM_START 0x20000000u
#define D1_SRAM_END 0x20020000u

/* Ожидание конца DMA-транзакции */
#define DISK_XFER_TIMEOUT_MS 1000u
/* Docs/03 §2: до 450 мс на блок — карта программирует флеш после DMA */
#define DISK_CARD_STATE_MS_PER_BLOCK 450u
/* L0: число попыток одной блочной записи до Abort + re-init (Docs/03 §2) */
#define DISK_WRITE_RETRY_COUNT 3u

static bool buf_is_dmable(const void *buf, uint32_t bytes)
{
    uint32_t addr = (uint32_t)buf;
    uint32_t last;

    if (buf == NULL) {
        return false;
    }
    if ((addr % 4u) != 0u) {
        return false; /* SDIO-DMA ходит словами (bsp_sdio.c: MDATAALIGN_WORD) */
    }
    if (addr < D1_SRAM_START || addr >= D1_SRAM_END) {
        return false;
    }

    /* Защита от переноса: bytes > размера D1 (0x20020000) обязан быть
       отвергнут ДО сложения — иначе addr + bytes заворачивается по uint32_t
       и «влезшая» в конец SRAM хвостом проверка врёт о чужом буфере */
    if (bytes > D1_SRAM_END) {
        return false;
    }

    last = addr + bytes;

    return (last <= D1_SRAM_END);
}

static bool wait_card_transfer(SD_HandleTypeDef *hsd, uint32_t blocks)
{
    uint32_t timeout = DISK_CARD_STATE_MS_PER_BLOCK * blocks;
    uint32_t start = HAL_GetTick();

    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - start) > timeout) {
            return false;
        }
    }

    return true;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != PD_DRV_SDIO) {
        return STA_NOINIT;
    }

    /* bsp_sdio_init(): HAL_SD_Init() (CMD0/8/ACMD41/CMD2/CMD3/CMD9/CMD10/CMD7)
       + HAL_SD_ConfigWideBusOperation(SDIO_BUS_WIDE_4B), Docs/03 §2 */
    if (!bsp_sdio_init()) {
        return STA_NOINIT;
    }

    return 0u;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != PD_DRV_SDIO) {
        return STA_NOINIT;
    }

    /* Готовность уже обеспечена disk_initialize(); извлечение карты на ходу
       на этапе M1 не отслеживается (после записи 3 повторов, Docs/03 §2) */
    return 0u;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    SD_HandleTypeDef *hsd;

    if (pdrv != PD_DRV_SDIO) {
        return RES_PARERR;
    }
    if (count == 0u) {
        return RES_PARERR;
    }
    if (!buf_is_dmable(buff, (uint32_t)count * 512u)) {
        return RES_PARERR; /* Docs/01 §5: DMA-буфер только в D1 SRAM */
    }

    hsd = bsp_sdio_handle();
    bsp_sdio_transfer_begin();

    if (HAL_SD_ReadBlocks_DMA(hsd, buff, sector, count) != HAL_OK) {
        /* Срыв транзакции обязателен (аудит M1, V4): иначе hsd остаётся BUSY и
           каждая следующая операция чтения будет возвращать RES_ERROR до
           перезапуска питания. abort_recover() = HAL_SD_Abort() + перепривязка
           DMA-потока */
        (void)bsp_sdio_abort_recover();
        return RES_ERROR;
    }
    if (!bsp_sdio_wait_transfer(DISK_XFER_TIMEOUT_MS)) {
        /* При таймауте/ошибке bsp_sdio_wait_transfer() срывает транзакцию и
           восстанавливает DMA-обвязку самостоятельно */
        return RES_ERROR;
    }

    return RES_OK;
}

/**
 * Одна попытка блочной записи: DMA -> ожидание конца -> CMD13 до TRANSFER.
 * Любой сбой — bsp_sdio_abort_recover() (= HAL_SD_Abort() + перепривязка
 * DMA-потока): иначе hsd остаётся BUSY и следующая попытка
 * гарантированно вернёт HAL_BUSY вместо новой транзакции (Docs/03 §2).
 */
static DRESULT sd_write_once(SD_HandleTypeDef *hsd, const BYTE *buff, DWORD sector, UINT count)
{
    bsp_sdio_transfer_begin();

    if (HAL_SD_WriteBlocks_DMA(hsd, (uint8_t *)buff, sector, count) != HAL_OK) {
        (void)bsp_sdio_abort_recover();
        return RES_ERROR;
    }
    if (!bsp_sdio_wait_transfer(DISK_XFER_TIMEOUT_MS)) {
        /* При таймауте/ошибке bsp_sdio_wait_transfer() сам срывает транзакцию
           и перепривязывает DMA-поток — повторно срыв не нужен, повторный
           HAL_SD_MspInit() по уже READY-потоку только съел бы время */
        return RES_ERROR;
    }

    /* Docs/03 §2: DMA закончен, когда данные ушли в буфер карты — сама карта
       ещё может программировать флеш (CMD13 до HAL_SD_CARD_TRANSFER) */
    if (!wait_card_transfer(hsd, (uint32_t)count)) {
        (void)bsp_sdio_abort_recover();
        return RES_ERROR;
    }

    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    SD_HandleTypeDef *hsd;

    if (pdrv != PD_DRV_SDIO) {
        return RES_PARERR;
    }
    if (count == 0u) {
        return RES_PARERR;
    }
    if (!buf_is_dmable(buff, (uint32_t)count * 512u)) {
        /* Docs/03 §2 требует копировать в промежуточный выровненный буфер
           4096 Б; намеренно не делаем: все буферы прошивки (ring_buf,
           spi_rx_buf) уже лежат в .d1ram и выровнены на 4 — лишние 4 КБ
           D1 SRAM дороже. Вызывающий обязан передавать такой буфер. */
        return RES_PARERR;
    }

    hsd = bsp_sdio_handle();

    /* L0 (Docs/03 §2): три попытки DMA-записи с ожиданием CMD13 */
    for (uint32_t attempt = 0u; attempt < DISK_WRITE_RETRY_COUNT; attempt++) {
        if (sd_write_once(hsd, buff, sector, count) == RES_OK) {
            return RES_OK;
        }
    }

    /* Все попытки исчерпаны: срыв транзакции с перепривязкой DMA-потока и
       L1 (Docs/03 §2) — полный перезапуск инициализации карты. Вызов
       bsp_sdio_init() «напрямую» ничего не делал: идемпотентный guard
       g_sdio_ready возвращал true сразу, и «переинициализация» была пустой */
    (void)bsp_sdio_abort_recover();
    bsp_sdio_reinit();

    return RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    SD_HandleTypeDef *hsd;

    if (pdrv != PD_DRV_SDIO) {
        return RES_PARERR;
    }

    hsd = bsp_sdio_handle();

    switch (cmd) {
    case CTRL_SYNC:
        /* Конечный автомат HAL доводится до READY в SDIO_IRQHandler/DMA IRQ */
        return (HAL_SD_GetCardState(hsd) == HAL_SD_CARD_TRANSFER) ? RES_OK : RES_ERROR;

    case GET_SECTOR_COUNT:
        if (buff == NULL) {
            return RES_PARERR;
        }
        *(DWORD *)buff = hsd->SdCard.LogBlockNbr;

        return RES_OK;

    case GET_SECTOR_SIZE:
        if (buff == NULL) {
            return RES_PARERR;
        }
        *(WORD *)buff = 512u; /* _MIN_SS == _MAX_SS == 512 (ffconf.h) */

        return RES_OK;

    case GET_BLOCK_SIZE:
        if (buff == NULL) {
            return RES_PARERR;
        }
        *(DWORD *)buff = 1u; /* f_mkfs() не используется (_USE_MKFS = 0) */

        return RES_OK;

    default:
        return RES_PARERR;
    }
}

DWORD get_fattime(void)
{
    /* Часов в приборе нет (_FS_NORTC = 1 в ffconf.h), формат VLOG timestamp
       не использует (Docs/04). Функция оставлена, чтобы объявление в diskio.h
       было закрыто, если _FS_NORTC когда-нибудь выключат. */
    return 0u;
}
