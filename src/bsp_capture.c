/**
 * bsp_capture.c — захват сырых байтов FIFO BMI160 по SPI1 DMA (Docs/02 §4, Docs/03 §1).
 *
 * Транзакция вычитки (одна, непрерывная, пока CS = 0):
 *   1. Quiesce: потоки остановлены, SPI1->CR2 RXDMAEN/TXDMAEN сброшены, SPI1->DR пуст.
 *   2. CS (PA4) = 0 и одиночный байт 0xA4 (0x80 | 0x24) опросом — открывается окно FIFO_DATA.
 *   3. DMA2 Stream 0/CH3: SPI1->DR -> dest_buf (len байт).
 *   4. DMA2 Stream 3/CH3: массив dummy-байтов -> SPI1->DR (len байт) — тактовые импульсы.
 *   5. TC потока приёма (stm32f4xx_it.c) -> bsp_capture_dma_complete() -> CS = 1.
 *
 * Обработка принятых байтов (заголовки 0x84/0x80/0x40, кольцевой буфер) на этом
 * этапе отсутствует — только доставка сырых данных.
 */

#include "bsp_capture.h"

#include "drv_bmi160.h"
#include "log_ring.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

/* Пара Stream + Channel обязана быть exactly из матрицы RM0090 (Docs/03 §1):
   неверная пара -> "0 байт передано" без какого-либо флага ошибки. */
#define CAPTURE_RX_DMA_INSTANCE DMA2_Stream0
#define CAPTURE_TX_DMA_INSTANCE DMA2_Stream3
#define CAPTURE_DMA_CHANNEL DMA_CHANNEL_3

/* Docs/03 §1: приоритет SPI1 строго выше SDIO (MEDIUM) — запись на карту
   никогда не должна задерживать чтение датчика. */
#define CAPTURE_DMA_PRIORITY DMA_PRIORITY_VERY_HIGH

/* PB5 = INT1 датчика, Active HIGH (Docs/01 §2, INT_OUT_CTRL = 0x0A) */
#define CAPTURE_INT1_PORT GPIOB
#define CAPTURE_INT1_PIN GPIO_PIN_5
#define CAPTURE_EXTI_IRQn EXTI9_5_IRQn
#define CAPTURE_EXTI_PREEMPT_PRIORITY 6 /* по заданию этапа M0 */
#define CAPTURE_DMA_PREEMPT_PRIORITY 5  /* выше EXTI: конец приёма не ждёт следующего FWM */

/* Команда чтения FIFO_DATA: 0x80 | 0x24 (Docs/02 §1). При чтении 0x24 адрес
   не инкрементируется — последовательно выгружается сам FIFO. */
#define BMI160_CMD_READ_FIFO_DATA 0xA4u
#define CAPTURE_DUMMY_BYTE 0xFFu

#define CAPTURE_SPI_TIMEOUT_MS 10u
/* Ждём сброса бита EN потока: заведомо больше времени последней памяти/DMA операции */
#define CAPTURE_DISABLE_GUARD 200000u

/* Флаги потока 0 (LIFCR) и потока 3 (LIFCR) — RM0090 DMA_LISR */
#define CAPTURE_RX_DMA_FLAGS                                                                                   \
    (DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4)
#define CAPTURE_TX_DMA_FLAGS                                                                                   \
    (DMA_FLAG_FEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TCIF3_7)

/* Заголовки фреймов BMI160 в FIFO (Docs/02_bmi160_ref.md §4) */
#define BMI160_FRAME_REGULAR_ACCEL 0x84u /* 1 + 6 байт {XL,XH,YL,YH,ZL,ZH} */
#define BMI160_FRAME_EMPTY 0x80u         /* конец валидных данных в FIFO     */
#define BMI160_FRAME_SKIP 0x40u          /* 1 + 1 байт счётчика пропусков    */
#define BMI160_FRAME_SENSORTIME 0x44u    /* 4 байта, игнорируем              */
#define BMI160_FRAME_FIFO_CONFIG 0x48u   /* 2 байта, игнорируем              */

static DMA_HandleTypeDef g_hdma_spi1_rx;
static DMA_HandleTypeDef g_hdma_spi1_tx;

/* Dummy-байты тактирования SPI1 (модуль BMI160 отдаёт данные только на фронтах SCLK).
   DMA-буфер -> обязателен D1 SRAM (Docs/01 §5). */
static uint8_t g_tx_dummy[BSP_CAPTURE_MAX_LEN] __attribute__((section(".d1ram"), aligned(4)));

static volatile uint32_t g_capture_errors;
static volatile uint32_t g_capture_skip_frames;

/* Вычитка в полёте: ставит bsp_capture_start_dma(), снимает
   DMA2_Stream0_IRQHandler (TC или ошибка). См. bsp_capture.h */
volatile bool g_dma_busy = false;

static void capture_fail(void)
{
    g_capture_errors++;
}

void bsp_capture_error_raise(void)
{
    capture_fail();
}

/**
 * Остановить поток и привести HAL-состояние в READY.
 * Наш DMA2_Stream0_IRQHandler снимает флаги сам и НЕ вызывает HAL_DMA_IRQHandler(),
 * поэтому hdma->State сбрасывается здесь явно — иначе HAL_DMA_Start_IT()
 * навсегда вернёт HAL_ERROR после первой вычитки.
 */
static bool capture_quiesce_stream(DMA_HandleTypeDef *hdma, uint32_t flags)
{
    uint32_t guard = 0u;

    __HAL_DMA_DISABLE(hdma);
    while ((hdma->Instance->CR & DMA_SxCR_EN) != 0u) {
        if (++guard >= CAPTURE_DISABLE_GUARD) {
            return false;
        }
    }

    __HAL_DMA_CLEAR_FLAG(hdma, flags);

    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    hdma->State = HAL_DMA_STATE_READY;

    return true;
}

/**
 * Освободить SPI1->DR от остатков предыдущей транзакции (Docs/02 §4 п.1).
 * Чтение DR снимает RXNE; чтение SR + DR снимает OVR (RM0090).
 */
static void capture_flush_spi_rx(void)
{
    uint32_t guard = 0u;

    while (((SPI1->SR & SPI_FLAG_RXNE) != 0u) || ((SPI1->SR & SPI_FLAG_OVR) != 0u)) {
        (void)SPI1->SR;
        (void)SPI1->DR;
        if (++guard >= 8u) {
            break;
        }
    }
}

void bsp_capture_dma_init(void)
{
    SPI_HandleTypeDef *hspi = drv_bmi160_spi_handle();

    __HAL_RCC_DMA2_CLK_ENABLE();

    for (uint32_t i = 0u; i < BSP_CAPTURE_MAX_LEN; i++) {
        g_tx_dummy[i] = CAPTURE_DUMMY_BYTE;
    }

    /* SPI1_RX: SPI1->DR -> память, инкремент только по памяти */
    g_hdma_spi1_rx.Instance = CAPTURE_RX_DMA_INSTANCE;
    g_hdma_spi1_rx.Init.Channel = CAPTURE_DMA_CHANNEL;
    g_hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    g_hdma_spi1_rx.Init.Priority = CAPTURE_DMA_PRIORITY;
    g_hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_hdma_spi1_rx) != HAL_OK) {
        capture_fail();
        return;
    }

    /* SPI1_TX: память -> SPI1->DR. Поток 3, Channel 3 — законная пара (Docs/03 §1) */
    g_hdma_spi1_tx.Instance = CAPTURE_TX_DMA_INSTANCE;
    g_hdma_spi1_tx.Init.Channel = CAPTURE_DMA_CHANNEL;
    g_hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    g_hdma_spi1_tx.Init.Priority = CAPTURE_DMA_PRIORITY;
    g_hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_hdma_spi1_tx) != HAL_OK) {
        capture_fail();
        return;
    }

    /* Ручки DMA привязываем к SPI1: корректное состояние HAL-драйвера SPI */
    hspi->hdmarx = &g_hdma_spi1_rx;
    hspi->hdmatx = &g_hdma_spi1_tx;

    (void)capture_quiesce_stream(&g_hdma_spi1_rx, CAPTURE_RX_DMA_FLAGS);
    (void)capture_quiesce_stream(&g_hdma_spi1_tx, CAPTURE_TX_DMA_FLAGS);
    SPI1->CR2 &= (uint32_t)~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    /* Прерывание ставим только на приём (Transfer Complete потока 0).
       Поток передачи завершается на такт позже и прерывания не требует. */
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, CAPTURE_DMA_PREEMPT_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void bsp_capture_exti_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB5: датчик выставил INT1 Active HIGH / push-pull (INT_OUT_CTRL = 0x0A),
       значит значимый фронт — нарастающий. Pull-down — страховка от ложных
       фронтов, когда датчик ещё не подключён (плата только приехала). */
    gpio.Pin = CAPTURE_INT1_PIN;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAPTURE_INT1_PORT, &gpio);

    /* Линии 6..9 на этом этапе не используются, но сбрасываем всю группу:
       мусор после POR не должен выстрелить первым же вызовом. */
    HAL_NVIC_ClearPendingIRQ(CAPTURE_EXTI_IRQn);
    HAL_NVIC_SetPriority(CAPTURE_EXTI_IRQn, CAPTURE_EXTI_PREEMPT_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(CAPTURE_EXTI_IRQn);
}

bool bsp_capture_start_dma(uint8_t *dest_buf, uint16_t len)
{
    SPI_HandleTypeDef *hspi = drv_bmi160_spi_handle();
    uint8_t cmd = BMI160_CMD_READ_FIFO_DATA;

    /* Reentrancy gate: предыдущая вычитка ещё в полёте (FWM мог подняться
       раньше, чем главный цикл разобрал пакет). Перезапуск потоков на середине
       транзакции изуродовал бы и буфер, и окно CS — отклоняем молча (false без
       инкремента ошибок: вызывающий не должен был считать, что данные пришли). */
    if (g_dma_busy) {
        return false;
    }

    if (dest_buf == NULL || len == 0u || len > BSP_CAPTURE_MAX_LEN) {
        capture_fail();
        return false;
    }

    g_dma_busy = true;
    g_dma_done = false;

    /* 1. Quiesce (Docs/02 §4 п.1) */
    SPI1->CR2 &= (uint32_t)~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    if (!capture_quiesce_stream(&g_hdma_spi1_rx, CAPTURE_RX_DMA_FLAGS) ||
        !capture_quiesce_stream(&g_hdma_spi1_tx, CAPTURE_TX_DMA_FLAGS)) {
        g_dma_busy = false;
        capture_fail();
        return false;
    }
    capture_flush_spi_rx();

    /* 2. CS = 0 и командный байт. HAL_SPI_Transmit() в 2-проводном режиме сам
       читает SR+DR в конце, т.е. оставляет RXNE/OVR сброшенными. */
    drv_bmi160_cs_write(true);
    if (HAL_SPI_Transmit(hspi, &cmd, 1u, CAPTURE_SPI_TIMEOUT_MS) != HAL_OK) {
        drv_bmi160_cs_write(false);
        capture_flush_spi_rx();
        g_dma_busy = false;
        capture_fail();
        return false;
    }

    /* CRITICAL FIX (M0 hardening): незакрытый pending-бит NVIC или старый
       TCIF0 потока 0 от прошлой транзакции выстрелили бы прерыванием в первые
       такты НОВОГО приёма и угнали бы его в bsp_capture_dma_complete() раньше
       времени. Снимаем и pending, и все флаги потока 0 в LIFCR (запись 1),
       непосредственно перед стартом потоков. */
    NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    DMA2->LIFCR = CAPTURE_RX_DMA_FLAGS;

    /* 3. Сначала включаем запрос и запускаем приём, только потом передачу:
       тактовые импульсы не должны появиться раньше готовности RX DMA. */
    SPI1->CR2 |= SPI_CR2_RXDMAEN;
    if (HAL_DMA_Start_IT(&g_hdma_spi1_rx, (uint32_t)&SPI1->DR, (uint32_t)dest_buf, len) != HAL_OK) {
        SPI1->CR2 &= (uint32_t)~SPI_CR2_RXDMAEN;
        drv_bmi160_cs_write(false);
        g_dma_busy = false;
        capture_fail();
        return false;
    }

    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    if (HAL_DMA_Start_IT(&g_hdma_spi1_tx, (uint32_t)&g_tx_dummy[0], (uint32_t)&SPI1->DR, len) != HAL_OK) {
        SPI1->CR2 &= (uint32_t)~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        (void)capture_quiesce_stream(&g_hdma_spi1_rx, CAPTURE_RX_DMA_FLAGS);
        drv_bmi160_cs_write(false);
        g_dma_busy = false;
        capture_fail();
        return false;
    }

    return true;
}

void bsp_capture_dma_complete(void)
{
    /* Приём завершён: SPI больше не должен просить DMA до следующей вычитки */
    SPI1->CR2 &= (uint32_t)~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    /* Транзакция BMI160 заканчивается фронтом CS = 1: без него датчик считает,
       что окно FIFO_DATA ещё открыто, и следующий командный байт будет
       воспринят как продолжение чтения 0x24. */
    drv_bmi160_cs_write(false);
}

/**
 * Разбор принятого пакета FIFO_DATA и складывание сэмплов в кольцо.
 * Вызывается из главного цикла после g_dma_done (Docs/05 §2 п.1).
 */
void bsp_capture_parse_fifo(uint8_t *buf, uint16_t len)
{
    uint16_t i = 0u;

    if (buf == NULL || len == 0u) {
        capture_fail();
        return;
    }

    while (i < len) {
        uint8_t hdr = buf[i];

        if (hdr == BMI160_FRAME_REGULAR_ACCEL) {
            int16_t x;
            int16_t y;
            int16_t z;

            if (((uint32_t)i + 7u) > (uint32_t)len) {
                break; /* неполный фрейм на конце пакета — ждём следующую вычитку */
            }

            /* {XL,XH,YL,YH,ZL,ZH} little-endian (Docs/02 §4, Docs/04 §3) */
            x = (int16_t)((uint16_t)buf[i + 1u] | ((uint16_t)buf[i + 2u] << 8));
            y = (int16_t)((uint16_t)buf[i + 3u] | ((uint16_t)buf[i + 4u] << 8));
            z = (int16_t)((uint16_t)buf[i + 5u] | ((uint16_t)buf[i + 6u] << 8));

            if (!ring_push_sample(x, y, z)) {
                capture_fail();
                return;
            }
            i = (uint16_t)(i + 7u);
        } else if (hdr == BMI160_FRAME_EMPTY) {
            break; /* Overread: Further валидных данных в пакете нет */
        } else if (hdr == BMI160_FRAME_SKIP) {
            if (((uint32_t)i + 2u) > (uint32_t)len) {
                break;
            }
            g_capture_skip_frames++; /* SENSOR_OVR для заголовка блока */
            i = (uint16_t)(i + 2u);
        } else if (hdr == BMI160_FRAME_SENSORTIME) {
            if (((uint32_t)i + 4u) > (uint32_t)len) {
                break;
            }
            i = (uint16_t)(i + 4u);
        } else if (hdr == BMI160_FRAME_FIFO_CONFIG) {
            if (((uint32_t)i + 2u) > (uint32_t)len) {
                break;
            }
            i = (uint16_t)(i + 2u);
        } else {
            break; /* заголовка нет в списке: сбой SPI или потеря синхронизации */
        }
    }
}

DMA_HandleTypeDef *bsp_capture_rx_dma_handle(void)
{
    return &g_hdma_spi1_rx;
}

DMA_HandleTypeDef *bsp_capture_tx_dma_handle(void)
{
    return &g_hdma_spi1_tx;
}

uint32_t bsp_capture_skip_frame_count(void)
{
    return g_capture_skip_frames;
}

uint32_t bsp_capture_error_count(void)
{
    return g_capture_errors;
}
