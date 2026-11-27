/**
 * stm32f4xx_it.c — обработчики прерываний уровня приложения.
 *
 * Штатный startup_stm32f405xx.S объявляет все обработчики как .weak,
 * поэтому определённые здесь функции перекрывают бесконечные заглушки.
 */

#include "stm32f4xx_it.h"

#include "stm32f4xx_hal.h"

#include "bsp_capture.h"
#include "bsp_sdio.h"

/* Фронт INT1 (PB5) и конец приёма FIFO по DMA — единственные данные, которые
   прерывания передают главному циклу. Сбрасывает их main.c. */
volatile bool g_int1_triggered;
volatile bool g_dma_done;

/* Линия BMI160 INT1: PB5 = линия EXTI5 (GPIO_PIN_x == номер линии EXTI, Docs/01 §2) */
#define IT_EXTI_INT1_LINE GPIO_PIN_5
#define IT_EXTI9_5_OTHER_LINES (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)

/* DMA2 Stream 0 = SPI1_RX (Docs/03 §1): флаги потока 0 сидят в DMA2_LIFCR */
#define IT_DMA_S0_FLAGS (DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4)

/* Флаги потока (RM0090): у DMA1 и DMA2 позиции битов одинаковые — флаги
   потоков 0..3 в LIFCR, потоков 4..7 в HIFCR. HAL покрывает обе половины
   одной группой масок (DMA_FLAG_FEIF3_7 = поток 3 в LIFCR или поток 7 в HIFCR).
   Заглушки: DMA1 Stream2/3 — альтернативная пара SPI1_RX/SPI1_TX по RM0090,
   DMA2 Stream3 — SPI1_TX по Docs/03 §1. */
#define IT_DMA_S2_FLAGS                                                                                  \
    (DMA_FLAG_FEIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_TEIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TCIF2_6)
#define IT_DMA_S3_FLAGS                                                                                  \
    (DMA_FLAG_FEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TCIF3_7)

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * PB5 = BMI160 INT1 — порог заполнения FIFO (FWM), Active HIGH (Docs/01 §2,
 * Docs/02 §3: INT_MAP_1 = 0x40, INT_OUT_CTRL = 0x0A).
 * Обработчик только поднимает флаг: вычитку запускает главный цикл, чтобы
 * прерывание датчика оставалось коротким и не мешало DMA SDIO.
 */
void EXTI9_5_IRQHandler(void)
{
    /* EXTI_PR: запись 1 в бит снимает флаг */
    __HAL_GPIO_EXTI_CLEAR_IT(IT_EXTI_INT1_LINE);
    g_int1_triggered = true;

    /* Линии 6..9 на этом этапе не задействованы: снимаем и их флаги, иначе
       чужой флаг группы будет поднимать прерывание повторно. */
    __HAL_GPIO_EXTI_CLEAR_IT(IT_EXTI9_5_OTHER_LINES);
}

/**
 * DMA2 Stream 0 = SPI1_RX: все байты FIFO_DATA приняты (Docs/03 §1).
 * Флаги снимаем сами (HAL_DMA_IRQHandler здесь не вызывается), поэтому
 * bsp_capture_start_dma() приводит состояние потоков в READY сам.
 */
void DMA2_Stream0_IRQHandler(void)
{
    /* DMA2_LIFCR: запись 1 в бит Cxxx снимает флаг потока */
    DMA2->LIFCR = IT_DMA_S0_FLAGS;

    /* Снимает DMA-запросы SPI1 и поднимает CS = 1 — конец транзакции BMI160 */
    bsp_capture_dma_complete();

    g_dma_done = true;
}

/* ---- безопасные заглушки на периферию будущих этапов (TIM2, SDIO) ----
   Пока NVIC этих периферий не включён, обработчики не вызываются. Если шум
   или случайный флаговый сброс поднимет прерывание, вектор не должен уйти в
   .weak бесконечный цикл startup: флаги снимаются, выход. */

/**
 * SDIO (запись/чтение блоков MicroSD). Диспетчер HAL обязателен: именно
 * HAL_SD_IRQHandler() доводит конечный автомат до HAL_SD_STATE_READY и
 * останавливает DMA SDIO. Обработчик, который только снимает флаги, оставил бы
 * hsd->State == BUSY навсегда, и каждая следующая запись вернула бы HAL_BUSY.
 * HAL сам снимает флаги SDIO (SDIO_ICR / DICLR) и читает STAT/SR при ошибках.
 */
void SDIO_IRQHandler(void)
{
    HAL_SD_IRQHandler(bsp_sdio_handle());
}

/**
 * DMA2 Stream 6 = SDIO (Docs/03 §1). HAL_DMA_IRQHandler() снимает флаги
 * LIFCR/HIFCR потока и вызывает колбэки SD_DMATransmitCplt / SD_DMAError,
 * которые дальше работают через HAL_SD_IRQHandler.
 */
void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(bsp_sdio_dma_handle());
}

void TIM2_IRQHandler(void)
{
    /* TIM_SR: флаг снимается записью 0 в соответствующий бит */
    TIM2->SR = 0u;
}

void SPI1_IRQHandler(void)
{
    /* RM0090: OVR/MODF/CRCERR снимаются чтением SPI_SR с последующим чтением SPI_DR */
    (void)SPI1->SR;
    (void)SPI1->DR;
}

void DMA1_Stream2_IRQHandler(void)
{
    /* DMA_LIFCR: запись 1 в бит Cxxx снимает флаг потока */
    DMA1->LIFCR = IT_DMA_S2_FLAGS;
}

void DMA1_Stream3_IRQHandler(void)
{
    DMA1->LIFCR = IT_DMA_S3_FLAGS;
}

void DMA2_Stream3_IRQHandler(void)
{
    DMA2->LIFCR = IT_DMA_S3_FLAGS;
}
