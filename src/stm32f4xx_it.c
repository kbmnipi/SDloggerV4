/**
 * stm32f4xx_it.c — обработчики прерываний уровня приложения.
 *
 * Штатный startup_stm32f405xx.S объявляет все обработчики как .weak,
 * поэтому определённые здесь функции перекрывают бесконечные заглушки.
 */

#include "stm32f4xx_it.h"

#include "stm32f4xx_hal.h"

/* Линии EXTI, которые будут заняты: PB5 = BMI160 INT1 (Docs/01 §2).
   Маска совпадает с номерами линий EXTI9..5 (GPIO_PIN_x == EXTI line x). */
#define IT_EXTI9_5_LINES (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)

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

/* ---- безопасные заглушки на периферию будущих этапов (FIFO/DMA) ----
   Пока NVIC этих периферий не включён, обработчики не вызываются. Если шум
   на линии INT1 или случайный флаговый сброс поднимет прерывание, вектор не
   должен уйти в .weak бесконечный цикл startup: флаги снимаются, выход.
   На этапе FIFO/DMA сюда придёт настоящая логика (кольцевой буфер, DMA). */

void EXTI9_5_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(IT_EXTI9_5_LINES);
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
