/**
 * stm32f4xx_it.h — обработчики прерываний прошивки (src/stm32f4xx_it.c).
 */

#ifndef STM32F4XX_IT_H
#define STM32F4XX_IT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** SysTick 1 мс — обязателен, иначе HAL_Delay() зависнет намертво. */
void SysTick_Handler(void);

/** Фронт INT1 датчика (PB5, порог FIFO FWM) — выставляется из EXTI9_5_IRQHandler. */
extern volatile bool g_int1_triggered;

/** Приём FIFO по SPI1_RX DMA завершён — выставляется из DMA2_Stream0_IRQHandler. */
extern volatile bool g_dma_done;

/* Заглушки на периферию будущих этапов (BMI160 FIFO / DMA2 / SDIO):
   только снятие флагов. Без них случайное срабатывание уходит в .weak
   бесконечный цикл из startup_stm32f405xx.S. */
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);
void SPI1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_IT_H */
