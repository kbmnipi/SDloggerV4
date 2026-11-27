/**
 * stm32f4xx_it.h — обработчики прерываний прошивки (src/stm32f4xx_it.c).
 */

#ifndef STM32F4XX_IT_H
#define STM32F4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/** SysTick 1 мс — обязателен, иначе HAL_Delay() зависнет намертво. */
void SysTick_Handler(void);

/* Заглушки на периферию будущих этапов (BMI160 FIFO / DMA2 / SDIO):
   только снятие флагов. Без них случайное срабатывание уходит в .weak
   бесконечный цикл из startup_stm32f405xx.S. */
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);
void SPI1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_IT_H */
