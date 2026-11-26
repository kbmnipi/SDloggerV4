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

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_IT_H */
