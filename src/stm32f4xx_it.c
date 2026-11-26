/**
 * stm32f4xx_it.c — обработчики прерываний уровня приложения.
 *
 * Штатный startup_stm32f405xx.S объявляет все обработчики как .weak,
 * поэтому определённые здесь функции перекрывают бесконечные заглушки.
 */

#include "stm32f4xx_it.h"

#include "stm32f4xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}
