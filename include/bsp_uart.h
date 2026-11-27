/**
 * bsp_uart.h — отладочный UART1: PA9 = TX, PA10 = RX (AF7), 115200 8N1
 * (Docs/01_hardware_and_build.md §2, Docs/06_error.md §4).
 *
 * К UART1 подключается WeAct MiniDebugger (его же SWD: PA13/PA14).
 *
 * Работа БЕЗ прерываний и БЕЗ DMA: передача блокирующая (HAL_UART_Transmit
 * опрашивает флаг TXE с таймаутом). Это сознательно: DMA2 Stream6/Channel 4
 * по матрице RM0090 = USART1_TX и уже занят SDIO (Docs/03 §1), а прерывание
 * USART1 только мешало бы захвату датчика.
 *
 * bsp_uart_printf() НЕ безопасна из ISR: общий статический буфер формата и
 * блокирующая передача. Вызывать только из контекста main().
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Инициализация USART1 (PA9/PA10, AF7, 115200 8N1, 16x oversampling).
 * Повторный вызов после успеха ничего не делает.
 *
 * Функция возвращает void (лог — вспомогательное средство, прошивка обязана
 * работать и без него), но сбой не молчит: он поднимает счётчик, доступный
 * через bsp_uart_error_count(), и гасит bsp_uart_ready(), поэтому весь
 * последующий лог возвращает -1, а диагностика остаётся на вспышках PB2
 * (Docs/06 §3).
 */
void bsp_uart_init(void);

/** true если HAL_UART_Init() прошёл успешно. */
bool bsp_uart_ready(void);

/** Число сбоев UART: таймаут инициализации, таймаут передачи, обрезка строки. */
uint32_t bsp_uart_error_count(void);

/**
 * Форматирование СВОИМ минимальным форматтером (без <stdio.h>, без newlib и
 * без единой аллокации — аудит M1 K2) в статический буфер и блокирующая
 * передача в UART1. Поддерживаются %s, %c, %d, %u, %x, %X, %% и ширина
 * с нулевым заполнителем ("%02X"); float и модификаторы длины — нет.
 * Возвращает число отправленных байт; -1 — UART не готов, ошибка формата,
 * обрезка строки или таймаут передачи (сбой всегда поднимает счётчик).
 * Атрибут format(printf) оставляет проверку пары «спецификация/аргумент»
 * за компилятором (-Wformat).
 */
int bsp_uart_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
