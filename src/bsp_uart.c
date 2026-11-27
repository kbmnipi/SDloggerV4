/**
 * bsp_uart.c — отладочный UART1 (PA9 TX / PA10 RX, AF7, 115200 8N1)
 * для отладочного вывода по Docs/06_error.md §4.
 *
 * ПЕРЕДАЧА БЛОКИРУЮЩАЯ: HAL_UART_Transmit() опрашивает флаг TXE до конца
 * строки или до таймаута. Прерывание USART1 и DMA не используются, NVIC для
 * USART1 не настраивается: DMA2 Stream6/CH4 (USART1_TX по матрице RM0090)
 * занят SDIO, а приоритет захвата датчика (EXTI INT1 = 6, SPI1_RX DMA = 5)
 * не должен ни от чего зависеть.
 *
 * Тактирование: USART1 висит на APB2 = 84 МГц, поэтому bsp_uart_init()
 * обязан вызываться ПОСЛЕ system_clock_84mhz_hse() — иначе BRR считается
 * от внутренней 16 МГц HSI и скорость окажется неверной.
 */

#include "bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

/* PA9 = TX, PA10 = RX, AF7 (Docs/01 §2) */
#define UART_INSTANCE USART1
#define UART_GPIO_PORT GPIOA
#define UART_TX_PIN GPIO_PIN_9
#define UART_RX_PIN GPIO_PIN_10
#define UART_GPIO_AF GPIO_AF7_USART1

/* 115200 8N1 (Docs/01 §2) */
#define UART_BAUD_RATE 115200u

/* Таймаут отправки одной строки: 128 байт на 115200 8N1 = ~11 мс,
   100 мс — запас на случай висящей линии без терминала */
#define UART_TX_TIMEOUT_MS 100u

/* Максимальная длина строки лога вместе с '\0' */
#define UART_PRINT_BUF_SIZE 128u

static UART_HandleTypeDef g_huart;

/* Один общий буфер формата: контекст main(), из ISR не вызывать */
static char g_uart_buf[UART_PRINT_BUF_SIZE];

static bool g_uart_ready;
static volatile uint32_t g_uart_error_count;

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance != UART_INSTANCE) {
        return;
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TX и RX: GPIO_PULLUP — линия не «висит» в воздух, когда MiniDebugger
       отключён, и на RX не ловятся ложные старт-биты */
    gpio.Pin = UART_TX_PIN | UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = UART_GPIO_AF;
    HAL_GPIO_Init(UART_GPIO_PORT, &gpio);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART_INSTANCE) {
        return;
    }

    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(UART_GPIO_PORT, UART_TX_PIN | UART_RX_PIN);
}

void bsp_uart_init(void)
{
    if (g_uart_ready) {
        return;
    }

    g_huart.Instance = UART_INSTANCE;
    g_huart.Init.BaudRate = UART_BAUD_RATE;
    g_huart.Init.WordLength = UART_WORDLENGTH_8B;
    g_huart.Init.StopBits = UART_STOPBITS_1;
    g_huart.Init.Parity = UART_PARITY_NONE;
    g_huart.Init.Mode = UART_MODE_TX_RX;
    g_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&g_huart) != HAL_OK) {
        /* Сбой не молчит: готовность снята (весь лог вернёт -1), счётчик
           доступен по bsp_uart_error_count() */
        g_uart_ready = false;
        g_uart_error_count++;
        return;
    }

    g_uart_ready = true;
}

bool bsp_uart_ready(void)
{
    return g_uart_ready;
}

uint32_t bsp_uart_error_count(void)
{
    return g_uart_error_count;
}

int bsp_uart_printf(const char *format, ...)
{
    va_list args;
    int len;

    if (!g_uart_ready) {
        return -1;
    }

    va_start(args, format);
    len = vsnprintf(g_uart_buf, (int)sizeof(g_uart_buf), format, args);
    va_end(args);

    if (len <= 0) {
        g_uart_error_count++;
        return -1;
    }

    if ((uint32_t)len >= sizeof(g_uart_buf)) {
        /* Строка не влезла: печатаем усечённой, но сбой считаем — иначе
           обрезанный лог выглядит как полный */
        len = (int)sizeof(g_uart_buf) - 1;
        g_uart_error_count++;
    }

    if (HAL_UART_Transmit(&g_huart, (const uint8_t *)g_uart_buf, (uint16_t)len,
                          UART_TX_TIMEOUT_MS) != HAL_OK) {
        g_uart_error_count++;
        return -1;
    }

    return len;
}
