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

/* ===========================================================================
 * Минимальный форматтер вместо vsnprintf() (аудит M1, находка K2).
 *
 * vsnprintf() из newlib-nano тянет в образ цепочку _svfprintf_r -> _malloc_r
 * -> _sbrk (подтверждено firmware.map до правки): куча при _Min_Heap_Size = 0
 * выросла бы от _end прямо в резерв стека MSP. Свой форматтер не аллоцирует
 * ничего и пишется строго в g_uart_buf.
 *
 * Поддерживается ровно то, что нужно логу: %s, %c, %d, %u, %x, %X, %% и
 * опциональные заполнитель/ширина ("%02X"). Модификаторы длины, '+'/'-',
 * точность и float НЕ поддерживаются: неизвестная спецификация печатается
 * дословно (сбой виден в логе), а не съедает аргумент молча.
 *
 * Все write-функции принимают счётчик n: он всегда растёт, а в buf попадает
 * только первые keep байт. На выходе n — длина строки БЕЗ учёта обрезки,
 * поэтому обрезка обнаруживается сравнением n c размером буфера.
 * =========================================================================== */

/* 32-битное беззнаковое в десятичном: максимум 10 цифр + запас */
#define UART_FMT_MAX_DIGITS 11u

static void uart_fmt_putc(uint32_t *n, char *buf, uint32_t keep, char ch)
{
    if (*n < keep) {
        buf[*n] = ch;
    }
    (*n)++;
}

static void uart_fmt_puts(uint32_t *n, char *buf, uint32_t keep, const char *s)
{
    if (s == NULL) {
        s = "(null)";
    }
    while (*s != '\0') {
        uart_fmt_putc(n, buf, keep, *s++);
    }
}

static void uart_fmt_putu(uint32_t *n, char *buf, uint32_t keep, uint32_t value, uint32_t base,
                          bool upper, char pad, uint32_t width)
{
    char tmp[UART_FMT_MAX_DIGITS];
    uint32_t num = 0u;

    /* цифры пишем от младшего разряда, выводим в обратном порядке */
    do {
        uint32_t rem = value % base;

        value /= base;
        if (rem < 10u) {
            tmp[num] = (char)('0' + (int)rem);
        } else {
            tmp[num] = upper ? (char)('A' + (int)(rem - 10u)) : (char)('a' + (int)(rem - 10u));
        }
        num++;
    } while (value != 0u);

    while (num < width) {
        tmp[num] = pad;
        num++;
    }

    while (num > 0u) {
        num--;
        uart_fmt_putc(n, buf, keep, tmp[num]);
    }
}

static void uart_fmt_puti(uint32_t *n, char *buf, uint32_t keep, int32_t value, char pad,
                          uint32_t width)
{
    if (value < 0) {
        uart_fmt_putc(n, buf, keep, '-');
        if (width > 0u) {
            width--;
        }
        /* (uint32_t)0 - (uint32_t)value: вычитание беззнаковое, потому что
           -(-2147483648) не представимо в int32_t (UB знаковое) */
        uart_fmt_putu(n, buf, keep, (uint32_t)0u - (uint32_t)value, 10u, false, pad, width);
    } else {
        uart_fmt_putu(n, buf, keep, (uint32_t)value, 10u, false, pad, width);
    }
}

static uint32_t uart_fmt(char *buf, uint32_t cap, const char *format, va_list args)
{
    uint32_t n = 0u;
    uint32_t keep = cap - 1u; /* последний байт всегда под терминирующий '\0' */

    while (*format != '\0') {
        char fmt = *format++;
        char pad = ' ';
        uint32_t width = 0u;

        if (fmt != '%') {
            uart_fmt_putc(&n, buf, keep, fmt);
            continue;
        }

        fmt = *format++;
        if (fmt == '\0') {
            break;
        }
        if (fmt == '%') {
            uart_fmt_putc(&n, buf, keep, '%');
            continue;
        }

        /* необязательные '0' и цифры ширины: %02X -> pad='0', width=2 */
        while ((fmt == '0') || ((fmt >= '1') && (fmt <= '9'))) {
            if (fmt == '0') {
                pad = '0';
            }
            if (width < 99u) {
                width = (width * 10u) + (uint32_t)(fmt - '0');
            }
            fmt = *format++;
            if (fmt == '\0') {
                break;
            }
        }
        if (fmt == '\0') {
            break;
        }

        switch (fmt) {
        case 's':
            uart_fmt_puts(&n, buf, keep, va_arg(args, const char *));
            break;
        case 'c':
            uart_fmt_putc(&n, buf, keep, (char)va_arg(args, int));
            break;
        case 'd':
            uart_fmt_puti(&n, buf, keep, (int32_t)va_arg(args, int), pad, width);
            break;
        case 'u':
            uart_fmt_putu(&n, buf, keep, va_arg(args, unsigned int), 10u, false, pad, width);
            break;
        case 'x':
            uart_fmt_putu(&n, buf, keep, va_arg(args, unsigned int), 16u, false, pad, width);
            break;
        case 'X':
            uart_fmt_putu(&n, buf, keep, va_arg(args, unsigned int), 16u, true, pad, width);
            break;
        default:
            uart_fmt_putc(&n, buf, keep, '%');
            uart_fmt_putc(&n, buf, keep, fmt);
            break;
        }
    }

    buf[(n < keep) ? n : keep] = '\0';
    return n;
}

int bsp_uart_printf(const char *format, ...)
{
    va_list args;
    int len;

    if (!g_uart_ready) {
        return -1;
    }

    va_start(args, format);
    len = (int)uart_fmt(g_uart_buf, (uint32_t)sizeof(g_uart_buf), format, args);
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
