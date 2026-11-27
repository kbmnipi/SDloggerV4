/**
 * main.c — этап M0: тактирование 84 МГц + BMI160 + захват FIFO по SPI1 DMA.
 *
 * Порядок инициализации (порядок фиксирован аудитом M0):
 *   1) bsp_meminit()              — обнуление CCM (секция .ccmram — NOLOAD)
 *   2) HAL_Init()                 — SysTick, приоритет grouped, NVIC
 *   3) led_init()                 — PB2 ДО тактирования: при сбое HSE/PLL код 1
 *                                  должен мигать уже настроенным выходом
 *   4) system_clock_84mhz_hse()   — любой сбой -> fault_blink_code1()
 *   5) drv_bmi160_spi_init()      — false -> fault_blink_code1()
 *   6) drv_bmi160_init_sensor()   — CHIP_ID != 0xD1 -> fault_blink_code1()
 *   7) ring_init()                — кольцо 16 x 4096 байт в D1 SRAM
 *   8) drv_bmi160_start_accel()   — Normal Mode + 1600 Гц / OSR2 / +/-16 g
 *   9) drv_bmi160_configure_fifo()— FIFO watermark 56 Б, FWM -> INT1 (Docs/02 §3)
 *  10) bsp_capture_dma_init()     — DMA2 Stream0/CH3 RX + Stream3/CH3 TX (Docs/03 §1)
 *  11) bsp_capture_exti_init()    — PB5 EXTI, приоритет 6
 *  12) bsp_capture_error_count()  — любой сбой DMA/EXTI -> fault_blink_code1()
 *  13) bsp_sdio_init()            — SDIO 4-bit 24 МГц + DMA2 Stream6/CH4 -> код 2
 *  14) f_mount(&fs, "", 1)        — монтирование FAT32 -> код 2 при ошибке
 *
 * Главный цикл: фронт INT1 -> динамика FIFO_LENGTH -> вычитка N = min(байт, 252)
 * по DMA -> разбор фреймов 0x84 -> 680 сэмплов в текущий блок кольца
 * (Docs/02 §4, Docs/05 §2 п.1).
 *
 * Индикация (Docs/06_error.md):
 *   карта смонтирована  -> 3 быстрые вспышки PB2, далее PB2 дёргается на каждый
 *                           разобранный пакет FIFO
 *   код 1 (1 вспышка/3 с) -> нет ответа от акселерометра либо сбой SPI1/DMA2
 *   код 2 (2 вспышки/3 с) -> SDIO не поднялся или f_mount вернул ошибку
 *
 * Создание файла, запись блоков по 4096 байт, заголовок блока (block_seq/CRC32)
 * и кнопка PC13 на этом этапе сознательно НЕ подключаются.
 */

#include "stm32f4xx_hal.h"

#include "drv_bmi160.h"
#include "bsp_capture.h"
#include "bsp_sdio.h"
#include "log_ring.h"
#include "stm32f4xx_it.h"

#include "ff.h"

#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_2

/* Docs/06_error.md §3: N вспышек по 200 мс горит / 200 мс пауза, затем длинная
   пауза до конца цикла в 3 с */
#define ERROR_FLASH_ON_MS 200u
#define ERROR_FLASH_OFF_MS 200u
#define ERROR_CYCLE_MS 3000u

/* Быстрые вспышки подтверждения: карта смонтирована (этап M1) */
#define MOUNT_OK_FLASHES 3u
#define MOUNT_OK_FLASH_MS 100u

/* Приёмный буфер вычитки FIFO_DATA (0x24). DMA2 видит только D1 SRAM,
   поэтому буфер обязан лежать в .d1ram (Docs/01 §5, Docs/03 §1). */
static uint8_t spi_rx_buf[BSP_CAPTURE_BUF_SIZE] __attribute__((section(".d1ram"), aligned(4)));

/* Контекст тома FatFs: FATFS + FIL содержат по секторному буферу 512 Б,
   поэтому стоят в .bss (D1 SRAM), а не в CCM (Docs/01 §5) */
static FATFS fs;

/* Счётчик SENSOR_OVR: FIFO датчика было почти полным (>= 1015 байт) и часть
   данных могла потеряться. Флаг OVERFLOW/SENSOR_OVR пойдёт в заголовок блока
   на этапе записи (Docs/04 §3). */
static volatile uint32_t g_sensor_ovr_count;

/* Границы секции .ccmram из scripts/STM32F405RGT6_CCM.ld. Секция NOLOAD:
   startup_stm32f405xx.S обнуляет только _sbss.._ebss в D1 SRAM и до CCM
   не доходит, поэтому CCM обнуляем явно (Docs/01 §5). */
extern uint32_t _sccmram;
extern uint32_t _eccmram;

/**
 * Код ошибки N вспышками (Docs/06_error.md §3): N вспышек по 200 мс, затем
 * пауза до конца цикла в 3 с. Бесконечно, пока питание не перезапустили.
 *   1 — нет ответа от акселерометра (Docs/06 §3)
 *   2 — проблема с SD-картой: SDIO не инициализирован либо f_mount с ошибкой
 */
static void fault_blink_code(uint32_t flash_count)
{
    while (1) {
        for (uint32_t i = 0u; i < flash_count; i++) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(ERROR_FLASH_ON_MS);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(ERROR_FLASH_OFF_MS);
        }
        HAL_Delay(ERROR_CYCLE_MS - (flash_count * (ERROR_FLASH_ON_MS + ERROR_FLASH_OFF_MS)));
    }
}

static void fault_blink_code1(void)
{
    fault_blink_code(1u);
}

static void fault_blink_code2(void)
{
    /* Docs/06_error.md §3: карта не вставлена, SDIO не поднялся или не FAT32 */
    fault_blink_code(2u);
}

/**
 * HSE 8 МГц -> SYSCLK 84 МГц (PLL M=8, N=336, P=4, Q=7), AHB 84, APB1 42,
 * APB2 84 (SPI1), PLL48CK 48 МГц (будущий SDIO). Динамическое переключение
 * частоты запрещено — сбивается тактирование SDIO.
 */
static void system_clock_84mhz_hse(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        fault_blink_code1();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2; /* 42 МГц: TIM2 гасится до 1 МГц */
    clk.APB2CLKDivider = RCC_HCLK_DIV1; /* 84 МГц: SPI1 = 84 / 16 = 5.25 МГц */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        fault_blink_code1();
    }
}

static void led_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/**
 * Обнулить CCM (0x10000000): секция .ccmram объявлена NOLOAD и лежит вне
 * диапазана обнуления startup (_sbss.._ebss — только D1 SRAM). Любое
 * состояние в CCM обязано стартовать с нуля, а не с мусора после POR.
 * Сейчас в .ccmram ничего не размещено (диапазон пуст), функция работает
 * и на последующих этапах (таблица CRC32, состояние FIFO).
 */
static void bsp_meminit(void)
{
    uint32_t *ccm = &_sccmram;

    while (ccm < &_eccmram) {
        *ccm++ = 0u;
    }
}

int main(void)
{
    uint8_t chip_id;
    /* Длина последнего принятого по DMA пакета FIFO — с ней и разбираем */
    uint16_t capture_len = 0u;

    bsp_meminit();
    HAL_Init();

    /* PB2 настраиваем до тактирования: сбой HSE/PLL должен отмигать код 1
       уже настроенным выходом (Docs/06_error.md §3) */
    led_init();

    system_clock_84mhz_hse();

    if (!drv_bmi160_spi_init()) {
        fault_blink_code1();
    }

    chip_id = drv_bmi160_init_sensor();

    if (chip_id != BMI160_CHIP_ID) {
        fault_blink_code1();
    }

    ring_init();

    /* Акселерометр в Normal Mode + профиль 1600 Гц / OSR2 / +/-16 g (Docs/02 §4 п.5).
       Строго ДО конфигурации FIFO: без Normal Mode FIFO не заполняется и
       порог FWM никогда не поднимает INT1 (PB5). */
    if (!drv_bmi160_start_accel()) {
        fault_blink_code1();
    }

    /* FIFO: watermark 56 байт, acc+header, FWM -> INT1 (PB5), Flush FIFO */
    if (!drv_bmi160_configure_fifo()) {
        fault_blink_code1();
    }

    bsp_capture_dma_init();  /* DMA2 Stream0/CH3 = SPI1_RX, Stream3/CH3 = SPI1_TX */
    bsp_capture_exti_init(); /* PB5 = EXTI5, нарастающий фронт, NVIC приоритет 6  */

    if (bsp_capture_error_count() != 0u) {
        fault_blink_code1();
    }

    /* SDIO 4-bit + DMA2 Stream6/CH4 (Docs/03 §1-2), код 2 при любой проблеме
       с картой (Docs/06 §3) */
    if (!bsp_sdio_init()) {
        fault_blink_code2();
    }

    /* f_mount(&fs, "", 1): третий аргумент 1 = монтировать том сразу, т.е.
       читать MBR/BOOT-сектор прямо здесь. Путь "" = логический диск 0
       (_VOLUMES = 1, ffconf.h) */
    if (f_mount(&fs, "", 1u) != FR_OK) {
        fault_blink_code2();
    }

    /* 3 быстрые вспышки PB2 — карта смонтирована (Docs/06 §2: штатно IDLE
       мигает раз в 2.5 с, но состояние кнопки PC13 появляется позже) */
    for (uint32_t i = 0u; i < MOUNT_OK_FLASHES; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(MOUNT_OK_FLASH_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(MOUNT_OK_FLASH_MS);
    }

    while (1) {
        if (g_int1_triggered) {
            uint16_t fifo_bytes;
            uint16_t read_len;

            g_int1_triggered = false;

            /* Docs/02 §4 п.2: сначала динамика FIFO, потом вычитка */
            fifo_bytes = drv_bmi160_get_fifo_length();
            if (fifo_bytes > BMI160_FIFO_LENGTH_MAX) {
                /* 0xFFFF: линия SPI1 сдохла — вычитку не запускаем, датчик не трогаем */
                continue;
            }
            if (fifo_bytes == 0u) {
                continue; /* Docs/02 §4 п.3: прерывание снято, читать нечего */
            }
            if (fifo_bytes >= BSP_CAPTURE_FIFO_OVR_BYTES) {
                g_sensor_ovr_count++; /* SENSOR_OVR в заголовок блока, Docs/04 §3 */
            }

            /* Docs/02 §4 п.4: N = min(fifo_bytes, 252) */
            read_len = (fifo_bytes > (uint16_t)BSP_CAPTURE_MAX_LEN) ? (uint16_t)BSP_CAPTURE_MAX_LEN
                                                                    : fifo_bytes;

            bsp_capture_start_dma(spi_rx_buf, read_len);
            if (bsp_capture_error_count() != 0u) {
                fault_blink_code1();
            }
            capture_len = read_len;
        }

        if (g_dma_done) {
            g_dma_done = false;

            /* Разбор принятого пакета и складывание {x, y, z} в текущий слот
               кольца (Docs/02 §4, Docs/05 §2 п.1) */
            bsp_capture_parse_fifo(spi_rx_buf, capture_len);
            if (bsp_capture_error_count() != 0u) {
                fault_blink_code1();
            }

            /* PB2 дёргается на каждый разобранный пакет — визуальная
               проверка всего цикла: INT1 -> DMA -> разбор -> кольцо */
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
    }
}
