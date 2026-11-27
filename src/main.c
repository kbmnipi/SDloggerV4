/**
 * main.c — этап M1, настольная проверка платы: тактирование 84 МГц + BMI160 +
 * SPI1 DMA захват + SDIO/FatFs + тестовая запись одного блока 4096 байт.
 *
 * Порядок инициализации (порядок фиксирован аудитом M0):
 *   1)  bsp_meminit()              — обнуление CCM (секция .ccmram — NOLOAD)
 *   2)  HAL_Init()                 — SysTick, приоритет grouped, NVIC
 *   3)  led_init()                 — PB2 ДО тактирования: при сбое HSE/PLL код 1
 *                                    должен мигать уже настроенным выходом
 *   4)  system_clock_84mhz_hse()   — любой сбой -> fault_blink_code1()
 *   5)  bsp_uart_init()            — отладочный лог UART1 PA9/PA10 (Docs/06 §4),
 *                                    строго после тактирования: BRR считается
 *                                    от APB2 = 84 МГц
 *   6)  drv_bmi160_spi_init()      — false -> fault_blink_code1()
 *   7)  drv_bmi160_init_sensor()   — CHIP_ID != 0xD1 -> fault_blink_code1()
 *   8)  ring_init()                — кольцо 16 x 4096 байт в D1 SRAM
 *   9)  drv_bmi160_start_accel()   — Normal Mode + 1600 Гц / OSR2 / +/-16 g
 *   10) drv_bmi160_configure_fifo()— FIFO watermark 56 Б, FWM -> INT1 (Docs/02 §3)
 *   11) bsp_capture_dma_init()     — DMA2 Stream0/CH3 RX + Stream3/CH3 TX (Docs/03 §1)
 *   12) bsp_capture_exti_init()    — PB5 EXTI, приоритет 6
 *   13) bsp_capture_error_count()  — любой сбой DMA/EXTI -> fault_blink_code1()
 *   14) bsp_sdio_init()            — SDIO 4-bit 24 МГц + DMA2 Stream6/CH4 -> код 2
 *   15) f_mount(&fs, "", 1)        — монтирование FAT32 -> код 2 при ошибке
 *
 * Настольный тест (контейнер VLOG пока НЕ делается): TEST.BIN открывается на
 * FA_CREATE_ALWAYS|FA_WRITE, в него пишется ОДИН блок 4096 байт из D1 SRAM
 * (16 байт заголовка заполнены 0xAA, далее сквозной счётчик), далее f_sync +
 * f_close, 4 быстрые вспышки PB2 и остановка в while(1). Цикл захвата
 * датчика на этом этапе сознательно НЕ запускается.
 *
 * Индикация (Docs/06_error.md):
 *   карта смонтирована  -> 3 быстрые вспышки PB2
 *   тест записи пройден  -> 4 быстрые вспышки PB2, затем остановка
 *   код 1 (1 вспышка/3 с) -> нет ответа от акселерометра либо сбой SPI1/DMA2
 *   код 2 (2 вспышки/3 с) -> SDIO не поднялся, f_mount с ошибкой либо сбой
 *                             тестовой записи (Docs/06 §3 код 4 по смыслу —
 *                             кода 4 в настольном тесте ещё нет)
 *
 * Контейнер VLOG (block_seq/CRC32), кнопка PC13 и запись по кольцу на этом
 * этапе сознательно НЕ подключаются.
 */

#include "stm32f4xx_hal.h"

#include "drv_bmi160.h"
#include "bsp_capture.h"
#include "bsp_sdio.h"
#include "bsp_uart.h"
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

/* Быстрые вспышки подтверждения: карта смонтирована / тест записи пройден */
#define MOUNT_OK_FLASHES 3u
#define TEST_OK_FLASHES 4u
#define OK_FLASH_MS 100u

/* Настольный тест: имя в 8.3 (_USE_LFN = 0 в ffconf.h) и один блок 4096 Б
   = 8 секторов FatFs за один вызов f_write (Docs/03 §2) */
#define TEST_FILE_NAME "TEST.BIN"
#define TEST_WRITE_BYTES BSP_SDIO_BLOCK_BYTES

/* 16 байт — размер заголовка блока VLOG (Docs/04 §3): в тесте он заполнен
   константой, чтобы на PC было видно границу заголовка и сквозного счётчика */
#define TEST_HEADER_BYTES 16u
#define TEST_HEADER_FILL 0xAAu

/* Пауза цикла остановки после успешного теста */
#define HALT_LOOP_MS 1000u

/* Тестовый блок 4096 байт. f_write -> disk_write -> HAL_SD_WriteBlocks_DMA,
   поэтому буфер обязан лежать в D1 SRAM и быть выровнен по 4 (Docs/01 §5,
   Docs/03 §1); проверка буфера в lib/FatFs/diskio.c иначе вернёт RES_PARERR */
static uint8_t test_block[TEST_WRITE_BYTES] __attribute__((section(".d1ram"), aligned(4)));

/* Контекст тома FatFs: FATFS и FIL содержат по секторному буферу 512 Б,
   поэтому стоят статическими в .bss (D1 SRAM), а не в CCM и не на стеке
   (Docs/01 §5) */
static FATFS fs;
static FIL test_file;


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

/**
 * Заполнить тестовый блок: первые 16 байт (размер заголовка блока VLOG по
 * Docs/04 §3) константой 0xAA, далее сквозной счётчик по модулю 256. Такая
 * картина на PC сразу показывает: пропущенный сектор (разрыв счёта на 512),
 * сдвиг данных (смещение счёта) и подмену сектора (повтор участка).
 */
static void fill_test_block(void)
{
    for (uint32_t i = 0u; i < TEST_WRITE_BYTES; i++) {
        test_block[i] = (i < TEST_HEADER_BYTES) ? (uint8_t)TEST_HEADER_FILL : (uint8_t)i;
    }
}

int main(void)
{
    uint8_t chip_id;
    FRESULT fr;
    UINT bw;

    bsp_meminit();
    HAL_Init();

    /* PB2 настраиваем до тактирования: сбой HSE/PLL должен отмигать код 1
       уже настроенным выходом (Docs/06_error.md §3) */
    led_init();

    system_clock_84mhz_hse();

    /* Отладочный лог включаем сразу после тактирования: USART1 висит на
       APB2 = 84 МГц, поэтому BRR обязан считаться уже от новых частот.
       Сбой UART не фатален (лог — вспомогательное средство, прошивка обязана
       работать и без него), но и не молчит: поднимается bsp_uart_error_count(),
       весь дальнейший лог вернёт -1, диагностика останется на вспышках PB2 */
    bsp_uart_init();

    bsp_uart_printf("[BOOT] System started\r\n");

    if (!drv_bmi160_spi_init()) {
        bsp_uart_printf("[BOOT] ERR: SPI1 init failed\r\n");
        fault_blink_code1();
    }

    chip_id = drv_bmi160_init_sensor();
    bsp_uart_printf("[BOOT] BMI160 CHIP_ID: 0x%02X\r\n", (unsigned int)chip_id);

    if (chip_id != BMI160_CHIP_ID) {
        bsp_uart_printf("[BOOT] ERR: CHIP_ID != 0xD1\r\n");
        fault_blink_code1();
    }

    ring_init();

    /* Акселерометр в Normal Mode + профиль 1600 Гц / OSR2 / +/-16 g (Docs/02 §4 п.5).
       Строго ДО конфигурации FIFO: без Normal Mode FIFO не заполняется и
       порог FWM никогда не поднимает INT1 (PB5). */
    if (!drv_bmi160_start_accel()) {
        bsp_uart_printf("[BOOT] ERR: accel Normal Mode failed\r\n");
        fault_blink_code1();
    }

    /* FIFO: watermark 56 байт, acc+header, FWM -> INT1 (PB5), Flush FIFO */
    if (!drv_bmi160_configure_fifo()) {
        bsp_uart_printf("[BOOT] ERR: FIFO config failed\r\n");
        fault_blink_code1();
    }

    bsp_capture_dma_init();  /* DMA2 Stream0/CH3 = SPI1_RX, Stream3/CH3 = SPI1_TX */
    bsp_capture_exti_init(); /* PB5 = EXTI5, нарастающий фронт, NVIC приоритет 6  */

    if (bsp_capture_error_count() != 0u) {
        bsp_uart_printf("[BOOT] ERR: SPI1 DMA/EXTI failed\r\n");
        fault_blink_code1();
    }

    /* SDIO 4-bit + DMA2 Stream6/CH4 (Docs/03 §1-2), код 2 при любой проблеме
       с картой (Docs/06 §3) */
    if (!bsp_sdio_init()) {
        bsp_uart_printf("[BOOT] ERR: SDIO init failed\r\n");
        fault_blink_code2();
    }

    /* f_mount(&fs, "", 1): третий аргумент 1 = монтировать том сразу, т.е.
       читать MBR/BOOT-сектор прямо здесь. Путь "" = логический диск 0
       (_VOLUMES = 1, ffconf.h) */
    fr = f_mount(&fs, "", 1u);
    if (fr != FR_OK) {
        bsp_uart_printf("[BOOT] ERR: f_mount failed (%u)\r\n", (unsigned int)fr);
        fault_blink_code2();
    }

    bsp_uart_printf("[BOOT] SDIO init and Mount OK\r\n");

    /* 3 быстрые вспышки PB2 — карта смонтирована (Docs/06 §2: штатно IDLE
       мигает раз в 2.5 с, но состояние кнопки PC13 появляется позже) */
    for (uint32_t i = 0u; i < MOUNT_OK_FLASHES; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(OK_FLASH_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(OK_FLASH_MS);
    }

    /* --- настольный тест пути записи: один блок 4096 байт в TEST.BIN --- */
    bsp_uart_printf("[BOOT] Writing test file...\r\n");

    fill_test_block();

    /* FA_CREATE_ALWAYS = создать заново (или стереть старый TEST.BIN),
       FA_WRITE = запись. Имя 8.3 — _USE_LFN = 0 в ffconf.h */
    fr = f_open(&test_file, TEST_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        bsp_uart_printf("[TEST] ERR: f_open failed (%u)\r\n", (unsigned int)fr);
        fault_blink_code2();
    }

    /* 4096 байт = 8 секторов FatFs за один вызов, т.е. один HAL_SD_WriteBlocks_DMA
       (Docs/03 §2). Файл начинается с границы кластера (кластер 32 КБ), поэтому
       FatFs не дробит этот запрос */
    bw = 0u;
    fr = f_write(&test_file, test_block, TEST_WRITE_BYTES, &bw);
    if (fr != FR_OK || bw != (UINT)TEST_WRITE_BYTES) {
        bsp_uart_printf("[TEST] ERR: f_write failed (%u, bytes=%u)\r\n", (unsigned int)fr,
                        (unsigned int)bw);
        fault_blink_code2();
    }

    bsp_uart_printf("[TEST] Write successful, bytes written: %u\r\n", (unsigned int)bw);

    /* f_sync — добить данные из окна FatFs на карту, f_close — дописать FAT/DIR.
       Обе проверяются: молчаливая потеря данных на этом тесте недопустима */
    fr = f_sync(&test_file);
    if (fr != FR_OK) {
        bsp_uart_printf("[TEST] ERR: f_sync failed (%u)\r\n", (unsigned int)fr);
        fault_blink_code2();
    }

    fr = f_close(&test_file);
    if (fr != FR_OK) {
        bsp_uart_printf("[TEST] ERR: f_close failed (%u)\r\n", (unsigned int)fr);
        fault_blink_code2();
    }

    bsp_uart_printf("[TEST] File write OK\r\n");

    /* 4 быстрые вспышки PB2 — путь записи пройден полностью */
    for (uint32_t i = 0u; i < TEST_OK_FLASHES; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(OK_FLASH_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(OK_FLASH_MS);
    }

    /* Захват датчика на этом этапе сознательно не запускается: остановка */
    while (1) {
        HAL_Delay(HALT_LOOP_MS);
    }
}
