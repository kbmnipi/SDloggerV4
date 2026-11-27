/**
 * main.c — этап M0: тактирование 84 МГц + проверка присутствия BMI160.
 *
 * Порядок инициализации (порядок фиксирован аудитом M0):
 *   1) bsp_meminit()            — обнуление CCM (секция .ccmram — NOLOAD)
 *   2) HAL_Init()               — SysTick, приоритет grouped, NVIC
 *   3) led_init()               — PB2 ДО тактирования: при сбое HSE/PLL код 1
 *                                должен мигать уже настроенным выходом
 *   4) system_clock_84mhz_hse() — любой сбой -> fault_blink_code1()
 *   5) drv_bmi160_spi_init()    — false -> fault_blink_code1()
 *
 * Индикация (Docs/06_error.md):
 *   CHIP_ID == 0xD1        -> мигаем PB2 с частотой 1 Гц (датчик на месте)
 *   CHIP_ID != 0xD1        -> код 1 (нет ответа от акселерометра): одна вспышка
 *                             PB2 каждые 3 с (200 мс горит / 2800 мс пауза)
 *
 * SD-карта, FatFs и кольцевой буфер на этом шаге сознательно НЕ подключаются.
 */

#include "stm32f4xx_hal.h"

#include "drv_bmi160.h"
#include "stm32f4xx_it.h"

#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_2

#define ERROR_CODE1_BLINK_MS 200u  /* Docs/06_error.md §3: 200 мс горит  */
#define ERROR_CODE1_PAUSE_MS 2800u /* ... и пауза до конца цикла в 3 с    */

/* Границы секции .ccmram из scripts/STM32F405RGT6_CCM.ld. Секция NOLOAD:
   startup_stm32f405xx.S обнуляет только _sbss.._ebss в D1 SRAM и до CCM
   не доходит, поэтому CCM обнуляем явно (Docs/01 §5). */
extern uint32_t _sccmram;
extern uint32_t _eccmram;

/**
 * Код ошибки 1 (Docs/06_error.md §3): нет ответа от акселерометра —
 * PB2 вспыхивает 1 раз каждые 3 секунды. Дальше кода 1 не defined,
 * поэтому сбои тактирования (HAL_RCC_* ниже) мигаем тем же кодом.
 */
static void fault_blink_code1(void)
{
    while (1) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(ERROR_CODE1_BLINK_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(ERROR_CODE1_PAUSE_MS);
    }
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

    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(500);
    }
}
