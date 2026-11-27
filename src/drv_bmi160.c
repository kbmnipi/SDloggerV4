/**
 * drv_bmi160.c — SPI1-интерфейс BMI160, присутствие датчика (presence gate).
 *
 * Протокол (Docs/02_bmi160_ref.md):
 *   чтение: 0x80 | (addr & 0x7F), запись: addr & 0x7F
 *   после soft reset интерфейс датчика возвращается в I2C -> нужен
 *   повторный "wiggle" чтения 0x7F (SPI_WAKEUP).
 */

#include "drv_bmi160.h"

#include "stm32f4xx_hal.h"

/* Регистры, нужные для presence gate */
#define BMI160_REG_CHIP_ID 0x00u
#define BMI160_REG_CMD 0x7Eu
#define BMI160_REG_SPI_WAKEUP 0x7Fu

#define BMI160_CMD_SOFT_RESET 0xB6u
#define BMI160_CMD_FIFO_FLUSH 0xB0u /* Docs/02 §2: очистить FIFO */
#define BMI160_ADDR_READ 0x80u

/* Регистры FIFO/прерываний (Docs/02_bmi160_ref.md раздел 2) */
#define BMI160_REG_FIFO_CONFIG_0 0x46u
#define BMI160_REG_FIFO_CONFIG_1 0x47u
#define BMI160_REG_INT_EN_1 0x51u
#define BMI160_REG_INT_OUT_CTRL 0x53u
#define BMI160_REG_INT_MAP_1 0x56u

/* Значения профиля 1600 Гц / +/-16 g (Docs/02_bmi160_ref.md раздел 3) */
#define BMI160_FIFO_CONFIG_0_VALUE 0x0Eu /* fifo_water_mark = 14 * 4 = 56 байт  */
#define BMI160_FIFO_CONFIG_1_VALUE 0x50u /* fifo_acc_en | fifo_header_en        */
#define BMI160_INT_EN_1_VALUE 0x40u      /* int_fwm_en                          */
#define BMI160_INT_MAP_1_VALUE 0x40u     /* int1_fwm -> INT1                    */
#define BMI160_INT_OUT_CTRL_VALUE 0x0Au  /* int1_output_en | int1_lvl (HIGH)    */

/* Прогрев FIFO после Flush перед началом записи (Docs/02 §4 п.7) */
#define BMI160_FIFO_FLUSH_SETTLE_MS 50u

/* PA4 — CS, активный LOW; PA5 — SCK, PA6 — MISO, PA7 — MOSI (AF5) */
#define BMI160_CS_PORT GPIOA
#define BMI160_CS_PIN GPIO_PIN_4
#define BMI160_SPI_GPIO_PORT GPIOA

#define BMI160_SPI_TIMEOUT_MS 10u

static SPI_HandleTypeDef g_hspi1;

static void bmi160_cs_low(void)
{
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);
}

static void bmi160_cs_high(void)
{
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);
}

static bool bmi160_xfer(uint8_t out, uint8_t *in)
{
    uint8_t rx = 0x00u;

    if (HAL_SPI_TransmitReceive(&g_hspi1, &out, &rx, 1u, BMI160_SPI_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    if (in != NULL) {
        *in = rx;
    }
    return true;
}

static uint8_t bmi160_read_reg(uint8_t addr)
{
    uint8_t value = 0x00u;

    bmi160_cs_low();
    if (!bmi160_xfer(BMI160_ADDR_READ | (addr & 0x7Fu), NULL)) {
        bmi160_cs_high();
        return 0x00u;
    }
    if (!bmi160_xfer(0xFFu, &value)) {
        bmi160_cs_high();
        return 0x00u;
    }
    bmi160_cs_high();

    return value;
}

static bool bmi160_write_reg(uint8_t addr, uint8_t value)
{
    bool ok_addr;
    bool ok_value;

    bmi160_cs_low();
    ok_addr = bmi160_xfer(addr & 0x7Fu, NULL);
    ok_value = bmi160_xfer(value, NULL);
    bmi160_cs_high();

    return ok_addr && ok_value;
}

bool drv_bmi160_spi_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* SCK (PA5) и MISO (PA6) — подтянуты вверх, MOSI (PA7) — вниз */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(BMI160_SPI_GPIO_PORT, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BMI160_SPI_GPIO_PORT, &gpio);

    /* CS — обычный push-pull выход, покоится в HIGH (SPI включается по
       первому нарастающему фронту CS на модуле GY-BMI160) */
    gpio.Pin = BMI160_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BMI160_CS_PORT, &gpio);
    bmi160_cs_high();

    g_hspi1.Instance = SPI1;
    g_hspi1.Init.Mode = SPI_MODE_MASTER;
    g_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    g_hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    g_hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH; /* Mode 3: CPOL = 1 */
    g_hspi1.Init.CLKPhase = SPI_PHASE_2EDGE; /* Mode 3: CPHA = 1 */
    g_hspi1.Init.NSS = SPI_NSS_SOFT;
    g_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 84 / 16 = 5.25 МГц */
    g_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    g_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_hspi1.Init.CRCPolynomial = 7u;
    if (HAL_SPI_Init(&g_hspi1) != HAL_OK) {
        return false;
    }

    __HAL_SPI_ENABLE(&g_hspi1);

    return true;
}

uint8_t drv_bmi160_init_sensor(void)
{
    /* 1. Одиночное чтение 0x7F переводит интерфейс модуля из I2C в SPI */
    (void)bmi160_read_reg(BMI160_REG_SPI_WAKEUP);
    HAL_Delay(5);

    /* 2. Soft Reset (после него интерфейс снова I2C). Сбой здесь проявится
       на шаге 4: CHIP_ID не будет 0xD1 -> вызывающий уходит в код ошибки 1. */
    (void)bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    HAL_Delay(10);

    /* 3. Повторно переключаем в SPI */
    (void)bmi160_read_reg(BMI160_REG_SPI_WAKEUP);
    HAL_Delay(5);

    /* 4. ID датчика: 0xD1, если BMI160 на месте и линия SPI жива */
    return bmi160_read_reg(BMI160_REG_CHIP_ID);
}

bool drv_bmi160_configure_fifo(void)
{
    /* Порядок по Docs/02 §4 п.6..7: сначала конфигурация, затем Flush FIFO.
       Каждая запись проверяется: сбой SPI1 -> false без исключений. */
    if (!bmi160_write_reg(BMI160_REG_FIFO_CONFIG_0, BMI160_FIFO_CONFIG_0_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_FIFO_CONFIG_1, BMI160_FIFO_CONFIG_1_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_INT_EN_1, BMI160_INT_EN_1_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_INT_MAP_1, BMI160_INT_MAP_1_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_INT_OUT_CTRL, BMI160_INT_OUT_CTRL_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH)) {
        return false;
    }

    /* Docs/02 §4 п.7: после Flush датчику нужно 50 мс до начала записи FIFO */
    HAL_Delay(BMI160_FIFO_FLUSH_SETTLE_MS);

    return true;
}

SPI_HandleTypeDef *drv_bmi160_spi_handle(void)
{
    return &g_hspi1;
}

void drv_bmi160_cs_write(bool asserted)
{
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
