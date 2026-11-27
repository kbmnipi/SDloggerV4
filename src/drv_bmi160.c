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
#define BMI160_ADDR_READ 0x80u

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

static uint8_t bmi160_xfer(uint8_t out)
{
    uint8_t in = 0x00u;

    if (HAL_SPI_TransmitReceive(&g_hspi1, &out, &in, 1u, BMI160_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0x00u;
    }
    return in;
}

static uint8_t bmi160_read_reg(uint8_t addr)
{
    uint8_t value;

    bmi160_cs_low();
    (void)bmi160_xfer(BMI160_ADDR_READ | (addr & 0x7Fu));
    value = bmi160_xfer(0xFFu);
    bmi160_cs_high();

    return value;
}

static void bmi160_write_reg(uint8_t addr, uint8_t value)
{
    bmi160_cs_low();
    (void)bmi160_xfer(addr & 0x7Fu);
    (void)bmi160_xfer(value);
    bmi160_cs_high();
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

    /* 2. Soft Reset (после него интерфейс снова I2C) */
    bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    HAL_Delay(10);

    /* 3. Повторно переключаем в SPI */
    (void)bmi160_read_reg(BMI160_REG_SPI_WAKEUP);
    HAL_Delay(5);

    /* 4. ID датчика: 0xD1, если BMI160 на месте и линия SPI жива */
    return bmi160_read_reg(BMI160_REG_CHIP_ID);
}
