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
#define BMI160_REG_PMU_STATUS 0x03u
#define BMI160_REG_FIFO_LENGTH_0 0x22u
#define BMI160_REG_FIFO_LENGTH_1 0x23u
#define BMI160_REG_ACC_CONF 0x40u
#define BMI160_REG_ACC_RANGE 0x41u
#define BMI160_REG_FIFO_CONFIG_0 0x46u
#define BMI160_REG_FIFO_CONFIG_1 0x47u
#define BMI160_REG_INT_EN_1 0x51u
#define BMI160_REG_INT_OUT_CTRL 0x53u
#define BMI160_REG_INT_MAP_1 0x56u

#define BMI160_CMD_ACC_NORMAL 0x11u /* Accel Normal Mode (Docs/02 §2) */

/* PMU_STATUS (0x03): acc_pmu_status = биты [5:4], Normal == 01 (Docs/02 §2) */
#define BMI160_PMU_ACC_STATUS_MASK 0x30u
#define BMI160_PMU_ACC_STATUS_NORMAL 0x10u

/* FIFO_LENGTH_1 (0x23): fifo_byte_counter[10:8], маска 0x07 (Docs/02 §2) */
#define BMI160_FIFO_LENGTH_HI_MASK 0x07u

/* Значения профиля 1600 Гц / +/-16 g (Docs/02_bmi160_ref.md раздел 3).
   Разбор побитово сверен с официальным Bosch BMI160 API (_Inbox/bmi160_defs.h,
   маски записаны скобками в комментариях) — значения совпадают:
     ACC_CONF       = BMI160_ACCEL_BW_OSR2_AVG2 (0x01<<4) | BMI160_ACCEL_ODR_1600HZ (0x0C)
     ACC_RANGE      = BMI160_ACCEL_RANGE_16G (0x0C)
     FIFO_CONFIG_1  = BMI160_FIFO_ACCEL (0x40) | BMI160_FIFO_HEADER (0x10)
     INT_OUT_CTRL   = BMI160_INT1_OUTPUT_EN (0x08) | int1_lvl HIGH (0x02);
                      BMI160_INT1_OUTPUT_MODE (0x04) = 0 -> Push-Pull, как требует
                      Docs/01 §2 (push-pull подтверждён bmi160.c:5084-5085:
                      output_mode ставится именно в бит 0x04) */
#define BMI160_ACC_CONF_VALUE 0x1Cu    /* acc_bwp = OSR2 (-3 дБ на 324 Гц), acc_odr = 1600 Гц */
#define BMI160_ACC_RANGE_VALUE 0x0Cu   /* +/-16 g, 2048 LSB/g (488 мкг/LSB)                 */
#define BMI160_FIFO_CONFIG_0_VALUE 0x0Eu /* fifo_water_mark = 14 * 4 = 56 байт              */
#define BMI160_FIFO_CONFIG_1_VALUE 0x50u /* BMI160_FIFO_ACCEL | BMI160_FIFO_HEADER            */
#define BMI160_INT_EN_1_VALUE 0x40u      /* int_fwm_en (бит 6)                                */
#define BMI160_INT_MAP_1_VALUE 0x40u     /* int1_fwm (бит 6) -> INT1                            */
#define BMI160_INT1_OUTPUT_EN_MASK 0x08u /* Bosch BMI160_INT1_OUTPUT_EN_MASK   — выход INT1 включён */
#define BMI160_INT1_LVL_HIGH_MASK 0x02u  /* Bosch bit1 (output_type) = 1       — Active HIGH        */
#define BMI160_INT1_OUTPUT_MODE_MASK 0x04u /* Bosch BMI160_INT1_OUTPUT_MODE_MASK — 0 = Push-Pull    */
#define BMI160_INT_OUT_CTRL_VALUE (BMI160_INT1_OUTPUT_EN_MASK | BMI160_INT1_LVL_HIGH_MASK)
/* INT1: output_en | lvl HIGH, OUTPUT_MODE (0x04) = 0 -> Push-Pull */

/* Пауза после CMD = 0x11 до проверки PMU_STATUS (Docs/02 §4 п.5) */
#define BMI160_ACC_NORMAL_SETTLE_MS 5u

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
    uint8_t chip_id = bmi160_read_reg(BMI160_REG_CHIP_ID);

    /* t_IDLE >= 450 мкс (Docs/02 §2): CS обязан отдыхать между транзакциями.
       Строго здесь, а не «на глаз» по таймингам UART-печати вызывающего:
       1 мс гарантированно перекрывает 450 мкс перед следующей командой
       (CMD = 0x11 в drv_bmi160_start_accel()) */
    HAL_Delay(1);

    return chip_id;
}

bool drv_bmi160_start_accel(void)
{
    uint8_t status;

    /* Docs/02 §4 п.5: Accel Normal Mode, выдержка 5 мс, проверка PMU_STATUS.
       Без этого шага акселерометр остаётся в SUSPEND: FIFO не заполняется,
       порог FWM никогда не достигается и INT1 (PB5) не поднимается. */
    if (!bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL)) {
        return false;
    }
    HAL_Delay(BMI160_ACC_NORMAL_SETTLE_MS);

    status = bmi160_read_reg(BMI160_REG_PMU_STATUS);
    if ((status & BMI160_PMU_ACC_STATUS_MASK) != BMI160_PMU_ACC_STATUS_NORMAL) {
        return false;
    }

    /* Docs/02 §4 п.6 + §3: профиль 1600 Гц / OSR2 / +/-16 g */
    if (!bmi160_write_reg(BMI160_REG_ACC_CONF, BMI160_ACC_CONF_VALUE)) {
        return false;
    }
    if (!bmi160_write_reg(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_VALUE)) {
        return false;
    }

    return true;
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

uint16_t drv_bmi160_get_fifo_length(void)
{
    uint8_t len0 = 0u;
    uint8_t len1 = 0u;

    /* Автоинкремент адреса включён всегда, кроме 0x24 (Docs/02 §1):
       один цикл CS = 0 -> 0x80|0x22 -> FIFO_LENGTH_0 -> FIFO_LENGTH_1. */
    bmi160_cs_low();
    if (!bmi160_xfer(BMI160_ADDR_READ | BMI160_REG_FIFO_LENGTH_0, NULL) ||
        !bmi160_xfer(0xFFu, &len0) ||
        !bmi160_xfer(0xFFu, &len1)) {
        bmi160_cs_high();
        return BMI160_FIFO_LENGTH_SPI_ERROR;
    }
    bmi160_cs_high();

    /* fifo_byte_counter[10:0]: младшие 8 байт + 3 старших бита (Docs/02 §2) */
    return (uint16_t)len0 | (uint16_t)((uint16_t)(len1 & BMI160_FIFO_LENGTH_HI_MASK) << 8);
}

SPI_HandleTypeDef *drv_bmi160_spi_handle(void)
{
    return &g_hspi1;
}

void drv_bmi160_cs_write(bool asserted)
{
    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
