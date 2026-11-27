/**
 * bsp_sdio.c — MicroSD на SDIO 4-bit + DMA2 Stream6/Channel 4 (Docs/03 §1-2).
 *
 * Тактирование SDIOCLK = PLL48CK 48 МГц (PLLQ = 7, main.c), SDIO_CK = 48 / (0 + 2)
 * = 24 МГц (Docs/01 §2, Docs/03 §2).
 *
 * Приоритеты (Docs/03 §1): DMA-приоритет SDIO = MEDIUM, т.е. строго ниже
 * SPI1_RX/SPI1_TX (VERY_HIGH); прерывания SDIO/DMA2_Stream6 — самый низкий
 * NVIC-приоритет (7), чтобы запись на карту никогда не задерживала захват
 * датчика (EXTI INT1 = 6, SPI1_RX DMA = 5).
 */

#include "bsp_sdio.h"

/* PC8..PC11 = D0..D3, PC12 = CK, PD2 = CMD, всё AF12 (Docs/01 §2) */
#define SDIO_GPIO_PORT GPIOC
#define SDIO_DATA_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
#define SDIO_CLK_PIN GPIO_PIN_12
#define SDIO_CMD_PORT GPIOD
#define SDIO_CMD_PIN GPIO_PIN_2

/* SDIO_CK = SDIOCLK / (ClockDiv + 2): 48 МГц / 2 = 24 МГц (Docs/03 §2) */
#define SDIO_CLOCK_DIV 0u

/* DMA2 Stream 6 + Channel 4 = SDIO по матрице RM0090 (Docs/03 §1) */
#define SDIO_DMA_INSTANCE DMA2_Stream6
#define SDIO_DMA_CHANNEL DMA_CHANNEL_4
#define SDIO_DMA_PRIORITY DMA_PRIORITY_MEDIUM

/* Самый низкий NVIC-приоритет: захват датчика имеет приоритет выше */
#define SDIO_IRQn_PRIORITY 7u
#define SDIO_DMA_IRQn_PRIORITY 7u

/* Таймаут ожидания конца DMA-транзакции (Docs/03 §2: до 450 мс на блок) */
#define SDIO_XFER_TIMEOUT_MS 1000u

static SD_HandleTypeDef g_hsd;
static DMA_HandleTypeDef g_hdma_sdio;

static bool g_sdio_ready;
static bool g_sdio_msp_error;
static volatile bool g_sdio_xfer_done;
static volatile bool g_sdio_xfer_error;

void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef gpio = {0};

    if (hsd->Instance != SDIO) {
        return;
    }

    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* D0..D3 и CMD: обязательный внутренний GPIO_PULLUP — на плате WeAct
       резисторов нет (Docs/01 §2); на D3 дополнительно внешний R11 100 кОм */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_SDIO;

    gpio.Pin = SDIO_DATA_PINS;
    HAL_GPIO_Init(SDIO_GPIO_PORT, &gpio);

    gpio.Pin = SDIO_CMD_PIN;
    HAL_GPIO_Init(SDIO_CMD_PORT, &gpio);

    /* CK — выход SDIO, подтяжка не используется (Docs/01 §2: GPIO_NOPULL) */
    gpio.Pin = SDIO_CLK_PIN;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDIO_GPIO_PORT, &gpio);

    /* Одинаковый поток на чтение и запись: HAL_SD сам переключает биты DIR
       (stm32f4xx_hal_sd.c: MODIFY_REG(..., DMA_SxCR_DIR, ...)) перед стартом.
       SDIO требует FIFO-режим DMA и-word переносы через регистр FIFO. */
    g_hdma_sdio.Instance = SDIO_DMA_INSTANCE;
    g_hdma_sdio.Init.Channel = SDIO_DMA_CHANNEL;
    g_hdma_sdio.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_hdma_sdio.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_sdio.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_hdma_sdio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_hdma_sdio.Init.Mode = DMA_NORMAL;
    g_hdma_sdio.Init.Priority = SDIO_DMA_PRIORITY;
    g_hdma_sdio.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    g_hdma_sdio.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    g_hdma_sdio.Init.MemBurst = DMA_MBURST_SINGLE;
    g_hdma_sdio.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&g_hdma_sdio) != HAL_OK) {
        g_sdio_msp_error = true;
    }

    __HAL_LINKDMA(hsd, hdmarx, g_hdma_sdio);
    __HAL_LINKDMA(hsd, hdmatx, g_hdma_sdio);

    HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
    HAL_NVIC_SetPriority(SDIO_IRQn, SDIO_IRQn_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);

    HAL_NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, SDIO_DMA_IRQn_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance != SDIO) {
        return;
    }

    HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
    __HAL_RCC_SDIO_CLK_DISABLE();
    HAL_GPIO_DeInit(SDIO_GPIO_PORT, SDIO_DATA_PINS | SDIO_CLK_PIN);
    HAL_GPIO_DeInit(SDIO_CMD_PORT, SDIO_CMD_PIN);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    g_sdio_xfer_done = true;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    g_sdio_xfer_done = true;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    g_sdio_xfer_error = true;
    g_sdio_xfer_done = true;
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    g_sdio_xfer_done = true;
}

bool bsp_sdio_init(void)
{
    if (g_sdio_ready) {
        return true;
    }

    g_sdio_msp_error = false;
    g_sdio_xfer_done = false;
    g_sdio_xfer_error = false;

    g_hsd.Instance = SDIO;
    g_hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    g_hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    g_hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    g_hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
    g_hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    g_hsd.Init.ClockDiv = SDIO_CLOCK_DIV;

    /* HAL_SD_Init() вызывает HAL_SD_MspInit(): пины, DMA2 Stream6/CH4, NVIC.
       Внутри: CMD0/8/ACMD41/CMD2/CMD3/CMD9/CMD10/CMD7 на 400 кГц + CSD/CID. */
    if (HAL_SD_Init(&g_hsd) != HAL_OK) {
        return false;
    }
    if (g_sdio_msp_error) {
        return false;
    }

    /* CMD6 (BUS_WIDTH[3:2] = 01) + перенастройка SDIO на 4 бит и 24 МГц */
    if (HAL_SD_ConfigWideBusOperation(&g_hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
        return false;
    }

    g_sdio_ready = true;

    return true;
}

bool bsp_sdio_ready(void)
{
    return g_sdio_ready;
}

SD_HandleTypeDef *bsp_sdio_handle(void)
{
    return &g_hsd;
}

DMA_HandleTypeDef *bsp_sdio_dma_handle(void)
{
    return &g_hdma_sdio;
}

void bsp_sdio_transfer_begin(void)
{
    g_sdio_xfer_done = false;
    g_sdio_xfer_error = false;
}

bool bsp_sdio_wait_transfer(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!g_sdio_xfer_done) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            /* Транзакцию снимаем, иначе HAL_SD навсегда останется BUSY и
               следующая запись вернёт HAL_BUSY (Docs/03 §2 — обработка сбоев) */
            (void)HAL_SD_Abort(&g_hsd);
            g_sdio_xfer_done = false;
            g_sdio_xfer_error = false;
            return false;
        }
    }

    if (g_sdio_xfer_error) {
        (void)HAL_SD_Abort(&g_hsd);
        g_sdio_xfer_error = false;

        return false;
    }

    return true;
}
