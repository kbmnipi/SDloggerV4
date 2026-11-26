# STM32F4 DMA2 Stream and Channel Mapping Reference (RM0090)

> **Project:** SDloggerV3_2 (STM32F405RGT6). **Scope: DMA2 only** (SPI1, SDIO; DMA1 — USART2/3, SPI2/3, I2C — out of scope).
> **Source:** RM0090, DMA2 request-mapping table (numbering varies between revisions — Table 27/63 etc.; verify against the PDF revision you use).
> **Two ground rules:** a stream serves only **one** channel at a time; the meaning of CHSEL depends **on the stream**, not only on the channel number — see the matrix in §3.

---

## 1. Final project assignment (SDloggerV3_2, §9) — fixed record

| Peripheral | Function | Stream | CHSEL | Comment |
|---|---|---|---|---|
| **SPI1_RX** | LIS3DSH data receive | DMA2 **Stream 0** | **3** | legal per matrix §3 |
| **SPI1_TX** | dummy-byte transmit | DMA2 **Stream 5** | **3** | legal per matrix §3; the V3.0 variant `S2/CH3` was **invalid — S2/CH3 belongs to SPI1_RX** |
| **SDIO** | SD write, 4-bit + DMA | DMA2 **Stream 3** | **4** | legal per matrix §3 |
| *(reserve)* | SDIO, backup | DMA2 Stream 6 | 4 | use if Stream 3 is ever needed for SPI1_TX |

**Conflict check:** streams 0 / 5 / 3 are all distinct → no two requests share a stream; the V3.2 assignment is fully legal and conflict-free.

**Fallback configuration** (if Stream 3 is given to SPI1_TX): `RX = S0/CH3, TX = S3/CH3, SDIO = S6/CH4`.

---

## 2. Quick lookup: logger peripherals → legal (stream, CHSEL) pairs

| Peripheral | Legal options | Chosen for the project |
|---|---|---|
| **SPI1_RX** | (Stream 0, CH3), (Stream 2, CH3) | **(Stream 0, CH3)** |
| **SPI1_TX** | (Stream 3, CH3), (Stream 5, CH3) | **(Stream 5, CH3)** |
| **SDIO** | (Stream 3, CH4), (Stream 6, CH4) | **(Stream 3, CH4)** |

Note: the CH3 combinations differ per peripheral — SPI1_RX has no (3,3)/(5,3) option, SPI1_TX has no (0,3)/(2,3) option. A wrong choice shows up as "the DMA never fired", not as an error (see §5 item 2).

---

## 3. Full DMA2 request-mapping matrix (RM0090)

Rows — streams (Stream 0…7); columns — the `CHSEL[2:0]` encoding of the `DMA_SxCR` register. SPI1/SDIO cells are bolded — the logger-critical ones.

| Stream | CH0 (000) | CH1 (001) | CH2 (010) | CH3 (011) | CH4 (100) | CH5 (101) | CH6 (110) | CH7 (111) |
|---|---|---|---|---|---|---|---|---|
| **0** | ADC1 | — | TIM8_CH1 | **SPI1_RX** | — | — | TIM1_TRIG | TIM8_UP |
| **1** | — | DCMI | TIM8_CH2 | — | — | USART6_RX | TIM1_CH1 | — |
| **2** | ADC1 | — | TIM8_CH3 | **SPI1_RX** | USART1_RX | USART6_RX | TIM1_CH2 | — |
| **3** | — | DCMI | — | **SPI1_TX** | **SDIO** | SPI4_RX | TIM1_CH1 | — |
| **4** | ADC1 | CRYPT_IN | — | — | USART1_RX | SPI4_TX | TIM1_UP | — |
| **5** | — | CRYPT_OUT | TIM8_UP | **SPI1_TX** | **SDIO** | — | TIM1_CH3 | — |
| **6** | — | HASH_IN | TIM8_CH4 / TRIG / COM | — | USART1_TX | USART6_TX | TIM1_CH4 | — |
| **7** | — | — | — | — | USART1_TX | USART6_TX | TIM1_COM | — |

---

## 4. Channel selection in registers (`CHSEL[2:0]` in `DMA_SxCR`)

| CHSEL | Channel | CHSEL | Channel |
|---|---|---|---|
| `000` | 0 | `100` | 4 (SDIO) |
| `001` | 1 | `101` | 5 |
| `010` | 2 | `110` | 6 |
| `011` | 3 (SPI1) | `111` | 7 |

Project example (HAL):

```c
hdma_spi1_rx.Instance = DMA2_Stream0;  hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
hdma_spi1_tx.Instance = DMA2_Stream5;  hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
SDIO_Init.hdma_sdio.Instance = DMA2_Stream3; /* Channel = DMA_CHANNEL_4 */
```

---

## 5. Rules and traps

1. **One stream — one request at a time.** Stream 3 cannot serve two peripherals simultaneously (SPI1_TX = S3/CH3 **or** SDIO = S3/CH4 — only one of them); running both requires distinct streams.
2. **CHSEL only takes effect within its own stream** and is stream-specific. A pair outside the §3 matrix → the request **is not routed**: the transfer never starts, which looks like a silent "0 bytes" / hang rather than an error. Watch: `DMA_LISR` flags (TEIF/FEIF/TIF) plus actual data transferred.
3. The table applies **to STM32F4 only** (F405/F407 etc.). STM32F7/H7 use DMAMUX instead of CHSEL — do not copy this mapping to other families.
4. DMA2 has no USART2/3/4/5, SPI2/3, I2C — those live on DMA1 (out of scope of this document).
5. TIM1/TIM8, DCMI, CRYPT, HASH, USART1/6 rows were carried over from the source table unchanged; the logger-critical SPI1/SDIO cells were cross-checked against SDloggerV3_2.md §9 — no discrepancies.

---

## 6. Bench verification (M0/M1 checklist)

- **M0 (SPI1):** loopback MOSI→MISO; full-duplex burst via `S0/CH3` (RX) + `S5/CH3` (TX); verify transferred volume and a known pattern; `DMA_LISR`: TEIF/FEIF = 0. This proves the CH3 requests are actually routed to streams 0 and 5.
- **M1 (SDIO):** 4-bit write/read-back of 4 KiB via `S3/CH4` with CRC (SDlogger M1 profile) — proves CH4 routing to stream 3.

