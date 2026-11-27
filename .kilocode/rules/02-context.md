# Project Context: SD Logger V4

## Hardware
- MCU: STM32F405RGT6 @ 84 MHz. 128 KB D1 SRAM, 64 KB CCM.
- Sensor: **BMI160** on SPI1 (Mode 3, 5.25 MHz). Pins: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4, INT1=PB5.
- Storage: MicroSD via SDIO 4-bit. Pins: PC8-PC11, PC12, PD2.
- User I/O: LED PB2 (Active HIGH), Button PC13 (Active HIGH).
- Debug: UART1 (PA9/PA10) at 115200.

## Critical Rules (Do not violate)
- No RTOS. No dynamic allocation.
- **Memory:** All DMA buffers and the Stack MUST be in D1 SRAM (`0x2000_0000`). Never in CCM.
- **SPI:** Mode 3 (CPOL=1, CPHA=1). The BMI160 requires strict delays (`t_IDLE`) during init (5ms/10ms).
- **SDIO:** Write strictly 4096 bytes per block. Use DMA2.
- **Format:** Block = 4096 B (16 B header + 680 samples × 6 B). Little-endian.