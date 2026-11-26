/*
 * SDloggerV4 — раскладка памяти STM32F405RGT6 (WeAct Studio Core Board)
 *
 *   FLASH  0x08000000  1024 KiB
 *   RAM    0x20000000   128 KiB   D1 SRAM: видит CPU и DMA2
 *   CCMRAM 0x10000000    64 KiB   CCM:     видит только CPU, DMA2 туда не имеет доступа
 *
 * Жёсткое правило (Docs/01_hardware_and_build.md, раздел 5):
 *   все DMA-буферы (ring[16][4096], spi_rx_buf[256], FATFS, FIL, diskio)
 *   и главный стек MSP (4 KiB) — строго в D1 SRAM; в CCM — только таблица
 *   CRC32 и переменные состояния, не связанные с DMA.
 *
 * Подключается хуком scripts/link_layout.py (-Wl,-T,...) вместо стандартного
 * скрипта пакета tool-ldscripts-ststm32.
 *
 * Соглашения для исходников:
 *   __attribute__((section(".d1ram"), aligned(4)))  -> DMA-буферы (zero-init)
 *   __attribute__((section(".ccmram"), aligned(4))) -> данные, доступные CPU
 * Секции NOLOAD -> начальные значения из flash не подтягиваются: .d1ram
 * обнуляется штатным startup (см. растянутый диапазон _sbss.._ebss), а .ccmram
 * инициализируется самим кодом прошивки (таблица CRC32 строится на лету).
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack.
   MSP обязан лежать в D1 SRAM: ORIGIN(RAM) + LENGTH(RAM) = 0x20020000 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size = 0x0000;  /* динамическая память (malloc/new) запрещена */
_Min_Stack_Size = 0x1000; /* MSP 4 KiB — резерв в самом верху D1 SRAM */
_stack_bottom = _estack - _Min_Stack_Size;

/* Memories definition */
MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Sections */
SECTIONS
{
  /* Векторная таблица — начало flash, как ожидает startup_stm32f405xx.S */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Инициализированные данные — в D1 SRAM (штатный startup копирует
     _sidata.._sdata.._edata). Никакого .bss-подобного мусора здесь нет. */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT>FLASH

  /* Обычные переменные состояния — D1 SRAM.
     Идёт ПЕРЕД .d1ram: две секции стоят непрерывно, и один штатный цикл
     обнуления _sbss.._ebss закрывает обе (правки в startup не нужны). */
  .bss :
  {
    . = ALIGN(4);
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
  } >RAM

  /* DMA-буферы: ring[16][4096], spi_rx_buf[256], FATFS/FIL, diskio.
     KEEP обязателен: сборка идёт с -Wl,--gc-sections, а железо обращается
     к буферу без явных ссылок из кода. */
  .d1ram (NOLOAD) :
  {
    . = ALIGN(4);
    _sd1ram = .;
    KEEP(*(.d1ram*))
    . = ALIGN(4);
    _ed1ram = .;
  } >RAM

  /* Конец диапазана обнуления — за .d1ram (см. комментарий к секции .bss) */
  _sbss = ADDR(.bss);
  __bss_start__ = _sbss;
  _ebss = _ed1ram;
  __bss_end__ = _ebss;

  /* Проверка запаса под стек MSP (4 KiB) сразу за DMA-буферами */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* CCM — видит только CPU. Здесь таблица CRC32 (8 KiB) и состояние,
     не участвующее в DMA. Никаких буферов SDIO/SPI1! */
  .ccmram (NOLOAD) :
  {
    . = ALIGN(4);
    _sccmram = .;
    KEEP(*(.ccmram*))
    . = ALIGN(4);
    _eccmram = .;
  } >CCMRAM

  /* ------------------------- границы раскладки ------------------------- */
  ASSERT(_ed1ram + _Min_Stack_Size <= ORIGIN(RAM) + LENGTH(RAM), "FATAL: D1 Stack collision")
  ASSERT(_sd1ram >= ORIGIN(RAM) && _ed1ram <= ORIGIN(RAM) + LENGTH(RAM), "FATAL: DMA buffer outside D1 SRAM")
  ASSERT(_eccmram <= ORIGIN(CCMRAM) + LENGTH(CCMRAM), "FATAL: CCM overflow")

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
