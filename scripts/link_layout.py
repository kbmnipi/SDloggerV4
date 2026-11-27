"""
SDloggerV4 — build hook раскладки памяти (platformio.ini: extra_scripts = pre).

Подключает проверенный скрипт линковщика scripts/STM32F405RGT6_CCM.ld через
-Wl,-T,..., из-за чего PlatformIO не подставляет стандартный скрипт пакета
tool-ldscripts-ststm32 (см. platformio/builder/tools/piobuild.py: BuildProgram
пропускает авто-`-T`, если в LINKFLAGS уже есть -Wl,-T).

Раскладка (Docs/01_hardware_and_build.md, раздел 5):
  D1 SRAM  0x20000000  128 KiB -> все DMA-буферы (.d1ram) + стек MSP 4 KiB наверху
  CCM      0x10000000   64 KiB -> только CPU: таблица CRC32, состояние (.ccmram)
"""

import os

from SCons.Script import Import

Import("env")

LD_SCRIPT = "STM32F405RGT6_CCM.ld"


def resolve_ld_script():
    ld_path = os.path.join(env.subst("$PROJECT_DIR"), "scripts", LD_SCRIPT)
    if not os.path.isfile(ld_path):
        raise Exception("link_layout: linker script not found: %s" % ld_path)
    # Прямые слеши: путь уходит в LINKFLAGS одним полем -Wl,-T,<path>
    return os.path.normpath(ld_path).replace("\\", "/")


def check_board():
    mcu = (env.BoardConfig().get("build.mcu", "") or "").lower()
    if mcu and not mcu.startswith("stm32f405"):
        print(
            "Warning: %s рассчитан на STM32F405RGT6 (D1 SRAM 128K + CCM 64K), "
            "текущая плата: %s" % (LD_SCRIPT, mcu)
        )


ld_script = resolve_ld_script()
check_board()

if not any("-Wl,-T" in f for f in env.get("LINKFLAGS", [])):
    env.Append(LINKFLAGS=["-Wl,-T,%s" % ld_script])

# env.Replace(LDSCRIPT_PATH=...) убран: PlatformIO всё равно подставляет
# скрипт из LDSCRIPT_PATH в BuildProgram, и единственный работающий способ
# отключить авто-`-T` — уже имеющийся -Wl,-T в LINKFLAGS (piobuild.py).
# env.Prepend(LIBPATH=[...]) убран: в каталоге scripts/ нет .a/.lib библиотек.

# ВАЖНО: строку "RAM:" в отчёте ststm32 считает по регулярке
# SIZEDATAREGEXP="^(?:\.data|\.bss|\.noinit)" (builder/main.py), которая
# подставляется ПОСЛЕ pre-скриптов, поэтому .d1ram/.ccmram в неё не попадают:
# полоса RAM занижает реальное использование D1 SRAM. Достоверные источники —
# ASSERT'ы в .ld (сборка падает при переполнении) и `arm-none-eabi-size -A`.

print("link_layout: D1 SRAM 128K (DMA + MSP 4K) / CCM 64K (CPU only) -> %s" % ld_script)
