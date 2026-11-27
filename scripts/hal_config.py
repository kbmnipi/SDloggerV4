"""
SDloggerV4 — pre-хук проектной конфигурации HAL (platformio.ini: extra_scripts = pre).

lib/stm32f4xx_hal_conf.h — наша копия конфигурации HAL 1.28.3 (Docs/01 §4),
в platformio.ini включено board_build.stm32cube.custom_config_header = yes,
поэтому платформа НЕ генерирует свой stm32f4xx_hal_conf.h в пакете
framework-stm32cubef4 (stm32cube.py: generate_hal_config_file).

Путь к lib/ добавляется именно здесь, а не в build_flags: build_flags
обрабатываются в BuildProgram (piobuild.py), а исходники HAL фреймворка
компилируются в клоне env, созданном раньше — в build_flags их -I уже не
попадает. Pre-скрипты выполняются до builder/main.py платформы
(platformio/builder/main.py: env.SConscript(env.GetExtraScripts("pre"))).
"""

import os

from SCons.Script import Import

Import("env")

HAL_CONF_HEADER = "stm32f4xx_hal_conf.h"


def hal_conf_dir():
    lib_dir = os.path.normpath(os.path.join(env.subst("$PROJECT_DIR"), "lib"))
    if not os.path.isfile(os.path.join(lib_dir, HAL_CONF_HEADER)):
        raise Exception("hal_config: not found: %s" % os.path.join(lib_dir, HAL_CONF_HEADER))
    return lib_dir.replace("\\", "/")


conf_dir = hal_conf_dir()

env.Append(CPPPATH=[conf_dir])

print("hal_config: %s -> %s" % (HAL_CONF_HEADER, conf_dir))
