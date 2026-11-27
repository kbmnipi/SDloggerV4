/**
 * syscalls.c — заглушка _sbrk() для newlib-nano (аудит M1, находка K2).
 *
 * Правило проекта: динамической памяти нет (Docs/01 §5, .kilocode/rules).
 * Линкерный скрипт scripts/STM32F405RGT6_CCM.ld объявляет _Min_Heap_Size = 0,
 * но это только намерение: стоит любой функции libc потянуть _malloc_r
 * (исторически это делал vsnprintf из bsp_uart.c), как newlib зовёт _sbrk(),
 * а стандартная заглушка libnosys поднимала кучу от _end — то есть писала
 * прямо в зарезервированный под стек MSP участок ._user_heap_stack. Тихая
 * порча стека, которую отладчиком не найти.
 *
 * Своя заглушка превращает любую будущую скрытую аллокацию в ГРОМКОЕ и
 * воспроизводимое событие: malloc() возвращает NULL с errno = ENOMEM, а не
 * затирает кадр вызова. Проверка:
 *   arm-none-eabi-nm --undefined-only .pio/build/release/firmware.elf   (пусто)
 *   arm-none-eabi-nm --defined-only .pio/build/release/firmware.elf | findstr sbrk
 *
 * _sbrk() в заголовках newlib не объявлена (только вызывается из libc), поэтому
 * объявление здесь ни с чем не конфликтует; caddr_t берётся из <sys/types.h>.
 */

#include <errno.h>
#include <sys/types.h>

caddr_t _sbrk(int incr)
{
    (void)incr;
    errno = ENOMEM;
    return (caddr_t)-1;
}
