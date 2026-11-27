# Coding Discipline (Strict)

## Scope Discipline
- Do exactly what the current task asks. No more.
- No scaffolding, no "while I'm here", no unsolicited refactors.
- If a task requires multiple files, list them and ask which to do first.

## Micro-step Execution (CRITICAL for local models)
- **One prompt = one compilable unit.** (One `.c`/`.h` pair, or one bounded modification).
- Never generate more than ~200 lines of new code in a single response. Break it into logical steps.
- After writing code, ALWAYS state:
  1. Which file(s) were modified.
  2. What to verify (build result).
  3. What the next logical step is (but DO NOT execute it).

## Coding Style
- C99. No C++ features. No VLAs.
- All file-scope variables are `static` unless shared via header.
- ISR functions (`xxx_IRQHandler`) contain ONLY flag setting, DMA manipulation, and ring buffer updates.
- **No `float` in ISR.** Fixed-point only.
- Every magic number is a `#define` with a comment citing the Doc section.
- **No silent failures.** Every error path must return a status, increment a counter, or trigger a state change.

## Task Modes
- **IMPLEMENT**: Write full C files. (Default mode).
- **AUDIT**: Review code. Output `BUG-N / WARN-N`. Do not modify files.
- **DEBUG**: Propose a fix as a diff. Do not rewrite the whole file.

## Stop Conditions
Stop and ask the user if:
- A required register or pin is missing from `Docs/`.
- You catch yourself repeating the same reasoning loop.
- The task requires changing the DMA streams or clock tree.