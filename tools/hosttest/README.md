# Host-side verification of the backend changes

Two differential tests that run on the build machine (no Pi, no ARM
toolchain, no QEMU) against the **patched** `external/nuked-sc55` tree.
They verify logic, not performance. They complement the QEMU golden gate
(which runs the real firmware) rather than replacing it: random states
cover register combinations the firmware never produces, and the
firmware produces sequences the random states never do.

## 1. `pcm_diff.cpp` — PCM_Update variants vs the baseline

For each seed: randomize a `pcm_t` (registers, config, wave ROMs, eram,
voice masks), copy it, run N ticks through `PCM_Update` and through each
scalar variant in `g_pcmVariants`, and compare every posted sample,
every interrupt request and the full state. Register-accumulator
variants are compared mod 2^20 on `accum_l/r` and `rcsum` (the only way
the engine reads them) and skip `ram1[31][1]` / `ram1[31][3]`.

Also runs `PCM_RomFastSelfTest` (exhaustive `PCM_RomFetch` vs
`PCM_ReadROM`: every hiaddr, every 20-bit address, both config modes,
MK1 and mkII).

Arguments: `pcm_diff.exe [seeds=150] [ticks=400] [max_reg_slots=32]`.
Run it twice, with 32 and with 31: a variant that passes only at 31 is
exact for the firmware's slot counts but relies on slot 31 never being
in the loop (slot 31's filter registers alias the mix bus words; see
`docs/variant-review-20261031.md`).

NEON variants (3, 4, 5, 9) carry AArch64 inline asm and are compiled
out on other hosts (`#if defined(__aarch64__)` in pcm.cpp); on an
AArch64 Linux host (or under qemu-user) the same test covers them too.

## 2. `timer_selftest_main.cpp` — lazy timer vs legacy timer

Runs `TIMER_SelfTest` (3M steps, both step tables, plain and with the
injected one-tick fault). Exit 0 = plain runs report 0 mismatches, the
injected runs report > 0, and the lazy fast path was exercised.

## Building

The backend headers need C++20/23. `host_shim.h` is force-included to
neutralise GCC attributes for MSVC and pull in `nuked_profile.h`;
`config.h` stands in for the CMake-generated one; `split_stub.h` replaces
`split_thread_circle.h` (Circle headers + AArch64 asm) for the PCM test.

MSVC (Developer PowerShell / after `vcvars64.bat`), from this directory:

```
$be = "..\..\external\nuked-sc55\src\backend"
(Get-Content "$be\pcm.cpp") -replace '#include "split_thread_circle.h"','#include "split_stub.h"' | Set-Content pcm_host.cpp
cl /nologo /std:c++latest /EHsc /O2 /FI host_shim.h /I $be /I . /Fe:pcm_diff.exe pcm_diff.cpp pcm_host.cpp "$be\decoder2\cache.cpp"
.\pcm_diff.exe 300 400 32
.\pcm_diff.exe 300 400 31
cl /nologo /std:c++20 /EHsc /O2 /I $be /I . /FI msvc_shim.h /Fe:timer_selftest.exe timer_selftest_main.cpp "$be\mcu_timer.cpp"
.\timer_selftest.exe
```

GCC/Clang (Linux, x86-64 or AArch64):

```
BE=../../external/nuked-sc55/src/backend
sed 's/#include "split_thread_circle.h"/#include "split_stub.h"/' $BE/pcm.cpp > pcm_host.cpp
g++ -std=c++23 -O2 -include nuked_profile.h -I $BE -I . -o pcm_diff pcm_diff.cpp pcm_host.cpp $BE/decoder2/cache.cpp
./pcm_diff 300 400 32 && ./pcm_diff 300 400 31
g++ -std=c++20 -O2 -I $BE -I . -o timer_selftest timer_selftest_main.cpp $BE/mcu_timer.cpp && ./timer_selftest
```

(On GCC drop `-include host_shim.h`; the attributes are native.)

## Results recorded 2026-10-31 (MSVC 19.44, x64)

```
romfast selftest MK1:  mismatches=0 -> PASS
romfast selftest mkII: mismatches=0 -> PASS
max_reg_slots=32: sched PASS, pipe FAIL (10/300 seeds, samples), romfast PASS, scalacc PASS, romacc PASS
max_reg_slots=31: sched PASS, pipe PASS,                          romfast PASS, scalacc PASS, romacc PASS
timer: MK1 and GENERIC tables, 3M steps: plain 0 mismatches, injected >1.8M, PASS
```
