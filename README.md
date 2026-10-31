# Nuked-SC55 dual-core test kernel (Pi Zero 2 W)

A minimal bare-metal Circle kernel that runs
[jcmoyer/Nuked-SC55](https://github.com/jcmoyer/Nuked-SC55) on two cores of a
Raspberry Pi Zero 2 W (BCM2710A1, 4x Cortex-A53 @ 1 GHz): the H8 MCU emulator
on core 2, the PCM voice mixer on core 3, with a JV880-shape synchronisation
design (shared PCM state, release-published step counters, acquire chase,
small tick batches, a state lock only around publication).

It also carries the measurement harness for that work: an auto-sequenced
variant A/B runner, per-core PMU cycle counters, and a MiSTer variant display.

## What this is not

This is **not** mt32-pi, and **not** the production port. It has no Munt,
FluidSynth, or EmuSC engines, no configuration UI, no menus, no networking,
and no MIDI input stack. It exists to answer one question in isolation: can
the dual-core split run Nuked-SC55 at 100% of real time on a Pi Zero 2 W,
and which code shape gets there? The production port — the same sync design
inside mt32-pi's `CNukedSC55Synth`, alongside the other engines — will land
in [ckeisc/mt32-pi](https://github.com/ckeisc/mt32-pi) separately, once the
design is proven here.

## Status (snapshot 2026-10-31, round 3 sync + timer)

Work in progress. This snapshot contains the RUN-table auto-sequence
(17 runs = PCM variant x sync profile x timer mode, see below), the
PMU cycle + event-counter tooling, the lazy-timer self-test and
shadow-check mode, and the MiSTer display wiring. Hardware
verification of this exact snapshot is pending. The round-3 sync/timer
work was written without an ARM toolchain at hand; the only piece
executed before commit is the lazy-timer self-test, built and run
with a host compiler (see `docs/optimization-round3-sync-20261031.md`).

## Hardware and build

- Target: Raspberry Pi Zero 2 W (Pi 3-class silicon). No USB, no network.
- Toolchain: `aarch64-none-elf` GCC + `ninja`, same as the Circle
  `circle-stdlib` build.
- Dependencies come in as submodules (see `.gitmodules`): `circle-stdlib`
  and `Nuked-SC55`. Check them out with
  `git submodule update --init --recursive`.
- Build with `make`. It produces `kernel8.img` (gzip-compressed kernel
  image). Copy it to the root of a FAT32 SD card as `kernel8.img`.

The Makefile applies `patches/nuked-sc55-0.8.0-circle.patch` to the
Nuked-SC55 submodule at build time, and
`patches/circle-45-gzip-kernel.patch` to Circle. Do not edit a patched file
without updating its patch hunk.

## SD card layout

- `kernel8.img` — this kernel, at the card root.
- `roms/sc55/` — your own SC-55 ROM set. The kernel scans this directory
  and hash-matches the files against the MK1 (v1.21) romset definitions.
  **No ROMs are in this repo, and none ever will be.**
- `shaped-var3-stats.log` — written by the kernel (see below).

## What the kernel does on boot

No keys, no input. It auto-starts E1M1 on run 0 (baseline) and plays it
through, then advances automatically on each song end. A **run** is a
(PCM_Update variant, sync profile, timer mode) triple; the table lives
in `src/minimal.cpp` (`g_runs`), the PCM variants in `pcm.cpp`
(`g_pcmVariants`), the sync profiles in `src/minimal.cpp`
(`g_splitSyncProfiles`, knobs defined in `split_thread_circle.h`).

| Run | Name | PCM variant | Sync profile | Timer | What it isolates |
|---|---|---|---|---|---|
| 0 | base | 0 baseline | 0 legacy | legacy | the reference: shaped-var behaviour, every knob at its legacy value |
| 1 | trim | 0 | 1 trim | legacy | sync overhead trim (padded state, raw lock, cheap lock timing, publish/8, ring check/16, IRQ0 flag, idle backoff) |
| 2 | tick | 0 | 2 tick | legacy | + per-tick lock hand-off (core-2 wait bounded by one PCM tick) |
| 3 | fifo | 0 | 3 fifo | legacy | + deferred-store FIFO (core 2 never waits; tick-boundary = single-threaded semantics) |
| 4 | fifolz | 0 | 3 fifo | lazy | + event-driven timer (the core-2 compute lever) |
| 5 | free | 0 | 4 free | legacy | no lock at all: upper bound of lock removal (EXPERIMENT, never ship) |
| 6 | fifo32 | 0 | 5 fifo32 | legacy | publish-interval sensitivity (8 -> 32) |
| 7 | tchk | 0 | 3 fifo | check | VALIDATION: lazy timer vs shadow legacy timer, mismatches logged (`tchk=`) |
| 8 | schedlz | 1 sched | 3 fifo | lazy | PCM variant on the best sync + timer |
| 9 | pipelz | 2 pipe | 3 fifo | lazy | " |
| 10 | neon2lz | 3 neon2 | 3 fifo | lazy | " |
| 11 | splitlz | 4 splitacc | 3 fifo | lazy | " |
| 12 | layoutlz | 5 layout | 3 fifo | lazy | " |
| 13 | romfstlz | 6 romfast | 3 fifo | lazy | round 3b: per-slot ROM bank pointers (host-verified bit-exact) |
| 14 | scalaclz | 7 scalacc | 3 fifo | lazy | round 3b: exact register accumulators, scalar (host-verified bit-exact) |
| 15 | romacclz | 8 romacc | 3 fifo | lazy | round 3b: romfast + scalacc (host-verified bit-exact) |
| 16 | layromlz | 9 layoutrom | 3 fifo | lazy | round 3b: layout + romfast (NEON part unchanged; QEMU gate pending) |

After the last run's song end, the kernel stops: silence, idle windows,
and a `sequence_done` line. **Silence is the done signal.** The LED
heartbeat keeps running (a frozen LED means wedge or stall, never
"done"). Every run's totals are written as a `run_total` line at its
song end, so a power cut after run k still leaves runs 0..k-1 complete.

The MiSTer display follows along: the Pi writes a status byte triple
`{0xA2, 0xFF, run}` to I2C slave `0x45` (documented minimal-kernel-only
convention; the byte carries the run number), and draws `VAR nn name`
on the SSD1306 mirror at `0x3C` (best-effort, core 1 only, never in the
audio path).

## Log format

`SD:/shaped-var3-stats.log`, windows flushed every 5 s. Run
`python tools/analyze_log.py SD/shaped-var3-stats.log` for the per-run
table (cycles per MCU step, cycles per PCM tick, deltas against run 0,
flags, timer-check verdict).

Boot line carries `kernel=mt32-shaped-var3`, `pmc2/pmc3=ok|fail` (PMU
cycle counters), `pmev2/pmev3=ok|fail` (PMU event counters: instructions,
branch mispredicts, L1D/L2 refills), and `timer_selftest=pass|FAIL` (the
lazy timer agrees with the legacy timer on a 400k-step pseudo-random
sequence AND the detector catches an injected one-tick event shift).
`runlist` lines describe each run's components. Window lines carry,
among others:

- `var=N` the RUN index; `pcm= sync= timer=` its components
- `pcm_cyc`, `mix_cyc` core-3 PCM task / voice-mix cycles; `ticks` PCM
  ticks rendered -> `pcm_cyc/ticks` = cycles per tick (core-3 comparator)
- `step_cyc` core-2 MCU stepping cycles (excluding lock spins); `steps`
  MCU steps -> `step_cyc/steps` = cycles per step (core-2 comparator);
  `lock_cyc` core-2 lock-spin cycles, `lock_waits`/`lock_cont` lock ops
  and contended ops
- `pubs` cycle-count publishes, `irq0` voice-end IRQs folded via the
  flag, `fifo_hw`/`fifo_full` deferred-store FIFO high water / full waits
  (must be 0)
- `tchk= tchk_cmp= tchk_mism= tchk_first= tchk_code=` lazy-timer shadow
  check (run 7 only; `off` elsewhere)
- `c2_inst c2_brmis c2_l1d c2_l2d` / `c3_*` PMU events per core over the
  window's work
- `pct` the real-time rate (wall-clock based); `devq_low` device queue
  underruns per window (the chop tell)

## Regenerating the song table

`src/songs.h` is generated, not hand-written, and is deliberately **not**
committed: it embeds MIDI event bytes. Regenerate it from your own MIDI
files with `tools/gen_songs.py` (edit the `SONGS` paths at the top of the
script to point at your files; it writes `src/songs.h`).

## Docs

- `docs/variant-review-20261031.md` — review of PCM variants 0-5 against
  their source ideas (correctness incl. a host differential test, whether
  each can reach its goal, headroom), plus the round-3b variants 6-9 it
  queued and the host test tool (`tools/hosttest`).
- `docs/optimization-round3-sync-20261031.md` — THIS round: where
  core 2's cycles go, the sync levers (lock modes, publish interval,
  layout, IRQ0 hand-off) and the lazy timer with their correctness
  arguments, the run table, the validation checks and the decision
  rules. Start here for the current kernel.
- `docs/optimization-round3-20261031.md` — upstream round 3 (PCM
  variants 4/5: split-accumulator and cross-slot NEON; harness gate
  results).
- `docs/a53-optimization-catalog.html` — every A53-relevant optimisation
  surfaced in this investigation, each with why it is or isn't relevant to
  this voice loop, what was tried, and the outcome with numbers. Searchable
  and filterable.
- `docs/a53-execution-research-20261031.md` — the underlying research
  (A53 microarchitecture, LLVM model, FFmpeg/dav1d tricks, checkasm/PMU
  tooling).
- `docs/optimization-20261031.md`, `docs/optimization-round2-20261031.md`
  — round-1 and round-2 harness reports (candidates, gates, numbers).

## Licenses

- This repo's own code: see `LICENSE` (GPLv3, inherited from the parent
  project).
- `circle-stdlib` and `Nuked-SC55` keep their own licenses; they are
  submodules, not vendored copies.
- No ROMs, no MIDI blobs, no SoundFonts are included.
