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

## Status (snapshot 2026-10-31)

Work in progress. This snapshot contains the auto-sequence variant runner
(variants 0-3, see below), the PMU cycle tooling, and the MiSTer display
wiring. Hardware verification of this exact snapshot is pending; follow-up
commits will land as the build is verified and as further variants gate.
Round-3 variants (split-accumulator NEON, restructured-layout NEON) are not
in this snapshot — they join only after passing a bit-exact harness gate.

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
- `shaped-var-stats.log` — written by the kernel (see below).

## What the kernel does on boot

No keys, no input. It auto-starts E1M1 on variant 0 (baseline) and plays it
through, then advances automatically on each song end:

| Variant | Name | What it is |
|---|---|---|
| 0 | baseline | Unmodified mix path (the reference) |
| 1 | sched | Round-1 lambda-pair slot scheduling |
| 2 | pipe | Round-2 head/tail pair split (software pipelining) |
| 3 | neon2 | Round-2 NEON mix tail, full-width loads, no lane inserts |

After variant 3's song end, the kernel stops: silence, idle windows, and a
`sequence_done` log line carrying per-variant totals. **Silence is the done
signal.** The LED heartbeat keeps running (a frozen LED means wedge or
stall, never "done").

The MiSTer display follows along: the Pi writes a status byte triple
`{0xA2, 0xFF, variant}` to I2C slave `0x45` (documented minimal-kernel-only
convention — there is no real SoundFont here, the byte carries the variant
number), and draws `VAR n name` on the SSD1306 mirror at `0x3C`
(best-effort, core 1 only, never in the audio path).

## Log format

`SD:/shaped-var-stats.log`, windows flushed every 5 s. Boot line carries
`kernel=mt32-shaped-var` and `pmc2=ok|fail pmc3=ok|fail` (PMU counter
validation — if a counter fails, its cycle fields report 0, never silently
valid zeros). Window lines carry, among others:

- `var=N` — the variant that produced the window
- `pcm_cyc` — core-3 PCM task cycles; `mix_cyc` — voice-mix portion
  (accumulated per batch, not per tick, so the reads don't distort the
  measurement)
- `step_cyc` — core-2 MCU stepping cycles (excluding lock spins);
  `lock_cyc` — core-2 state-lock spin cycles
- `pct` — the real-time rate (wall-clock based); `devq_low` — device
  queue underruns per window (the chop tell: `sound_drops`/`underruns`
  read 0 while audio chops; `pct` + `devq_low` are the real signals)

## Regenerating the song table

`src/songs.h` is generated, not hand-written, and is deliberately **not**
committed: it embeds MIDI event bytes. Regenerate it from your own MIDI
files with `tools/gen_songs.py` (edit the `SONGS` paths at the top of the
script to point at your files; it writes `src/songs.h`).

## Docs

- `docs/a53-optimization-catalog.html` — every A53-relevant optimisation
  surfaced in this investigation, each with why it is or isn't relevant to
  this voice loop, what was tried, and the outcome with numbers. Searchable
  and filterable; start here.
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
