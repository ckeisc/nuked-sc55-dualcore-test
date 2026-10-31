# Round 3 (sync + timer) — closing the last 20% on the Pi Zero 2 W (2026-10-31)

This round targets the core that binds: **core 2 (the H8 MCU interpreter
side)**, not the voice loop. Rounds 1–3 (PCM variants 1–5) worked on
core 3's `PCM_Update`; the model-grade numbers for those were ±2%, and
the shaped-var runs put core 2 at 98–99% step-loop wall against core 3 at
73–75% busy. Core 3 has headroom. Core 2 does not. This round adds levers
on core 2 and the coordination between the two cores, keeps every PCM
variant as an orthogonal axis, and extends the kernel so a single boot
measures all of it.

Everything here is **unverified on hardware** (this snapshot was written
without an ARM toolchain or a Pi at hand); the kernel is the test. The
one piece that was executed before commit is the lazy-timer self-test
(see §5) built with a host compiler — the log at the end of this file
says what it reported.

## 0. Bottom line (what to expect, and what decides)

| Lever | Where | Mechanism | Expected core-2 effect | Risk |
|---|---|---|---|---|
| Sync overhead trim (sync profile 1 `trim`) | `split_thread_circle.h`, `minimal.cpp`, `ring.h` | Cache-line padding of the split state and ring indices; raw TASK-level spinlock instead of Circle `CSpinLock` (which runs `EnterCritical`/`LeaveCritical` + WFE); lock timing only on contended ops with unfenced PMU reads instead of 2× `CTimer::GetClockTicks` (ISB + CNTPCT + 64-bit divide) + 2× ISB PMU reads per op; publish `cycles_pub` every 8 steps instead of every step (STLR + cross-core line bounce per step); ring `Avail()` every 16 steps instead of every step; IRQ0 via a flag instead of a cross-core RMW on `interrupt_pending`; idle core 3 backs off between polls. | 20–60 cycles/step (4–10%) | none to correctness (profile 0 reproduces the old behaviour exactly) |
| Per-tick lock hand-off (profile 2 `tick`) | `split_thread_circle.h` | Core 3 still locks the batch but yields the lock at the next tick boundary whenever core 2 announces a wait. Core-2 wait bounded by one PCM tick (~27 µs) instead of one batch. | depends on lock-op rate (log has it: `lock_waits`, `lock_cont`, `lock_cyc`) | none |
| Deferred-store FIFO (profile 3 `fifo`) | `split_thread_circle.h`, `pcm.cpp` (`PCM_WriteDeferred`) | Core 2 never waits: committing `ram1`/`ram2` stores go into a SPSC FIFO; core 3 applies them at each tick boundary before rendering. This IS the single-threaded emulator's ordering (a register write inside an instruction lands before the `PCM_Update` that follows the step). Core 2's own latch reads consult the pending records (read-your-own-write preserved). Status reads lock-free (atomic exchange on `irq_assert`). | removes `lock_cyc` entirely; plus whatever the lock's cache traffic cost | FIFO full (counted, must be 0); audible check |
| Lazy timer (timer mode 1) | `mcu_timer.cpp` | `TIMER_Clock` is ~140 instructions per MCU step (QEMU attribution, round 1: 18.9% of ALL instructions = ~45% of the MCU side). The event-driven version returns after one compare while no timer can match/wrap/re-issue, materializes skipped ticks arithmetically, and runs the ORIGINAL tick body for every event tick. | 100–130 cycles/step (**15–25%**) — the single biggest credible lever | exactness: see §5 (self-test + shadow run + QEMU gate) |
| No lock at all (profile 4 `free`) | — | Upper bound of lock removal; stores can be lost to core 3's read-modify-writes (`ram2[slot][7]`, `ram2[slot][8]`, `ram1[slot][4]`). | ceiling for `tick`/`fifo` | glitches possible; **never ship** |
| Publish-interval 32 (profile 5 `fifo32`) | — | Sensitivity of the publish knob. | small | none |

Decision rule, per run, from `tools/analyze_log.py`:

* The comparator for core 2 is **cycles per MCU step** (`step_cyc/steps`,
  excluding lock spins) plus **lock-spin cycles per step** (`lock_cyc/steps`).
  For core 3 it is **cycles per PCM tick** (`pcm_cyc/ticks`). These keep
  ranking runs after `pct` saturates at 100.
* Real time needs core 2 at ≤ ~500 cycles/step total (2 M steps/s at 1 GHz)
  and core 3 at ≤ ~31 000 cycles/tick (32 kHz). The shaped-var baseline
  measured ~85% real time with core 2 at 99% step-loop wall, i.e. ~580
  cycles/step; the target is a ≥ 20% reduction, ~460.
* A lever is credible when it lowers its comparator by ≥ 5% against run 0
  with **no flags** (underruns, `ring_drops`, `sound_drops`, `fifo_full`,
  `tchk=FAIL`) and the drop holds across the song's windows.
* Run 7 (`tchk`) decides whether the lazy timer may be used at all:
  `tchk=ok` with a large `tchk_cmp` means the lazy timer produced the same
  register state and the same interrupt requests as the legacy timer at
  every compared step of a full E1M1.

## 1. Where core 2's cycles go (the analysis behind the levers)

Numbers from the existing reports, combined:

* QEMU (round 1, prefix): MCU side 42.4% of instructions, PCM side 53.3%,
  `TIMER_Clock` 18.9%. Per MCU step: ~315 MCU-side instructions, of which
  ~140 are `TIMER_Clock`, leaving ~175 for the H8 dispatch + execute,
  `MCU_Interrupt_Handle`, UART/analog and the split glue.
* Hardware (shaped runs): ~85% real time, core 2 at 98–99% step-loop
  wall, core 3 at 73–75% busy. At 1 GHz that is ~580 cycles per step on
  core 2 and ~27 000 cycles per PCM tick on core 3 (26 000 instructions
  per tick in QEMU, so core 3 runs at CPI ≈ 1.0).
* If core 2 also ran at CPI 1.0 it would need ~315 cycles/step = 63% of
  the core, not 99%. The ~265-cycle gap is some mix of (a) interpreter
  CPI above 1 (indirect-branch mispredicts in decoder2's handler
  dispatch, ~9 cycles each on A53) and (b) costs QEMU never saw: the
  per-step STLR publish with core 3 polling the same line, the
  `CSpinLock` critical-section overhead and WFE spins on every PCM
  register store and status read, two `GetClockTicks` (ISB + CNTPCT +
  64-bit UDIV) and two ISB-fenced PMU reads per lock op, the ring
  `Avail()` LDAR per step on a line core 3 writes per sample, the
  `NukedProfileClock()` out-of-line call per step, and false sharing in
  `ThreadSplitStateCircle` (lock word, `cycles_pub`, core-3 stats and
  core-2 stats all on one 64-byte line) and `SkelRing` (both indices and
  the first slots on one line).

The round-1 diag kernel was built to split (a) from (b) but its result is
not in this repo. This round does not need the answer first: it removes
the (b) costs outright (profile `trim`), gives the lock three
alternatives, and attacks the largest (a) item that is NOT the
interpreter itself — the timer — with an exactness proof and a detector.
It also adds the PMU event counters (instructions, branch mispredicts,
L1D/L2 refills, per core, per window) so the next round can see the
interpreter's CPI directly instead of inferring it.

What this round deliberately does not touch: the H8 interpreter's
dispatch and memory-decode paths (a separate round, with the same
run-table mechanism), and the voice loop (PCM variants 1–5 stay as they
are and are re-measured on the best sync + timer combination).

## 2. The sync changes, with their correctness arguments

### 2.1 Publish interval (`publish_interval`, 1 → 8 or 32)

Core 3 renders only ticks ending at or before `published − 800`. A tick
ending at cycle E needs the register writes of every step k with
12(k−1) < E, i.e. 12k < E + 12. Core 3 renders E only after an ACQUIRE
load of a published value P ≥ E + 800; the RELEASE store of P happened
after step P/12, so every step with 12k ≤ P is visible. Since
E + 12 ≤ P − 788 < P, visibility holds for any publish spacing; the
800-cycle cap already provides 66 steps of slack. The kernel caps the
interval at 32 (384 cycles, half a tick) and publishes once more at the
end of every step loop so the last few steps never wait for the next
chunk.

### 2.2 Lock modes

* `BATCH` is the shipped behaviour.
* `TICK`: core 2 sets `writer_want` (RELEASE) before trying the lock and
  clears it once it holds the lock. Core 3 checks the flag at each tick
  boundary (the `DrainBeforeTick` hook every PCM variant already calls);
  if set, it releases, waits until core 2 has acquired, then re-acquires
  (and now queues behind core 2's single store). Bounded, fair, no lost
  wake-ups.
* `FIFO`: `PCM_Write` is refactored into `PCM_WriteImpl<Defer>`;
  `PCM_Write` is `Impl<false>` (unchanged behaviour by construction) and
  `PCM_WriteDeferred` is `Impl<true>`, which performs every side effect
  except the final `ram1`/`ram2` store and returns that store. Core 2
  pushes it (4096-entry SPSC ring, RELEASE/ACQUIRE indices, producer and
  consumer indices on separate lines). Core 3 drains at every tick
  boundary — one ACQUIRE load per tick when empty. Core 2's latch reads
  of `ram1`/`ram2` scan the pending records newest-first before reading
  live memory, so a read after a write returns the written value exactly
  as in the single-threaded engine. Words core 3 mutates (envelopes,
  addresses, key bits) read live, as before. If the FIFO is ever full,
  core 2 waits for core 3 (≤ 1 tick) and `fifo_full` counts it; it must
  stay 0.
* `FREE`: no lock. A core-2 store to a word core 3 is read-modify-writing
  in the same tick (`ram2[slot][7]`: key/nibble update; `ram2[slot][8]`:
  phase/b15/IRQ flag; `ram1[slot][4]`: address write-back; `ram1[1,3,5]`,
  `ram2[8..10]` on voice end) can be lost. It is in the table only to
  bound what lock removal is worth; it is not a candidate.

### 2.3 IRQ0 hand-off

Core 3's `MCU_Interrupt_SetRequest(IRQ0, 1)` is a read-modify-write of
`mcu.interrupt_pending`, a word core 2 also read-modify-writes every step
(timer Include/Exclude, interrupt take) **outside** the state lock. The
shipped design therefore has a lost-update window. With `legacy_irq = 0`
core 3 only sets a one-byte flag (`irq0_req`, RELEASE); core 2 folds it
into `interrupt_pending` at the top of its next step. The 0x3E status
read clears `irq_assert` with an atomic exchange. Profile 0 keeps the
old path so the baseline stays the baseline; every other profile uses
the flag. `irq0` in the window line counts the folds.

### 2.4 Layout

`ThreadSplitStateCircle` is now a static, 64-byte-aligned object with
the lock words, `cycles_pub`, core-3 stats, core-3 PMU snapshots, core-2
stats, FIFO producer index, read-mostly config and the FIFO storage each
starting on their own line. `SkelRing` keeps the producer index (+ its
private copy of the consumer index), the consumer index (+ its copy of
the producer index) and the data on separate lines, and each side
re-reads the other's index only when its private copy says it must
(standard SPSC optimisation; drop-oldest semantics unchanged because the
drop decision is still taken on a fresh read).

## 3. The lazy timer

Legacy `TIMER_Clock` runs `while (deadline < T) { body; deadline += stride; }`
per timer with T = cycles/2. On MK1 the FRT stride is 4 and T advances
6 per step, so each FRT runs 1–2 bodies per step, plus the TMR: ~140
instructions per step.

A tick body does one of two things: an **event** (counter equals OCRA or
OCRB → flags and optional CCLRA reset; or the increment wraps → overflow
flag) or **nothing but `++counter`**. It also re-issues
`SetRequest(src, 1)` for each (enable && flag) pair every tick — observable,
because `MCU_Interrupt_Handle` Excludes the source when it takes the
interrupt and the next tick puts it back until the firmware clears the
flag.

The lazy version computes, from the current state, the index of the
first tick that is not a pure increment: `e = min((ocra − frc) mod 2^16,
(ocrb − frc) mod 2^16, 0xffff − frc)`, or `e = 0` when any (enable &&
flag) pair is set. Ticks 0..e−1 are applied as one jump (`frc += n`,
`deadline += n·stride`, n ≤ e); tick e runs the **original body**. A
timer needs no attention until T reaches `deadline + e·stride + 1`, so
`TIMER_Clock` returns after one compare against the minimum attention
over all four timers. Register reads and writes materialize the affected
timer first (reads: so the value is current; writes: so pending
event-free ticks are applied under the OLD registers, then the attention
point is invalidated). Nothing is ever predicted; only ticks that
provably cannot be events are skipped, which is why this is exact where
round 1's closed form (which skipped per call with its own distance
math) was not — and why its per-step cost is a handful of instructions
rather than round 1's ~120.

## 4. The run table

| Run | Name | PCM variant | Sync profile | Timer | Purpose |
|---|---|---|---|---|---|
| 0 | base | 0 baseline | 0 legacy | legacy | **reference** — shaped-var behaviour, every knob at its legacy value |
| 1 | trim | 0 | 1 trim | legacy | sync overhead trim (layout, raw lock, cheap timing, publish/8, ring/16, IRQ0 flag, backoff) |
| 2 | tick | 0 | 2 tick | legacy | + per-tick lock hand-off |
| 3 | fifo | 0 | 3 fifo | legacy | + deferred-store FIFO (no core-2 waits) |
| 4 | fifolz | 0 | 3 fifo | **lazy** | + lazy timer — the expected best for core 2 |
| 5 | free | 0 | 4 free | legacy | upper bound of lock removal (experiment, not a candidate) |
| 6 | fifo32 | 0 | 5 fifo32 | legacy | publish-interval sensitivity (8 → 32) |
| 7 | tchk | 0 | 3 fifo | **check** | VALIDATION: lazy timer vs shadow legacy over a full E1M1; `tchk=` must stay `ok` |
| 8 | schedlz | 1 sched | 3 fifo | lazy | PCM variants re-measured on the best sync + timer |
| 9 | pipelz | 2 pipe | 3 fifo | lazy | " |
| 10 | neon2lz | 3 neon2 | 3 fifo | lazy | " |
| 11 | splitlz | 4 splitacc | 3 fifo | lazy | " (upstream round-3 variant) |
| 12 | layoutlz | 5 layout | 3 fifo | lazy | " (upstream round-3 variant, the model-positive one) |
| 13 | romfstlz | 6 romfast | 3 fifo | lazy | round 3b (see `docs/variant-review-20261031.md`): per-slot ROM bank pointers |
| 14 | scalaclz | 7 scalacc | 3 fifo | lazy | round 3b: exact register accumulators (scalar) |
| 15 | romacclz | 8 romacc | 3 fifo | lazy | round 3b: romfast + scalacc |
| 16 | layromlz | 9 layoutrom | 3 fifo | lazy | round 3b: layout + romfast (gate pending) |

E1M1 is ~85 s at real time; slower runs take proportionally longer.
Seventeen runs ≈ 26 minutes. The sequence stops after run 16
(`sequence_done`); silence is the done signal, as before.

**How this relates to the upstream PCM variants:** the PCM variants are
one axis of a run; the sync profile and timer mode are the other two.
The sync/timer levers do not appear as new PCM variants and do not
change any existing one. Runs 0–7 all use PCM variant 0 so the core-2
levers are isolated; runs 8–12 take each PCM variant onto the best
sync + timer combination, so the core-3 variants are measured where
they would ship. Adding a PCM variant later is one line in
`g_pcmVariants` (pcm.cpp) plus one line in `g_runs` (minimal.cpp).

## 5. Validation — what the kernel checks, and what the log must show

### 5.1 Timer exactness (the user-flagged risk)

Three independent checks, in increasing coverage:

1. **Boot self-test with fault injection** (`TIMER_SelfTest`, run twice
   in `Initialize`, result on the boot line as `timer_selftest=pass|FAIL`
   with `selftest_plain_mism` and `selftest_inject_mism`). A legacy and a
   lazy timer (each with an interrupt sink word, no `mcu_t` needed) run
   400 000 MCU steps of a pseudo-random sequence: timer register writes
   (clock selects, flag clears, compare values near the counter, counter
   values near the wrap, immediate matches, the TMR CCLR modes and the
   stopped clock), register reads (values compared), and simulated
   interrupt takes (random pending bit cleared on both sides). Per step
   the per-call interrupt-request set and the sink words are compared;
   the full register state is compared at every materialization and at
   least every 256 steps. The plain run must report **0** mismatches.
   The second run injects a fault into the lazy jump — it skips one tick
   PAST the next event, the round-1 failure class — and must report
   **> 0**. A `pass` therefore means both "lazy agrees with legacy on
   this sequence" and "the detector would have caught a one-tick event
   shift". A `FAIL` does not stop the sequence but marks every lazy-timer
   run untrustworthy.

2. **Run 7 (`tchk`)**: the real emulator runs a full E1M1 with the lazy
   timer while a shadow legacy timer is stepped in lockstep (mirroring
   every register read/write). Every `TIMER_Clock` call compares the
   interrupt requests issued; the register state is compared at every
   materialization and forced at least every 1024 calls; every register
   read compares the returned byte. The window and `run_total` lines
   carry `tchk=ok|FAIL`, `tchk_cmp` (compare count), `tchk_mism`,
   `tchk_first` (MCU cycle of the first mismatch) and `tchk_code`
   (which field, per `mcu_timer.h`). `tchk=ok` with `tchk_cmp` in the
   hundreds of millions is the pass.

3. **QEMU harness gate** (not in this repo; the workflow the earlier
   rounds used): `PCM_Write` is `PCM_WriteImpl<false>` and the lazy
   timer is bit-exact by construction, but both should still go through
   the prefix + full-E1M1 golden run before anything ships. Build the
   harness with the patched `mcu_timer.cpp` in LAZY mode (call
   `TIMER_SetMode(timer, TIMER_MODE_LAZY)` after init) and expect 0
   failures, including the checkpoint-22/step-575 000 boot window that
   failed round 1.

### 5.2 Sync correctness and health

* `fifo_full` must be 0 in every window of every FIFO run (a non-zero
  value means core 2 waited on core 3; the FIFO is 4096 deep and drains
  every tick, so this would indicate a stall, not a capacity problem).
* `irq0` (folds) should be of the same order as the legacy run's
  voice-end IRQs; a run with `irq0=0` while notes end means the flag
  path is broken.
* `underruns`, `ring_drops`, `sound_drops` must be 0 once the run keeps
  up; any run with `underruns > 0` at `pct=100` is wedging somewhere.
* `lock_waits` should be identical across profiles 0–2 for the same
  PCM variant (same firmware activity); `lock_cont` and `lock_cyc` are
  what the lock levers change. FIFO/free runs report `lock_waits=0`.
* Listen. The `free` run is expected to produce occasional wrong or stuck
  notes; if it sounds clean that is information too (the RMW windows are
  rarely hit), not a licence to ship it.
* The MiSTer display shows `VAR nn name`; the run names fit the 5×7 font
  table (digits, lowercase, V/A/R).

### 5.3 Measurement validity

* Boot line: `pmc2/pmc3` (cycle counters) and `pmev2/pmev3` (event
  counters) must be `ok`; failed counters report 0 in their fields.
* Core-2 event counters are sampled per step-loop call (~5 ms), core-3
  per batch, as 32-bit deltas, so the 4-second wrap cannot bite.
* `steps` and `ticks` per window let `analyze_log.py` compute the two
  comparators; `run_total` lines carry the per-run sums so a power cut
  after run k still leaves runs 0..k−1 complete.

## 6. Expected outcome and what to do with it

* If run 1 (`trim`) alone drops cycles/step by ≥ 5% and run 4 (`fifolz`)
  by ≥ 20% with `tchk=ok` in run 7: the production port gets profile 3's
  coordination and the lazy timer (after the QEMU gate), and the PCM
  variant choice is read from runs 8–12.
* If `trim` is flat and `tick`/`fifo` are flat but `free` is not: the
  lock traffic matters but something in the hand-off/FIFO costs as much
  as it saves — check `fifo_hw` (a deep FIFO means core 3 drains late)
  and the per-tick hook cost.
* If every sync run is flat and only `fifolz` moves: the gap was MCU
  work, not coordination; the next round is the interpreter (decoder2
  dispatch, `MCU_Read`/`MCU_Write` page-decode fast paths,
  `MCU_Interrupt_Handle` per step), using the new `c2_brmis`/`c2_inst`
  fields to see the mispredict rate directly.
* If core 2 reaches 100% and core 3 becomes binding (`c3cpt` near
  31 000 and `core3_pct` near 100): the voice-loop levers are the
  `layout` variant (run 12), then a restrict/local-copy slot body, then
  a two-core PCM split (cores 0/1 are idle).

## 7. Files touched

* `patches/nuked-sc55-0.8.0-circle.patch` (regenerated from the patched
  tree): `split_thread_circle.h` (rewritten: profiles, layout, lock
  modes, FIFO, IRQ0 flag), `mcu_timer.h/.cpp` (lazy + check + self-test),
  `pcm.h/.cpp` (`PCM_WriteDeferred` via `PCM_WriteImpl<Defer>`, per-tick
  hook in the baseline `PCM_Update`, `SplitAssertIrq0` at all six IRQ
  sites), `nuked_profile.h`/`emu.cpp` (inline profile-clock accessor:
  the per-step out-of-line call is gone).
* `include/minimal.h`, `src/minimal.cpp`: run table, sync profiles,
  static split state, profile-driven step loop and PCM task, PMU event
  counters, timer self-test, `run`/`run_total`/`runlist` log lines,
  extended `win` line, 5×7 font table for the run names, stats file
  `shaped-var3-stats.log`, boot `kernel=mt32-shaped-var3`.
* `include/ring.h`: padded indices with cached counterpart indices.
* `tools/analyze_log.py`: per-run table with the two comparators,
  deltas against run 0, flags, and the timer-check verdict.

## 8. Host self-test log

`mcu_timer.cpp` is host-portable, so `TIMER_SelfTest` was built with
MSVC 2022 (`cl /std:c++20`, stub `config.h`, `__builtin_ctz` shim) and
run on the authoring machine, 3 000 000 MCU steps per run, both step
tables, with the fast-path guard (the lazy timer must have taken its
skip path on at least 1/8 of the calls, or the run counts as a failure):

```
table=MK1     steps=3000000 plain_mismatches=0 inject_mismatches=1887818 -> PASS
table=GENERIC steps=3000000 plain_mismatches=0 inject_mismatches=2062813 -> PASS
```

So on the host: the lazy timer matched the legacy timer's register
state, read values, per-call interrupt requests and pending-interrupt
set at every compared step; the fault-injected build (jump one tick
past an event) was caught; and the skip path was exercised. The same
function runs at boot on the Pi (400 000 steps per run, result on the
boot line). This is evidence about the timer logic, not about the
firmware's use of it — run 7 (`tchk`) and the QEMU golden gate remain
the gates for that.

Two earlier iterations of the self-test sequence are worth recording:
the first version compared state after materializing every step, which
would have hidden a skip-path bug (fixed: compare only at natural
materializations plus a forced compare every 256 steps); the second
never acknowledged timer flags, so the (enable && flag) rule kept the
lazy timer on its legacy-exact slow path and the fast path was almost
never taken (fixed: the simulated interrupt take performs the firmware's
read-TCSR / write-TCSR acknowledge, with 1 in 5 takes left
unacknowledged to keep the re-issue path covered). The guard that
exposed the second problem is now part of the test.
