# Overnight optimization experiments — 2026-10-31 (mt32-pi Nuked-SC55 split)

This work was authorized 2026-10-31 ~02:25 PDT. Two tracks.
No pushes, no changes to shipped kernel images, original trees, step-4
artifacts, or git state. All work in copies under
`hidden_files/opt-20261031/` (candidates) and a copied kernel tree
(`~/workspace/mt32-pi-shaped-diag/`). This report is the deliverable;
The hardware run is the verdict for every candidate.

## 0. Bottom line

| Rank | Candidate | Bit-exact | QEMU insns (prefix) | Host wall (prefix) | Verdict |
|---|---|---|---|---|---|
| — | **sched** (lambda-pair slot loop) | GREEN (prefix + full E1M1 + full BELMO) | n/a — scheduling does not change counts | n/a — host wall is misleading for scheduling | **QUEUED for HW A/B** — A53 model estimates **+0.09% cycles (no gain)** |
| — | **neon64** (asm mix-tail, 64-bit) | GREEN (prefix) | **+1.68%** (regression) | +3.9% | **REJECTED** |
| — | **neon128** (asm mix-tail, 128-bit) | GREEN (prefix) | **+1.68%** (regression) | −1.9% (TCG distortion, see §1) | **REJECTED** |
| — | **timer** (TIMER_Clock closed form) | **FAILED** (2 failures prefix, 2 boot) | −1.04% (diagnostic only, NOT a win) | −3.1% (diagnostic only) | **FAILED — not ranked** |

**No count-changing candidate wins.** Within-slot NEON mix-tail is a
regression at both widths; the TIMER_Clock closed form fails
bit-exactness and would gain only ~1% even if fixed (§4). The
scheduling candidate is bit-exact but the A53 model estimates no gain
(§3). **The gating question remains the core-2 split (Track B, §5):
until the morning run answers whether the ~39-point core-2 gap is lock
spins or MCU work, no engine optimization can be claimed as the
real-time fix.** NEON voices remains justified only as a total-work
reduction, not as the proven fix (step-4 caveat, restated §6).

Baseline for all deltas: `opt-20261031/cand-base` (pristine backup
tree + step-4 harness files), prefix workload (truncated E1M1, 30 s +
3 s tail, 54,999,240 steps): **40,831,173,616 insns, host wall
55.451 s, 0 failures**. This reproduces step 4 exactly (step-4 total
40,831,173,458; +158 insns negligible; PCM 53.26%, MCU 42.42%,
PCM_Update 43.21%, PCM_ReadROM 5.91%, calc_tv 4.04% — all match).

## 1. Measurement basis and caveats (read before the numbers)

- **Guest insns are NOT Pi wall time.** QEMU counts TCG instructions;
  a 128-bit NEON op counts as 1 insn regardless of its A53 latency,
  AGU occupancy, or dual-issue behavior. **NEON insn deltas OVERSTATE
  wall-time gains** (and, here, understate the regression: the NEON
  candidates' +1.68% insn increase would be worse on A53 wall time,
  where NEON ops cost 2–3 cycles and the volatile-asm barriers block
  dual-issue scheduling).
- **Host wall time is distorted by TCG translation** (x86 out-of-order
  execution hides exactly the stalls that matter on the in-order
  A53). It is reported as a second signal only. The neon128 case
  proves the distortion: insns +1.68% but host wall −1.9% — the wall
  number is TCG artifact, not evidence.
- **Scheduling candidates are NEVER graded by insn delta or host
  wall** (parent correction 2026-10-31): reordering changes neither
  the count nor, on x86, the visible stalls. Sched is graded ONLY by
  the ordered-trace + A53 cycle model (§3), and ultimately by
  hardware A/B (shape-kernel pct).
- **Bit-exactness gate is absolute**: goldens green via
  `grep 'verify:'`, never tail output. A failing candidate is
  reported FAILED, never tuned to fit goldens. Prefix = truncated
  E1M1 (30 s + 3 s tail); full E1M1 + BELMO goldens are required
  before claiming engine-semantics success (sched: done, §3;
neon/timer: verdicts stand on prefix results as reported).
- QEMU for the harness is `~/workspace/tools/qemu-user-build/qemu-aarch64`
  (8.2.2). Plugin output goes to QEMU's log (`-d plugin -D <file>`),
  not stdout. MIDI paths kept short (ReadGolden truncates at 63 chars).

## 2. NEON mix-tail (asm, both widths) — GREEN but REJECTED

**What:** vectorize the per-slot `multi()` bundles of the mix tail
ONLY — volume pairs (multiv1/2 from `sample`, multiv3/4 from
`sample2`) and the pan/reverb quad (sampl/sampr/rc0/rc1 from
`sample3`). Fetch/envelope head stays scalar (no gather on A53;
per-voice ROM fetch by phase is a gather and packing eats the gain).
Serial accumulation (`addclip20` chains into `ram1[31][1]/[3]` and
`rcsum`) untouched and in order — cross-slot accumulation reordering
is NOT bit-exact in general (an intermediate sum exceeding 20 bits
wraps when re-read through `sx20`), so it was never attempted.
`multi(a,b) = sx20(a) * b` with int32 wrap maps exactly to
`SHL/SSHR #12` + `MUL` low 32 bits. Code:
`opt-20261031/cand-neon64/tree/src/backend/pcm.cpp` and
`cand-neon128/...` (same call sites, width selected by
`-DNEON_MIX_TAIL_WIDTH`).

**Toolchain finding (important):** the FIRST implementation used NEON
intrinsics. GCC 11.3.1 at `-Ofast -mcpu=cortex-a53` **scalarizes
those intrinsics outright — 0 vector instructions even in
isolation**. Intrinsics are not a viable NEON route on this
toolchain. The shipped candidates use `volatile asm` to force
emission. Vector code confirmed present in the binaries: neon64 has
4× `mul v0.2s` (vol ×2 + pan ×2), neon128 has 3× `mul v0.4s`
(vol ×2 + pan ×1). (Grep lesson: objdump renders asm `ins` as `mov`
and `umov` as `mov`, and backslash-t grep patterns silently break
through this tool layer — verify against full disassembly dumps,
never a single narrow grep.)

**Results (prefix):**

- neon64 (`83347ba3…`): **41,518,178,621 insns (+1.68%)**,
  PCM_Update 18,328,871,411 (+3.89%), host wall 57.623 s (+3.9%),
  **0 failures — bit-exact GREEN, but a regression**.
- neon128 (`042a33de…`): **41,518,178,610 insns (+1.68%)**,
  PCM_Update identical 18,328,871,411, host wall 54.390 s (−1.9%,
  TCG distortion — see §1), **0 failures — bit-exact GREEN, but a
  regression**.

**Why it loses (+27 insns/slot-tick):** pack/unpack overhead (dup +
ins×2/4 + umov×2/4 + shl + sshr ≈ 8–12 asm insns per bundle)
exceeds the 2–4 scalar multis replaced, AND `volatile asm` is a
scheduling barrier (forces multiv values through stack slots instead
of registers, blocks dual-issue scheduling across blocks). Width is
insn-neutral here (64-bit vs 128-bit differ by 11 insns total) —
the bottleneck is packing + barriers, not vector width. This
confirms the pre-analysis (wash or negative) and adds the barrier
cost. **Within-slot NEON mix-tail is rejected as a performance
direction.** The remaining NEON shapes for the voice loop are all
serial chains (sample fetch address-advance, DPCM, interpolation,
filter) — there is no obvious second NEON target in this loop; see
§6 for what this implies.

## 3. Scalar scheduling (lambda-pair slot loop) — GREEN, no modeled gain, queued for HW A/B

**What:** the slot loop body (~465 lines, `pcm.cpp` slot loop) is
executed as an `always_inline` lambda called twice per pair
iteration (`process_slot(s)`, `process_slot(s+1)`, odd remainder
handled), so the compiler can schedule the second slot's
independent head (ROM fetch, envelope) across the first slot's
serial mix tail. Execution ORDER is identical (s, s+1, …), so all
shared-state updates (ram1[31] mix bus, rcsum, slot2 cross-writes)
keep their original sequence. No `return`/`goto` in the body; all
`break`s are inside switches. Code:
`opt-20261031/cand-sched/tree/src/backend/pcm.cpp`.

**Dead end first:** `#pragma GCC unroll 2` on the slot loop has NO
effect (PCM_Update object byte-identical to pristine, 0x2584).
GCC 11.3.1's unroller refuses this body even with
`--param max-unrolled-insns=3000` and `-funroll-loops` (with
`-funroll-loops` only a small inner loop grew, 0x2584 → 0x268c).
The lambda-pair restructure was the replacement that actually
changes codegen: PCM_Update 0x2584 → 0x37b4 (+0x1230), no
`process_slot` symbol (fully inlined).

**Bit-exactness:** prefix **0 failures — GREEN**
(`cand-sched/jit-test-arm`, md5 `67423e6a…`). Full-song goldens:
**E1M1 164,997,213 steps, 164 checkpoints, 0 failures — GREEN;
BELMO 441,939,679 steps, 441 checkpoints, 0 failures — GREEN.**
Full engine-semantics success for the sched candidate (prefix +
both full songs green).

**A53 estimate (the only sched grade, per the parent correction):**
ordered-trace plugin (`opt-20261031/trace_plugin.c` +
`libtrace.so`) + hand-built A53 stall simulator
(`opt-20261031/a53sim.py`). Window: 5,000,000 insns starting at
insn 20,000,000,000 (mid-song, steady-state voice rendering;
window in insn count aligns both binaries because scheduling does
not change the executed count). Model: in-order dual-issue
(2/cycle, 1 LS/cycle single AGU), ALU 1, multiply 3, NEON 3,
load→use 4 (L1 hit assumed, no cache modeling), memory RAW
exact-address +1, branches consume a slot with no mispredict
penalty (path identical in both schedules, so branch costs cancel
in the delta). No llvm-mca (documented wrong for A53).

- Baseline window: 5,000,000 insns → **4,286,700 cycles, CPI
  0.8573**, 43.97% of insns in PCM_Update, 0 unknown PCs.
- Sched window: 5,000,000 insns → **4,290,718 cycles, CPI
  0.8581**, 43.88% in PCM_Update, 0 unknown PCs.
- **Delta: +4,018 cycles (+0.09%) — no gain, within model noise.**

**Why no gain (reasoned argument):** GCC's scheduler reorders
WITHIN basic blocks, and the slot body is fragmented by switches
(DPCM, filter, MK1/JV880 paths) and branches into many small basic
blocks — the pair structure does not create larger scheduling
regions, it duplicates the same blocks. The accumulator chain
(ram1[31], rcsum) serializes slot tails regardless. The stalls the
model does see (~38% of cycles) are load-use chains (ROM fetches,
table lookups) with limited independent work in the same basic
block to fill the latency. **Queued for hardware A/B:**
shape-kernel pct is the ground truth — the model assumes L1 hits
and no icache effects, and a real HW run could differ (including
adversely, if the larger code hurts icache). Do not present the
+0.09% as a Pi percentage; it is a ranking-grade estimate that says
"no modeled gain."

## 4. TIMER_Clock closed form — FAILED (not ranked)

**What:** event-driven skip inside `TIMER_Clock`
(`opt-20261031/cand-timer/tree/src/backend/mcu_timer.cpp`, function
only). A tick that is neither a compare-match nor a counter wrap
only increments the counter, so a run of event-free ticks is
skipped arithmetically and only the event tick runs the original
`ClockFrt`/`ClockTmr` body (flag, CCLRA-clear, overflow, interrupt,
deadline semantics identical by construction; wrap excluded since
rem ≤ d_wrap keeps the counter ≤ 0xffff/0xff). Original bodies
untouched. Edit scope confirmed clean: ONLY `mcu_timer.cpp` differs
from the backup tree.

**Bit-exactness: FAILED.** Prefix: **2 failures** (checkpoint 0 @
step 1,000,000 + final state). Boot: **2 failures** (checkpoint 22
@ step 575,000 + final state, of 400 checkpoints). Control:
pristine tree (`cand-base`) verifies **0 failures** on boot, so the
divergence IS the timer change, not tree drift. HashState covers
only r/pc/sr/sram/ram/dev_register (timer internals are NOT
hashed) and audio matched, so the divergence is a TRANSIENT
firmware-execution phase difference — an interrupt taken at a
slightly different step (state differs at the checkpoint instant,
reconvergences by the next checkpoint). The closed form therefore
shifts SOME timer event by ≥1 step somewhere before step 575,000.
The tick-count / deadline / flag logic was re-proven correct on
paper (distances are mod-2^16/2^8 differences, CCLRA/CCLR and
wrap/overflow cases all handled by running the original body on
the event tick, deadline accounting identical per tick), so the
shift mechanism is NOT yet identified — it is not a logic error
visible by inspection. The decisive next debug step (not done for
time): build baseline + timer debug binaries that dump all timer
fields (frc/tcsr/ocra/ocrb/tcr/stride/deadline, tmr equivalents,
interrupt_pending) at every checkpoint on boot and diff — the
first differing checkpoint localizes the shift, then a step-window
dump around it pins the tick.

**Insn result (diagnostic only — a FAILED candidate is not
ranked):** total 40,407,398,270 (**−1.04%**), TIMER_Clock
7,358,746,020 (18.21%) vs baseline 7,717,003,576 (18.90%)
(**−4.64% of TIMER_Clock**), host wall 53.754 s (−3.1%).
**Why so small (vs step-4's assumed −50% of TIMER_Clock →
−9.5%):** rem is almost always 1–3 ticks (target advances 6/call,
strides ≥ 4), so the per-call distance math (~120 insns:
TicksToTarget + 3 distance computations + branches, ×3 FRTs +
TMR, every one of 55M calls) costs MORE than the 1–3 tick bodies
it skips in the common case. The net −4.6% comes only from rarer
large-rem windows. Step-4's −50% assumed loop overhead dominated;
in fact event-tick bodies + per-call overhead do. **Even if the
bit-exactness bug were fixed, this shape caps at ~1% total —
TIMER_Clock is NOT the easy 9.5% win.** A future timer attempt
would need near-zero per-call overhead (e.g., a cheaper event
test) or a different structure, and must first pass the
bit-exactness gate.

## 5. Track B — diagnostic kernel (for the morning run)

**File:** `~/workspace/kernel8-debug-shaped-diag-20261031.img`
(copy of the tree's `kernel8.img`; tree
`~/workspace/mt32-pi-shaped-diag/`, a copy of
`~/workspace/mt32-pi-shaped-skel/` — the original tree and image
are untouched: shaped-skel `kernel8.img` still md5
`a741249e363af21a1100f1cfd763efe8`).

- **Size:** 582,225 B. **md5:**
  `1f12844ef17e35d25434840deedcf46c` (≠ `a741249e…`).
- **Guards (all pass):** `_end` 0x2ef56c < 0x480000; startup64
  `.word 0x004a0000` (4 MB map); production config (NO_SDHOST
  count 0 in Config.mk); 19 `CI2SSoundBaseDevice` symbols (I2S
  present) and 16 `CI2CMaster` symbols (same as shaped-skel);
  sev present after `stlr` in the built ELF (Circle lock unchanged
  — the Circle lib was not rebuilt, `.done` skipped); decompressed
  image strings: `shaped-diag-stats.log` ×1,
  `kernel=mt32-shaped-diag` ×1, `lock_us=` ×1, `mcu_us=` ×1,
  old names (`shaped-skel-stats.log`, `kernel=mt32-shaped-skel`)
  ×0.
- **Build notes:** stale `src/*.o/*.d` + `kernel8.*` deleted,
  `build-nuked` removed (backend lib rebuilt with the
  instrumentation), `build/circle-newlib` recreated empty, plain
  `make` with the ARM toolchain + ninja on PATH. One build failure
  on the way: this kernel builds with `-Werror=volatile`, and
  volatile compound assignment (`+=`) is an error in C++20 — fixed
  by plain assignment (`x = x + y`) in the header AND, identically,
  in the patch hunk (line-count unchanged, so the Makefile's patch
  step still reverse-verifies and skips). **Patch discipline for
  future diag edits: never edit a patched file without its patch
  hunk — a direct header edit breaks the build (reverse dry-run
  fails on content mismatch, forward apply fails on the new-file
  hunk). The patch hunk's `@@` count was updated 262 → 277 for the
  15 added lines; reverse dry-run exits 0.**

**What it measures:** core-2 `StepUntilFill` time split into MCU
stepping vs state-lock spin waits. `ThreadSplitStateCircle` gains
`lock_wait_us` + `lock_waits`, written ONLY by the MCU thread
(core 2) in `SkelPCMWrite` (committing-store branch) and
`SkelPCMRead` (status 0x3c/0x3e branch) around `Acquire`, timed
with `CTimer::GetClockTicks()` (µs resolution; uncontended
acquires read 0 µs, so `lock_us` counts contended waits and
`lock_waits` counts every lock op). Core-3 batch holds are NOT
counted (they are what core 2 waits ON). The `win` line gains
three fields (appended at the end, existing field positions
unchanged): `lock_us` (window spin-wait wall),
`lock_waits` (window lock ops), `mcu_us` (window step-loop wall
MINUS `lock_us` = pure MCU stepping + peripherals + publish +
ring-check overhead inside StepUntilFill). Log:
`SD:/shaped-diag-stats.log`; boot marker
`kernel=mt32-shaped-diag`. **Flash as `kernel8.img`** (the
SD-card file must be named exactly that). This kernel is
diagnostic-only (two extra timer reads per lock op on the hot
path) — never ship it.

**How to read the morning run (the decision rule):**

- If `lock_us` is a large fraction of the step-loop wall
  (`mcu_us` ≪ step wall, i.e. core2 step time mostly spins):
  the ~39-point core-2 gap is **lock spins** — core 2 waits on
  core 3's whole-batch holds (up to 128 ticks). The fix is sync
  design (finer lock granularity, shorter batches, or lock-free
  publication), NOT engine speed. Do not pursue NEON voices as
  the fix.
- If `lock_us` ≈ 0 (`mcu_us` ≈ step wall): the gap is **MCU
  work** — the H8 decoder2 dispatch costs more per instruction
  on A53 than QEMU insn counts suggest (branch/table-heavy
  dispatch vs pipelinable float voice loops), and/or core-2
  extras (publish, peripherals, ring checks). The fix is
  MCU-side and/or total-work reduction; NEON voices becomes
  credible again as one lever, and the sched candidate's HW A/B
  (§3) is worth running.
- Either way, report sustained `pct` + `devq_low` + `sound_drops`
  as before (the chop metric from the shape runs); the new
  fields explain the pct, they don't replace it.

## 6. What to run on hardware next (in order)

1. **Diag kernel run (the gating measurement).** Flash
   `kernel8-debug-shaped-diag-20261031.img` as `kernel8.img`,
   play E1M1 + BELMO (buttons as in shaped-skel: Button 1
   play/pause, Button 2 cycle from the beginning), and read the
   `win` lines' `lock_us` / `lock_waits` / `mcu_us` per §5.
   This answers the 39-point gap question that blocks ALL
   optimization claims.
2. **Sched HW A/B (only after, or alongside, the diag run).**
   No sched hardware kernel was built in this task (the candidate
   exists only as a harness binary). If the diag run shows MCU
   work (not lock spins) and the A/B is requested, build a
   shape-kernel variant with the lambda-pair loop and compare
   sustained pct against the shaped-skel 83–85% baseline. The
   model predicts no gain — treat the A/B as a check on the
   model, not as an expected win.
3. **Do NOT build a NEON hardware kernel from the mix-tail
   candidates.** Both widths are insn regressions with a
   barrier cost; a hardware run would most likely confirm the
   regression (or, at best, measure TCG-flattering noise). If a
   NEON direction is revisited, it needs a fundamentally
   different shape than within-slot bundles (and the
   scalarization lesson of §2: use asm, not intrinsics, on this
   toolchain).
4. **Timer:** do not revive the closed form without first fixing
   bit-exactness via the checkpoint-dump debug path (§4) AND
   redesigning for near-zero per-call overhead — the current
   shape caps at ~1%.

## 7. Open threads and things NOT done

- **Sched full-song goldens:** DONE — E1M1 + BELMO both green
  (§3). No pending golden runs remain for any candidate reported
  here (neon/timer prefix results stand as reported; neon
  candidates are rejected on performance, timer on bit-exactness,
  so full-song runs would not change their verdicts).
- **Timer 1-step shift:** mechanism unidentified; debug path in
  §4 (checkpoint timer-state dumps, then step-window dump).
- **Known-safe pending win (not tasked, footnote):** item-4
  PCM_ReadROM inlining, −0.76% from optimization round 1 (pushed
  to `feature/nuked-sc55-optimization` as `f8a0a2e`, NOT in the
  harness tree or main). It remains the only previously measured
  safe win not yet in the split; it is small and was not
  re-measured here.
- **Shape follow-ups (blocked, unchanged):** MiSTer check
  (was one connected during either shape run? zero MiSTer lines
  in both logs); idle-state switch ring_drops trigger test;
  end-of-run-#2 pop (deferred 2026-10-31 — power-off
  transient vs pause-drain pop).
- **Step-4 caveat restated:** the Pi binding core is core 2
  (98–99% step-loop wall) vs core 3 at 73–75%, with ~39 points
  unexplained by engine work. Nothing in this task's Track A
  resolves that gap — only the Track B morning run can. Until
  then, every engine-speed claim stays provisional.

## 8. Artifacts produced (all under the goal's hidden_files unless noted)

- This report: `hidden_files/optimization-20261031.md`.
- Working notes: `hidden_files/opt-20261031/NOTES.md`.
- Candidates: `hidden_files/opt-20261031/cand-{base,sched,timer,neon64,neon128}/`
  (each: harness files + `tree/` copy + binary + verify logs;
  `cand-base` is the pristine control).
- Measurement: `hidden_files/opt-20261031/measure.sh`,
  `*-plugin.log`, `*-attr.txt`, `*-wall.txt`, `*-symbols.txt`.
- Trace + model: `hidden_files/opt-20261031/trace_plugin.c`,
  `libtrace.so`, `trace-base.bin`, `trace-sched.bin`,
  `dis-base.txt`, `dis-sched.txt`, `a53sim.py`.
- Diagnostic kernel: `~/workspace/kernel8-debug-shaped-diag-20261031.img`
  (582,225 B, md5 `1f12844ef17e35d25434840deedcf46c`); tree
  `~/workspace/mt32-pi-shaped-diag/`; build logs
  `~/workspace/mt32-pi-shaped-diag-build.log` (failed,
  `-Werror=volatile`) and
  `~/workspace/mt32-pi-shaped-diag-build2.log` (success).
- Untouched: shipped shaped-skel image + tree, step-4 artifacts,
  original backup tree, git state (no commits, no pushes).

