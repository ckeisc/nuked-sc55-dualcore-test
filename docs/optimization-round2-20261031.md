# Optimization round 2 — mt32-pi Nuked-SC55 voice loop (2026-10-31)

Second round applying the FFmpeg/dav1d A53 clues (research file Appendix 3)
to the voice loop in the QEMU harness. Reports to the parent only; no
hardware kernel, no pushes, no user-facing message.

## Scope and gates

- **E1M1 ONLY this round** (2026-10-31 ~07:57 — BELMO dropped from
  the gate until 100% on one song). Prefix = truncated E1M1 (first 30 s
  song time + 3 s tail, 54,999,240 steps, 54 checkpoints). Full E1M1 =
  164,997,213 steps, 164 checkpoints. Bit-exact gate ABSOLUTE; a failing
  candidate is FAILED with its checkpoint, never tuned goldens.
- Baseline for all deltas: round-1 `opt-20261031/cand-base` (pristine),
  prefix 40,831,173,616 insns, host wall 55.451 s (TCG-caveated).
- Verdicts captured with `grep 'verify:'` (verify exit status is masked
  when piped through `tail`). MIDI/golden paths kept short (ReadGolden
  truncates at 63 chars). Toolchain: GCC 11.3.1 scalarizes NEON
  intrinsics outright — working NEON candidates use volatile asm only.

## Summary

| Candidate | Bit-exact | Grade | Verdict |
|---|---|---|---|
| A. NEON mix-tail v2 (no INS, full-width loads) | GREEN (prefix) | +2.29% insns, +3.9% wall | **REJECTED** — NEON mix-tail direction CLOSED |
| B. Head/tail pair split (software pipelining) | GREEN (prefix + full E1M1) | A53 model +0.19% cycles | **GREEN but NO GAIN** — model-negative, do not build on it |
| C. Prefetch next-tick ROM data | n/a (design only) | harness cannot measure | **QUEUED as HW-design note** — not implemented, see §3 |
| D. Post-increment pointer audit | n/a (audit) | no post-increment loads exist | **NO-OP** — nothing to change |

**No round-2 candidate wins.** The scheduling direction now has two
independent model-negative results (round-1 lambda-pair +0.09%, round-2
pair split +0.19%), and the NEON mix-tail direction has failed in both
shapes. The gating question is unchanged from round 1: the core-2 split
(diag-kernel run) decides whether engine speed work matters at all.

## 1. A — NEON mix-tail v2 (no lane inserts) — REJECTED

**What:** single asm block per slot-tick for the whole mix-tail multi
chain, zero INS. Coefficient vectors built by scalar C code into a
16-byte-aligned stack array (`neon_pack[4]`, pairs packed as `cA |
cB<<32` etc.) and loaded with full-width `ldr q1`/`ldr q2` (vol pair in
q1 low/high halves, pan/rc pair in q2); multiplicands DUP'd full-width
(`.4s`); serial addclip20 chain including the cin rounding bit runs as
scalar instructions inside the block (sx20(x>>k) via SBFX: `sbfx #6,#20`
for sx20(m>>6), `sbfx #13,#19` for sx20(m>>13) — the latter disassembles
as the `asr` alias, which is correct, not a bug); only
sampl/sampr/rc0raw/rc1raw leave via UMOV (unavoidable scalar interface
to the serial sum chain, which stays in order). Serial accumulation
into ram1[31]/rcsum untouched. Single-accumulator (FFmpeg trick 1),
per the brief. Split-accumulator variant was not attempted (serial sum
preserved by design).

**Code:** `opt-round2-20261031/cand-neon2/tree/src/backend/pcm.cpp`
(mix region, `#if defined(NEON_MIX_TAIL_V2)`). Binary md5
`4a5fa16ca8fe3c13e778b4d384e420d6`; PCM_Update 0x2584 → 0x2640; new
code confirmed present in disassembly (`ldr q1`/`ldr q2`, `dup v0.4s`,
w17/w18 sample2/sample3 adds).

**Gate:** GREEN on prefix (54,999,240 steps, 54 checkpoints, 0
failures). Full E1M1 not run: a candidate rejected on prefix does not
need a full run (round-1 precedent).

**Numbers (prefix, plugin counts + host wall, both TCG-caveated):**
total insns 41,764,790,169 vs baseline 40,831,173,616 =
**+933,616,553 (+2.29%)**; PCM_Update +933,616,539 (+5.29%);
≈ +36.8 insns/slot-tick; host wall 57.619 s vs 55.451 s (+3.9%).
WORSE than round-1 v1 (+1.68%).

**Why (from disassembly):** pack construction ≈ 20 insns (extracts +
ubfiz/orr + stp/str + address computation + 2× ldr q); the block itself
is 38 insns vs ≈ 37 for the scalar region it replaces (roughly a wash);
plus `memory`-clobber spill/reload traffic around the block.

**Lesson:** INS was the cheaper construction (8 insns for all
coefficient vectors vs ~20 for pack+load). The FFmpeg full-width-load
trick pays only when data is ALREADY in memory — for computed scalars,
building a memory vector to load it back is strictly more work than
inserting lanes. On A53 wall time the regression would be worse (NEON
ops cost 2–3 cycles, block is a barrier).

**Status: DEAD.** NEON mix-tail is now CLOSED in both shapes (v1 INS
shape, v2 full-width-load shape). Do NOT build a NEON hardware kernel
from either. (The split-accumulator variant remains untested, but it
would need to overcome a +2.29% deficit before it even starts, and
reordering intermediate sums through sx20 is not bit-exact in general
— it would have to pass the gate first, and the gate is the only thing
that could save it. Not recommended.)

## 2. B — Hand-interleaved software pipelining — GREEN but NO GAIN

**What:** the slot loop restructured so voice N+1's ROM fetch issues
while voice N mixes. This is NOT round-1's lambda-pair (which kept
execution order identical) — it changes order across voices: each pair
of slots runs head(s), head(s+1), tail(s), tail(s+1), where head =
per-slot setup → address generator → 4 ROM fetches → DPCM →
interpolation → filter → `ram1[5] = reference` → IRQ check, and tail =
calc_tv ×3 → mix tail → sums → key update → !active cleanup. The
serial addclip20 sum stays in order (tails run in slot order).

**Load-order question (FFmpeg trick 2):** the two orders worth
comparing are the original order (tail(s) before head(s+1)) and the
pair order (head(s+1) before tail(s)). Both are now measured: the
original order IS the baseline (4,286,700 model cycles), the pair order
is this candidate (4,294,669). Neither wins; there is no third order
to test — a "load-first" shape with all heads before all tails would
break IRQ order (heads must stay in slot order, which the pair shape
preserves) and would still not lengthen any load→use distance (see
mechanism below).

**Code:** `opt-round2-20261031/cand-pipe/` (fresh copy of round-1
cand-base, NOT built on neon2 — B is measured against the pristine
baseline). Surgery script `cand-pipe/split-loop.py` (mechanical,
assertion-checked): loop at `pcm.cpp:1105`; head = body lines
1107..1458, tail = lines 1459..1577; boundary is the IRQ-check close /
`int volmul1 = 0;` line. Cross-slot state carried in a 7-field
`PipeCtx` (v3, usenew, newnibble, old_nibble, key, kon, active); all
other body locals are head-internal or tail-internal (tail redeclares
its own ram1/ram2 from slot). Diff vs the pristine tree is exactly the
mechanical transform (198 lines, renames + structure) — no other file
differs. No return/continue in the span (grep-verified); switches'
breaks are unaffected by lambda wrapping. Binary md5
`42f50ee4e9f89f9d0abb0753875eea9c`; no head_slot/tail_slot symbols
(fully inlined); PCM_Update 0x4ecff0 size 0x4ab0 (baseline 0x2584 at
the same address — ~1.9× code growth, an icache consideration for any
future HW run, not a gate failure).

**Safety case (source-verified before building):**
- `pcm.ram2[ram2[7] & 31][0]` is read cross-slot in the head but NEVER
  written anywhere in pcm.cpp (stable during a tick, order-independent).
- `rcadd`/`rcadd2` are computed pre-loop (lines 692, 913–1015) and
  loop-invariant during the slot loop.
- Mix bus zeroed pre-loop (`ram1[31][1]/[3] = 0`, `rcsum = 0`).
- Heads in slot order preserve IRQ order (`pcm.irq_assert`, PushIrq);
  tails in slot order preserve accumulation order.
- Address writeback (`ram1[4] = next_address`, `ram2[8]` b15 writeback)
  is IN the head, so no tail writeback is needed.
- Cross-block fields the tail needs from the head: v3, usenew,
  newnibble, old_nibble, key, kon, active — all assigned in the head
  on every path (both filter branches assign v3; all others assigned
  at declaration points) before the tail reads them.

**Gate: GREEN.** Prefix 54,999,240 steps, 54 checkpoints, 0 failures
(`cand-pipe/verify-pipe.log`); full E1M1 164,997,213 steps, 164
checkpoints, 0 failures (`cand-pipe/verify-pipe-full.log`). The
reorder is bit-exact, as the safety case predicted.

**Grade — ordered-trace + A53 cycle model ONLY (never insn delta or
host wall for scheduling):**
- Tooling: round-1 `trace_plugin.c`/`libtrace.so` (copied to
  `opt-round2-20261031/`) + `a53sim.py` (in-order dual-issue, 2/cycle,
  1 LS/cycle, ALU 1, mul 3, NEON 3, load→use 4 L1, mem RAW exact-addr
  +1, branches consume a slot, no mispredict penalty — ranking-grade
  only, HW A/B is ground truth).
- Window: 5,000,000 insns from insn 20,000,000,000 (mid-song,
  steady-state voice rendering; window in insn count aligns both
  binaries because scheduling does not change the executed count).
  Trace `opt-round2-20261031/trace-pipe.bin` (63,371,450 B);
  disassembly `opt-round2-20261031/dis-pipe.txt` (full objdump).
- Baseline reproduction first (sanity): `a53sim.py trace-base.bin
  dis-base.txt` → 4,286,700 cycles, CPI 0.8573, stall ~1,649,708,
  unknown 0 — EXACTLY round-1's published baseline. Invocation
  validated.
- Pipe: `a53sim.py trace-pipe.bin dis-pipe.txt` → **4,294,669 cycles,
  CPI 0.8589, stall ~1,637,236, unknown 0.**
- **Delta: +7,969 cycles (+0.19%), CPI +0.0016 — NO MODELED GAIN,
  slightly negative.** Window PCM_Update share: baseline 43.97%
  (matches round-1 exactly — method validated), pipe 44.06%.
- (For the record only, NOT the grade: prefix host wall 66.9 s vs
  baseline 55.451 s — TCG distortion, meaningless for scheduling.)

**Why no gain (mechanism, independently of the model number):**
load→use distances WITHIN each head are unchanged by the split (each
head's fetches are still immediately followed by its own DPCM/interp/
filter uses); calc_tv calls and IRQ/filter branches block GCC's
scheduler from moving work across the new boundaries; on an in-order
core, reordering alone creates no overlap — only longer load→use
distances do, and the split does not lengthen any. What the split DOES
add is cost: PipeCtx zero-init + stack round-trips for the 7
cross-fields (head stores, tail loads) + pair-loop control. The model
reflects exactly this: stall cycles actually fell slightly (−12,472)
but total cycles rose — the added instructions cost more dual-issue
slots than the reorder saves. This is the same structural reason
round-1's lambda-pair (+0.09%) failed, now with an added memory
round-trip per slot.

**Status: GREEN but NOT a performance direction.** Queued for
hardware A/B ONLY as a low-priority check (the model is ranking-grade
and HW is ground truth, so a HW run could in principle overturn it —
but the mechanism analysis independently predicts no gain, the code
grew ~1.9×, and there is nothing for HW to find that the model +
mechanism haven't already excluded). Do NOT build a kernel around it.
If the parent wants a scheduling candidate in a future HW A/B sweep,
round-1's sched (+0.09%, smaller code growth) outranks this one.

## 3. C — Prefetch next-tick ROM data — DESIGN ONLY, not implemented

**Why not implemented (three reasons, stated plainly):**
1. The only non-no-op prefetch points (cross-slot samp0, cross-tick
   samp0) require computing a HOST pointer from a ROM address, and
   PCM_ReadROM (`pcm.cpp:33`) maps ROM→host through a ~30-line
   bank/mode switch (config_reg_3d bit 0x20 selects the bank shift;
   MK1/JV880 variants per bank; waverom1/2/3/card/exp). A prefetch
   would have to replicate that switch — a duplicated mapping that
   must track future ReadROM changes, for a hint.
2. The harness CANNOT measure any cache effect (TCG has no cache
   model), so an implementation would be gradeable only as "still
   bit-exact + added insn cost" — which proves nothing about the gain.
   Per the brief, no gain is claimed from QEMU numbers.
3. Expected HW gain is small anyway: per-voice ROM reads advance
   slowly through wave data, so the current region is overwhelmingly
   L1-resident (~4-cycle exposed latency, not DRAM ~150). The
   in-loop alternative (prefetch the current slot's fetches right
   before their calls) is a no-op: prefetch→load distance ~0.
   (A prefetch hint cannot change results by construction — prfm
   never faults — so bit-exactness would be unaffected; that is not
   a reason to add unmeasurable code.)

**HW-design note (queued, not built):** if a future HW A/B wants to
test prefetch, the clean implementation is cross-tick samp0 prefetch
at tick end — next tick's samp0 address is computable then
((hiaddr<<20)|ram1[slot][4]; hiaddr = ram2[7] bits [11:8], untouched
by the key update which only touches bits 15:12 and 5). Implement via
a SHARED address-translation helper (not a duplicated switch);
estimated cost ~5–8 insns/slot-tick. If the parent wants C implemented
despite the caveats, it is a ~30-line change + prefix verify.

## 4. D — Post-increment pointer audit — NO-OP

Baseline `PCM_Update` disassembly contains exactly one post-index
instruction (function-epilogue `ldp x29, x30, [sp], #352`) and one
pre-index (prologue `stp x29, x30, [sp, #-352]!`); zero
post-increment loads in the hot path. GCC already avoids them here.
No variant built, nothing to grade. DEAD as a direction (not a
failure — the audit found the code already in the desired shape).

## What the harness cannot measure (explicit)

- Cache effects: candidate C's entire gain hypothesis, plus any L1/L2
  behavior of the voice loop (the model assumes L1 hits).
- True Pi wall time for ANY candidate (TCG distorts NEON wall time and
  hides in-order stalls; host wall is never a scheduling grade).
- Icache effects of code growth (pipe PCM_Update 0x4ab0 vs baseline
  0x2584; neon2 0x2640) — the model has no icache.
- The core-2 binding question itself: whether the ~39-point core-2
  gap is lock spins or MCU work. That remains gated on the
  diag-kernel run (`kernel8-debug-shaped-diag-20261031.img`, log
  `SD:/shaped-diag-stats.log`, decision rule: lock_us large →
  sync design, not engine speed; lock_us ≈ 0 → MCU work, NEON voices
  credible again). No round-2 result changes that gate.

## What queues for hardware A/B, and what is dead

**Queued (low priority, model-grade only):**
- Round-1 sched lambda-pair (+0.09% model, smaller code growth than
  pipe) — still the best scheduling candidate, if a HW A/B sweep
  wants one.
- Round-2 pipe (+0.19% model) — behind sched; include only as a
  negative-control-style check, not as a hoped-for win.
- C prefetch design note (§3) — needs the shared-helper implementation
  first; no HW kernel from the current tree state.

**Dead (do not revive without new evidence):**
- NEON mix-tail, both shapes (v1 INS +1.68%, v2 full-width-load
  +2.29%).
- TIMER_Clock closed form (round 1: FAILED bit-exactness, caps ~1%
  even if fixed).
- Pipe pair split as a performance direction (GREEN but
  model-negative + mechanism-negative).
- Post-increment loads as a voice-loop issue (audit: none exist).

## Artifacts

- Report (this file):
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/optimization-round2-20261031.md`
- Working notes:
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/opt-round2-20261031/NOTES.md`
- Round-2 work area:
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/opt-round2-20261031/`
  - `cand-neon2/` (candidate A tree + binary + logs)
  - `cand-pipe/` (candidate B tree + binary + logs + `split-loop.py`)
  - `trace_plugin.c`, `libtrace.so`, `a53sim.py` (copies of round-1
    tooling), `measure.sh`, `dis-base.txt`, `dis-base-pcmupdate.txt`,
    `dis-pipe.txt`, `trace-pipe.bin`, `trace-pipe.log`,
    `control-verify-plain.log` (round-1 cand-base control re-verify,
    GREEN, 54 checkpoints, 0 failures)
- Round-1 inputs reused unchanged: `../opt-20261031/` (report, notes,
  traces, disassembly, cand-base tree).
