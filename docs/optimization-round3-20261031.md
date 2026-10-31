# Optimization round 3 — mt32-pi Nuked-SC55 voice loop (2026-10-31)

Third round: two new NEON mix-tail candidates for the variant kernel
(2026-10-31 ~09:39 — split-accumulator + restructured-layout as
variants 4/5, PENDING this harness bit-exact gate; full E1M1 green
required for inclusion). Reports to the parent only; no hardware
kernel, no pushes, no user-facing message.

## Scope and gates

- **E1M1 ONLY** (BELMO dropped from all gates). Prefix = truncated
  E1M1 (first 30 s song time + 3 s tail, 54,999,240 steps, 54
  checkpoints). Full E1M1 = 164,997,213 steps, 164 checkpoints.
  Bit-exact gate ABSOLUTE; a failing candidate is FAILED with its
  checkpoint, never tuned goldens.
- Baseline for all deltas: round-1 `opt-20261031/cand-base`
  (pristine), prefix 40,831,173,616 insns, host wall 55.451 s
  (TCG-caveated). Candidates are compared against the PRISTINE
  baseline, never against each other or against neon2.
- Verdicts captured with `grep 'verify:'` (verify exit status is
  masked when piped through `tail`). MIDI/golden paths kept short
  (ReadGolden truncates at 63 chars). Toolchain: GCC 11.3.1
  scalarizes NEON intrinsics outright — volatile asm ONLY for NEON.
- The full-E1M1 golden was recorded for this round from the pristine
  baseline: `step4/e1m1-full-golden.txt`, spec
  `song:/home/hatch/workspace/step4/e1m1.mid` (byte-identical copy
  of `~/workspace/user/files/M_E1M1.mid`). The golden's
  `audio_hash` + `audio_frames` cover the mix output directly, so
  the gate validates the mix bit-exactly (not just MCU state).
- Grading: insn counts + host wall are TCG-caveated context ONLY,
  never the verdict; A53 cycle-model grading for scheduling
  aspects; NO hardware claims from QEMU (QEMU is correctness-only;
  the parent handles the hardware side). Do not re-litigate
  round-1/2 verdicts.

## Shared exactness case (both candidates)

The mix tail's serial chains are the only reason split/vector
accumulation was previously "not bit-exact in general". The case
that it IS exact here rests on three verified facts:

1. **sx20 is a ring homomorphism.** `sx20(x) = (x<<12)>>12` is
   sign-extension of the low 20 bits: `sx20(x)` is the unique
   representative in [-2^19, 2^19) of `x mod 2^20`. Therefore
   `sx20(sx20(a)+sx20(b)+c) = sx20(a+b+c)` for any integers a, b, c.
   `addclip20(a,b,cin) = sx20(a)+sx20(b)+cin`, so a chain of
   addclip20 steps observes, at each sx20 read, exactly the
   mod-2^20 sum of all raw terms and cin bits — independent of
   grouping. Reordering or splitting the accumulation cannot change
   any value observed through sx20.
2. **Every downstream observation reads through sx20 or low bits.**
   `accum_l/r` are read only via `addclip20` (sx20) in final
   mixing and via `& write_mask`, where `write_mask` is in {0,3,15}
   (pcm.cpp lines 540/563/568) — low bits only, hence determined
   by the value mod 2^20. `rcsum` is read only via `multi()`
   (= sx20) in next tick's pre-loop reverb. The bus
   `ram1[31][1]/[3]` is read only by the replaced accumulation
   itself (verified by grep across the backend) and zeroed pre-loop
   each tick; the state hash excludes `pcm_t`.
3. **Cin bits are grouping-independent.** Every cin in the tail
   (`(sampl>>5)&1`, `rc0&1`, `rcadd[k]&1`, `((mv2>>12)|(mv1>>5))&1`)
   is computed from raw per-slot values or pre-loop constants,
   never from a partial sum. Splitting therefore cannot change any
   cin.
4. **No int32 overflow.** Per-slot terms are `sx20(term)+cin` with
   |term| < 2^19+1; a 2-way split accumulator holds <= ~19 terms
   (< 2^25), a 4-lane vector accumulator holds <= 32 terms plus
   injections (< 2^25), and combined tick totals are < 2^26 —
   all far from int32 overflow. (The original serial sums are
   bounded the same way; the argument needs no wrap behavior.)

What this case does NOT cover (and the gate must): the multi chain
itself (`multi` = `sx20(a)*b`, shifts, byte extracts) must be
reproduced exactly lane-for-lane — that is what the bit-exact gate
checks, and any lane error fails it outright.

## Summary

| Candidate | Bit-exact | Grade | Verdict |
|---|---|---|---|
| 1. splitacc (NEON v2 block + even/odd split accumulators) | GREEN (prefix + full E1M1) | insns +0.74%, model per-tick +2.70% (negative) | **CLEARED** for variant-kernel inclusion (full E1M1 green); lowest performance priority |
| 2. layout (cross-slot NEON, no per-slot packing) | GREEN (prefix + full E1M1) | insns -2.19%, model per-tick -1.19% (positive) | **CLEARED** for variant-kernel inclusion (full E1M1 green); highest performance priority of the round |

## 1. Splitacc — NEON v2 block + split accumulators

**Design.** Reuse the round-2 NEON v2 block unchanged (single asm
block per slot-tick, full-width loads, no lane inserts — proven
green on prefix) and replace ONLY the accumulation: instead of the
serial bus/rcsum chains through memory (`ram1[31][1]/[3]` load /
add / store per slot, 32-deep dependency chain), each slot's terms
(`sx20(term)+cin`, plus its slot2 injection terms under the same
mapping as the original two switches, merged into one) go to an
even/odd accumulator pair held in registers, dispatched by a single
alternating parity branch. Combined once at tick end. The delta vs
neon2 is therefore purely the chain-break effect: no per-slot bus
memory traffic, no 32-deep serial chain, two independent chains
that dual-issue can pair.

**Why split (and why two, not more).** The serial chain was cited
in round 1 as serializing slot tails regardless of scheduling.
Two accumulators halve the chain depth with one predictable branch
and no extra state; four would need two branch bits or a rotating
index for no additional chain benefit at this chain length (the
remaining chain is 16 deep, already below the NEON latency hiding
window the dual-issue core can fill from neighboring slots' heads).
Two is also the FFmpeg same-register-accumulation complement: where
FFmpeg found one accumulator fastest on A53 (forwarding path),
here the chain is ACROSS slots through memory, so splitting the
memory chain — not the register chain — is the mechanism.

**Code:** `opt-round3-20261031/cand-splitacc/` (copy of round-2
`cand-neon2` tree; surgery `splitacc.py`, assertion-checked,
purely additive — diff vs neon2 shows ZERO deletions). Build
define `-DNEON_MIX_TAIL_V2 -DSPLITACC`. Binary md5
`e05fdd1cd7b279f4c2e78d76542d8e93`; PCM_Update 0x4ecff0 size
0x25d0 (neon2 0x2640, baseline 0x2584).

**Gate:** prefix GREEN (54,999,240 steps, 54 checkpoints, 0
failures; host wall 53.581 s — NOT a grade). Full E1M1 GREEN
(164,997,213 steps, 164 checkpoints, 0 failures; verify wall
156.4 s). The split-accumulator exactness case is confirmed by
measurement, not just by proof: the full song's audio hash
(`dcdc6735e13c77a0`) matches the pristine baseline exactly.

**Numbers (prefix, plugin counts + host wall, both TCG-caveated;
baseline TOTAL 40,831,173,616, PCM_Update 17,641,866,415):**
TOTAL 41,131,678,522 (**+300,504,906, +0.74%**); PCM_Update
17,942,371,430 (**+300,505,015, +1.70%**; the 109-insn difference
between the two deltas is attribution noise). Per-slot-tick:
**+11.8 insns** (vs neon2's +36.8). So splitacc recovers
633,111,647 insns vs neon2 (the bus memory traffic + 32-deep
chain cost ~25 insns/slot-tick) but does NOT get below baseline
on instructions — the v2 pack cost (~20/slot-tick) remains and
outweighs the chain saving. Host wall 53.581 s vs baseline
55.451 s (-3.4%, TCG-caveated context only).

**A53 model (window 5M insns from 20B, same tooling as rounds
1-2; baseline 4,286,700 cycles, CPI 0.8573, stall ~1,649,708):**
4,370,105 cycles, CPI 0.8740, stall ~1,638,595, unknown 0. Raw
window delta +83,405 (+1.95%) — but for count-changing
candidates the raw window comparison is confounded (a 5M-insn
window covers fewer ticks when insns/tick rise, unlike sched/pipe
where counts were identical). Normalized per unit of work
(CPI ratio x insn ratio): **+2.70% cycles per tick —
model-negative.** The chain-break does reduce stalls slightly
(1,638,595 vs 1,649,708) but the pack overhead + NEON latency +
barriers cost more issue slots than the break saves. The model
says splitacc is not a performance direction; hardware could in
principle overturn it (ranking-grade model), but the mechanism
and the insn count point the same way.

**Disassembly evidence:** vector code confirmed present in
PCM_Update: the v2 block's `ldr q1/q2`, `dup v0.4s`,
`shl/sshr #12`, 3x `mul v0.4s` (vol pair, vol2 pair, pan/rc quad),
plus `umov` (rendered `mov` by objdump). 30 vector-arrangement
instructions total in PCM_Update.

**Status:** CLEARED for variant-kernel inclusion as variant 4
("splitacc"). Performance numbers below are context for the
hardware run, not a second gate: the parent's brief clears on
full-E1M1 green, and hardware is the verdict on speed.

## 2. Layout — restructured mix data flow, cross-slot NEON

**Design (written before building, per the brief).** The packing
cost that killed v1 (~8-12 insns/slot-tick) and v2 (~20
insns/slot-tick) exists because per-slot coefficients are computed
as scalars and must be packed into vectors before use. The layout
change removes the packing instead of relocating it:

- The slot loop keeps its head (setup, address gen, DPCM,
  interpolation, filter, IRQ check) untouched and in slot order.
  The tail is split: its scalar part (calc_tv x3, sample/pan/rc
  selects, slot2 injection-term resolution, key update, !active
  cleanup) stays IN the loop, in the same iteration as its head —
  so NO cross-slot locals are needed (contrast round-2 pipe, which
  needed a 7-field PipeCtx because it ran head(s+1) before
  tail(s)). The scalar tail stores all mix inputs to nine
  contiguous, 16-byte-aligned, zero-initialized `[32]` arrays
  (`mix_sample/vm1/vm2/pan/rc/injL/injR/injC0/injC1`; `[32]` is
  exact because ram1/ram2 are `[32][...]` and reg_slots <= 32).
- After the loop, a NEON pass runs the whole mix chain in `.4s`
  lanes over groups of 4 slots: coefficient vectors are loaded
  from the contiguous arrays (9 full-width loads per group, no
  per-slot packing — the packing cost amortizes 4x and the 8
  multiplies become 2 vector multiplies per stage). The serial
  addclip20 steps (sample2, sample3) and the term/cin computations
  are lane-wise vector shifts/adds/ands, exactly mirroring the
  scalar formulas (sx20 via SHL/SSHR #12; byte extracts via
  SSHR + SHL/SSHR #24 for signed low bytes, SSHR + AND-broadcast
  for 7-bit fields). Accumulation is into 4 vector accumulators
  (`+w` operands, never hardcoded registers), combined once per
  tick via STR Q + C horizontal sums.
- Lanes beyond reg_slots are zero in all arrays and contribute
  exactly zero terms; key update/cleanup run only for real slots.

**Why the reorder is safe (dependency case, source-verified):**
- `ram1`/`ram2` inside the loop are per-slot pointers
  (`pcm.ram1[slot]`, `pcm.ram2[slot]`): the tail's key update and
  !active cleanup write only the current slot's state, so no later
  slot's head can observe them. The head reads no bus/rcsum/accum
  state (grep-verified: the head touches neither `ram1[31]` nor
  `rcsum`); the bus/rcsum accumulations are replaced by the
  homomorphism argument above.
- IRQ assertions stay in slot order (heads in order); tails never
  read IRQ state, and the MCU processes latched IRQs at SM_Tick
  (next tick), so intra-tick tail/head interleaving is
  unobservable.
- `tv_counter` is read-only during the loop (decremented pre-loop);
  `nfs`/`voice_active` are tick-constant; `rcadd`/`rcadd2` are
  pre-loop constants; ROM is read-only.
- The combine runs after the loop but before the post-loop
  `ram2[31][7] |= 0x20` / `nfs = true` writes, which do not touch
  accum/rcsum — so combine placement relative to them is
  unobservable.

**What was NOT built (exactly one layout design, per the brief).**
Alternatives considered and set aside: a full two-phase
(heads-for-all-slots, then tails-for-all-slots) split — rejected
because it would need PipeCtx-style cross-slot locals for v3/key
state with no benefit once the scalar tail stays in-loop; and a
2-slot group width — rejected because 4-wide matches the NEON
register width and amortizes packing further (2-wide would leave
half the vector idle and double the group-loop overhead).

**Code:** `opt-round3-20261031/cand-layout/` (copy of pristine
round-1 `cand-base`; surgery `layout.py`, assertion-checked,
purely additive — diff vs cand-base shows ZERO deletions). Build
define `-DNEON_LAYOUT`. Binary md5
`40f8dc7bc140102c23955c7e64afd936`; PCM_Update 0x4ecff0 size
0x281c (baseline 0x2584, neon2 0x2640, splitacc 0x25d0 — larger
because the vector pass is a separate loop body, but it is a
single loop body, not 8 unrolled copies, so icache impact stays
small).

**Gate:** prefix GREEN (54,999,240 steps, 54 checkpoints, 0
failures; host wall 69.645 s — NOT a grade; TCG translates the
vector loop poorly, so the host wall overstates layout's cost).
Full E1M1 GREEN (164,997,213 steps, 164 checkpoints, 0 failures;
verify wall 180.8 s). The cross-slot reorder + homomorphism case
is confirmed by measurement: full-song audio hash matches the
pristine baseline exactly.

**Numbers (prefix, plugin counts + host wall, both TCG-caveated;
same baseline):** TOTAL 39,937,005,421 (**-894,168,195, -2.19%**);
PCM_Update 16,747,698,298 (**-894,168,117, -5.07%**; 78-insn
attribution-noise residue). Per-slot-tick: **-35.2 insns**. Layout
is the FIRST NEON candidate in any round to come in BELOW
baseline on instructions — the cross-slot design removes more
work (scalar mix region ~37/slot + accumulation traffic) than
the vector pass + array stores add (~24 + ~15/slot, with the
packing cost amortized 4x). Host wall 69.645 s vs baseline
55.451 s (+25.6%) CONTRADICTS the insn reduction — which is
exactly why host wall is never the grade for NEON: TCG translates
guest NEON to x86 at a much higher per-insn cost than guest
scalar, so wall and count diverge.

**A53 model (same window/tooling; baseline as above):**
4,330,434 cycles, CPI 0.8661, stall ~1,613,242, unknown 0. Raw
window delta +43,734 (+1.02%) — again confounded for a
count-changing candidate (a 5M-insn window covers MORE ticks
when insns/tick fall). Normalized per unit of work:
**-1.19% cycles per tick — model-positive, modest.** Layout is
the only round-3 candidate positive on both measures (insns
-2.19%, model per-tick -1.19%), and the only NEON candidate in
any round positive on either. The gain is modest because the
vector chain is serial within each group (sample2 -> sample3 ->
sampl, NEON latency 3 in the model) — the win comes from
executing fewer instructions, not from hiding latency. A
2-group-interleaved single-block variant could in principle do
better by overlapping groups; it is NOT built here (correctness
first, one design per the brief) and is noted as the natural
follow-up if the hardware run confirms layout's direction.

**Disassembly evidence:** vector code confirmed present in
PCM_Update: 121 vector-arrangement instructions in a single loop
body — 9 `ldr q` (the mix-array loads), 8 `mul` (exactly the
cross-slot chain: mv1/mv2/mv3/mv4/sampl/sampr/rc0/rc1), `dup`
constants, 8 `and` + 2 `orr` in `.16b`, 16 `add`, 4 `st1`
accumulator stores. The `.16b` logicals and `st1` stores are
notable: an early build failed because GAS rejects `.4s` logicals
and `str vN.16b` — both fixed in `layout.py` (logical ops moved
to `.16b`, bitwise-identical; stores moved to `st1`, same data).

**Status:** CLEARED for variant-kernel inclusion as variant 5
("layout"). Performance numbers below are context for the
hardware run, not a second gate, same as splitacc.

## What the harness cannot measure (explicit)

- Cache effects, true Pi wall time for ANY candidate, icache
  effects of code growth — same caveats as round 2.
- The core-2 binding question itself (diag-kernel run, pending
  with the parent). No round-3 result changes that gate.

## Artifacts

- Report (this file):
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/optimization-round3-20261031.md`
- Working notes:
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/opt-round3-20261031/NOTES.md`
- Round-3 work area:
  `~/workspace/goals/mt32-pi-nuked-sc55-engine/hidden_files/opt-round3-20261031/`
  - `cand-splitacc/` (candidate 1 tree + binary + logs)
  - `cand-layout/` (candidate 2 tree + binary + logs)
- Shared inputs: `../step4/e1m1-30s-golden.txt` (prefix),
  `../step4/e1m1-full-golden.txt` (full, recorded this round from
  pristine cand-base), `../opt-20261031/` (round-1 baseline).
