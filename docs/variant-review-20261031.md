# Review of the six PCM_Update variants against their source ideas (2026-10-31)

Scope: variants 0–5 in `pcm.cpp` (`g_pcmVariants`), each checked against
the idea it implements in `docs/a53-optimization-catalog.html` (C7, C8,
C4, C5, C6) and the round reports, on three questions: is the
implementation a correct realisation of the idea (bit-exact), can the
idea deliver the performance it was meant to, and is there headroom
inside the same idea. The review ends with four new variants (6–9) that
the kernel now queues as runs 13–16, and with the untried ideas that
remain.

Method:

* Each `PCM_Update_*` body was diffed against the baseline `PCM_Update`
  text, so only the transform itself was reviewed (the bodies are
  1000+ lines each, but the diffs are 80–400 lines).
* The NEON blocks were checked instruction by instruction against the
  scalar formulas they replace.
* A host differential test (`tools/hosttest/pcm_diff.cpp`, MSVC) runs
  every scalar variant against the baseline on 300 randomized `pcm_t`
  states × 400 ticks, MK1 and mkII, comparing every sample, every
  interrupt request and the full state. The NEON variants cannot run on
  x86; their review is by reading plus the upstream QEMU gate.

## 0. The finding that cuts across variants 2, 4, 5: slot 31 aliases the mix bus

The slot loop accumulates the mix bus in `pcm.ram1[31][1]` / `[3]` and
`pcm.rcsum`. Slot 31's own registers are `pcm.ram1[31][*]`: when the
loop reaches slot 31 (`reg_slots == 32`), that voice's filter reads
`reg1 = ram1[1]`, `reg3 = ram1[3]` — the bus — and writes its filter
output back into the same words, which the mix tail then reads as the
bus. The emulated chip's register file really does alias them (it is
why the "31" address-generator block before the loop also lives in
slot 31's registers). Any variant that reorders slot 31's head relative
to slot 30's tail (pipe), or that stops writing the bus words (splitacc,
layout, and my scalacc before its guard), is therefore exact only while
`reg_slots ≤ 31`.

Evidence: the host test fails `pipe`, `scalacc` and `romacc` on the
same 10/300 seeds when states may have 32 slots, and passes all of
them when slots are capped at 31; `scalacc`/`romacc` pass all states
once they fall back to the serial loop for 32 slots.

Impact: the SC-55 firmware runs ≤ 31 slots (the split's 800-cycle tick
cap already assumes it), so this does not affect the E1M1 gate or the
kernel runs. It is a latent exactness hole in `pipe`, `splitacc` and
`layout` that the QEMU gate cannot see. The fix is the one-line guard
scalacc uses (serial loop when `reg_slots == 32`); it is not applied to
the upstream variants in this commit because they are upstream's and
two of them cannot be re-verified here.

## 1. Variant 1 — `sched` (catalog C7, lambda-pair slot loop)

* **Idea.** Inline the slot body twice per loop iteration so the
  compiler can schedule slot s+1's independent head (ROM fetches,
  envelope) across slot s's serial mix tail.
* **Implementation.** An `always_inline` lambda called for s and s+1,
  execution order unchanged. Correct: host test PASS on all states.
* **Can it reach the goal?** No. GCC's scheduler works within basic
  blocks; the body is split into dozens of blocks by the DPCM/filter
  switches, the MK1/mkII branch, the IRQ check and three `calc_tv`
  calls, so two inlined copies give it nothing new to interleave. The
  round-1 A53 model said +0.09%; the mechanism says ~0.
* **Headroom inside the idea.** None in C. Interleaving across slots on
  an in-order core needs hand-scheduled asm (or the compiler being able
  to prove the ROM loads independent of the stores around them, which
  the `uint32_t*`/`uint16_t*` aliasing forbids).
* **Recommendation.** Keep for one hardware measurement (it is cheap
  and settles the question); drop from later sequences.

## 2. Variant 2 — `pipe` (catalog C8 / B7, head–tail pair split)

* **Idea.** Software pipelining: issue slot s+1's ROM fetches before
  slot s's mix tail so the fetch latency overlaps the tail.
* **Implementation.** `head(s), head(s+1), tail(s), tail(s+1)` with a
  7-field context struct. Correct for `reg_slots ≤ 31` (host PASS);
  diverges at 32 (§0). The cross-slot safety case in the round-2 report
  is right except for that one omission.
* **Can it reach the goal?** No, for the reason the round-2 report
  found: the split does not lengthen any load→use distance, because
  each head's four fetches are consumed inside the same head, and the
  compiler will not hoist them past `calc_tv` and the stores. It adds
  the context round-trips and doubles code size (icache pressure on a
  32 KB I-cache the A53 model does not see).
* **Headroom.** The only way to realise this idea is to compute slot
  s+1's `samp0`/`newnibble` addresses (they depend only on `ram1[4]`,
  `ram1[0]`, `ram1[2]`, `ram2[7]`, `ram2[8]`) and issue the loads or a
  `prfm` for them before slot s's tail — catalog B8's cross-slot
  prefetch. Expected gain is small (the fetch region is L1-resident
  most of the time) and the harness cannot measure it; only hardware
  can. Not implemented here.

## 3. Variant 3 — `neon2` (catalog C4, per-slot NEON mix tail)

* **Idea.** Vectorise the per-slot multiply chain (volume pairs, pan/rc
  quad) in one asm block without lane inserts.
* **Implementation.** Correct (gate green; the asm maps `multi` to
  `shl/sshr #12; mul`, `sx20(x>>k)` to `sbfx`, keeps the serial
  `addclip20` chain scalar inside the block).
* **Can it reach the goal?** No: +2.3% instructions. Per slot it
  replaces ~37 scalar instructions with ~20 of pack construction plus a
  38-instruction block plus `memory`-clobber spills. On A53 the block
  is also a scheduling barrier and each NEON op costs 2–3 cycles.
* **Headroom.** None; superseded by `layout`, which removes the pack.
* **Recommendation.** One hardware measurement at most; it is the
  negative control for the NEON direction.

## 4. Variant 4 — `splitacc` (catalog C5 / B2, NEON v2 + even/odd accumulators)

* **Idea.** Break the 32-deep serial accumulation chain through memory
  into two register accumulators.
* **Implementation.** Correct for `reg_slots ≤ 31` (gate green on full
  E1M1); the sx20-homomorphism argument in the round-3 report is right:
  `addclip20(a,b,cin) = sx20(a)+sx20(b)+cin`, `sx20` is reduction mod
  2^20, so any grouping gives the same value at every sx20 read, the
  cin bits never depend on partial sums, and the exact partial sums
  stay far from int32 overflow. The one gap is §0 (slot 31 reads the
  bus through its filter; the argument assumed "the bus is read only by
  the accumulation").
* **Can it reach the goal?** No as built: it keeps the neon2 block, so
  it inherits neon2's packing cost (+0.74% instructions, model +2.7%
  per tick). The accumulator change itself is sound and saves ~25
  instructions per slot of bus load/store traffic.
* **Headroom inside the idea.** Yes — drop the NEON block. That is
  variant 7 (`scalacc`) below: baseline scalar mix tail + exact register
  accumulators, host-verified. Whether two accumulators beat one on
  A53 is moot here: the memory round trip was the cost, not the chain
  (and the FFmpeg finding says A53 forwards a single accumulator
  fastest).

## 5. Variant 5 — `layout` (catalog C6, cross-slot NEON mix)

* **Idea.** Store each slot's mix inputs into contiguous arrays and run
  the whole mix chain in `.4s` lanes over groups of 4 slots, so the
  packing cost disappears.
* **Implementation.** The asm was checked line by line against the
  scalar formulas: `sx20` as `shl/sshr #12`; `(int8)(v>>8)` as
  `sshr #8; shl #24; sshr #24`; `(v>>1)&127` as `sshr #1; and 127`;
  `multi` as `mul` on the sx20'd operand; `sample2`/`sample3` as
  `sx20(mv1>>6) + sx20(mv2>>13) + (((mv2>>12)|(mv1>>5))&1)`; `rc0/rc1`
  with the extra `>>5`; the four term/cin derivations; lanes beyond
  `reg_slots` zero and contributing zero. All correct. Same §0
  exposure. Gate green on full E1M1.
* **Can it reach the goal?** Only partly. It is the first NEON variant
  below baseline on instructions (−2.2% total, −5% of PCM_Update) and
  model-positive, but the mix tail it vectorises is ~37 of ~820
  instructions per slot: the ceiling of this idea is about 5% of
  core 3. It cannot by itself move the real-time number.
* **Headroom inside the idea** (none implemented here; all need the
  QEMU gate):
  1. The four injection arrays (`mix_injL/R/C0/C1`) are unnecessary:
     there are at most six injection terms per tick, so they can go
     straight into four scalar totals added at the combine. Saves
     512 bytes of zeroing, 128 stores, 32 vector loads and 8 vector adds
     per tick.
  2. `sx20(x >> k)` for k ≤ 12 is `shl #(12−k); sshr #12` (2 ops) rather
     than `sshr #k; shl #12; sshr #12` (3 ops): ~6 of the ~90 ops per
     group.
  3. Interleave two groups per asm block to cover the 3-cycle NEON
     latency of the serial `sample2 → sample3 → sampl` chain (the
     round-3 report's own follow-up). Needs v20–v29 for the second
     group's temporaries; roughly 30–40% faster vector pass.
  Together perhaps 1–2% of core 3. Worth doing only after the hardware
  run confirms layout's direction.

## 6. Where core 3's time actually goes (why 1–5 moved little)

Round-1 attribution (QEMU, per slot-tick, ~820 instructions):
`PCM_ReadROM` ~90 (5 calls, 11% of the PCM side), `calc_tv` ~66 (3
calls, 8%), the mix tail ~37 (5%), and the address generator + DPCM +
interpolation + filter + IRQ check + key update ~630 (76%), which is
one long serial chain per voice. Variants 1–5 all worked on the 5%
region. The two cheapest untouched regions are the ROM fetch (a real
call with a 30-line bank switch per fetch) and the memory round trips
of the accumulation; both are scalar and bit-exact by construction.

## 7. New variants queued (round 3b)

| # | Name | Transform | Exactness | Host test | Expected (core 3) |
|---|---|---|---|---|---|
| 6 | romfast | Resolve `PCM_ReadROM`'s bank switch once per slot (`PCM_RomBanks`: the two possible banks for address bit 19 clear/set) and fetch with select + indexed load (`PCM_RomFetch`); JV880 and `a ≥ 2^20` fall back to `PCM_ReadROM`. | By construction; `PCM_RomFastSelfTest` compares all 2^24 addresses × both config modes × MK1/mkII | PASS all states; exhaustive ROM test PASS | −8 to −10% (removes ~85 of 90 instructions per slot plus the call's spills) |
| 7 | scalacc | Baseline scalar mix tail with exact register accumulators (the splitacc/layout combine without the NEON block); serial loop when `reg_slots == 32` (§0). | sx20 homomorphism (as C5/C6) + the slot-31 guard | PASS all states | −2 to −4% (8 fewer AGU slots per slot, no `pcm.ram1`/`rcsum` aliasing in the tail) |
| 8 | romacc | 6 + 7 | as above | PASS all states | ~−12% |
| 9 | layoutrom | `layout` + the romfast fetch path (NEON pass unchanged) | NEON part: upstream gate; fetch part: as 6 | not runnable on x86 | layout's gain + romfast's |

These are the kernel's runs 13–16, all on sync profile 3 (fifo) with
the lazy timer, so they are measured where they would ship. The
generator is `tools/pcm_surgery_round3b.py` (assertion-checked textual
surgery, the method rounds 1–3 used). **Variant 9 still needs the QEMU
golden gate before it is trusted; 6–8 should also go through it even
though the host test is far broader than E1M1 on register coverage.**

The boot log now also carries `romfast_selftest=pass|FAIL` (the
exhaustive fetch comparison run against the real ROMs on the device).

## 8. Ideas not yet tried (design notes, in order of expected value)

1. **Local-copy slot registers.** Copy `ram1[0..5]` and `ram2[1..11]`
   into locals at slot start and write back only what the slot modifies.
   Every store through `uint32_t*`/`uint16_t*` today forces reloads of
   every other value of that type (the body re-reads `ram2[6]`,
   `ram2[7]`, `ram2[8]`, `ram1[1]`, `ram1[3]`, `ram1[5]` several times).
   Scalar, bit-exact by construction, host-testable. Plausibly 5–10%
   of core 3; the only change that reaches the 76% region.
2. **Cross-slot `samp0`/`newnibble` prefetch** (catalog B8) using
   `PCM_RomBanks` as the shared address translation the design note
   asked for: ~5 instructions per slot, hardware-only measurement.
3. **`calc_tv` restructuring**: three calls per slot with a branchy
   `type` derivation that depends only on `adjust` (a register) and
   `nfs`; a 256-entry table per speed byte would remove most branches.
   Scalar, host-testable; ~3–5%.
4. **Layout headroom** items 1–3 of §5.
5. **Two-core PCM** (cores 0 and 1 are idle): per-slot heads are
   independent until the accumulation; with exact register
   accumulators the slot set could be split across two cores and
   summed in order. Large redesign; only worth it if core 3 becomes
   binding after the core-2 work.

None of this changes the round-3 conclusion: core 2 is the binding core
and its levers (sync trim, FIFO, lazy timer) are where the 20% is
expected; core 3's variants decide how much headroom is left there.
