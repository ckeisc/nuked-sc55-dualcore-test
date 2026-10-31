# Pi Zero 2 W execution research (2026-10-31)

Question: is the Pi Zero 2 W's NEON real performance hardware, or
compatibility-only silicon that can't actually run vector code faster?

## The exact chip

Pi Zero 2 W = RP3A0 SiP: Broadcom BCM2710A1 (same silicon as the Pi 3),
4x Cortex-A53 @ 1 GHz, 512 MB LPDDR2.
Sources: Wevolver, Zephyr, Farnell, DigiKey listings agree on all four facts.

## Verdict: the instinct is essentially right

A53 NEON is real hardware (full ARMv8-A 128-bit NEON), not emulation — but
it is a cost-optimized, narrow implementation where NEON only wins for the
right code shapes. For the wrong shapes it ties or loses to well-scheduled
scalar code. There is a documented case of NEON intrinsics at -O3 showing
no significant gain over scalar C on A53.

## The traps, each mapped to our PCM voice loop

1. **In-order, partial dual-issue.** A53 executes strictly in order and can
   issue two instructions per cycle ONLY when they are independent and use
   different pipes. It cannot skip over a stalled instruction. Badly ordered
   code stalls at full latency; well-ordered code wins big. This is the
   core's real 2x lever, and it is a scheduling lever, not a width lever.
   (Community finding: a contiguous-accumulator rewrite that enables the
   forwarding path helps A53/A55 by ~39%, neutral on A73/A76.)
2. **NEON latency ~2-3 cycles, chains stall.** Only VEOR was observed at
   1-cycle latency; typical NEON ops are >=2 cycles, fmul->fadd chains
   stall ~3 cycles. Per-voice envelope/phase/accumulator chains are
   sequential, so naive NEON vectorization of the voice loop stalls at
   full latency. Fix: multiple independent accumulators, interleaved
   voices — helps scalar and NEON alike.
3. **No gather.** NEON has no gather instruction. Our voice loop fetches
   wave ROM per voice by phase — random access, per-voice pointers.
   Vectorizing the fetch head requires scalar loads + lane packing
   (INS/TBL), and that pack overhead typically eats the vector gain.
   The fetch head is NEON-hostile; the mix/accumulate tail (streaming,
   sequential, independent lanes) is NEON-friendly. Split the loop on
   that boundary; do not vectorize it whole.
4. **64-bit load path, 128-bit store path (TRM-documented asymmetry).**
   `ldr q0` takes 2 cycles to issue and does not dual-issue; multi-register
   loads take 2 cycles per register. Stores are 1 cycle per register and
   can dual-issue with non-NEON work. Loading voice data into vectors is
   therefore load-bandwidth-bound even before compute.
5. **Single AGU: one load OR store per cycle.** No dual-issue of
   load+store, or of two 64-bit stores. The voice loop is load-heavy
   (ROM fetch + voice state + envelope tables), so it is AGU-bound as
   much as ALU-bound.
6. **128-bit NEON ops occupy both 64-bit halves.** A full-width NEON op
   blocks all other NEON work that cycle; two 64-bit-half ops on
   independent registers CAN dual-issue. Counterintuitive consequence:
   64-bit vectors (4x16-bit samples) sometimes schedule better than
   128-bit vectors (8x16-bit) on this core. Test both widths.
7. **Memory latency cannot be hidden (no out-of-order).** L1 hit 4 cycles,
   L2 10-12, DRAM ~150 cycles; branch mispredict ~7-10 cycles. Wave ROM
   tables are megabytes, so a voice fetch missing the caches stalls the
   core for ~150 cycles with nothing else to run. ROM-fetch locality
   (and the already-landed item-4 cheap fetch path) matters more than
   ALU width. Software prefetch of the next tick's ROM data, issued
   early, is a real lever on an in-order core.
8. **No public timing documentation.** ARM never published an A53
   Software Optimization Guide (NDA only); all A53 timing knowledge is
   microbenchmark folklore. LLVM's A53 model (llvm-mca) is documented
   as wrong for this core, so compiler auto-vectorization and
   scheduling models are unreliable here. Hand-scheduled code (or
   careful intrinsics + unrolling + multiple accumulators) is where
   the gain is, and the only trustworthy numbers are measurements.

## Consequences for the optimization plan

- The NEON candidate must be SPLIT: NEON for the mix/accumulate tail
  only; scalar + interleaved scheduling for the fetch/envelope head.
  A whole-loop NEON port is likely to repeat the earlier stages' ~3%.
- Add a scalar-scheduling candidate as first-class: interleave 2-4
  voices, multiple accumulators, break chains, align data, prefetch
  next-tick ROM fetches. On A53 this can plausibly beat naive NEON.
- QEMU instruction counts CANNOT adjudicate NEON-vs-scalar on this
  core: they count a 128-bit NEON op as 1 instruction and ignore
  latency, AGU contention, and dual-issue. QEMU ranks scalar-scheduling
  variants; the NEON question needs a hardware A/B run (NEON/scalar
  switch in a kernel run here).
- Do not use llvm-mca for A53, and do not trust compiler -O3
  auto-vectorization claims for this core without a hardware check.

## Appendix: LLVM's A53 model — has it got better? (Challenge noted 2026-10-31)

A challenge to the blanket "LLVM's A53 model is wrong": LLVM is
not static, A53 is popular, and "wrong" is not binary. Findings from
reading current LLVM main's AArch64SchedA53.td (2026-10-31) plus its
commit history:

- Timeline: origin Feb 2014 (D2829, no NEON at first; NEON added in a
  later patch), then incremental small fixes — the last substantive
  timing-related fix found is Oct 2023 (David Green, postinc operand
  order). Touches since (2024–2026) are mechanical: new-feature
  bookkeeping (FEAT_FPv2, C1-Nano, Ampere1B UnsupportedFeatures
  updates), typo fixes. NO systematic re-measurement or rework.
  The Feb 2025 Estevez critique postdates the last substantive fix.
- What is RIGHT in current main's model (read, not assumed): in-order
  (MicroOpBufferSize=0), IssueWidth=2, single LdSt resource (matches
  the single AGU), 2 ALU units, MispredictPenalty=9 (matches measured
  ~7–10), latencies sourced from ARM's own "Cortex-A53 Software
  Optimisation Specification - Instruction Timings v1.0 Spreadsheet"
  (cited in the .td), and multi-register vector load/store issue
  cycles modeled explicitly (VLD1–5: latency 4–8, release 2–5).
- What is WRONG or MISSING (structural, still in current main):
  single FPALU resource — does not model the two 64-bit NEON halves
  dual-issuing; plain 128-bit load occupancy not modeled as 2-cycle
  (Estevez's `ldr q0` complaint); forwarding assumed — the .td
  comment itself admits "These latencies will be incorrect when the
  result cannot be forwarded"; LoadLatency=3 optimistic (assumes
  L1 hit + bypass).
- Tool-scope limits (not model bugs): llvm-mca analyzes a straight-line
  block in isolation — per its own man page it does not model branch
  prediction or fetch/decode, and it never models caches or memory
  addresses. Even a perfect pipeline model cannot capture ROM-fetch
  cache misses or per-voice branches. So "wrong" for our purpose is
  partly "incomplete by design."
- VERIFIED by empirical retest 2026-10-31
  (`llvm-mca-a53-retest-20261031.md`, LLVM 23.1.4, Oct 2026):
  all three Feb 2025 complaints STILL reproduce — `ldr q0`
  1.0–1.04 cycles/load (reality 2.0); fadd+add retire-together
  (add D@3, both E@6; 3.04 cycles/pair steady-state);
  64-bit fadd+ldr no dual-issue (2.05 cycles/pair). Scalar
  control PASSES (independent adds dual-issue, IPC 1.89;
  order-swapped add-first dual-issues; dependent-add forwarding
  works). So the wrongness is LOCALIZED to NEON-follower
  scheduling, not global — the split answer to "has it got
  better?": scalar parts fine, NEON complaints unchanged
  ~20 months later.
- Revised guidance (supersedes the blanket "do not use llvm-mca"):
  llvm-mca's A53 numbers are usable as a rough pipeline sanity check
  for straight-line scalar code, but NOT trustworthy for NEON
  dual-issue, vector-load throughput, or anything involving memory
  or branches — and never as the grading tool for scheduling (own
  cycle model from microbenchmarks, or hardware A/B, remains the way).

## Appendix: reverse-engineering sources + has understanding improved? (Challenge noted 2026-10-31)

The challenge is right: "ARM never published it" does not mean the
knowledge does not exist. Findings:

- The A53-specific spreadsheet is still not public as far as searching
  finds: the only hits for its exact title are LLVM's own .td comment
  (and mirrors). Destevez's NDA claim stands for the A53 doc itself.
- BUT the big transferable source exists and is public: ARM's
  Cortex-A55 Software Optimization Guide (ARM-EPM-128372, Non-
  Confidential since issue 2.0, Jan 2018; issue 4.0, Aug 2022, r2p0,
  48 pp., on documentation-service.arm.com). It has pipeline +
  forwarding paths (§3.1.1), dual-issue (§3.2), load/store + AGU
  (§3.3), FP/NEON (§3.5), full per-instruction tables (§4: loads,
  stores, NEON int/float/misc, FP, crypto, CRC), "similar instruction
  performance" (§5.7) and "A64 low latency pointer forwarding"
  (§5.11). The A55 retains most of the A53 backend (Chips and Cheese:
  same 2-wide in-order formula, 8-stage integer pipeline, inherited
  backend, robust dual-issue) — so the A55 tables are the closest
  thing to an official A53 timing doc.
- Transfer caveats (measured, not assumed): A53 != A55 on some pairs
  (ARM forum measurement: SMLAL.int8 + 64-bit load = 0.97 IPC on A53
  vs 1.72 on A55); A55 has a predecoded I-cache and a perceptron
  predictor vs A53's GShare, so branch/frontend behavior does not
  transfer; even the A55 guide contradicts itself on SMLAL dual-issue
  slots (01 in one place, 00 in another). Rule: A55 tables = strong
  prior, A53 measurements arbitrate conflicts.
- Reverse-engineering timeline (understanding HAS improved):
  2016 ARMageddon (USENIX Security): cache hierarchy, inclusivity,
  LRU vs pseudo-random replacement, eviction costs in cycles on A53
  phones (Snapdragon 410: L1 32 KB 4-way, L2 512 KB 16-way,
  instr-inclusive / data-non-inclusive). 2017 Wiggers et al. (ODROID-C2
  paper): PMU cycle-counter microbenchmarks for instruction
  characteristics ARM neglected to publish (e.g. independent loads =
  2 cycles each; Appendix A findings). 2020–22: Tencent ncnn
  A53/A55 dual-issue wiki (their own benchmarking); Cranelift A53
  SIMD throughput microbenchmarks; Sonos tract article (tight-loop
  issue-latency + result-latency measurements, counterintuitive load
  trick, later adopted by TFLite Ruy's A53/A55 kernel); Hardforum
  latency-hiding experiments (UAI +21.8%, pipelining +28.8% on A53).
  2025 Destevez: systematic NEON microbenchmarks (single AGU,
  64-bit load path, NEON half-pipes, llvm-mca critique). Oct 2026
  VideoLAN Developer Days: ARM engineers via FFmpeg — A53/A55/A520/
  C1-Nano have a dedicated accumulator forwarding path, so
  accumulating into the SAME register is ~40% faster there than
  alternating registers (opposite of naive chain-breaking advice).
- Consequence for our scheduling candidate: do NOT assume multiple
  independent accumulators win on A53. The FFmpeg finding is about
  accumulation chains (scalar FIR; whether it extends to NEON fadd
  mix accumulators is UNMEASURED — test both styles on hardware).
- Still missing: the A53 spreadsheet itself; one consolidated
  per-instruction A53 table (knowledge is scattered across guides,
  papers, wikis, kernels); a full dual-issue pair matrix; forwarding
  coverage per op pair; Pi-specific cache config (SoC cache sizes are
  implementation choices — phone-SoC numbers do not transfer exactly
  to BCM2710A1; do not assert Pi cache sizes without a Pi source).
- llvm-exegesis exists for AArch64 but must run ON A53 hardware under
  Linux with libpfm; no published A53 exegesis tables were found, and
  it cannot run on our bare-metal Circle kernel. A Linux-booted Pi 3
  / Zero 2 W could run it — a separate hardware exercise, not
  available from here.
- Practical rule for our cycle model: cite provenance per number
  (A55-guide vs A53-measured vs LLVM-only); start from the A55
  tables for loads/stores/NEON, validate against A53
  microbenchmarks, and let hardware A/B arbitrate conflicts.

## Sources

- Daniel Estevez, "Coding NEON kernels for the Cortex-A53"
  (destevez.net, 2025-02): in-order/dual-issue, 64/128-bit load/store
  asymmetry, single AGU, NDA-only docs, llvm-mca wrong, fmul->fadd
  3-cycle stall, microbenchmark links (incl. Tencent ncnn A53/A55
  dual-issue wiki).
- AsiaBSDcon 2026, "Notes on Runtime" (Neville-Neil): A53 branch
  mispredict ~7-10 cycles; L1 4 / L2 10-12 / DRAM ~150 cycles.
- systemonchips.com: NEON intrinsics at -O3 no better than scalar on
  A53 (alignment + redundant accumulate caveats).
- Nettle mailing list (Granlund measurements): NEON latency >=2 cycles
  (VEOR excepted), single shift unit.
- Pi Zero 2 W specs: Wevolver / Zephyr / Farnell / DigiKey listings
  (BCM2710A1, 4x A53 @ 1 GHz, 512 MB LPDDR2, RP3A0).

## Appendix 3 — How FFmpeg / dav1d optimize for A53-class cores (2026-10-31, the "additional clues" question)

### The source: VDD26 Session 4, Martin Storsjö, "AArch64 Assembly Tips and Tricks"

Martin Storsjö (FFmpeg + dav1d ARM assembly maintainer) gave the talk at VideoLAN
Developer Days 2026 (Rome, Oct 18). Full report: VideoLAN Developer Days 2026
Rome Report Part 3 (Tokyo Video Tech, Medium). The talk is not a how-to-write-asm
talk; it is decision guidance starting from CPU architecture. Key doctrines:

- **The best code differs from CPU to CPU.** "If you have several machines,
  measure on them in parallel and look for a balance between changes that speed
  one up and slow another down." Small/efficiency cores (A53, A55) "leave plenty
  of room for tuning." Martin says he does NOT read vendors' optimisation guides;
  he works from a feel for which operations are fast vs slow.
- **Scheduling decides performance.** "Issue independent loads first, then line up
  the calculations that use them." "In simple functions, unrolling the loop is
  needed to give the CPU more independent data." "Code with broken dependency
  chains runs faster than code that looks tidy." Macros reduce scheduling freedom:
  "writing plenty of comments and interleaving separate calculations" instead.
- **Post-increment creates dependencies.** A load with post-increment makes the
  next instruction using the advanced pointer wait, and so must calculations using
  the loaded data. Remedies: separate base registers for even/odd rows with
  doubled stride, or alternate x2 vs x2+x3 and advance x2 at the end (more
  instructions, faster on large cores).
- **Tree reduction vs serial chain is core-dependent.** Adding x0..x7 serially is
  a 7-deep chain; a tree reduction is 3 steps — on out-of-order cores. On
  A53/A55 the forwarding path (trick 1 below) reverses this.
- **Measure-or-don't-submit.** "Before submitting a patch, measure it. If it is
  not faster than the existing C implementation, adding assembly has no value."
- **Don't copy x86 control flow.** x86 addressing idioms (e.g. indexing from the
  end of a range) differ from ARM64's. "To learn idiomatic NEON, read small NEON
  functions in FFmpeg."
- **SVE offers little** (until very recently SVE2 was 128-bit-only; variable length
  gives up unrolling). Moot for A53 (no SVE), but confirms NEON is the only vector
  path.

### The three counterintuitive tricks (from Arm engineers, via Martin)

1. **Accumulate into the SAME register (A53/A55/A520/C1-Nano forwarding path).**
   Breaking an FIR accumulation across two alternating registers seems sensible,
   but these cores have a dedicated forwarding path passing the previous result
   straight to the next instruction. Rewriting to single-accumulator made A53, A55
   and A520 ~40% faster, at ~1% cost on out-of-order cores. A510 LACKS the path
   and the same code is much slower there — but A510 is always paired with larger
   cores, so dropping it from consideration is an option. (This is the finding
   already recorded in Part 27 / the Oct 2026 FFmpeg note; VDD26 confirms it with
   the 40% number and the A510 exception.)
2. **dav1d 8-pixel function: start the slow load FIRST, insert another
   instruction, then use the loaded value.** The old code started from a value
   already at hand and deferred the slow load; Arm's suggestion was the opposite.
   Result: a few % SLOWER on efficiency cores, 9–13% faster on large cores. For an
   A53-only target the direction inverts: deferring the load (the old shape) is
   the better bet — but the real lesson is that the optimal schedule differs by
   core class, so both orders must be measured on the Pi, not assumed.
3. **Partial-load width fix: 3x from one instruction.** Where four bytes suffice,
   a load into PART of a 16-byte register leaves the rest unchanged, creating an
   implicit dependency on whatever the previous iteration wrote there. Replacing
   it with a load that REPLICATES the four bytes across the register (LD1R) or
   one that ZEROES the remainder lets the next iteration start immediately.
   One-instruction change: 3x faster, single-digit losses elsewhere; combined
   with trick 2: 3.37x. (Core attribution for the 3x is not pinned in the report;
   do not quote it as an A53 number specifically.)

### FFmpeg's working method (from commit history, not just the talk)

- **checkasm + per-core columns on real hardware.** Every ARM asm patch carries
  before/after checkasm numbers across cores, A53 included. Examples: 2017 vp9
  loop filters 2–6x on A53 (e.g. mix2_v_44_16: A53 5.70x); 2019 vp8
  put_epel16_h6v6: A53 2183.0 → 1729.4 (~21%). Recent (2025) benches run on
  Raspberry Pi 5 (A76) and AWS Graviton 3 — the A53 columns faded as hardware
  moved on; the A53 knowledge lives in the older commits and in the VDD26 talk.
- **Explicit in-order scheduling commits.** 2022-01-04, h264_qpel avg form:
  "Don't use the loaded registers directly, avoiding stalls on in order cores.
  Use vrhadd.u8 with q registers where easily possible." That is the doctrine
  applied: insert independent work between a load and its first use.
- **Runtime dispatch.** dav1d detects NEON/DotProd/I8MM/SVE via getauxval(AT_HWCAP)
  (src/arm/cpu.c) and dispatches to hand-written GAS assembly, with
  size-specific kernels and :128 alignment hints. FFmpeg does the same via
  av_get_cpu_flags. Same pattern as OpenBLAS (per-CPU micro-kernels; OpenBLAS
  changelog notes optimized CGEMM/ZGEMM kernels specifically for Cortex-A53/A55).

### Mapping to OUR voice loop (the additional clues, ranked)

1. **Lane inserts are the partial-load trap (trick 3).** Our rejected NEON
   mix-tail used INS (pack) + UMOV (unpack) per bundle — exactly the
   dependency-creating lane operations the FFmpeg trick avoids. Any future NEON
   attempt must prefer full-width loads (LD1R replicate, full 64-bit loads) over
   lane inserts, and keep the accumulate in full registers. (Scalar GPR note:
   AArch64 W-register writes zero-extend, so scalar code has no equivalent
   partial-register trap — this trap is NEON-specific.)
2. **Hand-interleaved software pipelining is the strongest untested candidate
   class.** Our sched candidate failed because GCC fragments basic blocks and
   the slot body is switch-heavy; FFmpeg's answer is hand-written asm that
   interleaves two independent streams: fetch voice N+1's ROM data while mixing
   voice N. This is NOT the same as the lambda-pair experiment (which kept
   execution order identical). It changes order across voices — which is legal
   here because voices are independent until the final mix sum, and the sum order
   must be preserved (or proven bit-exact under reordering — it is NOT in
   general, since intermediate sums wrap through sx20; the safe shape is
   interleaved FETCH + per-voice work, with the serial addclip20 sum kept in
   order). Measure both load orders (trick 2) on the Pi.
3. **Single-accumulator vs tree for ram1[31]/rcsum (trick 1).** Test both on
   hardware. Prior: single-accumulator wins ~40% on A53-class cores for FIR-style
   accumulation; tree wins on OoO. Our current code already accumulates serially
   — the question is whether the NEON mix-tail (if revived) should keep one
   accumulator or split it. Do not assume multi-accumulator wins (contra the
   naive chain-breaking instinct).
4. **Pointer advance at loop end, not post-increment (post-increment rule).**
   The voice loop advances per-voice phase pointers; a post-increment load in the
   hot loop makes the next pointer use wait. Advance at loop end or use
   even/odd base registers.
5. **Prefetch + layout (OpenBLAS lesson, already in our plan).** Wave ROM tables
   are MBs; on an in-order core a cache miss (~150 cycles DRAM) has nothing to
   hide behind. Prefetch next tick's ROM data early, issued ahead of use — the
   one scheduling lever that does not depend on dual-issue at all.
6. **Meta-clue: we get to be MORE single-minded than FFmpeg.** FFmpeg balances
   across cores and accepts per-core regressions; our target is exactly one core
   (A53 @ 1 GHz in the Pi Zero 2 W). A510/OoO costs are irrelevant to us. Tune
   for A53 alone and measure on the Pi — which is also FFmpeg's rule
   (measure-or-don't-submit; our shape-kernel pct + bit-exact gate is the same
   discipline).

### What does NOT transfer

- Video kernels are streaming throughput loops over large blocks; our voice loop
  is short, branchy, per-voice stateful (envelope, phase), with a 128-tick batch
  bound and a state lock. FFmpeg's unroll-and-interleave doctrine transfers; its
  block sizes and streaming shapes do not.
- FFmpeg's kernels are mostly pure NEON streaming. Ours mixes a scalar
  fetch/envelope head (no gather — stays scalar) with a vector mix tail. The
  FFmpeg playbook agrees with the split: do not vectorize the gather part.
- The 3x / 3.37x / 40% numbers are dav1d/FIR numbers, NOT predictions for our
  loop. They set priors and candidate shapes, not expected gains.

## Appendix 4 — FFmpeg's measurement tooling: checkasm (2026-10-31, cloned FFmpeg master to /tmp/ffmpeg)

Question: does FFmpeg's "measure before submit" rule rest on real A53 profiling
tooling in the dev setup? YES — checkasm, now in its dav1d-derived rewrite
(tests/checkasm/ext/, 2025, Niklas Haas). Key files: ext/src/checkasm.c (engine),
ext/src/perf.c (timer selection), ext/include/checkasm/perf/aarch64.h (pmccntr),
ext/src/perf_internal.h (pmccntr enable), ext/include/checkasm/test.h (bench macros),
ext/src/stats.c (log-mean/log-variance estimator), libavutil/aarch64/timer.h
(classic timer).

### The dev setup IS the target hardware

checkasm is built for the target and run ON it: `./checkasm --bench
--test=<pattern> --function=<pattern> --affinity=<core> --duration=<us>`.
No simulator anywhere in the loop. QEMU is correctness-only (checkasm without
--bench). The FATE network is volunteers running checkasm on their own boards.
This matches our Part 20/25 conclusion: QEMU cannot grade perf.

### Two timing layers — units correction

- CLASSIC (libavutil/timer.h → libavutil/aarch64/timer.h): reads `cntvct_el0`,
  the generic virtual counter (typically 62.5 MHz). Its own comment: "cntvct_el0
  has lower resolution than pmccntr_el0, but is usually accessible from user
  space by default." CORRECTION to earlier messages: the old A53 commit-message
  numbers (vp9 5.70x, vp8 2183.0 → 1729.4) are in cntvct TICKS, not CPU cycles.
  Ratios/speedups are unaffected; absolute "cycles" claims from those numbers
  would be wrong.
- NEW ext perf layer (perf.c, tried in order): (a) direct `pmccntr_el0`
  (isb + mrs) = TRUE CPU cycles, probed with a signal handler (fault → fall
  back); enabled via checkasm_pmccntr_enable (PMCR.E=1, PMCNTENSET bit 31,
  PMCCFILTR bits 30/28 cleared) run on ALL cores; validated (must increment
  within 20 us or it errors, never silently reports zeros). (b) Linux
  perf_event (PERF_COUNT_HW_CPU_CYCLES via perf_event_open). (c) ARM-specific
  perf init (perf/arm.c). (d) gettime-nanosecond fallback.

### Bench methodology worth copying

- Adaptive: iteration count grows toward a time budget (default 1000 us per
  function, --duration= flag) but ALWAYS gathers >= 30 samples even past budget.
- Two loop shapes: SIMPLE (start; N calls; stop) when no asm counter or count
  < 128; ASM (unrolled 32-call batches with inline outlier rejection: a batch
  is kept only if t*count <= sum*4, i.e. batches > 4x the running trimmed mean
  are dropped — kills interrupt/SIMD-warmup outliers).
- NOP calibration: measures an empty call (checkasm_measure_nop_cycles) and
  SUBTRACTS it from every result (adjustedCycles = raw - nop). Timer overhead
  never pollutes the number.
- Statistics: log-mean + log-variance (geometric treatment); reports mode +/-
  stddev, C-vs-asm ratio, and perf_scale (perf-units → nanoseconds) so it
  reports both cycles and ns.
- Correctness + bench unified: without --bench each function is called once with
  validation (bit-exact C-vs-asm comparison); with --bench the same inputs are
  timed. That is our harness's verify + measure pairing, unified.
- CPU affinity flag (--affinity) pins to one core — matters on multicore.

### Port to our bare-metal kernel (proposed, NOT built yet)

- Clock: PMCCNTR_EL0 via isb + mrs, enabled once at boot with checkasm's exact
  3-register setup. NO permission problem on bare metal (privileged kernel; the
  Linux-userspace PMUSERENR problem does not exist). Boot-time validation: if
  it does not increment, log an error, never report zeros.
- What to time: cycles around (a) PCM_Update slot loop per window, (b) the
  voice-mix section, (c) core-2 step loop (cycles version of mcu_us), (d) lock
  spins (cycles version of lock_us). New win-line fields (pcm_cyc, mix_cyc,
  step_cyc) beside the existing us fields. Per-window values → min across song
  windows is the stable comparator.
- NOP calibration: measure the empty timed region once at boot, subtract from
  per-region numbers (matters for short regions like a single tick).
- Phase 2 (small): PMU event counters (PMXEVCNTR) — instructions retired + L1D
  refill + branch mispredict — giving IPC and stall attribution: cycles +
  instructions distinguishes "more work" from "more stalls", which is exactly
  the core-2 39-point question.
- Vehicle: the variant kernel (Button-2 A/B/C/D) — each variant's win lines
  carry its cycle counts. NOT the diag kernel (do not churn the pending gating
  run); NOT the harness (QEMU cannot read PMU meaningfully).
