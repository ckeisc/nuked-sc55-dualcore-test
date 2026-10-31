//
// minimal.cpp -- mt32-pi-shaped-skel kernel app
//
// Design: the PROVEN skeletal JV880-shape sync (togglefix baseline)
// inside a kernel with mt32-pi's outward shape (ported from the old
// shaped kernel):
// - REAL audio device. Frame demand is device-paced: core 2 waits
//   until the I2S device queue has FREE space for a full 256-frame
//   chunk at 48 kHz, steps the MCU until the native ring holds what
//   the streaming resampler needs for that chunk, resamples 32 kHz
//   -> 48 kHz with the port's resampler verbatim, converts float ->
//   s32 exactly like mt32-pi's AudioTask, and Writes. The device
//   clock (48,000 frames/s) is the independent reference: the
//   window pct is wall-clock due vs produced, so a slow engine
//   shows even though the device queue absorbs transient slips.
//   The User Port carries NO audio; sound leaves via the Pi's own
//   I2S output, NOT through the MiSTer.
// - The bounded 16384-frame native ring lets the producer (core 3)
//   run ahead of the drain; the drain is deliberately NOT lockstep
//   frame-by-frame, so the cores' parallelism is preserved (core 2
//   steps the MCU while core 3 renders PCM batches concurrently).
//   A transient slip smaller than the ring + device queue is
//   absorbed; a sustained deficit is the failure.
// - DMA starvation semantics: if the device software queue drains
//   fully, DMA may terminate (StateIdle) and later Writes do NOT
//   restart it. So the loop restarts explicitly: after a Write, if
//   the device is inactive, it waits 20 ms (Terminating -> Idle
//   takes one DMA buffer period; Start during Terminating asserts)
//   and then calls Start() -- this covers pause/resume (DMA dies
//   during every pause) and starvation recovery alike. A Start()
//   failure is never retried (a Failed DMA state asserts on Start);
//   the kernel keeps stepping + writing into the dead queue and the
//   log shows it.
// - Cycle/pause pipeline latency is honest, not hidden: a song cycle
//   clears the NATIVE ring, but up to one device queue (2048 frames,
//   ~43 ms) of already-resampled old audio still plays out before
//   the new song is heard; pause likewise tails out up to ~43 ms.
//   There is no queue-flush API in Circle, so this tail is inherent
//   to any chunked-write design, mt32-pi's included.
// - EVENT LOGGING: every incoming CONTROL event is logged to
//   shaped-var3-stats.log with name + parameters + t= timestamp:
//   MiSTer I2C init/down/synth/romset/soundfont/mute/writefail,
//   variant advances, and sequence_done. The MIDI stream is NEVER
//   logged (hundreds of events/s would flood). No OSD text is
//   attempted: the 3-byte status protocol has no text field, and
//   the MiSTer's own fixed pop-ups on status change are unchanged.
//   The variant NAME travels via the SSD1306 mirror (0x3C,
//   best-effort, core 1 only); the variant NUMBER travels via the
//   status byte (0xA2 hack, see below).
// - NO HARDWARE KEYS (2026-10-31 ~09:23): core 0 spins inertly.
//   The variant auto-sequence is: boot variant 0 (baseline) E1M1 ->
//   song_end -> variant 1 (sched) E1M1 -> song_end -> variant 2
//   (pipe) E1M1 -> song_end -> variant 3 (neon2) E1M1 -> song_end ->
//   variant 4 (splitacc) E1M1 -> song_end -> variant 5 (layout) E1M1
//   -> song_end -> STOP (post-song_end idle: silence, clean idle
//   windows, LED heartbeat unchanged, never restart, never loop).
//   The advance
//   path (DoRunAdvance) is SEPARATE from the deleted song-toggle
//   DoSwitch: same Reset + GS_RESET + InitSplitState(true) +
//   resampler reset, but NO song change, NO cycle line, NO abort
//   line (the old variant's song completed).
// - MiSTer VARIANT DISPLAY (2026-10-31 ~08:00 design): the
//   status NUMBER is the soundfont byte: core 1 writes
//   {0xA2, 0xFF, variant} to slave 0x45 on init, on received-status
//   change, and on variant advance (polled, <= 10 ms after the
//   advance). 0xA2 is a DOCUMENTED minimal-kernel-only hack:
//   upstream cores only know 0xA1 (MT-32) / 0xA2 (SoundFont), so
//   the truthful 0xA4 would likely display as Unknown, and there
//   is no real SoundFont here -- the byte carries the variant
//   number, nothing else. The variant NAME is drawn on the SSD1306
//   via slave 0x3C (MiSTer-simulated display) at the same
//   occasions, best-effort only (NACK/no display tolerated
//   silently; core 1 only, never the audio path).
// - PMU CYCLE TOOLING (FFmpeg checkasm port): per-core PMCCNTR_EL0.
//   Each core enables its own counter once at loop start (PMCR.E=1,
//   PMCNTENSET bit 31=1, PMCCFILTR bits 30+28 cleared), never
//   resets it, validates it (spin until it changes or ~1 ms wall
//   timeout), and measures a back-to-back double-read NOP cost.
//   Core 2: step_cyc (MCU stepping only = StepUntilFill wall minus
//   lock spins, NOP-corrected) + lock_cyc (state-lock spins only,
//   NOP-corrected per op). Core 3: pcm_cyc (total PCM task work,
//   same scope as pcm_task_us) + mix_cyc (voice-mix portion per
//   batch, inside the PCM_Update variants). The boot line carries
//   pmc2/pmc3 ok|fail; a failed counter's fields report 0.
// - SHAPED-SKEL FIX 1 (device-drain failure, critical path): the old
//   shaped AudioLoop waited for device space with
//   `while (GetQueueFramesAvail() < OutChunkFrames) Yield();` -- but
//   GetQueueFramesAvail() returns QUEUED frames, not free space
//   (driver: GetQueueBytesAvail() = InPtr - OutPtr = data waiting to
//   be consumed; the working mt32-pi app computes free space as
//   QueueSize - Avail). So the old loop waited for the queue to
//   FILL (>= 256 frames QUEUED), then wrote a fixed 256-frame chunk
//   into the nearly-full queue: chronic short Writes (the run log's
//   369 sound_drops, got 1024 = 128 frames = one DMA buffer, or 0),
//   with the queue pinned full because drain ≈ producer add. This
//   kernel waits for FREE space instead:
//   `while (GetQueueSizeFrames() - GetQueueFramesAvail() < chunk)
//   MiniPause();`
//   Free space can only GROW between the wait exit and the Write
//   (the DMA handler dequeues = frees; this loop is the sole writer),
//   so the Write that follows is guaranteed full: sound_drops are
//   impossible by construction, not just unlikely. Any residual
//   drain slowness then shows as PACING (the loop waits longer for
//   free space; window chunk counts + pct reveal the true rate),
//   never as click-then-silence.
//   What was NOT proven from code (and is NOT claimed here): the exact
//   DMA drain rate implied by the old log (~10-17k frames/s from drop
//   timing). The I2S driver, clock, IRQ setup and device params are
//   byte-for-byte the working mt32-pi app's path on this board
//   (the config: output_device = i2s), so no code defect was found
//   that provably throttles DMA; the shape run's stats (wall-clock
//   pct, devq_maxavail, sound_drops) measure the true rate on
//   hardware. If sound is slow/gappy but present, the drain rate is
//   the follow-up, with hardware instrumentation (DMA IRQ counts)
//   as the next step -- not another blind code change.
// - SHAPED-SKEL FIX 2 (Yield hang): the old shaped kernel registers
//   no CTasks, yet its KeyLoop, MisterLoop and AudioLoop wait all
//   called CScheduler::Get()->Yield() unconditionally -- and Yield()
//   never returns with zero tasks (scheduler.cpp: GetNextTask()
//   always yields MAX_TASKS with m_nTasks == 0). Cores 0/1 were
//   therefore stuck from boot, so the old shaped buttons + MiSTer
//   link could never run (consistent with zero key/mister events in
//   its run log). Every Yield in this kernel is MiniPause() (ARM
//   yield hint -- the proven minimal-kernel fix). This file does not
//   even include <circle/sched/scheduler.h>, so a Yield call cannot
//   sneak back in unnoticed.
// - SHAPED-SKEL FIX 3 (devq semantics): the old shaped window code
//   measured devq_maxavail/devq_low on Avail (QUEUED), which inverted
//   both fields against their comments. Here devq_maxavail = peak
//   FREE space (near 2048 = device nearly starved) and devq_low =
//   chunks with fewer than one chunk QUEUED at Write time (device
//   nearly starved). Same field names, corrected computation.
// - Sync is the skeletal JV880 shape, UNCHANGED from togglefix (see
//   split_thread_circle.h): both cores share one pcm_t; core 2
//   publishes mcu.cycles (RELEASE) after each StepMCU; core 3
//   ACQUIRE-loads it and renders bounded batches (at most 128
//   ticks, capped at published-800) under the state lock; the
//   sample ring is Sterr1's lock-free monotonic-index discipline
//   (drop-oldest). The unconditional PCMLoop (togglefix fix) is
//   kept verbatim: gating on m_bSplitActive starved the parked
//   branch and wedged the first toggle in DoSwitch (skel runs
//   #2/#3). MK1 v1.21 only, decoder2 only, oversampling off (native
//   32 kHz).
// - Thread discipline: while the split runs, ONLY core 2 touches
//   the emulator's MCU side and ONLY core 3 touches its PCM side;
//   core 0 touches nothing (inert spin, no GPIO reads, no events);
//   core 1 never touches the emulator or the file, it only posts
//   MiSTer events and sends the variant status + LCD draw. The song
//   event tables (songs.h) are read-only shared data; the cursor
//   lives on core 2, which posts events at exact emulated-cycle
//   boundaries.
// - The stats file is written ONLY by core 2, so no file lock is
//   ever needed and no other core can wedge on it. Every line is
//   f_sync'd: the log must survive a power cut, since the operator reads the
//   card after.
// - No priming burst at init: like the harness song start, the engine
//   goes Reset -> GS_RESET -> split activation with MCU cycles at 0,
//   so the song tables (cycles absolute from 0) post exactly. The
//   audio loop pre-fills the native ring (4 chunks) and Writes two
//   output chunks into the device queue BEFORE Start(), so DMA
//   starts with data queued (Start with an empty queue fails).
// - INHERITED WART (togglefix full-pass run, NOT fixed here by
//   design): ring_drops=15832 appeared exactly at the toggle window
//   and then stayed flat (no new drops in 80 windows). One-time
//   ring overflow at the switch, mechanism unpinned (published-cap
//   reasoning says a core-2 stall alone cannot fill the ring). The
//   counter is kept and logged exactly as togglefix does, so the
//   wart stays measurable; the ring and the switch are NOT
//   redesigned to chase it in this build.
// - ENGINE CEILING (read the windows with this in mind): togglefix
//   measured the engine at ~85% real-time (533 chunks/5 s vs 625
//   needed at 32 kHz). The shape loop demands 48 kHz output through
//   the resampler from the same engine, so sustained window pct
//   ≈ 80-85 is the EXPECTED good outcome (due grows at 48k, prod at
//   ~40k), NOT a failure -- and underruns (silence-filled output)
//   stay small or zero while the engine keeps up with demand. The
//   drain-fix criterion is sound_drops = 0 + audible sound, not
//   pct = 100: pct = 100 needs a real-time engine (NEON voice work,
//   queued separately).
//

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

#include <circle/actled.h>
#include <circle/interrupt.h>
#include <circle/memory.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/synchronize.h>
#include <circle/timer.h>
#include <fatfs/ff.h>

#include "minimal.h"
#include "songs.h"

// Nuked-SC55 backend headers are only included here (this file is compiled
// as C++20; see Kernel.mk). The compat shim must come first: it supplies
// the C++23 library functions the bare-metal libstdc++ lacks (see the
// header itself); it is also force-included into the backend build.
#include "nukedcirclecompat.h"

#include "emu.h"
#include "pcm.h"
#include "rom.h"
#include "split_thread_circle.h"
#include "mcu_timer.h"
#include "rom_io.h"
#include "sha256.h"
#include "standard_romsets.h"

// Dual-core split state (see split_thread_circle.h): owned by the app
// instance (m_pSplitState), published here while the split is active so
// the backend's dispatch and hooks can reach it. Null whenever the split
// is inactive -- the backend then calls PCM_Read / PCM_Write directly.
ThreadSplitStateCircle* g_splitT = nullptr;

// ROMs live on the SD card only in this kernel (no USB stack).
const char MinimalROMDirectory[] = "roms/sc55";

// Sanity limits for the ROM directory scan (same as the port: a complete
// SC-55 set is five files totalling ~3.3 MiB).
constexpr size_t MaxROMFiles    = 24;
constexpr size_t MaxROMFileSize = 4 * 1024 * 1024;

// Step budget per chunk (same formula as the port with ratio 1: the
// port's (nFrames * ratio + 2) * 300, i.e. ~5.8x headroom over the
// ~13,312 MCU steps a 256-frame chunk needs). Budget exhaustion with a
// short ring IS the starvation the stats measure -- it never loops
// forever on a dead PCM task.
constexpr u64 StepsPerFrameBudget = 300;

// Variant auto-sequence (2026-10-31 ~09:23/09:29): NO hardware
// keys. Variant numbers: 0 baseline, 1 sched, 2 pipe, 3 neon2.
// Boot starts variant 0; each E1M1 song_end advances (reset +
// E1M1 from the beginning); after variant 5's song_end the kernel
// stops into post-song_end idle. Button GPIOs are NEVER read.
// The variant table itself lives in pcm.cpp (g_pcmVariants,
// data-driven per 2026-10-31 ~09:39); MaxVariants sizes the
// per-variant accumulator arrays (headroom for the pending
// variants 4/5 -- adding them extends sequence_done's explicit
// fields mechanically, no dispatch rewrite).
// (Round 3) MaxVariants is gone: the RUN table below (g_runs) drives the
// auto-sequence; per-run totals are scalar accumulators, printed per run.

// MiSTer link: Pi is the I2C master, MiSTer User Port slave at 0x45,
// 3-byte status {synth, romset, soundfont}. Poll at 10 ms (5x
// mt32-pi's 50 ms). Synth byte values from the port's misterstatus.h.
// Writeback is sent on init, on received-status change, AND on
// variant advance (polled): always OUR variant reality {SoundFont
// 0xA2, 0xFF, variant} -- the DOCUMENTED minimal-kernel-only hack
// (truthful 0xA4 likely displays as Unknown; no real SoundFont
// exists here). Every other selection is logged and IGNORED, never
// acted on. Mute counts as link-active (a successful Read) but is
// otherwise ignored. The SSD1306 mirror lives at slave 0x3C
// (MiSTer-simulated display; variant name only, best-effort).
constexpr u8 MisterI2CAddress = 0x45;
constexpr u8 MisterLCDAddress = 0x3C;
constexpr u64 MisterPollUs = 10000;
constexpr u8 MisterSynthMute = 0xA0;
constexpr u8 MisterSynthNuked = 0xA4;
constexpr u8 MisterSynthVariant = 0xA2;

// Audio conversion: mt32-pi's AudioTask constant verbatim (signed
// 24-bit positive max; the shift binds after the subtraction, so this
// is (1 << 23) - 1 = 8388607, the same value the port writes).
constexpr float Sample24BitMax = static_cast<float>((1 << 23) - 1);

// Stats file (SD root) and window length.
const char MinimalStatsPath[] = "SD:/shaped-var3-stats.log";
constexpr u64 WindowUs = 5000000ULL;	// 5 s -> 240,000 frames at 48 kHz

// Scheduler-free pause for spin waits (this kernel registers no
// CScheduler tasks, so CScheduler::Yield() can never return --
// GetNextTask() always yields MAX_TASKS with m_nTasks == 0). The ARM
// YIELD hint is the correct primitive here: no scheduler, no context
// switch, safe on every core. (This is the togglefix MiniPause,
// unchanged.)
static inline void MiniPause()
{
    __asm__ volatile ("yield" ::: "memory");
}

// PMU cycle counter, per core (FFmpeg checkasm port; bare metal has
// no PMUSERENR problem: the kernel runs privileged, so PMCCNTR_EL0
// reads directly). PMU_ReadCycles() itself lives in
// split_thread_circle.h (the backend needs it too); enable/validate/
// calibrate live here (core loops only). Enable once per core at
// loop start, with checkasm's exact setup: PMCR.E (bit 0) = 1;
// PMCNTENSET bit 31 = 1; PMCCFILTR bits 30 and 28 CLEARED (count at
// all levels). The counter is NEVER reset. An isb after the enables
// makes them take effect before the first read.
static inline void PMU_Enable()
{
    uint64_t nCfg = 0;
    __asm__ volatile ("mrs %0, pmcr_el0" : "=r" (nCfg));
    nCfg = nCfg | 1ULL;
    __asm__ volatile ("msr pmcr_el0, %0" :: "r" (nCfg) : "memory");
    __asm__ volatile ("msr pmcntenset_el0, %0" :: "r" (1ULL << 31) : "memory");
    uint64_t nFiltr = 0;
    __asm__ volatile ("mrs %0, pmccfiltr_el0" : "=r" (nFiltr));
    nFiltr = nFiltr & ~((1ULL << 30) | (1ULL << 28));
    __asm__ volatile ("msr pmccfiltr_el0, %0" :: "r" (nFiltr) : "memory");
    __asm__ volatile ("isb" ::: "memory");
}

// Boot validation (checkasm-style, mandatory): after enable, spin
// until the counter changes or ~1 ms wall timeout. A stuck counter
// (never counts) fails; the caller reports ok|fail in the boot line
// and the window writer gates that core's cycle fields to 0.
static bool PMU_Validate()
{
    const uint64_t nFirst = PMU_ReadCycles();
    const u64 nStartUs = CTimer::GetClockTicks();
    while (true)
    {
        if (PMU_ReadCycles() != nFirst)
            return true;
        if (CTimer::GetClockTicks() - nStartUs >= 1000ULL)
            return false;
        MiniPause();
    }
}

// NOP calibration: a back-to-back double read, once per core at
// boot. Subtracted from every reported cycle value (clamped at 0).
static uint64_t PMU_CalibrateNop()
{
    const uint64_t nFirst = PMU_ReadCycles();
    return PMU_ReadCycles() - nFirst;
}

// ---------------------------------------------------------------------------
// ROUND 3 SYNC (2026-10-31): sync profiles, timer modes, and the RUN table.
//
// A RUN is (PCM_Update variant, sync profile, timer mode). The
// auto-sequence plays E1M1 once per run, in table order, and stops
// after the last one. Run 0 is the shaped-var baseline: every knob at
// its legacy value, so the reference does not move. Each later run
// adds ONE lever (or runs a validation), so the log isolates each
// lever's effect on core-2 cycles/step, core-3 cycles/tick and pct.
// The PCM variants (pcm.cpp g_pcmVariants, 0-5) are one AXIS of a run;
// the sync/timer levers are the other two. See
// docs/optimization-round3-sync-20261031.md.
// ---------------------------------------------------------------------------

// Sync profiles (coordination knobs, see split_thread_circle.h).
//   number, short, full, lock_mode, legacy_lock, legacy_timing, legacy_irq, publish_interval, ring_check_interval, core3_backoff
const SplitSyncProfile g_splitSyncProfiles[] = {
	{0, "legacy", "shaped-var sync exactly: batch lock (CSpinLock IRQ_LEVEL), CTimer+ISB timing on every lock op, direct IRQ0, publish every step, ring check every step, tight idle spin",
	 SPLIT_LOCK_BATCH, 1, 1, 1, 1, 1, 0},
	{1, "trim", "overhead trim only: padded state, raw TASK lock, contended-only timing, IRQ0 flag, publish/8, ring check/16, idle backoff 64 (batch lock kept)",
	 SPLIT_LOCK_BATCH, 0, 0, 0, 8, 16, 64},
	{2, "tick", "trim + per-tick lock hand-off (core-2 wait bounded by one PCM tick)",
	 SPLIT_LOCK_TICK, 0, 0, 0, 8, 16, 64},
	{3, "fifo", "trim + deferred-store FIFO (core 2 never waits; stores land at tick boundaries = single-threaded semantics)",
	 SPLIT_LOCK_FIFO, 0, 0, 0, 8, 16, 64},
	{4, "free", "trim + NO lock (EXPERIMENT: upper bound of lock removal; lost-update hazard, never ship)",
	 SPLIT_LOCK_FREE, 0, 0, 0, 8, 16, 64},
	{5, "fifo32", "fifo with publish every 32 steps (publish-interval sensitivity)",
	 SPLIT_LOCK_FIFO, 0, 0, 0, 32, 16, 64},
};
const int g_splitSyncProfileCount = sizeof(g_splitSyncProfiles) / sizeof(g_splitSyncProfiles[0]);

// Run table.
struct TRun
{
	const char* name;	// <= 8 chars (LCD "VAR nn name")
	int pcm;		// g_pcmVariants number (0 baseline, 1 sched, 2 pipe, 3 neon2, 4 splitacc, 5 layout)
	int sync;		// g_splitSyncProfiles number
	int timer;		// TIMER_MODE_LEGACY / LAZY / CHECK
};
static const TRun g_runs[] = {
	{"base",    0, 0, TIMER_MODE_LEGACY},	// 0: the reference (shaped-var behaviour)
	{"trim",    0, 1, TIMER_MODE_LEGACY},	// 1: + sync overhead trim (layout, raw lock, publish/8, ring/16, IRQ0 flag, cheap timing)
	{"tick",    0, 2, TIMER_MODE_LEGACY},	// 2: + per-tick lock hand-off
	{"fifo",    0, 3, TIMER_MODE_LEGACY},	// 3: + deferred-store FIFO (no core-2 waits)
	{"fifolz",  0, 3, TIMER_MODE_LAZY},	// 4: + lazy timer (the core-2 compute lever)
	{"free",    0, 4, TIMER_MODE_LEGACY},	// 5: upper bound: no lock at all (experiment)
	{"fifo32",  0, 5, TIMER_MODE_LEGACY},	// 6: publish-interval sensitivity (8 -> 32)
	{"tchk",    0, 3, TIMER_MODE_CHECK},	// 7: VALIDATION: lazy timer vs shadow legacy, mismatches logged (tchk=)
	{"schedlz", 1, 3, TIMER_MODE_LAZY},	// 8: PCM variant sched on fifo + lazy timer
	{"pipelz",  2, 3, TIMER_MODE_LAZY},	// 9: PCM variant pipe, same
	{"neon2lz", 3, 3, TIMER_MODE_LAZY},	// 10: PCM variant neon2, same
	{"splitlz", 4, 3, TIMER_MODE_LAZY},	// 11: PCM variant splitacc (upstream round 3), same
	{"layoutlz", 5, 3, TIMER_MODE_LAZY},	// 12: PCM variant layout (upstream round 3), same
};
static const int g_runCount = sizeof(g_runs) / sizeof(g_runs[0]);

static inline const TRun& RunDesc(int nRun)
{
	if (nRun < 0 || nRun >= g_runCount)
		nRun = 0;
	return g_runs[nRun];
}
static inline int RunPcm(int nRun) { return RunDesc(nRun).pcm; }
static inline int RunSync(int nRun) { return RunDesc(nRun).sync; }
static inline int RunTimer(int nRun) { return RunDesc(nRun).timer; }
static const SplitSyncProfile& SyncProfile(int nNumber)
{
	for (int i = 0; i < g_splitSyncProfileCount; ++i)
		if (g_splitSyncProfiles[i].number == nNumber)
			return g_splitSyncProfiles[i];
	return g_splitSyncProfiles[0];
}

// The split state is a STATIC object (64-byte aligned by its type):
// its cache-line layout is the point of the round-3 padding, and
// Circle's operator new gives no alignment guarantee for over-aligned
// types.
static ThreadSplitStateCircle s_SplitState;

// PMU event counters (round 3). Four events per core, in this order:
//   0: INST_RETIRED (0x08), 1: BR_MIS_PRED (0x10),
//   2: L1D_CACHE_REFILL (0x03), 3: L2D_CACHE_REFILL (0x17).
// 32-bit counters: they wrap in ~4 s at 1 GHz, so they are sampled
// per step-loop call (core 2, ~5 ms) and per batch (core 3) as 32-bit
// deltas into 64-bit accumulators. Filter bits 31/30 cleared = count
// at every exception level (same filter as PMCCFILTR above).
static inline void PMU_EnableEvents()
{
	__asm__ volatile ("msr pmevtyper0_el0, %0" :: "r" ((uint64_t)0x08) : "memory");
	__asm__ volatile ("msr pmevtyper1_el0, %0" :: "r" ((uint64_t)0x10) : "memory");
	__asm__ volatile ("msr pmevtyper2_el0, %0" :: "r" ((uint64_t)0x03) : "memory");
	__asm__ volatile ("msr pmevtyper3_el0, %0" :: "r" ((uint64_t)0x17) : "memory");
	__asm__ volatile ("msr pmcntenset_el0, %0" :: "r" ((uint64_t)0xF) : "memory");
	__asm__ volatile ("isb" ::: "memory");
}
static inline void PMU_ReadEvents(u32 pOut[4])
{
	uint64_t v0, v1, v2, v3;
	__asm__ volatile ("mrs %0, pmevcntr0_el0" : "=r" (v0));
	__asm__ volatile ("mrs %0, pmevcntr1_el0" : "=r" (v1));
	__asm__ volatile ("mrs %0, pmevcntr2_el0" : "=r" (v2));
	__asm__ volatile ("mrs %0, pmevcntr3_el0" : "=r" (v3));
	pOut[0] = static_cast<u32>(v0);
	pOut[1] = static_cast<u32>(v1);
	pOut[2] = static_cast<u32>(v2);
	pOut[3] = static_cast<u32>(v3);
}
// Validation: the instruction counter must advance across a short
// busy loop (~1 ms wall timeout). A stuck counter -> the core's event
// fields report 0 and the boot line says pmevN=fail.
static bool PMU_ValidateEvents()
{
	u32 a[4], b[4];
	PMU_ReadEvents(a);
	const u64 nStartUs = CTimer::GetClockTicks();
	while (true)
	{
		PMU_ReadEvents(b);
		if (b[0] != a[0])
			return true;
		if (CTimer::GetClockTicks() - nStartUs >= 1000ULL)
			return false;
	}
}


namespace
{

struct TROMFile
{
	char Name[64];
	SHA256_Digest Digest;
	std::vector<uint8_t> Data;
};

bool ReadROMFile(const char* pPath, std::vector<uint8_t>& OutData)
{
	FIL File;
	if (f_open(&File, pPath, FA_READ) != FR_OK)
		return false;

	const FSIZE_t nSize = f_size(&File);
	OutData.resize(static_cast<size_t>(nSize));

	size_t nReadTotal = 0;
	bool bOK          = true;
	while (nReadTotal < OutData.size())
	{
		UINT nRead = 0;
		if (f_read(&File, OutData.data() + nReadTotal, static_cast<UINT>(OutData.size() - nReadTotal), &nRead) != FR_OK || nRead == 0)
		{
			bOK = false;
			break;
		}
		nReadTotal += nRead;
	}

	f_close(&File);
	return bOK && nReadTotal == OutData.size();
}

// Scans the SD ROM directory and hashes every plausible file it finds.
// Returns false if no usable files were found at all.
bool ScanROMFiles(std::vector<TROMFile>& OutFiles)
{
	char DirectoryPath[32];
	snprintf(DirectoryPath, sizeof(DirectoryPath), "SD:/%s", MinimalROMDirectory);

	DIR Dir;
	FILINFO FileInfo;
	FRESULT Result = f_findfirst(&Dir, &FileInfo, DirectoryPath, "*");

	while (Result == FR_OK && *FileInfo.fname)
	{
		if (!(FileInfo.fattrib & (AM_DIR | AM_HID | AM_SYS)) && FileInfo.fsize > 0 && FileInfo.fsize <= MaxROMFileSize && OutFiles.size() < MaxROMFiles)
		{
			TROMFile File;
			snprintf(File.Name, sizeof(File.Name), "%s/%s", DirectoryPath, FileInfo.fname);

			if (ReadROMFile(File.Name, File.Data) && SHA256_HashBytes(File.Data, File.Digest))
				OutFiles.push_back(std::move(File));
		}

		Result = f_findnext(&Dir, &FileInfo);
	}

	f_closedir(&Dir);

	return !OutFiles.empty();
}

// Finds a complete MK1 romset definition whose ROM hashes all match
// scanned files. MK1 ONLY in this kernel: any other complete set is a
// failure (the split is MK1-gated in the backend dispatch, and the
// golden songs are MK1 runs).
const RomsetDefinition* MatchRomsetMK1(std::vector<TROMFile>& Files, TROMFile* pMatchedFiles[ROMLOCATION_COUNT])
{
	for (const RomsetDefinition& Definition : GetStandardRomsetDefinitions())
	{
		if (Definition.romset != Romset::MK1)
			continue;

		bool bComplete = true;
		for (size_t i = 0; i < ROMLOCATION_COUNT; ++i)
			pMatchedFiles[i] = nullptr;

		for (const RomLocation Location : Definition.GetValidLocations())
		{
			const SHA256_Digest& Wanted = Definition.GetHash(Location);
			TROMFile* pMatch            = nullptr;
			for (TROMFile& File : Files)
			{
				if (File.Digest == Wanted)
				{
					pMatch = &File;
					break;
				}
			}

			if (!pMatch)
			{
				bComplete = false;
				break;
			}

			pMatchedFiles[static_cast<size_t>(Location)] = pMatch;
		}

		if (bComplete)
			return &Definition;
	}

	return nullptr;
}

const char* SongName(int nSong)
{
	return nSong == 0 ? "e1m1" : "belmo";
}

// Variant number -> short name, from the data-driven table in
// pcm.cpp (single source of truth: the table owns the names, so a
// follow-up adding variants needs no change here). Unknown numbers
// report "unknown" (never a wrong name).
// Run index -> short name (LCD + log). Unknown indexes report
// "unknown" (never a wrong name).
const char* RunName(int nRun)
{
	if (nRun >= 0 && nRun < g_runCount)
		return g_runs[nRun].name;
	return "unknown";
}

// MiSTer synth byte -> name (port's misterstatus.h values).
const char* MisterSynthName(u8 nSynth)
{
	switch (nSynth)
	{
	case 0xA0:
		return "mute";
	case 0xA1:
		return "mt32";
	case 0xA2:
		return "soundfont";
	case 0xA3:
		return "emusc";
	case MisterSynthNuked:
		return "nuked";
	case 0xFF:
		return "unknown";
	default:
		return "other";
	}
}

} // namespace

CMinimalApp::CMinimalApp(CI2CMaster* pI2CMaster, CInterruptSystem* pInterrupt)
	: CMultiCoreSupport(CMemorySystem::Get()),
	  m_pEmulator(nullptr),
	  m_pRomsetInfo(nullptr),
	  m_pRomsetName(nullptr),
	  m_pSplitState(nullptr),
	  m_bSplitActive(false),
	  m_bPCMIdle(true),
	  m_Ring{},
	  m_nRingDrops(0),
	  m_nWrittenFrames(0),
	  m_pEvents(nullptr),
	  m_pBlob(nullptr),
	  m_nEventCount(0),
	  m_nEndCycle(0),
	  m_nNextEvent(0),
	  m_nCurrentSong(0),
	  m_bSongDone(false),
	  m_bPaused(false),
	  m_nCurrentRun(0),
	  m_bSequenceDone(false),
	  m_bPmu3Done(false),
	  m_bPmu3Ok(false),
	  m_bPmu2Ok(false),
	  m_MisterRing{},
	  m_nMisterDrops(0),
	  m_nMisterDropsLogged(0),
	  m_nMisterUps(0),
	  m_nMisterDowns(0),
	  m_nCycles(0),
	  m_pI2CMaster(pI2CMaster),
	  m_pInterrupt(pInterrupt),
	  m_pSound(nullptr),
	  m_dResampleRatio(0.0),
	  m_dReadFrac(0.0),
	  m_SampleA{0.0f, 0.0f},
	  m_SampleB{0.0f, 0.0f},
	  m_bWindowValid(false),
	  m_bSoundStarted(false),
	  m_bSoundRestartPending(false),
	  m_bSoundStartFailed(false),
	  m_nSoundRestartWaitStartUs(0),
	  m_nSoundRestarts(0),
	  m_nSoundDrops(0),
	  m_nNativeRate(0),
	  m_StatsFile{},
	  m_bStatsOpen(false),
	  m_nWinStartUs(0),
	  m_nWinIndex(0),
	  m_nWinProd(0),
	  m_nWinShort(0),
	  m_nWinUnderruns(0),
	  m_nWinCore2Us(0),
	  m_nWinResampleUs(0),
	  m_nWinChunks(0),
	  m_nWinDevQMaxAvail(0),
	  m_nWinDevQLow(0),
	  m_nWinSoundRestarts(0),
	  m_nWinSoundDrops(0),
	  m_nLEDChunks(0),
	  m_bLEDOn(false),
	  m_nLastPCMUs(0),
	  m_nLastLockUs(0),
	  m_nLastLockWaits(0),
	  m_nLastPCMCyc(0),
	  m_nLastMixCyc(0),
	  m_nLastLockCyc(0),
	  m_nWinStepWallCyc(0),
	  m_nWinStepCalls(0),
	  m_nSongDue(0),
	  m_nSongProd(0),
	  m_nSongShort(0),
	  m_nSongUnderruns(0),
	  m_nCompletions(0),
	  m_bSongEndPending(false)
{
}

CMinimalApp::~CMinimalApp()
{
	// Never runs in practice: the core loops run until power cut, and
	// main never returns past Run(). (Deliberately no StopSplitPCM
	// here: parking the PCM task would hang against the run-forever
	// PCM loop if this ever did run.)
}

void CMinimalApp::OnSample(s32 nLeft, s32 nRight)
{
	// Normalize() in the backend maps full scale (2^29) to 1.0f.
	// The backend idles at the SC-55 DAC mid-scale bias (0x1000000),
	// i.e. +0.03125 DC; center the ring at zero, exactly like the
	// port (a real SC-55's analog output is AC-coupled). The device
	// loop drains these values through the resampler, but the
	// callback runs the port's exact code so its cost stays in the
	// measured load.
	constexpr float Scale = 1.0f / 536870912.0f;
	constexpr float DCBias = 16777216.0f / 536870912.0f;
	const TSample Frame{ nLeft * Scale - DCBias, nRight * Scale - DCBias };
	// SkelRing always enqueues; the return tells whether it had to
	// drop the oldest frame to make room (must stay 0; see the
	// inherited wart note in the header comment).
	if (m_Ring.Enqueue(Frame))
		m_nRingDrops = m_nRingDrops + 1;
	m_nWrittenFrames = m_nWrittenFrames + 1;
}

void CMinimalApp::StopSplitPCM()
{
	if (!m_bSplitActive)
		return;

	// Park the PCM task: it finishes its current batch (bounded: at
	// most 128 ticks under the state lock), then sees the flag, acks
	// idle, and yields until reactivated. The MCU thread is not
	// stepping here (the caller is core 2, at a chunk boundary), so
	// once the PCM task has parked, nothing touches the backend
	// anymore.
	m_bSplitActive = false;
	SplitDMB();
	while (!m_bPCMIdle)
	{
		SplitDMB();
		MiniPause();
	}
	SplitDMB();
	g_splitT = nullptr;
	SplitDMB();
}

void CMinimalApp::InitSplitState(bool bClearRing)
{
	// Caller guarantees: PCM task parked (or never started) and the
	// MCU thread not stepping (chunk boundary, or pre-activation).
	if (!m_pSplitState)
		m_pSplitState = &s_SplitState;

	// Round 3: install the current run's sync profile and timer mode
	// BEFORE the state is (re)initialized and published. Both are read
	// by the backend on every step/batch; changing them here is safe
	// because the PCM task is parked and the MCU is not stepping.
	{
		const int nRun = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
		m_pSplitState->sync = SyncProfile(RunSync(nRun));
		TIMER_SetMode(*m_pEmulator->GetMCU().timer, static_cast<uint8_t>(RunTimer(nRun)));
	}

	m_pSplitState->InitFromPCM(m_pEmulator->GetPCM(), m_pEmulator->GetMCU());

	if (bClearRing)
	{
		// Fresh audio after a reset: the queued frames belong to the
		// pre-reset song and must not play after the reboot.
		// (Safe here: the PCM task is parked, so no producer runs.)
		m_Ring.Clear();
	}
	// Activation (bClearRing false) keeps queued frames; at init the
	// ring is empty anyway.

	// Publish order: initialized state, then g_splitT, then the
	// active flag, each separated by a barrier, so the PCM task can
	// never observe the flag before the state it gates on.
	SplitDMB();
	g_splitT = m_pSplitState;
	SplitDMB();
	m_bSplitActive = true;
	SplitDMB();
}

void CMinimalApp::SetSongCursor(int nSong)
{
	if (nSong == 0)
	{
		m_pEvents = song_e1m1_events;
		m_pBlob = song_e1m1_blob;
		m_nEventCount = song_e1m1_event_count;
		m_nEndCycle = song_e1m1_end_cycle;
	}
	else
	{
		m_pEvents = song_belmo_events;
		m_pBlob = song_belmo_blob;
		m_nEventCount = song_belmo_event_count;
		m_nEndCycle = song_belmo_end_cycle;
	}
}

void CMinimalApp::ResetWindow()
{
	m_nWinProd = 0;
	m_nWinShort = 0;
	m_nWinUnderruns = 0;
	m_nWinCore2Us = 0;
	m_nWinResampleUs = 0;
	m_nWinChunks = 0;
	m_nWinDevQMaxAvail = 0;
	m_nWinDevQLow = 0;
	m_nWinSoundRestarts = 0;
	m_nWinSoundDrops = 0;
	m_nWinStepWallCyc = 0;
	m_nWinStepCalls = 0;
	m_nWinSteps = 0;
	for (int i = 0; i < 4; ++i)
		m_nWinC2Ev[i] = 0;
	if (m_pSplitState)
	{
		ThreadSplitStateCircle& S = *m_pSplitState;
		SplitDMB();
		m_nLastPCMUs = S.pcm_task_us;
		m_nLastLockUs = S.lock_wait_us;
		m_nLastLockWaits = S.lock_waits;
		m_nLastPCMCyc = S.pcm_task_cyc;
		m_nLastMixCyc = S.mix_cyc;
		m_nLastLockCyc = S.lock_wait_cyc;
		m_nLastTicks = S.ticks;
		m_nLastPublishes = S.publishes;
		m_nLastLockContended = S.lock_contended;
		m_nLastIrq0Folded = S.irq0_folded;
		m_nLastFifoFull = S.fifo_full_waits;
		for (int i = 0; i < 4; ++i)
			m_nLastC3Ev[i] = S.c3_ev[i];
	}
	m_nWinStartUs = CTimer::GetClockTicks();
}

void CMinimalApp::LogLine(const char* pFormat, ...)
{
	// Core 2 only (the sole file writer, so no lock is needed; the
	// init-time lines run pre-multicore, single-threaded). Every
	// line is f_sync'd: the log must survive a power cut, since the operator
	// reads the card after the run. Best effort: if a write fails,
	// later lines just fail the same way (a truncated log is itself
	// evidence of an SD problem, not of a split problem). Every line
	// is prefixed with t=<us since boot> so control events are
	// timestamped (the event-logging requirement).
	if (!m_bStatsOpen)
		return;

	char Buffer[1024];
	const int nPrefix = snprintf(Buffer, sizeof(Buffer), "t=%llu ",
	                             static_cast<unsigned long long>(CTimer::GetClockTicks()));
	va_list Args;
	va_start(Args, pFormat);
	vsnprintf(Buffer + nPrefix, sizeof(Buffer) - static_cast<size_t>(nPrefix), pFormat, Args);
	va_end(Args);

	size_t nLen = strlen(Buffer);
	Buffer[nLen++] = '\n';

	UINT nWritten = 0;
	f_write(&m_StatsFile, Buffer, static_cast<UINT>(nLen), &nWritten);
	f_sync(&m_StatsFile);
}

void CMinimalApp::WriteWindow(bool bPaused)
{
	// Window line: tick/lookahead/rpc waits+us are 0 by construction
	// (that machinery was removed with the FIFO/RPC split); max_lag
	// is the peak published-minus-rendered lag since (re)activation;
	// write_hw/irq_hw/write_ovf/irq_ovf are 0 by construction;
	// ring_drops is the drop-oldest total since boot (must stay 0;
	// see the inherited wart note). due is WALL-CLOCK based
	// (elapsed * 48000): the device clock is the independent
	// reference, so a slow engine shows as pct < 100 even though the
	// device queue absorbs transient slips. Paused windows carry
	// paused=1 and due=0 -- a paused stretch must NOT read as
	// shortfall. core2_pct is the core-2 step-loop wall time as a
	// share of the window; core3_pct is the PCM task's work time as
	// a share of the window; resample_us is the resampler's wall
	// time. devq_maxavail is the peak device FREE space in the
	// window (near the 2048-frame queue size = the device nearly
	// starved); devq_low counts chunks where fewer than one chunk
	// remained QUEUED at Write time. (Both devq fields are CORRECTED
	// vs old shaped, which measured them on Avail = queued and
	// thereby inverted both.) sound_restarts/sound_drops are the
	// window's device events (drops must be 0 by construction: the
	// corrected wait guarantees a full chunk fits before Writing).
	ThreadSplitStateCircle& S = *m_pSplitState;
	SplitDMB();
	const u64 nTickWaits      = 0;
	const u64 nTickUs         = 0;
	const u64 nLookaheadWaits = 0;
	const u64 nLookaheadUs    = 0;
	const u64 nRpcWaits       = 0;
	const u64 nRpcUs          = 0;
	const u64 nPCMUs          = S.pcm_task_us - m_nLastPCMUs;
	const u64 nElapsedUs      = CTimer::GetClockTicks() - m_nWinStartUs;
	const u64 nDue            = bPaused ? 0 : nElapsedUs * OutRate / 1000000ULL;
	const u64 nPct            = nDue > 0 ? m_nWinProd * 100 / nDue : 100;
	const u64 nCore2Pct       = nElapsedUs > 0 ? m_nWinCore2Us * 100 / nElapsedUs : 0;
	// DIAG: core-2 state-lock spin waits in this window (S counters
	// are core-2-only, so the delta is exact); mcu_us is the
	// step-loop wall MINUS lock spins = pure MCU stepping + the
	// peripherals + publish + ring-check overhead inside StepUntilFill.
	// If mcu_us << step wall, the core-2 gap is lock spins; if
	// mcu_us ~= step wall, it is MCU work. (Uncontended Acquires
	// read 0 us at the 1 MHz timer, so lock_us counts contended
	// waits; lock_waits counts every lock op.)
	// PMU (shaped-var): cycle deltas in this window (S counters are
	// single-writer each, so the deltas are exact). pcm_cyc = core-3
	// total PCM task work (same scope as pcm_task_us); mix_cyc =
	// core-3 voice-mix portion per batch (inside the PCM_Update
	// variants); lock_cyc = core-2 state-lock spin cycles
	// (NOP-corrected per op at accumulation); step_cyc = core-2 MCU
	// stepping cycles ONLY, derived exactly like mcu_us but in
	// cycles: step_wall_raw - nop2*step_calls - lock_raw, where
	// lock_raw = lock_cyc + nop2*lock_waits (the per-op NOP
	// correction is added back: the wall contains the lock reads'
	// latency, and lock_cyc already subtracted it once). If a
	// core's counter failed validation (boot pmc2/pmc3=fail), its
	// cycle fields report 0 -- never silent zeros: the boot line
	// is the validity signal. var=N is the variant this window ran
	// under; per-variant totals accumulate into that variant's slot
	// (until sequence_done, after which idle windows stop adding).
	// Round 3: with the cheap (contended-only, unfenced) lock timing
	// there are no CTimer reads, so lock_us is DERIVED from the spin
	// cycles at 1 GHz; the legacy profile keeps the measured value.
	const u64 nLockUs         = S.sync.legacy_timing ? (S.lock_wait_us - m_nLastLockUs)
	                                                 : ((m_bPmu2Ok ? S.lock_wait_cyc - m_nLastLockCyc : 0) / 1000ULL);
	const u64 nLockWaits      = S.lock_waits - m_nLastLockWaits;
	const u64 nMcuUs          = m_nWinCore2Us >= nLockUs ? m_nWinCore2Us - nLockUs : 0;
	const u64 nCore3Pct       = nElapsedUs > 0 ? nPCMUs * 100 / nElapsedUs : 0;
	// PMU cycle deltas (see the comment above). step_cyc derivation:
	// wall_raw includes the lock spins AND both read-pair latencies
	// (wall reads: nop2 per StepUntilFill call; lock reads: nop2 per
	// lock op, already subtracted once inside lock_cyc). So:
	// step_cyc = wall_raw - nop2*calls - (lock_cyc + nop2*waits).
	// Every reported cycle value is NOP-corrected, clamped at 0.
	// A failed counter gates its fields to 0 (boot line is the
	// validity signal).
	const u64 nPCMCyc = m_bPmu3Ok ? S.pcm_task_cyc - m_nLastPCMCyc : 0;
	const u64 nMixCyc = m_bPmu3Ok ? S.mix_cyc - m_nLastMixCyc : 0;
	const u64 nLockCyc = m_bPmu2Ok ? S.lock_wait_cyc - m_nLastLockCyc : 0;
	u64 nStepCyc = 0;
	if (m_bPmu2Ok)
	{
		const u64 nNop2 = S.nop_cyc2;
		const u64 nLockRaw = nLockCyc + nNop2 * nLockWaits;
		const u64 nWallCorrected = m_nWinStepWallCyc >= nNop2 * m_nWinStepCalls ? m_nWinStepWallCyc - nNop2 * m_nWinStepCalls : 0;
		nStepCyc = nWallCorrected >= nLockRaw ? nWallCorrected - nLockRaw : 0;
	}
	const int nVariant = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
	m_nSongDue = m_nSongDue + nDue;
	// Round-3 window deltas.
	const u64 nTicks        = S.ticks - m_nLastTicks;
	const u64 nPubs         = S.publishes - m_nLastPublishes;
	const u64 nLockCont     = S.lock_contended - m_nLastLockContended;
	const u64 nIrq0         = S.irq0_folded - m_nLastIrq0Folded;
	const u64 nFifoFull     = S.fifo_full_waits - m_nLastFifoFull;
	u64 nC3Ev[4];
	for (int i = 0; i < 4; ++i)
		nC3Ev[i] = m_bPmuEv3Ok ? S.c3_ev[i] - m_nLastC3Ev[i] : 0;
	u64 nC2Ev[4];
	for (int i = 0; i < 4; ++i)
		nC2Ev[i] = m_bPmuEv2Ok ? m_nWinC2Ev[i] : 0;
	const TimerCheckStatus tchk = TIMER_GetCheckStatus(*m_pEmulator->GetMCU().timer);
	const char* pTchk = tchk.active ? (tchk.mismatches == 0 ? "ok" : "FAIL") : "off";
	if (!m_bSequenceDone && !bPaused)
	{
		m_nRunProd = m_nRunProd + m_nWinProd;
		m_nRunDue = m_nRunDue + nDue;
		m_nRunUnderruns = m_nRunUnderruns + m_nWinUnderruns;
		m_nRunPcmCyc = m_nRunPcmCyc + nPCMCyc;
		m_nRunMixCyc = m_nRunMixCyc + nMixCyc;
		m_nRunStepCyc = m_nRunStepCyc + nStepCyc;
		m_nRunLockCyc = m_nRunLockCyc + nLockCyc;
		m_nRunLockWaits = m_nRunLockWaits + nLockWaits;
		m_nRunLockCont = m_nRunLockCont + nLockCont;
		m_nRunSteps = m_nRunSteps + m_nWinSteps;
		m_nRunTicks = m_nRunTicks + nTicks;
		for (int i = 0; i < 4; ++i)
		{
			m_nRunC2Ev[i] = m_nRunC2Ev[i] + nC2Ev[i];
			m_nRunC3Ev[i] = m_nRunC3Ev[i] + nC3Ev[i];
		}
	}

	LogLine("win n=%u song=%s done=%u paused=%u chunks=%llu due=%llu prod=%llu short=%llu pct=%llu underruns=%llu core2_pct=%llu core3_pct=%llu resample_us=%llu tick_waits=%llu tick_us=%llu lookahead_waits=%llu lookahead_us=%llu rpc_waits=%llu rpc_us=%llu max_lag=%llu write_hw=%llu irq_hw=%llu write_ovf=%llu irq_ovf=%llu ring_drops=%llu devq_maxavail=%llu devq_low=%llu sound_restarts=%llu sound_drops=%llu lock_us=%llu lock_waits=%llu mcu_us=%llu var=%d pcm_cyc=%llu mix_cyc=%llu step_cyc=%llu lock_cyc=%llu steps=%llu ticks=%llu pubs=%llu lock_cont=%llu fifo_hw=%llu fifo_full=%llu irq0=%llu pcm=%d sync=%d timer=%d tchk=%s tchk_cmp=%llu tchk_mism=%llu tchk_first=%llu tchk_code=%u lazy_skips=%llu c2_inst=%llu c2_brmis=%llu c2_l1d=%llu c2_l2d=%llu c3_inst=%llu c3_brmis=%llu c3_l1d=%llu c3_l2d=%llu",
	        m_nWinIndex,
	        SongName(m_nCurrentSong),
	        m_bSongDone ? 1 : 0,
	        bPaused ? 1 : 0,
	        static_cast<unsigned long long>(m_nWinChunks),
	        static_cast<unsigned long long>(nDue),
	        static_cast<unsigned long long>(m_nWinProd),
	        static_cast<unsigned long long>(m_nWinShort),
	        static_cast<unsigned long long>(nPct),
	        static_cast<unsigned long long>(m_nWinUnderruns),
	        static_cast<unsigned long long>(nCore2Pct),
	        static_cast<unsigned long long>(nCore3Pct),
	        static_cast<unsigned long long>(m_nWinResampleUs),
	        static_cast<unsigned long long>(nTickWaits),
	        static_cast<unsigned long long>(nTickUs),
	        static_cast<unsigned long long>(nLookaheadWaits),
	        static_cast<unsigned long long>(nLookaheadUs),
	        static_cast<unsigned long long>(nRpcWaits),
	        static_cast<unsigned long long>(nRpcUs),
	        static_cast<unsigned long long>(S.max_pcm_lag),
	        static_cast<unsigned long long>(0),
	        static_cast<unsigned long long>(0),
	        static_cast<unsigned long long>(0),
	        static_cast<unsigned long long>(0),
	        static_cast<unsigned long long>(m_nRingDrops),
	        static_cast<unsigned long long>(m_nWinDevQMaxAvail),
	        static_cast<unsigned long long>(m_nWinDevQLow),
	        static_cast<unsigned long long>(m_nWinSoundRestarts),
	        static_cast<unsigned long long>(m_nWinSoundDrops),
	        static_cast<unsigned long long>(nLockUs),
	        static_cast<unsigned long long>(nLockWaits),
	        static_cast<unsigned long long>(nMcuUs),
	        nVariant,
	        static_cast<unsigned long long>(nPCMCyc),
	        static_cast<unsigned long long>(nMixCyc),
	        static_cast<unsigned long long>(nStepCyc),
	        static_cast<unsigned long long>(nLockCyc),
	        static_cast<unsigned long long>(m_nWinSteps),
	        static_cast<unsigned long long>(nTicks),
	        static_cast<unsigned long long>(nPubs),
	        static_cast<unsigned long long>(nLockCont),
	        static_cast<unsigned long long>(S.fifo_hw),
	        static_cast<unsigned long long>(nFifoFull),
	        static_cast<unsigned long long>(nIrq0),
	        RunPcm(nVariant), RunSync(nVariant), RunTimer(nVariant),
	        pTchk,
	        static_cast<unsigned long long>(tchk.compares),
	        static_cast<unsigned long long>(tchk.mismatches),
	        static_cast<unsigned long long>(tchk.first_cycles),
	        tchk.first_code,
	        static_cast<unsigned long long>(tchk.lazy_skips),
	        static_cast<unsigned long long>(nC2Ev[0]),
	        static_cast<unsigned long long>(nC2Ev[1]),
	        static_cast<unsigned long long>(nC2Ev[2]),
	        static_cast<unsigned long long>(nC2Ev[3]),
	        static_cast<unsigned long long>(nC3Ev[0]),
	        static_cast<unsigned long long>(nC3Ev[1]),
	        static_cast<unsigned long long>(nC3Ev[2]),
	        static_cast<unsigned long long>(nC3Ev[3]));

	m_nWinIndex = m_nWinIndex + 1;
	ResetWindow();
}

void CMinimalApp::WriteSongEnd()
{
	const u64 nPct = m_nSongDue > 0 ? m_nSongProd * 100 / m_nSongDue : 100;
	LogLine("song_end song=%s due=%llu prod=%llu short=%llu pct=%llu underruns=%llu completions=%llu cycles=%llu var=%d pcm=%d sync=%d timer=%d name=%s",
	        SongName(m_nCurrentSong),
	        static_cast<unsigned long long>(m_nSongDue),
	        static_cast<unsigned long long>(m_nSongProd),
	        static_cast<unsigned long long>(m_nSongShort),
	        static_cast<unsigned long long>(nPct),
	        static_cast<unsigned long long>(m_nSongUnderruns),
	        static_cast<unsigned long long>(m_nCompletions),
	        static_cast<unsigned long long>(m_nCycles),
	        __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE),
	        RunPcm(__atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE)),
	        RunSync(__atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE)),
	        RunTimer(__atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE)),
	        RunName(__atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE)));
}

void CMinimalApp::WriteRunTotal()
{
	// Core 2 only, from DrainMailboxes after the song_end + window
	// flush, so every song window of this run is included. The two
	// derived ratios are THE comparators across runs: cyc_per_step =
	// core-2 MCU stepping cycles per MCU step (excludes lock spins),
	// cyc_per_tick = core-3 PCM task cycles per PCM tick. pct is the
	// real-time rate; both ratios still move when pct saturates at
	// 100, which is what makes them useful once a run is fast enough.
	const int nRun = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
	const u64 nPct = m_nRunDue > 0 ? m_nRunProd * 100 / m_nRunDue : 100;
	const u64 nCycPerStep = m_nRunSteps > 0 ? m_nRunStepCyc / m_nRunSteps : 0;
	const u64 nCycPerTick = m_nRunTicks > 0 ? m_nRunPcmCyc / m_nRunTicks : 0;
	const u64 nLockCycPerStep1000 = m_nRunSteps > 0 ? m_nRunLockCyc * 1000 / m_nRunSteps : 0;
	const TimerCheckStatus tchk = TIMER_GetCheckStatus(*m_pEmulator->GetMCU().timer);
	LogLine("run_total n=%d name=%s pcm=%d sync=%d timer=%d due=%llu prod=%llu pct=%llu underruns=%llu steps=%llu ticks=%llu step_cyc=%llu lock_cyc=%llu pcm_cyc=%llu mix_cyc=%llu cyc_per_step=%llu lock_cyc_per_kstep=%llu cyc_per_tick=%llu lock_waits=%llu lock_cont=%llu c2_inst=%llu c2_brmis=%llu c2_l1d=%llu c2_l2d=%llu c3_inst=%llu c3_brmis=%llu c3_l1d=%llu c3_l2d=%llu tchk=%s tchk_cmp=%llu tchk_mism=%llu tchk_first=%llu tchk_code=%u",
	        nRun, RunName(nRun), RunPcm(nRun), RunSync(nRun), RunTimer(nRun),
	        static_cast<unsigned long long>(m_nRunDue),
	        static_cast<unsigned long long>(m_nRunProd),
	        static_cast<unsigned long long>(nPct),
	        static_cast<unsigned long long>(m_nRunUnderruns),
	        static_cast<unsigned long long>(m_nRunSteps),
	        static_cast<unsigned long long>(m_nRunTicks),
	        static_cast<unsigned long long>(m_nRunStepCyc),
	        static_cast<unsigned long long>(m_nRunLockCyc),
	        static_cast<unsigned long long>(m_nRunPcmCyc),
	        static_cast<unsigned long long>(m_nRunMixCyc),
	        static_cast<unsigned long long>(nCycPerStep),
	        static_cast<unsigned long long>(nLockCycPerStep1000),
	        static_cast<unsigned long long>(nCycPerTick),
	        static_cast<unsigned long long>(m_nRunLockWaits),
	        static_cast<unsigned long long>(m_nRunLockCont),
	        static_cast<unsigned long long>(m_nRunC2Ev[0]),
	        static_cast<unsigned long long>(m_nRunC2Ev[1]),
	        static_cast<unsigned long long>(m_nRunC2Ev[2]),
	        static_cast<unsigned long long>(m_nRunC2Ev[3]),
	        static_cast<unsigned long long>(m_nRunC3Ev[0]),
	        static_cast<unsigned long long>(m_nRunC3Ev[1]),
	        static_cast<unsigned long long>(m_nRunC3Ev[2]),
	        static_cast<unsigned long long>(m_nRunC3Ev[3]),
	        tchk.active ? (tchk.mismatches == 0 ? "ok" : "FAIL") : "off",
	        static_cast<unsigned long long>(tchk.compares),
	        static_cast<unsigned long long>(tchk.mismatches),
	        static_cast<unsigned long long>(tchk.first_cycles),
	        tchk.first_code);
}

void CMinimalApp::WriteSequenceDone()
{
	// Core 2 only, after the last run's song_end + run_total. The
	// per-run totals live in the run_total lines (one per run); this
	// line is the terminal marker.
	LogLine("sequence_done runs=%d", g_runCount);
}

void CMinimalApp::PCMTaskIteration()
{
	if (!m_bSplitActive || !m_pEmulator || !m_pSplitState)
	{
		// Parked: ack the stop handshake (if one is waiting) and idle.
		// This branch always returns, so a stale idle ack (written
		// just before a reactivation) can never be followed by work
		// without a fresh gate read first; conversely, m_bPCMIdle is
		// false throughout any work batch, so a stop waiter can never
		// observe true mid-batch.
		m_bPCMIdle = true;
		SplitDMB();
		MiniPause();
		return;
	}

	ThreadSplitStateCircle& S = *m_pSplitState;
	pcm_t& PCM = m_pEmulator->GetPCM();

	// Skeletal JV880 shape: ACQUIRE-load the published cycle count.
	// A batch needs more than one max tick (800 cycles) of headroom:
	// PCM_Update renders the tick crossing its target, so the batch
	// target is capped at published-800 -- the cap that guarantees
	// no tick ever ends past published (max tick = (31+1)*25 = 800).
	// With less headroom no batch could advance the render position,
	// so yield-hint pause and re-check. (pcm.cycles is core-3-owned:
	// only PCM_Update advances it, so reading it unlocked is safe;
	// reg_slots is read per batch and may change mid-batch via a
	// lock-free core-2 config write, exactly as in Sterr1 -- the
	// no-overshoot proof holds for any tick size up to 800.)
	const uint64_t nPublished = __atomic_load_n(&S.cycles_pub, __ATOMIC_ACQUIRE);
	if (nPublished <= PCM.cycles + ThreadSplitStateCircle::MaxTickCycles)
	{
		// Idle. Round 3: back off per profile before the next poll
		// (0 = the legacy tight spin), so an idle core 3 does not
		// hammer the line core 2 publishes on.
		for (unsigned i = S.sync.core3_backoff; i > 0; --i)
			MiniPause();
		MiniPause();
		return;
	}

	m_bPCMIdle = false;
	const u64 nWorkStartUs = CTimer::GetClockTicks();
	const uint64_t nWorkStartCyc = PMU_ReadCycles();

	// Bounded batch: at most 128 ticks (Sterr1's 128-sample bound;
	// this backend renders one sample per tick at native rate),
	// capped at published-800 (the no-overshoot cap above). The
	// whole batch holds the state lock, as in Sterr1.
	const uint64_t nTick = (uint64_t)(PCM.config.reg_slots + 1) * 25;
	uint64_t nBatchEnd = PCM.cycles + ThreadSplitStateCircle::MaxBatchTicks * nTick;
	const uint64_t nCap = nPublished - ThreadSplitStateCircle::MaxTickCycles;
	if (nBatchEnd > nCap)
		nBatchEnd = nCap;
	// Variant dispatch (read at batch start): the variant changes
	// ONLY while the PCM task is parked (inside DoRunAdvance),
	// so the batch always runs one variant. Baseline keeps the
	// plain PCM_Update name (its single-threaded callers are
	// untouched); the variants are separate implementations over
	// the same shared pcm_t.
	const int nRun = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
	const int nPcmVariant = RunPcm(nRun);
	// Round 3: the lock follows the profile (BATCH/TICK hold it for
	// the batch, FREE/FIFO take none), and the 4 PMU event counters
	// are sampled around the batch (work only, not the idle spin).
	u32 ev0[4], ev1[4];
	PMU_ReadEvents(ev0);
	const bool bLock = S.Core3LocksBatch();
	if (bLock)
		S.Core3Acquire();
	PCM_Update_Variant(PCM, nBatchEnd, nPcmVariant);
	if (bLock)
		S.Core3Release();
	PMU_ReadEvents(ev1);
	for (int i = 0; i < 4; ++i)
		S.c3_ev[i] = S.c3_ev[i] + static_cast<u32>(ev1[i] - ev0[i]);

	// Publish the render position (stats only now) and the peak lag
	// for the stats lines. Published only grows, and the batch never
	// renders past published-800, so the fresh lag is >= 0.
	SplitDMB();
	S.pcm_rendered = PCM.cycles;
	const uint64_t nFreshPublished = __atomic_load_n(&S.cycles_pub, __ATOMIC_ACQUIRE);
	const int64_t nLag = static_cast<int64_t>(nFreshPublished - PCM.cycles);
	if (nLag > static_cast<int64_t>(S.max_pcm_lag))
		S.max_pcm_lag = static_cast<uint64_t>(nLag);
	S.pcm_task_us = S.pcm_task_us + CTimer::GetClockTicks() - nWorkStartUs;
	// PMU: total PCM task work in cycles, NOP-corrected (mirrors
	// pcm_task_us's scope exactly: batch + publish + lag). A stuck
	// counter (failed validation) yields delta 0, and the window
	// writer gates the field to 0 anyway.
	{
		const uint64_t nDelta = PMU_ReadCycles() - nWorkStartCyc;
		const uint64_t nNop = S.nop_cyc3;
		S.pcm_task_cyc = S.pcm_task_cyc + (nDelta >= nNop ? nDelta - nNop : 0);
	}
}

void CMinimalApp::PostDueEvents()
{
	// Post every song event whose cycle the MCU has reached, in table
	// order, between steps -- exactly the harness barrier. Event
	// cycles are multiples of 12 (600 ns steps) and MCU cycles advance
	// by 12 per step from 0, so each event posts at exactly its step
	// boundary, never late, never early.
	mcu_t& MCU = m_pEmulator->GetMCU();
	while (m_nNextEvent < m_nEventCount)
	{
		const SongEvent& Event = m_pEvents[m_nNextEvent];
		if (Event.cycle > static_cast<uint64_t>(MCU.cycles))
			break;
		m_pEmulator->PostMIDI(std::span<const uint8_t>(m_pBlob + Event.offset, Event.len));
		m_nNextEvent = m_nNextEvent + 1;
	}
}

void CMinimalApp::StepUntilFill(size_t nNeed, u64* pStepUs)
{
	// Core 2 only. Step the MCU + peripherals until the ring holds
	// nNeed frames, or the step budget is exhausted (budget exhaustion
	// with a short ring IS the starvation the stats measure -- it
	// never loops forever on a dead PCM task). Skeletal JV880 shape:
	// NO tick rule, NO lookahead bound, NO IRQ drain -- the MCU never
	// waits on the PCM core for anything. Each step publishes
	// mcu.cycles with a RELEASE store (after StepMCU, before
	// StepPeripherals, mirroring Sterr1's publish right after the
	// cycles increment); the RELEASE orders every direct register
	// write that step made before the published count, so core 3's
	// ACQUIRE-load sees them. StepPeripherals touches no pcm_t, so
	// publishing before it is safe.
	mcu_t& MCU = m_pEmulator->GetMCU();
	ThreadSplitStateCircle& S = *m_pSplitState;
	const SplitSyncProfile& P = S.sync;
	const u64 nStartUs = CTimer::GetClockTicks();
	const uint64_t nStepStartCyc = PMU_ReadCycles();
	u64 nSteps = 0;
	const u64 nStepBudget = static_cast<u64>(nNeed + 2) * StepsPerFrameBudget;

	// Round 3 knobs (all 1/legacy in profile 0, which makes this loop
	// exactly the shaped-var loop): publish cycles_pub every
	// publish_interval steps (see split_thread_circle.h for why any
	// interval <= 66 is safe; the table uses <= 32), check the ring
	// every ring_check_interval steps (one ACQUIRE load of a line
	// core 3 writes per sample -- checking more often than once per
	// ~66 steps cannot observe a new frame anyway), and fold core 3's
	// IRQ0 flag at the top of each step when the profile routes IRQ0
	// through the flag.
	const unsigned nPubInterval = P.publish_interval ? P.publish_interval : 1;
	const unsigned nRingInterval = P.ring_check_interval ? P.ring_check_interval : 1;
	unsigned nPubCountdown = nPubInterval;
	unsigned nRingCountdown = 1;
	const bool bFoldIrq = (P.legacy_irq == 0);
	u64 nPublishes = 0;

	while (nSteps < nStepBudget)
	{
		if (--nRingCountdown == 0)
		{
			if (m_Ring.Avail() >= nNeed)
				break;
			nRingCountdown = nRingInterval;
		}
		if (bFoldIrq)
			SplitFoldIrq0(S, MCU);
		PostDueEvents();
		m_pEmulator->StepMCU();
		// RELEASE publish (every step in the legacy profile = Sterr1's
		// per-instruction granularity; every N steps otherwise).
		if (--nPubCountdown == 0)
		{
			__atomic_store_n(&S.cycles_pub, (uint64_t)MCU.cycles, __ATOMIC_RELEASE);
			nPubCountdown = nPubInterval;
			nPublishes = nPublishes + 1;
		}
		m_pEmulator->StepPeripherals();
		nSteps = nSteps + 1;

		// Song end (once): the end cycle includes the harness's 3 s
		// tail, so crossing it means the whole song + tail rendered.
		// The summary line itself is deferred to the chunk epilogue
		// (after that chunk's Write, so its totals are complete).
		if (!m_bSongDone && static_cast<uint64_t>(MCU.cycles) >= m_nEndCycle)
		{
			m_bSongDone = true;
			m_nCompletions = m_nCompletions + 1;
			m_bSongEndPending = true;
		}
	}

	// Flush: publish the final position once more so core 3 never
	// waits for the next chunk's loop to see the last < N steps.
	if (nPubCountdown != nPubInterval)
	{
		__atomic_store_n(&S.cycles_pub, (uint64_t)MCU.cycles, __ATOMIC_RELEASE);
		nPublishes = nPublishes + 1;
	}
	S.publishes = S.publishes + nPublishes;
	m_nWinSteps = m_nWinSteps + nSteps;
	SampleCore2Events();

	if (pStepUs)
		*pStepUs = CTimer::GetClockTicks() - nStartUs;
	// PMU: raw step-loop wall cycles in this call (core 2's own
	// counter, enabled at AudioLoop start). Accumulated RAW: the
	// window writer subtracts the read-pair cost (nop2 x calls) and
	// the lock spins to derive step_cyc. A stuck counter (failed
	// validation) yields delta 0, and the window writer gates the
	// field to 0 anyway.
	m_nWinStepWallCyc = m_nWinStepWallCyc + (PMU_ReadCycles() - nStepStartCyc);
	m_nWinStepCalls = m_nWinStepCalls + 1;
}

void CMinimalApp::SampleCore2Events()
{
	// Core 2 only. 32-bit counters, sampled once per step-loop call
	// (~5 ms, far below the ~4 s wrap), accumulated as deltas. Covers
	// the whole step loop (MCU work + peripherals + lock spins), which
	// is the core-2 IPC the analysis wants.
	u32 nNow[4];
	PMU_ReadEvents(nNow);
	for (int i = 0; i < 4; ++i)
	{
		m_nWinC2Ev[i] = m_nWinC2Ev[i] + static_cast<u32>(nNow[i] - m_nC2EvLast[i]);
		m_nC2EvLast[i] = nNow[i];
	}
}

size_t CMinimalApp::ResampleChunk(float* pOut, size_t nFrames, size_t* pSilence)
{
	// The port's CNukedSC55Synth streaming resampler, verbatim logic,
	// adapted only in the ring type (this kernel's SkelRing: Dequeue
	// returns false when empty, no fill counter to maintain) and in
	// the scale: the port multiplies by gain * volume/100; here gain
	// is 1.0 and volume is 100, so the scale is 1.0 and the lerp
	// output is written directly.
	// The window (m_SampleA, m_SampleB) brackets the next output
	// position (m_dReadFrac relative to A). The only behavioral
	// difference from a from-scratch resampler is the underrun path,
	// kept exactly as the port's: instead of resyncing to the newest
	// frame, the drain plays queued frames in order and emits
	// silence once the ring empties, then resets the window
	// (m_dReadFrac = 0, window invalid) so the next chunk re-primes
	// from whatever the ring holds then.
	auto DequeueOne = [&](TSample& Out) -> bool
	{
		return m_Ring.Dequeue(Out);
	};

	size_t nDone = 0;
	for (; nDone < nFrames; ++nDone)
	{
		if (!m_bWindowValid)
		{
			TSample A, B;
			if (!DequeueOne(A) || !DequeueOne(B))
				break;
			m_SampleA = A;
			m_SampleB = B;
			m_bWindowValid = true;
		}
		bool bStarved = false;
		while (m_dReadFrac >= 1.0)
		{
			m_SampleA = m_SampleB;
			if (!DequeueOne(m_SampleB))
			{
				bStarved = true;
				break;
			}
			m_dReadFrac -= 1.0;
		}
		if (bStarved)
			break;

		const float nFrac = static_cast<float>(m_dReadFrac);
		pOut[nDone * 2]     = m_SampleA.nLeft + (m_SampleB.nLeft - m_SampleA.nLeft) * nFrac;
		pOut[nDone * 2 + 1] = m_SampleA.nRight + (m_SampleB.nRight - m_SampleA.nRight) * nFrac;

		m_dReadFrac += m_dResampleRatio;
	}
	if (nDone < nFrames)
	{
		// Underrun (emulator too slow, or still booting): silence for
		// the rest of the chunk; the next chunk re-primes from
		// whatever the ring holds then.
		for (size_t j = nDone; j < nFrames; ++j)
		{
			pOut[j * 2]     = 0.0f;
			pOut[j * 2 + 1] = 0.0f;
		}
		m_dReadFrac = 0.0;
		m_bWindowValid = false;
	}
	if (pSilence)
		*pSilence = nFrames - nDone;
	return nDone;
}

void CMinimalApp::DoRunAdvance()
{
	// Core 2 only, called from DrainMailboxes (the chunk epilogue,
	// after WriteSongEnd + the window flush) -- never mid-step.
	// SEPARATE from the deleted song-toggle DoSwitch: the song NEVER
	// changes (E1M1 throughout), there is NO cycle line and NO abort
	// line (the old variant's song completed). Same reset path
	// (Reset + GS_RESET + InitSplitState(true) + resampler reset),
	// then the variant advances: the new variant is stored BEFORE
	// InitSplitState, while the PCM task is parked, so no batch can
	// observe a mid-batch change (core 3 reads the variant at batch
	// start). Core 1 learns the new variant by polling (MisterLoop)
	// and sends the MiSTer status + LCD update from there, never
	// from an audio core. The variant line is written directly:
	// this runs between chunks, so SD-sync cost never delays
	// stepping. The window restarts at the advance (the backend
	// counters were reset by InitFromPCM, so the snapshots must
	// restart too -- and the pre-advance flush in DrainMailboxes
	// already attributed the old variant's last window to it).
	// NOTE: the device queue is NOT flushed (Circle has no
	// queue-flush API), so up to one device queue (~43 ms) of
	// already-resampled old-variant audio still plays out before
	// the new variant is heard; the new variant itself renders from
	// the beginning. n is both the advance count and the new variant
	// (advances are sequential from 0, so the two coincide).
	if (m_bSplitActive)
		StopSplitPCM();
	m_pEmulator->Reset();
	m_pEmulator->PostSystemReset(EMU_SystemReset::GS_RESET);
	const int nOld = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
	const int nNew = nOld + 1;
	__atomic_store_n(&m_nCurrentRun, nNew, __ATOMIC_RELEASE);
	SplitDMB();
	InitSplitState(true);

	// The resampler window brackets the OLD variant's samples; reset
	// it so the first post-advance chunk re-primes from the new
	// variant's ring instead of lerping one stale frame across.
	m_bWindowValid = false;
	m_dReadFrac = 0.0;

	// E1M1 from the beginning (the cursor never leaves E1M1, but
	// re-point it explicitly: the advance restarts the song).
	SetSongCursor(0);
	m_nCurrentSong = 0;
	m_nNextEvent = 0;
	m_bSongDone = false;
	m_bSongEndPending = false;
	m_nSongDue = 0;
	m_nSongProd = 0;
	m_nSongShort = 0;
	m_nSongUnderruns = 0;
	m_nRunProd = 0;
	m_nRunDue = 0;
	m_nRunUnderruns = 0;
	m_nRunPcmCyc = 0;
	m_nRunMixCyc = 0;
	m_nRunStepCyc = 0;
	m_nRunLockCyc = 0;
	m_nRunLockWaits = 0;
	m_nRunLockCont = 0;
	m_nRunSteps = 0;
	m_nRunTicks = 0;
	for (int i = 0; i < 4; ++i)
	{
		m_nRunC2Ev[i] = 0;
		m_nRunC3Ev[i] = 0;
	}
	// The core-2 event counters keep counting across the reset; resync
	// the last-read values so the first window of the new run does not
	// inherit the reset's instructions.
	PMU_ReadEvents(m_nC2EvLast);
	ResetWindow();

	LogLine("run n=%d from=%d to=%d name=%s pcm=%d sync=%d timer=%d sync_name=%s", nNew, nOld, nNew, RunName(nNew),
	        RunPcm(nNew), RunSync(nNew), RunTimer(nNew), SyncProfile(RunSync(nNew)).short_name);
}

void CMinimalApp::DrainMailboxes()
{
	// Core 2 only. Called between chunks (playing epilogue) or in the
	// pause loop; never mid-step. Song-end (set while stepping) is
	// written first, then the partial window is flushed (so every
	// window belongs entirely to one variant), then the sequence
	// advances (variant < g_pcmVariantCount - 1) or finishes (last variant: sequence_done,
	// then post-song_end idle forever). No keys exist in this kernel,
	// so there is no key branch and no pause/resume path from input
	// (the AudioLoop pause branch is unreachable: m_bPaused is never
	// set). Every event is logged with its action.
	if (m_bSongEndPending)
	{
		m_bSongEndPending = false;
		WriteSongEnd();
		// Flush the partial window NOW: the advance below resets the
		// snapshots, and without the flush the straddling window
		// would be attributed to the new variant. Same for the
		// finish path, so the sequence_done totals are complete at
		// that line.
		if (m_nWinChunks > 0)
			WriteWindow(false);
		else
			ResetWindow();
		WriteRunTotal();	// this run's totals (one line per run; survives a power cut later on)
		const int nVariant = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
		if (nVariant < g_runCount - 1)
		{
			DoRunAdvance();	// logs the run line; E1M1 restarts
		}
		else if (!m_bSequenceDone)
		{
			WriteSequenceDone();
			m_bSequenceDone = true;
			// No reset, no restart: post-song_end idle continues
			// EXACTLY as today (stepping idle MCU, silence output,
			// clean idle windows, LED heartbeat unchanged). Audio
			// silence + the sequence_done line ARE the done signals;
			// LED-off is NEVER a done signal (frozen LED already
			// means wedge/stall in this codebase).
		}
	}

	// MiSTer events, in order. Every selection is logged and IGNORED:
	// this kernel has one engine, so there is nothing to switch to,
	// and the writeback always reports the variant reality. No OSD
	// text is attempted (the 3-byte protocol has no text field).
	TMisterEvent MisterEvent;
	while (m_MisterRing.Dequeue(MisterEvent))
	{
		switch (MisterEvent.type)
		{
		case 0:
			m_nMisterDowns = m_nMisterDowns + 1;
			LogLine("mister_down n=%llu", static_cast<unsigned long long>(m_nMisterDowns));
			break;
		case 1:
			m_nMisterUps = m_nMisterUps + 1;
			LogLine("mister_init n=%llu rx_synth=0x%02x rx_name=%s rx_romset=0x%02x rx_sf=0x%02x tx=0xA2,0xFF,0x%02x action=writeback",
			        static_cast<unsigned long long>(m_nMisterUps),
			        MisterEvent.synth, MisterSynthName(MisterEvent.synth),
			        MisterEvent.romset, MisterEvent.sf, MisterEvent.our_variant);
			break;
		case 2:
			LogLine("mister_synth synth=0x%02x name=%s action=ignored", MisterEvent.synth, MisterSynthName(MisterEvent.synth));
			break;
		case 3:
			LogLine("mister_romset romset=0x%02x action=ignored", MisterEvent.romset);
			break;
		case 4:
			LogLine("mister_sfindex index=0x%02x action=ignored", MisterEvent.sf);
			break;
		case 5:
			LogLine("mister_mute rx_synth=0xA0 action=ignored");
			break;
		case 6:
			LogLine("mister_writefail rx_synth=0x%02x rx_romset=0x%02x rx_sf=0x%02x", MisterEvent.synth, MisterEvent.romset, MisterEvent.sf);
			break;
		default:
			break;
		}
	}

	// Drops (producer-side counters, core 2 reports changes).
	if (m_nMisterDrops != m_nMisterDropsLogged)
	{
		m_nMisterDropsLogged = m_nMisterDrops;
		LogLine("mister_drop total=%llu", static_cast<unsigned long long>(m_nMisterDrops));
	}
}

void CMinimalApp::IdleLoop()
{
	// Core 0 only, NO HARDWARE KEYS (2026-10-31 ~09:23):
	// playback is wired to audio, so no input is needed or wanted.
	// This core spins inertly (yield hint) forever: it NEVER reads
	// GPIO 17/27, never posts events, and cannot affect playback or
	// variants in any way. MiniPause, NOT CScheduler::Yield(): this
	// kernel registers no CTasks, so Yield() can never return.
	while (true)
		MiniPause();
}

// Minimal 5x7 font for the LCD variant line, column bytes (LSB = top
// row). Only the glyphs "VAR n name" needs: V, A, R, space, digits
// 0-3, and lowercase b, a, s, e, l, i, n, c, h, d, p, o. Each glyph
// was sketched on a 5-wide x 7-tall grid and converted column by
// column; a slightly imperfect glyph is still readable, and the
// status number + log lines are authoritative anyway. Unknown
// characters render as blanks.
static const u8* VariantFontGlyph(char ch)
{
	// Classic 5x7 column font (LSB = top row) for every character the
	// run names use ("VAR nn name": digits, lowercase, a few capitals).
	// Unknown characters render as blanks.
	struct TGlyph { char ch; u8 col[5]; };
	static const TGlyph Glyphs[] = {
		{' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
		{'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
		{'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
		{'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
		{'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
		{'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
		{'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
		{'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
		{'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
		{'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
		{'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
		{'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
		{'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
		{'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
		{'a', {0x20, 0x54, 0x54, 0x54, 0x78}},
		{'b', {0x7F, 0x48, 0x44, 0x44, 0x38}},
		{'c', {0x38, 0x44, 0x44, 0x44, 0x20}},
		{'d', {0x38, 0x44, 0x44, 0x48, 0x7F}},
		{'e', {0x38, 0x54, 0x54, 0x54, 0x18}},
		{'f', {0x08, 0x7E, 0x09, 0x01, 0x02}},
		{'h', {0x7F, 0x08, 0x04, 0x04, 0x78}},
		{'i', {0x00, 0x44, 0x7D, 0x40, 0x00}},
		{'k', {0x7F, 0x10, 0x28, 0x44, 0x00}},
		{'l', {0x00, 0x41, 0x7F, 0x40, 0x00}},
		{'m', {0x7C, 0x04, 0x18, 0x04, 0x78}},
		{'n', {0x7C, 0x08, 0x04, 0x04, 0x78}},
		{'o', {0x38, 0x44, 0x44, 0x44, 0x38}},
		{'p', {0x7C, 0x14, 0x14, 0x14, 0x08}},
		{'r', {0x7C, 0x08, 0x04, 0x04, 0x08}},
		{'s', {0x48, 0x54, 0x54, 0x54, 0x20}},
		{'t', {0x04, 0x3F, 0x44, 0x40, 0x20}},
		{'u', {0x3C, 0x40, 0x40, 0x20, 0x7C}},
		{'y', {0x0C, 0x50, 0x50, 0x50, 0x3C}},
		{'z', {0x44, 0x64, 0x54, 0x4C, 0x44}},
	};
	for (const TGlyph& g : Glyphs)
	{
		if (g.ch == ch)
			return g.col;
	}
	return Glyphs[0].col;
}

void CMinimalApp::DrawVariantLCD(int nVariant)
{
	// Core 1 only, BEST-EFFORT ONLY. Draws "VAR n name" (e.g. "VAR 1
	// sched") on the SSD1306 at slave 0x3C (MiSTer-simulated
	// display; needs a supported core + LCD-On/Auto mode on the
	// MiSTer side to be visible). Every Write's return value is
	// ignored: NACK / no display is tolerated silently.
	// CI2CMaster::Write returns promptly on NACK (S_ERR+S_DONE);
	// only a physically stuck bus could spin it, which would
	// equally wedge the existing 0x45 polling -- and core 1 wedging
	// cannot stop audio (cores 2/3 are independent). This is never
	// called from an audio core, and only while the link is active
	// (the 0x3C display exists only via MiSTer simulation).
	// The init sequence + page-0 frame are re-sent on every draw
	// (self-healing if the display reset); ~30 ms of core-1 I2C per
	// draw, once per variant change + init.
	char Text[16];
	snprintf(Text, sizeof(Text), "VAR %d %s", nVariant, RunName(nVariant));
	static const u8 Init[] = {
		0x00,	// control: commands follow
		0xAE,	// display off
		0xD5, 0x80,	// display clock divide / oscillator
		0xA8, 0x3F,	// multiplex ratio 1/64
		0xD3, 0x00,	// display offset 0
		0x40,	// start line 0
		0x8D, 0x14,	// charge pump on
		0xA1,	// segment remap (column 127 -> SEG0)
		0xC8,	// COM scan direction remapped
		0xDA, 0x12,	// COM pins hardware config
		0x81, 0xCF,	// contrast
		0xD9, 0xF1,	// pre-charge period
		0xDB, 0x40,	// VCOMH deselect level
		0xA4,	// display all on resume (GDDRAM content)
		0xA6,	// normal display (not inverted)
		0x20, 0x00,	// horizontal addressing mode
		0x21, 0x00, 0x7F,	// column range 0..127
		0x22, 0x00, 0x00,	// page range 0..0
		0xAF	// display on
	};
	(void)m_pI2CMaster->Write(MisterLCDAddress, Init, sizeof(Init));
	// Page-0 frame: control 0x40 (data) + 128 columns (text glyphs
	// + blanks to the end of the page, so no stale columns linger).
	u8 Frame[1 + 128];
	Frame[0] = 0x40;
	for (int i = 1; i < 129; ++i)
		Frame[i] = 0x00;
	int nCol = 0;
	for (const char* p = Text; *p != '\0' && nCol + 6 <= 128; ++p)
	{
		const u8* pGlyph = VariantFontGlyph(*p);
		for (int c = 0; c < 5; ++c)
			Frame[1 + nCol + c] = pGlyph[c];
		nCol += 6;	// 5 glyph columns + 1 blank column
	}
	(void)m_pI2CMaster->Write(MisterLCDAddress, Frame, sizeof(Frame));
}

void CMinimalApp::MisterLoop()
{
	// Core 1 only. MiSTer link: the Pi is the I2C master, the MiSTer
	// User Port slave is at 0x45, and the protocol is a 3-byte
	// status {synth, romset, soundfont}. EVERY received control
	// event is REPORTED to the mailbox so core 2 can log it.
	// Poll every 10 ms. On the first successful Read after inactive
	// / failed reads (the MiSTer's "Use MT32-pi" ON switch), mark
	// the link active, write back OUR variant reality {0xA2, 0xFF,
	// variant} (the DOCUMENTED minimal-kernel-only hack: upstream
	// cores only know 0xA1/0xA2, so truthful 0xA4 would likely
	// display as Unknown), draw the LCD "VAR n name" (best-effort),
	// and queue a mister_up (init) event. On link-down (Read fails
	// after active -- the OFF switch), mark inactive, queue
	// mister_down, and KEEP THE ENGINE RUNNING (the test must not
	// depend on the MiSTer staying connected). Every received-field
	// change while active queues its own event (synth / romset /
	// sfindex; Mute gets its own type but counts as link-active,
	// exactly like mt32-pi's Mute path) and re-sends the variant
	// status (correct the MiSTer back to reality). A variant
	// advance (core 2, DoRunAdvance) is picked up by POLLING:
	// each poll, if the committed variant differs from the last
	// sent one while the link is active, re-send the status + LCD
	// (<= 10 ms after the advance; a failed send retries next poll
	// because lastSent only updates on success). The log's variant
	// line is ground truth; this loop is only the display. Core 1
	// never touches the emulator or the file; a full ring drops the
	// event and counts it (core 2 logs the drop). MiniPause, NOT
	// CScheduler::Yield(): no CTasks are registered, so Yield()
	// can never return.
	assert(m_pI2CMaster != nullptr);

	bool bActive = false;
	u8 LastSynth = 0xFF;
	u8 LastRomset = 0xFF;
	u8 LastSf = 0xFF;
	int nLastSentVariant = -1;	// -1 = never sent (forces the init-path send)
	u64 nLastPollUs = CTimer::GetClockTicks();

	while (true)
	{
		const u64 nNowUs = CTimer::GetClockTicks();
		if (nNowUs - nLastPollUs >= MisterPollUs)
		{
			nLastPollUs = nNowUs;
			// Variant advance while the link is active: re-send the
			// status + LCD now. A failed send does NOT update
			// lastSent, so the next poll retries (self-healing).
			// (When the link is down this is skipped: the init path
			// below sends the CURRENT variant when the link comes
			// back up, so no advance is ever missed.)
			const int nPollVariant = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
			if (bActive && nPollVariant != nLastSentVariant)
			{
				const u8 VariantStatus[3] = {MisterSynthVariant, 0xFF, static_cast<u8>(nPollVariant)};
				if (m_pI2CMaster->Write(MisterI2CAddress, VariantStatus, sizeof(VariantStatus)) == 3)
				{
					nLastSentVariant = nPollVariant;
					DrawVariantLCD(nPollVariant);
				}
			}
			u8 Status[3];
			const int nRead = m_pI2CMaster->Read(MisterI2CAddress, Status, sizeof(Status));
			if (nRead != 3)
			{
				if (bActive)
				{
					bActive = false;
					LastSynth = 0xFF;
					LastRomset = 0xFF;
					LastSf = 0xFF;
					const TMisterEvent Ev{0, 0xFF, 0xFF, 0xFF};
					if (!m_MisterRing.Enqueue(Ev))
						m_nMisterDrops = m_nMisterDrops + 1;
				}
			}
			else
			{
				const bool bChanged = !bActive || Status[0] != LastSynth || Status[1] != LastRomset || Status[2] != LastSf;
				if (!bActive)
				{
					// First contact: write back first, then report
					// active only on a successful write (mt32-pi's
					// order: a failed first write leaves the link
					// inactive so the next poll re-inits). The
					// status carries the CURRENT variant (the link
					// may have come up mid-sequence, so variant 0
					// must NOT be assumed here).
					const int nInitVariant = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
					const u8 InitStatus[3] = {MisterSynthVariant, 0xFF, static_cast<u8>(nInitVariant)};
					const int nWritten = m_pI2CMaster->Write(MisterI2CAddress, InitStatus, sizeof(InitStatus));
					if (nWritten == 3)
					{
						bActive = true;
						nLastSentVariant = nInitVariant;
						DrawVariantLCD(nInitVariant);
						LastSynth = Status[0];
						LastRomset = Status[1];
						LastSf = Status[2];
						const TMisterEvent Ev{1, Status[0], Status[1], Status[2], static_cast<u8>(nInitVariant)};
						if (!m_MisterRing.Enqueue(Ev))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
					else
					{
						const TMisterEvent Ev{6, Status[0], Status[1], Status[2], static_cast<u8>(nInitVariant)};
						if (!m_MisterRing.Enqueue(Ev))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
				}
				else if (bChanged)
				{
					// One event per changed field, in field order.
					if (Status[0] != LastSynth)
					{
						const TMisterEvent Ev{Status[0] == MisterSynthMute ? static_cast<u8>(5) : static_cast<u8>(2), Status[0], Status[1], Status[2]};
						if (!m_MisterRing.Enqueue(Ev))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
					if (Status[1] != LastRomset)
					{
						const TMisterEvent Ev{3, Status[0], Status[1], Status[2]};
						if (!m_MisterRing.Enqueue(Ev))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
					if (Status[2] != LastSf)
					{
						const TMisterEvent Ev{4, Status[0], Status[1], Status[2]};
						if (!m_MisterRing.Enqueue(Ev))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
					LastSynth = Status[0];
					LastRomset = Status[1];
					LastSf = Status[2];
					// Correct the MiSTer back to reality (it may have
					// selected an engine that does not exist here):
					// the variant status, never 0xA4/0xFF/0xFF (that
					// would clobber the displayed variant number).
					// lastSent is NOT updated here: the poll block
					// above owns variant-change sends (it re-sends
					// promptly if this writeback's variant differs
					// from the last sent one).
					const int nChangeVariant = __atomic_load_n(&m_nCurrentRun, __ATOMIC_ACQUIRE);
					const u8 ChangeStatus[3] = {MisterSynthVariant, 0xFF, static_cast<u8>(nChangeVariant)};
					const int nWritten = m_pI2CMaster->Write(MisterI2CAddress, ChangeStatus, sizeof(ChangeStatus));
					if (nWritten != 3)
					{
						bActive = false;
						LastSynth = 0xFF;
						LastRomset = 0xFF;
						LastSf = 0xFF;
						const TMisterEvent EvDown{0, 0xFF, 0xFF, 0xFF};
						if (!m_MisterRing.Enqueue(EvDown))
							m_nMisterDrops = m_nMisterDrops + 1;
						const TMisterEvent EvFail{6, Status[0], Status[1], Status[2]};
						if (!m_MisterRing.Enqueue(EvFail))
							m_nMisterDrops = m_nMisterDrops + 1;
					}
				}
				// Steady state: no change, no write, no event.
			}
		}
		MiniPause();
	}
}

void CMinimalApp::AudioLoop()
{
	// Core 2 only. Device-paced demand in 256-frame output chunks at
	// 48 kHz (mt32-pi's chunk), from the 32 kHz native ring via the
	// port's streaming resampler. The device clock (48,000 frames/s)
	// is the independent reference: the window pct is wall-clock due
	// vs produced, so a slow engine shows even though the device
	// queue absorbs transient slips.
	// Startup: step until the native ring holds 4 chunks (1024
	// frames), resample+Write TWO output chunks into the device
	// queue (startup cushion, unaccounted -- window 0 therefore reads
	// ~99.8% even on a perfect run; judge sustained windows, not
	// window 0), then Start() (Start with an empty queue fails; DMA
	// pulls two 128-frame buffers at Start, leaving one output chunk
	// queued). The window starts after Start.
	// Per playing chunk:
	//  - if a DMA restart is pending and 20 ms have not elapsed: top
	//    up the native ring only (it absorbs ~512 ms) and run the
	//    epilogue; the restart attempt happens once the wait expires
	//    (state guaranteed Idle by then).
	//  - otherwise wait until the device queue has FREE space for a
	//    full chunk (CORRECTED vs old shaped: Avail returns QUEUED
	//    frames, so the old `Avail < chunk` wait waited for the queue
	//    to FILL; free = queue size - queued -- see the header
	//    comment), step until the ring holds what the resampler
	//    needs, resample, convert float -> s32 (mt32-pi's AudioTask
	//    conversion verbatim), Write. A short Write is a device drop
	//    (counted + logged; 0 by construction with the corrected
	//    wait: free can only grow between the wait exit and the
	//    Write, since this loop is the sole writer and the DMA
	//    handler only frees). If the device went inactive, arm the
	//    restart wait instead of calling Start immediately (Start
	//    during Terminating asserts).
	//  - epilogue: DrainMailboxes (song-end + key/mister/drop lines),
	//    window check (5 s wall), ACT heartbeat (togglefix semantics:
	//    blink ~1 Hz while the window is clean, hold Off for a bad
	//    window).
	// Pause branch: no stepping, no demand, no Writes (the device
	// queue drains and DMA terminates -- revived by the restart
	// check on resume). Mailboxes still drain (resume / cycle /
	// mister events) and paused windows emit on the 5 s schedule.
	// A paused stretch counts due=0, never shortfall. The LED freezes
	// during pause (no chunks flow) -- itself a visible paused state.
	float FloatBuffer[OutChunkFrames * 2];
	s32 IntBuffer[OutChunkFrames * 2];
	auto WriteOneChunk = [&](bool bAccount, size_t* pRendered, size_t* pSilence)
	{
		const u64 nResampleStartUs = CTimer::GetClockTicks();
		size_t nSilence = 0;
		const size_t nRendered = ResampleChunk(FloatBuffer, OutChunkFrames, &nSilence);
		const u64 nResampleUs = CTimer::GetClockTicks() - nResampleStartUs;
		for (size_t i = 0; i < OutChunkFrames * 2; ++i)
			IntBuffer[i] = static_cast<s32>(FloatBuffer[i] * Sample24BitMax);
		const int nWritten = m_pSound->Write(IntBuffer, sizeof(IntBuffer));
		if (nWritten != static_cast<int>(sizeof(IntBuffer)))
		{
			m_nSoundDrops = m_nSoundDrops + 1;
			m_nWinSoundDrops = m_nWinSoundDrops + 1;
			LogLine("sound_drop expected=%u got=%d", static_cast<unsigned>(sizeof(IntBuffer)), nWritten);
		}
		if (bAccount)
		{
			m_nWinResampleUs = m_nWinResampleUs + nResampleUs;
			m_nWinProd = m_nWinProd + nRendered;
			m_nSongProd = m_nSongProd + nRendered;
			m_nWinShort = m_nWinShort + nSilence;
			m_nSongShort = m_nSongShort + nSilence;
			if (nSilence > 0)
			{
				m_nWinUnderruns = m_nWinUnderruns + 1;
				m_nSongUnderruns = m_nSongUnderruns + 1;
			}
			m_nWinChunks = m_nWinChunks + 1;
		}
		if (pRendered)
			*pRendered = nRendered;
		if (pSilence)
			*pSilence = nSilence;
	};

	// PMU (core 2): enable + validate + NOP-calibrate BEFORE any
	// stepping (StepUntilFill reads the counter below). The boot
	// line carries pmc2/pmc3, so it is logged HERE, not in
	// Initialize: core 3's result is only known once its loop has
	// started. Core 2 waits up to 100 ms for core 3's validation
	// flag (normally ~1 ms; the cap only bites if core 3 never
	// started, in which case pmc3=fail is the correct report and
	// the kernel runs degraded, never hung).
	PMU_Enable();
	m_bPmu2Ok = PMU_Validate();
	PMU_EnableEvents();
	m_bPmuEv2Ok = PMU_ValidateEvents();
	PMU_ReadEvents(m_nC2EvLast);
	if (m_pSplitState)
		m_pSplitState->nop_cyc2 = PMU_CalibrateNop();
	SplitDMB();
	const u64 nPmuWaitStartUs = CTimer::GetClockTicks();
	while (!m_bPmu3Done && CTimer::GetClockTicks() - nPmuWaitStartUs < 100000ULL)
		MiniPause();
	SplitDMB();
	const bool bPmu3Ok = m_bPmu3Done && m_bPmu3Ok;
	LogLine("boot kernel=mt32-shaped-var3 split=on song=%s rate=%u out_rate=%u out_chunk=%u devq=%u romset=%s audio=i2s pmc2=%s pmc3=%s pmev2=%s pmev3=%s timer_selftest=%s selftest_plain_mism=%llu selftest_inject_mism=%llu seq=auto runs=%d pcm_variants=%d",
	        SongName(m_nCurrentSong),
	        m_nNativeRate,
	        OutRate,
	        static_cast<unsigned>(OutChunkFrames),
	        DeviceQueueFrames,
	        m_pRomsetName != nullptr ? m_pRomsetName : "unknown",
	        m_bPmu2Ok ? "ok" : "fail",
	        bPmu3Ok ? "ok" : "fail",
	        m_bPmuEv2Ok ? "ok" : "fail",
	        (m_bPmu3Done && m_bPmuEv3Ok) ? "ok" : "fail",
	        m_bTimerSelfTestPass ? "pass" : "FAIL",
	        static_cast<unsigned long long>(m_nTimerSelfTestPlain),
	        static_cast<unsigned long long>(m_nTimerSelfTestInject),
	        g_runCount, g_pcmVariantCount);
	for (int i = 0; i < g_runCount; ++i)
	{
		const SplitSyncProfile& P = SyncProfile(g_runs[i].sync);
		LogLine("runlist n=%d name=%s pcm=%d pcm_name=%s sync=%d sync_name=%s lock_mode=%u legacy_lock=%u legacy_timing=%u legacy_irq=%u publish_interval=%u ring_check_interval=%u core3_backoff=%u timer=%d",
		        i, g_runs[i].name, g_runs[i].pcm,
		        (g_runs[i].pcm >= 0 && g_runs[i].pcm < g_pcmVariantCount) ? g_pcmVariants[g_runs[i].pcm].short_name : "?",
		        P.number, P.short_name, P.lock_mode, P.legacy_lock, P.legacy_timing, P.legacy_irq,
		        P.publish_interval, P.ring_check_interval, P.core3_backoff, g_runs[i].timer);
	}
	LogLine("legend r3: var = RUN index (see runlist; pcm/sync/timer fields give the run's components); steps = MCU steps in the window, ticks = PCM ticks rendered; step_cyc/steps = core-2 cycles per MCU step (THE core-2 comparator; excludes lock spins), pcm_cyc/ticks = core-3 cycles per PCM tick (THE core-3 comparator); pubs = cycles_pub publishes; lock_cont = lock ops whose first try failed (round-3 timing only); fifo_hw = deferred-store FIFO high water (FIFO modes), fifo_full = pushes that waited (must be 0); irq0 = voice-end IRQs folded via the flag; tchk = lazy-timer check (off when the run's timer mode is not CHECK; FAIL = any mismatch; tchk_first = mcu cycles of the first mismatch, tchk_code per mcu_timer.h), lazy_skips = TIMER_Clock fast-path returns; c2_*/c3_* = PMU events per core over the window's work (inst = instructions retired, brmis = branch mispredicts, l1d/l2d = data cache refills); run_total lines carry the per-run sums + cyc_per_step/cyc_per_tick; sequence_done is the terminal marker. Boot: pmev2/pmev3 validate the event counters (fail -> those fields read 0); timer_selftest=pass requires the plain lazy-vs-legacy self-test to report 0 mismatches AND the fault-injected run to report > 0 (proves the detector sees a one-tick event shift).");
	LogLine("legend win: due is wall-clock (elapsed*48000); pct = prod*100/due; window 0 excludes the 2 startup chunks (512 frames) so it reads ~99.8%% on a perfect run -- judge sustained windows; tick/lookahead/rpc waits+us, write_hw/irq_hw, write_ovf/irq_ovf are 0 (machinery removed); max_lag is the peak published-minus-rendered lag since (re)activation; ring_drops is the drop-oldest total since boot (inherited wart: togglefix showed 15832 at the toggle, then flat -- mechanism unpinned, reported not hidden); core2_pct is the step-loop wall share; resample_us is the resampler wall time; devq_maxavail is peak device FREE space (near 2048 = device nearly starved; CORRECTED vs old shaped, which measured queued); devq_low counts chunks with <1 chunk QUEUED at Write time; sound_drops must be 0 by construction (corrected free-space wait); sustained pct ~80-85 is the ENGINE ceiling (togglefix: ~85%% real-time), not a failure -- pct=100 needs a real-time engine; DIAG fields: lock_us = core-2 state-lock spin-wait wall in the window, lock_waits = lock ops taken, mcu_us = step-loop wall minus lock_us (pure MCU stepping + peripherals); PMU fields: var = variant this window ran under (0 baseline, 1 sched, 2 pipe, 3 neon2, 4 splitacc, 5 layout); pcm_cyc = core-3 total PCM task cycles, mix_cyc = core-3 voice-mix cycles per batch, step_cyc = core-2 MCU stepping cycles only, lock_cyc = core-2 state-lock spin cycles; every cycle value is NOP-corrected (double-read cost subtracted, clamped at 0); a failed counter (boot pmc2/pmc3=fail) reports 0 for its fields -- the boot line is the validity signal");
	LogLine("legend events: every control event is logged with t= timestamp + action -- variant (n/from/to/name on each advance), song_end (with var), sequence_done (per-variant totals vN_prod/vN_pcm_cyc/vN_mix_cyc/vN_step_cyc/vN_lock_cyc), mister_init/mister_down/mister_synth/mister_romset/mister_sfindex/mister_mute/mister_writefail, sound_drop/sound_restart/sound_start_fail, mister_drop. NO keys exist: no key lines, no cycle lines, no song_aborted lines (advances happen only on completed songs). The MIDI stream is NEVER logged. No OSD text is attempted (3-byte protocol has no text field); the MiSTer's own pop-ups on status change are unchanged; the variant NUMBER is the status soundfont byte {0xA2, 0xFF, variant} (documented minimal-kernel-only hack) and the variant NAME is the SSD1306 mirror at 0x3C (best-effort, core 1 only). Audio leaves via the Pi I2S output, NOT through the MiSTer. LED: core-2 heartbeat, ~1 Hz while windows clean, held Off on bad windows, frozen while paused or device-space stalled; LED-off is NEVER a done signal.");

	// ACT heartbeat (core 2 only, togglefix semantics): LED off from
	// here; it blinks only once chunks flow (see the epilogue below).
	// Frozen Off = wedged in pre-fill or before; frozen at any level
	// for > 10 s = wedged (or device-space stalled, or paused).
	CActLED* pLED = CActLED::Get();
	if (pLED)
		pLED->Off();
	m_bLEDOn = false;
	m_nLEDChunks = 0;

	u64 nDummyUs = 0;
	StepUntilFill(ChunkFrames * 4, &nDummyUs);
	{
		size_t nRendered = 0, nSilence = 0;
		WriteOneChunk(false, &nRendered, &nSilence);
		WriteOneChunk(false, &nRendered, &nSilence);
	}
	if (m_pSound->Start())
	{
		m_bSoundStarted = true;
	}
	else
	{
		m_bSoundStartFailed = true;
		LogLine("sound_start_fail");
	}
	ResetWindow();

	while (true)
	{
		if (m_bPaused)
		{
			DrainMailboxes();
			if (CTimer::GetClockTicks() - m_nWinStartUs >= WindowUs)
				WriteWindow(true);
			// 1 ms cadence: responsive without a hot spin.
			const u64 nPauseTick = CTimer::GetClockTicks();
			while (CTimer::GetClockTicks() - nPauseTick < 1000ULL)
				MiniPause();
			continue;
		}

		if (m_bSoundRestartPending)
		{
			if (CTimer::GetClockTicks() - m_nSoundRestartWaitStartUs >= SoundRestartWaitUs)
			{
				m_bSoundRestartPending = false;
				if (!m_bSoundStartFailed)
				{
					if (m_pSound->Start())
					{
						m_bSoundStarted = true;
						m_nSoundRestarts = m_nSoundRestarts + 1;
						m_nWinSoundRestarts = m_nWinSoundRestarts + 1;
						LogLine("sound_restart n=%llu", static_cast<unsigned long long>(m_nSoundRestarts));
					}
					else
					{
						m_bSoundStartFailed = true;
						LogLine("sound_start_fail");
					}
				}
			}
			else
			{
				// Still waiting out Terminating -> Idle: top up the
				// native ring only (no resample, no Write).
				const size_t nNeed = static_cast<size_t>(m_dReadFrac + static_cast<double>(OutChunkFrames) * m_dResampleRatio) + 3;
				u64 nStepUs = 0;
				StepUntilFill(nNeed, &nStepUs);
				m_nWinCore2Us = m_nWinCore2Us + nStepUs;
				DrainMailboxes();
				if (CTimer::GetClockTicks() - m_nWinStartUs >= WindowUs)
					WriteWindow(false);
				MiniPause();
				continue;
			}
		}

		// Wait for device FREE space for a full chunk (skipped once
		// sound Start has permanently failed, so the log keeps
		// flowing). CORRECTED vs old shaped (see the header comment):
		// GetQueueFramesAvail() returns QUEUED frames, so waiting on
		// Avail >= chunk waits for the queue to FILL; free space is
		// the queue size minus queued. MiniPause, NOT Yield: no
		// CTasks are registered, so Yield() can never return.
		if (!m_bSoundStartFailed)
		{
			while (m_pSound->GetQueueSizeFrames() - m_pSound->GetQueueFramesAvail() < OutChunkFrames)
				MiniPause();
		}

		// Frames the streaming resampler will need for this chunk:
		// the position advance, plus the interpolation endpoint, the
		// entry slide, and rounding (the port's nNeed formula).
		const size_t nNeed = static_cast<size_t>(m_dReadFrac + static_cast<double>(OutChunkFrames) * m_dResampleRatio) + 3;
		u64 nStepUs = 0;
		StepUntilFill(nNeed, &nStepUs);
		m_nWinCore2Us = m_nWinCore2Us + nStepUs;

		// Device queue state at Write time (after stepping, so the
		// drain during stepping is included). Avail = QUEUED frames;
		// free = queue size - queued. devq_maxavail tracks peak FREE
		// space (near 2048 = device nearly starved); devq_low counts
		// chunks where fewer than one chunk remained QUEUED (device
		// nearly starved). (Old shaped measured both on Avail
		// directly, inverting both fields.)
		const unsigned nQueuedBeforeWrite = m_pSound->GetQueueFramesAvail();
		const unsigned nFreeBeforeWrite = m_pSound->GetQueueSizeFrames() - nQueuedBeforeWrite;
		if (nFreeBeforeWrite > m_nWinDevQMaxAvail)
			m_nWinDevQMaxAvail = nFreeBeforeWrite;
		if (nQueuedBeforeWrite < OutChunkFrames)
			m_nWinDevQLow = m_nWinDevQLow + 1;

		size_t nRendered = 0, nSilence = 0;
		WriteOneChunk(true, &nRendered, &nSilence);

		// DMA restart check: DMA terminates when its queue drains
		// (every pause; any starvation). Arm the wait; the restart
		// itself happens at the top of a later iteration, once the
		// state is guaranteed Idle.
		if (!m_bSoundRestartPending && !m_bSoundStartFailed && !m_pSound->IsActive())
		{
			m_bSoundRestartPending = true;
			m_nSoundRestartWaitStartUs = CTimer::GetClockTicks();
			LogLine("sound_restart_wait");
		}

		DrainMailboxes();
		if (CTimer::GetClockTicks() - m_nWinStartUs >= WindowUs)
			WriteWindow(false);

		// ACT heartbeat (core 2 only, togglefix semantics): blink
		// ~1 Hz while the window is clean; hold Off for a bad
		// window; resume on the next clean window (ResetWindow
		// cleared the counters). Mirrors DemandLoop's order (window
		// check first, then heartbeat on the current window).
		if (pLED)
		{
			if (m_nWinShort > 0 || m_nWinUnderruns > 0)
			{
				if (m_bLEDOn)
				{
					pLED->Off();
					m_bLEDOn = false;
				}
			}
			else
			{
				m_nLEDChunks = m_nLEDChunks + 1;
				if ((m_nLEDChunks % 64) == 0)
				{
					m_bLEDOn = !m_bLEDOn;
					if (m_bLEDOn)
						pLED->On();
					else
						pLED->Off();
				}
			}
		}
	}
}

void CMinimalApp::PCMLoop()
{
	// Core 3 only: PCM tick batches. PMU first: enable + validate +
	// NOP-calibrate BEFORE any batch (PCMTaskIteration reads the
	// counter), then flag the result for core 2's boot line
	// (DMB-ordered, like the split handshake). A failed counter
	// means pcm/mix report 0 (the boot line is the validity signal,
	// never silent zeros). The iteration itself parks when
	// inactive (sets m_bPCMIdle and returns), so it must be called
	// UNCONDITIONALLY: gating on m_bSplitActive starved the parked
	// branch, and StopSplitPCM's wait for m_bPCMIdle could never
	// terminate -- the first registered switch wedged core 2 in the
	// old song-switch path (skel runs #2/#3). The unconditional call
	// lets the parked branch acknowledge idle so the switch can
	// proceed.
	PMU_Enable();
	m_bPmu3Ok = PMU_Validate();
	PMU_EnableEvents();
	m_bPmuEv3Ok = PMU_ValidateEvents();
	if (m_pSplitState)
		m_pSplitState->nop_cyc3 = PMU_CalibrateNop();
	SplitDMB();
	m_bPmu3Done = true;
	SplitDMB();
	while (true)
	{
		PCMTaskIteration();
	}
}

void CMinimalApp::Run(unsigned nCore)
{
	switch (nCore)
	{
	case 0:
		IdleLoop();
		break;

	case 1:
		MisterLoop();
		break;

	case 2:
		AudioLoop();
		break;

	case 3:
		PCMLoop();
		break;

	default:
		break;
	}
}

bool CMinimalApp::Initialize()
{
	// Stats file FIRST (the SD card is already mounted by the kernel):
	// every later failure is recorded here when possible, and a kernel
	// that cannot write stats cannot produce evidence, so it must not
	// run at all. FA_CREATE_ALWAYS: each boot is one run's log.
	if (f_open(&m_StatsFile, MinimalStatsPath, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
		return false;
	m_bStatsOpen = true;

	std::vector<TROMFile> Files;
	if (!ScanROMFiles(Files))
	{
		LogLine("error no ROM files found in %s", MinimalROMDirectory);
		return false;
	}

	TROMFile* pMatchedFiles[ROMLOCATION_COUNT];
	const RomsetDefinition* pDefinition = MatchRomsetMK1(Files, pMatchedFiles);
	if (!pDefinition)
	{
		LogLine("error ROM files in %s do not form a complete MK1 romset", MinimalROMDirectory);
		return false;
	}
	m_pRomsetName = pDefinition->name;

	// Hand the backend the raw (still scrambled) ROM images; LoadRomset()
	// unscrambles the wave ROMs in place. The buffers are moved (not
	// copied) out of the scan results so the images are only held once.
	m_pRomsetInfo = new RomsetInfo;
	for (size_t i = 0; i < ROMLOCATION_COUNT; ++i)
	{
		if (pMatchedFiles[i])
			m_pRomsetInfo->rom_data[i] = std::move(pMatchedFiles[i]->Data);
	}
	Files.clear();
	Files.shrink_to_fit();

	RomLoadStatusSet LoadStatus;
	if (!LoadRomset(*m_pRomsetInfo, &LoadStatus))
	{
		LogLine("error LoadRomset failed for romset %s", pDefinition->name);
		return false;
	}

	m_pEmulator = new Emulator;

	EMU_Options Options;
	Options.lcd_backend = nullptr;
	if (!m_pEmulator->Init(Options))
	{
		LogLine("error Emulator init failed");
		return false;
	}

	// Oversampling OFF, so the native rate is 32 kHz
	// (PCM_GetOutputFrequency: 64000 / 2 for MK1) and the port's
	// streaming resampler carries it to the 48 kHz I2S device (ratio
	// set below, once the native rate is known). The resampler's
	// wall time is measured per chunk (resample_us in the windows),
	// so the audio path's load stays visible.
	m_pEmulator->GetPCM().enable_oversampling = false;

	if (!m_pEmulator->LoadRoms(pDefinition->romset, *m_pRomsetInfo))
	{
		LogLine("error LoadRoms failed for romset %s", pDefinition->name);
		return false;
	}

	m_nNativeRate = PCM_GetOutputFrequency(m_pEmulator->GetPCM());
	m_dResampleRatio = static_cast<double>(m_nNativeRate) / static_cast<double>(OutRate);
	m_dReadFrac = 0.0;
	m_bWindowValid = false;

	// Sound device (I2S, 48 kHz, 256-frame DMA chunk words -- mt32-pi's
	// AudioChunkSize -- queue 2048 frames, Signed24_32, exactly like
	// mt32-pi's I2S path, minus the Pisound slave branch: this kernel
	// never probes for a Pisound, so bSlave is always false and the
	// kernel's I2C master is always passed for DAC probing (address 0
	// = auto-probe 0x4C/0x4D; no DAC found means PCM5102A, no
	// controller needed -- same as mt32-pi). Start() itself runs in
	// AudioLoop after the device queue is pre-filled (Start with an
	// empty queue fails); its DAC probe then shares the bus with
	// core 1's MiSTer polling, serialized by Circle's I2C spinlock.
	m_pSound = new CI2SSoundBaseDevice(m_pInterrupt, OutRate, OutChunkFrames, false, m_pI2CMaster);
	m_pSound->SetWriteFormat(TSoundFormat::SoundFormatSigned24_32, 2);
	if (!m_pSound->AllocateQueueFrames(DeviceQueueFrames))
	{
		LogLine("error AllocateQueueFrames failed");
		return false;
	}

	m_pEmulator->SetSampleCallback([](void* pUserData, const AudioFrame<int32_t>& Frame) {
		static_cast<CMinimalApp*>(pUserData)->OnSample(Frame.left, Frame.right);
	}, this);

	// Decoder2 only in this kernel (the port's default H8 engine; the
	// JIT and legacy paths are not part of the validity question).
	m_pEmulator->SetH8Engine(H8Engine::Decoder2);

	// No priming burst: like the harness song start, Reset -> GS_RESET
	// -> split activation with MCU cycles at 0, so the song tables
	// (cycles absolute from 0) post exactly. LoadRoms() copies every
	// ROM into the backend's internal buffers, so the staging images
	// are released here, as the port does.
	m_pEmulator->Reset();
	m_pRomsetInfo->PurgeRomData();
	m_pEmulator->PostSystemReset(EMU_SystemReset::GS_RESET);

	// Round 3: lazy-timer self-test (single-threaded, before any core
	// starts). Plain run must report 0 mismatches; the fault-injected
	// run (lazy jump skips one tick past an event = the round-1
	// failure class) MUST report > 0, or the detector itself is
	// broken. Both numbers go to the boot line. A FAIL does not stop
	// the sequence (the runs still measure), but the lazy-timer runs
	// are then not trustworthy and tchk must be read with that in mind.
	m_nTimerSelfTestPlain = TIMER_SelfTest(true, 400000, false);
	m_nTimerSelfTestInject = TIMER_SelfTest(true, 400000, true);
	m_bTimerSelfTestPass = (m_nTimerSelfTestPlain == 0 && m_nTimerSelfTestInject > 0);
	LogLine("timer_selftest result=%s plain_mism=%llu inject_mism=%llu steps=400000",
	        m_bTimerSelfTestPass ? "pass" : "FAIL",
	        static_cast<unsigned long long>(m_nTimerSelfTestPlain),
	        static_cast<unsigned long long>(m_nTimerSelfTestInject));

	// Split activation (MK1 only -- enforced by the romset match above).
	// Run 0's sync profile + timer mode are installed inside.
	InitSplitState(false);

	// Song 0 (E1M1) starts automatically at boot with variant 0
	// (baseline). NO keys: the auto-sequence advances variants on
	// song_end (DrainMailboxes) and stops after variant 5.
	SetSongCursor(0);
	m_nCurrentSong = 0;

	// No boot line here: it carries pmc2/pmc3, which are only known
	// once cores 2/3 have validated their PMU counters, so core 2
	// logs boot + legends at AudioLoop start (after its PMU setup
	// and its wait for core 3's validation flag).

	// Start the other cores only now that everything is live: core 2's
	// audio loop pre-fills from an already-active split, core 3's PCM
	// loop finds work immediately, core 1 polls the MiSTer link.
	// (This ordering also avoids the port's startup races: no core
	// touches the emulator before activation.)
	if (!CMultiCoreSupport::Initialize())
	{
		LogLine("error multicore Initialize failed");
		return false;
	}

	return true;
}
