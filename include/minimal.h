//
// minimal.h -- mt32-pi-shaped-skel kernel app
//
// What this kernel is: the PROVEN skeletal JV880-shape sync (togglefix
// baseline: shared pcm_t, core 2 RELEASE-publishes mcu.cycles after
// every StepMCU, core 3 ACQUIRE-chases in batches of at most 128
// ticks capped at published-800 under the state lock, no FIFO, no
// RPC, unconditional PCMLoop so the parked branch can ack idle)
// inside a kernel with mt32-pi's outward shape, ported from the old
// shaped kernel: core 0 is IDLE (no hardware keys: the variant
// auto-sequence needs no input), core 1 polls the MiSTer I2C link
// (10 ms, slave 0x45, variant status writeback {0xA2, 0xFF, variant}
// + best-effort SSD1306 "VAR n name" at 0x3C), core 2 runs the
// device-paced demand loop (MCU stepping + song event posting +
// streaming resample 32 kHz -> 48 kHz + I2S Writes + stats file +
// the variant auto-sequence), core 3 runs the PCM task. E1M1 ONLY
// (the BELMO tables are retained but unreachable: no selection path
// exists in this kernel) and the MK1-only ROM loading are unchanged
// from togglefix.
//
// What it is NOT: no mt32-pi code, no UI, no physical LCD, no USB,
// no network, no LCD mirror, no OSD text (the 3-byte status protocol
// has no text field; the MiSTer's own pop-ups on status change are
// unchanged). The stats file (shaped-var2-stats.log) is the
// observation channel for EVERY incoming control event, logged with
// name + parameters + timestamp; the MIDI stream itself is NEVER
// logged (hundreds of events/s would flood).
//
// SHAPED-VAR (this kernel): the shaped-diag base + the variant
// auto-sequence + PMU cycle tooling. NO HARDWARE KEYS (
// 2026-10-31 ~09:23): boot auto-starts E1M1 with variant 0
// (baseline); each E1M1 song_end advances to the next variant
// (1 sched, 2 pipe, 3 neon2) with an emulator reset and E1M1 from
// the beginning, via a SEPARATE variant-advance path (NOT the
// deleted song-toggle DoSwitch); after variant 5's song_end the
// kernel STOPS into post-song_end idle (silence, clean idle windows,
// LED heartbeat unchanged, never restart, never loop). Audio silence
// + the sequence_done line are the done signals; LED-off is NEVER a
// done signal (frozen LED already means wedge/stall). Every win line
// carries var=N; every song_end line carries var=N; sequence_done
// carries per-variant totals (prod + pcm/mix/step/lock cycles).
// PMU (FFmpeg checkasm port): per-core PMCCNTR_EL0, enabled once per
// core at loop start (PMCR.E=1, PMCNTENSET bit 31=1, PMCCFILTR bits
// 30+28 cleared), never reset; boot-validated (pmc2/pmc3 ok|fail in
// the boot line); NOP-calibrated (double-read cost subtracted from
// every reported cycle value, clamped at 0). Core 2 reports step_cyc
// (MCU stepping only) + lock_cyc (state-lock spins only); core 3
// reports pcm_cyc (total PCM task work) + mix_cyc (voice-mix portion
// per batch). If a core's counter fails, its cycle fields report 0
// and the boot line is the validity signal -- never silent zeros.
//
// SHAPED-SKEL FIXES vs the old shaped kernel (see minimal.cpp):
// (1) The device-space wait is CORRECTED: GetQueueFramesAvail()
// returns QUEUED frames, not free space, so the old loop waited for
// the queue to FILL and then wrote into the nearly-full queue
// (chronic short Writes, got 1024/0). This kernel waits for FREE
// space (queue size - queued) and writes only when a full chunk
// fits, so short Writes are impossible by construction.
// (2) Every CScheduler::Yield() is replaced with MiniPause() (ARM
// yield hint): this kernel registers no CTasks, so Yield() can never
// return (scheduler.cpp: GetNextTask() always yields MAX_TASKS with
// m_nTasks == 0) -- the old shaped cores 0/1 were stuck in Yield
// from boot, so its buttons + MiSTer link could never run.
// (3) The devq window fields measure what their names + comments say:
// devq_maxavail = peak FREE space (near 2048 = device nearly
// starved); devq_low = chunks with fewer than one chunk QUEUED at
// Write time. The old shaped code measured both on Avail (queued),
// which inverted both fields.
//
// Core roles: core 0 = idle (no keys, no mailbox posts);
// core 1 = MiSTer poll (10 ms, init/down/change events) -> mailbox,
// plus the variant status writeback and LCD draw (see above);
// core 2 = device-paced demand loop (MCU stepping + song event
// posting + resample + I2S Write + stats file + ACT heartbeat +
// variant auto-sequence + PMU step/lock cycles);
// core 3 = PCM task (unconditional iteration) + PMU pcm/mix cycles.
// The backend classes are only forward-declared here; minimal.cpp
// includes the real headers and is compiled as C++20.
//

#ifndef _minimal_h
#define _minimal_h

#include <circle/multicore.h>
#include <circle/types.h>
#include <fatfs/ff.h>

#include "ring.h"

// Backend types, forward-declared (see minimal.cpp for the real headers).
class Emulator;
struct RomsetInfo;
struct ThreadSplitStateCircle;
struct SongEvent;
class CI2CMaster;
class CInterruptSystem;
class CI2SSoundBaseDevice;

class CMinimalApp : public CMultiCoreSupport
{
public:
	CMinimalApp(CI2CMaster* pI2CMaster, CInterruptSystem* pInterrupt);
	virtual ~CMinimalApp() override;

	bool Initialize();
	void Run(unsigned nCore) override;

private:
	struct TSample
	{
		float nLeft;
		float nRight;
	};
	static constexpr size_t RingFrames = 16384;
	// Native-rate top-up unit for the step loop (256 native frames
	// = 8 ms at 32 kHz). The pre-fill steps 4 of these before the
	// first resample, so the first output chunks never starve.
	static constexpr size_t ChunkFrames = 256;
	// Output: 48 kHz I2S, 256-frame Writes (the mt32-pi chunk),
	// device queue 2048 frames (~43 ms: absorbs epilogue SD-sync
	// stalls without masking a rate shortfall -- the window pct is
	// wall-clock based, so a slow engine still shows).
	static constexpr unsigned OutRate = 48000;
	static constexpr size_t OutChunkFrames = 256;
	static constexpr unsigned DeviceQueueFrames = 2048;
	// DMA restart guard: after DMA terminates (queue drained: pause
	// or starvation), CDMASoundBuffers needs one buffer period to
	// reach Idle before Start() is legal again (Start during
	// Terminating asserts). 20 ms is ~7 DMA buffer periods.
	static constexpr u64 SoundRestartWaitUs = 20000ULL;

	// Sample callback (core 3, inside PCM_Update): normalize exactly
	// like the mt32-pi port (full scale 2^29 -> 1.0f, minus the SC-55
	// DAC mid-scale DC bias) and enqueue into the ring. The device
	// loop drains the ring through the resampler; the callback's
	// cost stays in the measured load, so the split is tested under
	// the port's load.
	void OnSample(s32 nLeft, s32 nRight);

	// Split lifecycle (skeletal JV880 shape, UNCHANGED from
	// togglefix): while the split runs, core 2 steps the MCU and
	// publishes cycle counts, core 3 renders PCM batches up to the
	// published count under the state lock, and both share one
	// pcm_t (no FIFO, no RPC, no shadows). Core 0 never touches
	// the emulator at all (it spins inertly: no keys in this
	// kernel); core 1 never touches the emulator or the file, it
	// only posts MiSTer events and sends the variant status + LCD
	// draw. StopSplitPCM parks the PCM task and clears g_splitT;
	// InitSplitState (re)activates from a live pcm_t, optionally
	// clearing the ring (reset path).
	void StopSplitPCM();
	void InitSplitState(bool bClearRing);

	// Core loops.
	void IdleLoop();	// core 0: inert spin (NO keys in this kernel)
	void MisterLoop();	// core 1: MiSTer I2C poll (10 ms) -> mister mailbox + variant status/LCD
	void AudioLoop();	// core 2: device-paced demand + stepping + resample + stats + LED + auto-sequence
	void PCMLoop();		// core 3: PCM task iterations (UNCONDITIONAL, togglefix fix)

	// Core-2 helpers.
	void PostDueEvents();	// post song events due at the current MCU cycle (NEVER logged: MIDI stream)
	void StepUntilFill(size_t nNeed, u64* pStepUs);	// step MCU until the ring holds nNeed frames (or budget out)
	size_t ResampleChunk(float* pOut, size_t nFrames, size_t* pSilence);	// port resampler, verbatim logic
	void DoVariantAdvance();	// variant advance (emulator reset, E1M1 from the beginning, NO song change)
	void SetSongCursor(int nSong);	// point the cursor at a song table
	void ResetWindow();	// fresh window accumulators + counter snapshots
	void DrainMailboxes();	// core 2: song-end + mister events + drops (chunk boundary / pause loop)
	void DrawVariantLCD(int nVariant);	// core 1 only: best-effort SSD1306 "VAR n name" at 0x3C
	void LogLine(const char* pFormat, ...);	// core 2 only (also init, pre-multicore), f_write+f_sync, t= prefixed
	void WriteWindow(bool bPaused);	// core 2 only: one 5 s stats window line
	void WriteSongEnd();	// core 2 only: per-song summary (completed)
	void WriteSequenceDone();	// core 2 only: per-variant totals after variant 5's song_end

	// One PCM-task iteration (core 3, skeletal shape, UNCHANGED from
	// togglefix): ACQUIRE-load the published cycle count; if it is
	// more than one max tick (800 cycles) past the rendered
	// position, render one BOUNDED batch (at most 128 ticks, capped
	// at published-800 so no tick ever ends past published) under
	// the state lock, then publish the render position + lag + work
	// time. Yield-hint pause when there is no work. No-op when the
	// split is inactive (it parks the task and acks the stop
	// handshake instead).
	void PCMTaskIteration();

	Emulator* m_pEmulator;
	RomsetInfo* m_pRomsetInfo;
	const char* m_pRomsetName;	// matched romset name (Initialize) for the AudioLoop boot line
	::ThreadSplitStateCircle* m_pSplitState;

	// Split handshake flags (volatile, DMB-ordered at the handoff
	// points, exactly like togglefix): m_bSplitActive gates the PCM
	// task; m_bPCMIdle acks that the PCM task has parked.
	volatile bool m_bSplitActive;
	volatile bool m_bPCMIdle;

	SkelRing<TSample, RingFrames> m_Ring;
	volatile u64 m_nRingDrops;	// drop-oldest overflows (must stay 0; see inherited wart note)
	volatile u64 m_nWrittenFrames;	// total native frames produced

	// Song state. The tables are read-only shared data (songs.h);
	// the cursor lives on core 2 (the only thread that posts events).
	// E1M1 ONLY in this kernel: the cursor is always 0 (BELMO data
	// retained but unreachable -- no selection path exists).
	const SongEvent* m_pEvents;
	const u8* m_pBlob;
	size_t m_nEventCount;
	u64 m_nEndCycle;
	size_t m_nNextEvent;
	int m_nCurrentSong;		// core 2 only (always 0: E1M1)
	bool m_bSongDone;		// core 2 only
	bool m_bPaused;			// core 2 only: UNREACHABLE without keys (kept for structure; always false)

	// Variant auto-sequence (2026-10-31 ~09:23/09:29). Core 2
	// owns m_nCurrentVariant; cores 1/3 read it ONLY via __atomic
	// builtins (plain field, never plain-read -- same discipline as
	// S.cycles_pub). The variant changes ONLY while the PCM task is
	// parked (inside DoVariantAdvance), so no batch can observe a
	// mid-batch change. Per-variant totals accumulate in WriteWindow
	// into the CURRENT variant's slot; after sequence_done they stop
	// (idle windows still carry var=3 but do not add to the totals,
	// so the sequence_done line stays the authoritative totals).
	// Dispatch is DATA-DRIVEN (2026-10-31 ~09:39): the variant
	// table lives in pcm.cpp (g_pcmVariants: number, short name,
	// full name, function pointer) and minimal.cpp reads names +
	// count from it -- adding a variant later is one table entry +
	// its PCM_Update_* function, not a rewrite. The accumulator
	// arrays are sized MaxVariants (headroom for the pending
	// variants 4/5); sequence_done prints the CURRENT variants
	// explicitly (0-3), and a follow-up adding variants extends
	// those fields mechanically.
	int m_nCurrentVariant;		// 0 baseline, 1 sched, 2 pipe, 3 neon2
	bool m_bSequenceDone;		// core 2 only: last variant's song_end logged, now idle forever
	u64 m_nVarProd[8];		// per-variant output frames (song windows only)
	u64 m_nVarPcmCyc[8];		// per-variant pcm_cyc sums
	u64 m_nVarMixCyc[8];		// per-variant mix_cyc sums
	u64 m_nVarStepCyc[8];		// per-variant step_cyc sums
	u64 m_nVarLockCyc[8];		// per-variant lock_cyc sums

	// PMU handshake (core 3 sets, core 2 reads, DMB-ordered like the
	// split handshake): m_bPmu3Done = core 3 finished its PMU
	// validation; m_bPmu3Ok = its counter counts. m_bPmu2Ok is core
	// 2 only. If a core's counter fails, its cycle fields report 0
	// and the boot line (pmc2/pmc3) is the validity signal.
	volatile bool m_bPmu3Done;
	volatile bool m_bPmu3Ok;
	bool m_bPmu2Ok;

	// MiSTer event mailbox (single producer core 1, single consumer
	// core 2; the MinimalRing lock makes it safe). Core 1 posts link
	// transitions and received-status changes; core 2 drains at chunk
	// boundaries (playing) or every 1 ms (paused) and logs every
	// event + the action taken. The MIDI stream is NEVER logged.
	// our_variant (init/writefail events only) is OUR status byte at
	// write time, so the mister_init log line reports the exact
	// bytes written (other events leave it 0: it is never logged).
	struct TMisterEvent
	{
		u8 type;	// 0=down, 1=init, 2=synth, 3=romset, 4=sfindex, 5=mute, 6=writefail
		u8 synth;	// received status bytes for this event
		u8 romset;
		u8 sf;
		u8 our_variant = 0;	// our status variant byte at write time (init/writefail only)
	};
	MinimalRing<TMisterEvent, 16> m_MisterRing;
	volatile u64 m_nMisterDrops;
	u64 m_nMisterDropsLogged;	// core 2 only
	u64 m_nMisterUps;			// core 2 only
	u64 m_nMisterDowns;			// core 2 only
	u64 m_nCycles;				// core 2 only: song cycles (always 0: no song cycling exists here; kept so the song_end format is unchanged)

	CI2CMaster* m_pI2CMaster;		// core 1 (MiSTer poll); also the sound device's DAC-probe master
	CInterruptSystem* m_pInterrupt;		// sound device construction
	CI2SSoundBaseDevice* m_pSound;		// core 2 only (created in Initialize)

	// Streaming resampler state (the port's CNukedSC55Synth resampler,
	// same window/frac logic, scale 1.0 here: gain 1.0, volume 100 --
	// no master-volume scaling in this kernel).
	double m_dResampleRatio;	// native frames consumed per output frame (32000/48000)
	double m_dReadFrac;		// fractional carry of the streaming resampler
	TSample m_SampleA;		// window bracketing the next output position
	TSample m_SampleB;
	bool m_bWindowValid;

	// Audio device state (core 2 only).
	bool m_bSoundStarted;		// Start() has succeeded at least once
	bool m_bSoundRestartPending;	// DMA dead: waiting out the Terminating -> Idle window
	bool m_bSoundStartFailed;	// Start() failed: never retry (Failed state asserts on Start)
	u64 m_nSoundRestartWaitStartUs;
	u64 m_nSoundRestarts;		// successful restarts (total since boot)
	u64 m_nSoundDrops;		// short Writes (total since boot; 0 by construction, see AudioLoop)

	// Native rate (core 2 only).
	unsigned m_nNativeRate;

	// Stats (core 2 only, except the split-state counters, which
	// core 3 / the backend update and core 2 snapshots per window).
	FIL m_StatsFile;
	bool m_bStatsOpen;
	u64 m_nWinStartUs;
	unsigned m_nWinIndex;
	u64 m_nWinProd;		// output frames written (rendered, non-silence)
	u64 m_nWinShort;		// output frames silence-filled
	u64 m_nWinUnderruns;	// chunks with any silence fill
	u64 m_nWinCore2Us;	// step-loop wall time
	u64 m_nWinResampleUs;	// resampler wall time
	u64 m_nWinChunks;	// output chunks written
	u64 m_nWinDevQMaxAvail;	// peak device FREE space (frames) -- near queue size = near starvation
	u64 m_nWinDevQLow;	// chunks where <1 chunk remained QUEUED at Write time
	u64 m_nWinSoundRestarts;
	u64 m_nWinSoundDrops;
	// ACT heartbeat (core 2 only): good-chunk counter + LED state.
	// Same semantics as togglefix: blink ~1 Hz while the window is
	// clean; hold Off for a bad window; frozen during pause (no
	// chunks flow) or a device-space stall.
	u64 m_nLEDChunks;
	bool m_bLEDOn;
	// Last-window snapshot of the PCM task's work time (the only
	// split-state counter left; tick/lookahead/RPC machinery was
	// removed with the FIFO/RPC split, so those window fields
	// report 0).
	u64 m_nLastPCMUs;
	u64 m_nLastLockUs;	// DIAG: last-window snapshot of S.lock_wait_us
	u64 m_nLastLockWaits;	// DIAG: last-window snapshot of S.lock_waits
	// PMU snapshots (same delta discipline): core-3 pcm/mix cycles
	// and core-2 lock cycles, from S.pcm_task_cyc / S.mix_cyc /
	// S.lock_wait_cyc. Step-wall cycles + call count are core-2
	// window accumulators (no S field needed: StepUntilFill runs on
	// core 2, which also writes the windows).
	u64 m_nLastPCMCyc;
	u64 m_nLastMixCyc;
	u64 m_nLastLockCyc;
	u64 m_nWinStepWallCyc;	// raw StepUntilFill wall cycles in this window
	u64 m_nWinStepCalls;	// StepUntilFill calls in this window
	// Per-song totals in OUTPUT frames (reset at each cycle).
	u64 m_nSongDue;
	u64 m_nSongProd;
	u64 m_nSongShort;
	u64 m_nSongUnderruns;
	u64 m_nCompletions;
	// Deferred log lines (core 2 only). SD-sync cost must never delay
	// stepping, so the song-end line is queued here by the step loop
	// and written in the chunk epilogue, after that chunk's Write,
	// so its totals are complete. Everything else is written
	// directly in the epilogue context that creates it --
	// DrainMailboxes and DoVariantAdvance run between chunks, never
	// mid-step, so their SD-sync cost never delays stepping. Rare
	// device lines (sound_drop, sound_restart) are written directly
	// at the Write site: they must be 0 in a good run, and a
	// drop/restart IS the news.
	bool m_bSongEndPending;
};

#endif
