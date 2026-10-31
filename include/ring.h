//
// ring.h -- skeletal lock-free sample ring (Sterr1/Mini-JV880pi shape)
//
// Replaces the old locked MinimalRing (CSpinLock at IRQ_LEVEL,
// modulo indices, one slot kept empty, drop-NEW on full). The new
// discipline mirrors Sterr1's MCU_PostSample + core-2 drain exactly:
// monotonic uint64 write/read indices, modulo ONLY for slot
// indexing, drop-OLDEST on overflow with a counter, RELEASE publish
// on the producer side and ACQUIRE consume on the consumer side.
// No lock anywhere in the sample path (the old ring's lock was one
// more wfe sleeper in the hot path; the JV880 shape removes it).
//
// Producer (core 3, sample callback inside PCM_Update): load
// write_idx RELAXED (own index), write the slot, load read_idx
// ACQUIRE, if the count would exceed N advance read_idx to
// (write+1-N) with a RELEASE store (dropping the oldest frames)
// and count the drop, then publish write_idx+1 with a RELEASE
// store. Consumer (core 2, demand loop): load write_idx ACQUIRE,
// read_idx RELAXED, copy while items remain, publish read_idx
// with a RELEASE store. Avail() is the same computation, for the
// step loop's fill condition. Clear() resets both indices to 0
// and is called ONLY when the PCM task is parked and the MCU is
// not stepping (song reset path), so plain stores are safe there
// (the app's DMB publication of the reactivated state orders them).
//
// Known property, inherited from Sterr1 (not introduced here):
// when the ring is full, a concurrent consumer dequeue can make
// the producer's drop store move read_idx backward by a few slots,
// re-delivering already-consumed frames. That only fires when the
// ring is full, and the pass bar requires ring_drops == 0 (never
// full), so it cannot affect a clean run. Duplicates, never
// corruption, and never a wedge.
//
// Single producer / single consumer in this kernel (core 3 posts,
// core 2 drains); the __atomic discipline is what makes that safe
// without a lock.
//

#ifndef _minimal_ring_h
#define _minimal_ring_h

#include <cstdint>
#include <circle/types.h>

template <class T, size_t N>
class SkelRing
{
public:
	SkelRing()
		: m_nWriteIdx(0),
		  m_nReadIdx(0),
		  m_Data{}
	{
		static_assert((N & (N - 1)) == 0, "ring size must be a power of 2");
	}

	// Producer only (core 3). Always enqueues; returns true if it
	// had to drop the oldest frame(s) to make room (caller counts
	// this; must stay 0 for a clean run).
	bool Enqueue(const T& Item)
	{
		uint64_t nCur = __atomic_load_n(&m_nWriteIdx, __ATOMIC_RELAXED);
		m_Data[nCur & BufferMask] = Item;
		uint64_t nNext = nCur + 1;
		bool bDropped = false;
		uint64_t nRead = __atomic_load_n(&m_nReadIdx, __ATOMIC_ACQUIRE);
		if ((nNext - nRead) > N)
		{
			// Full: advance the read index so exactly N frames
			// remain (drop-oldest). See the header comment for
			// the concurrent-dequeue property of this store.
			__atomic_store_n(&m_nReadIdx, nNext - N, __ATOMIC_RELEASE);
			bDropped = true;
		}
		__atomic_store_n(&m_nWriteIdx, nNext, __ATOMIC_RELEASE);
		return bDropped;
	}

	// Consumer only (core 2). Returns false when empty.
	bool Dequeue(T& OutItem)
	{
		uint64_t nWrite = __atomic_load_n(&m_nWriteIdx, __ATOMIC_ACQUIRE);
		uint64_t nRead = __atomic_load_n(&m_nReadIdx, __ATOMIC_RELAXED);
		if (nWrite == nRead)
			return false;
		OutItem = m_Data[nRead & BufferMask];
		__atomic_store_n(&m_nReadIdx, nRead + 1, __ATOMIC_RELEASE);
		return true;
	}

	// Consumer only (core 2, step-loop fill condition): frames
	// currently queued. Same computation as Sterr1's drain.
	size_t Avail() const
	{
		uint64_t nWrite = __atomic_load_n(&m_nWriteIdx, __ATOMIC_ACQUIRE);
		uint64_t nRead = __atomic_load_n(&m_nReadIdx, __ATOMIC_RELAXED);
		return static_cast<size_t>(nWrite - nRead);
	}

	// Reset path only (PCM parked, MCU not stepping): drop all
	// queued frames. Plain stores are safe here by the caller's
	// guarantee; the app's DMB publication orders them.
	void Clear()
	{
		m_nWriteIdx = 0;
		m_nReadIdx = 0;
	}

private:
	static constexpr size_t BufferMask = N - 1;

	uint64_t m_nWriteIdx;
	uint64_t m_nReadIdx;
	T m_Data[N];
};

// MinimalRing -- the OLD shaped kernel's locked mailbox ring, kept
// verbatim for the key/MiSTer event mailboxes ONLY (the audio ring
// above stays SkelRing; this class does not touch the sample path).
// Semantics mirror the lock-protected ring the mt32-pi port uses for
// its sample ring (CSpinLock at IRQ_LEVEL, volatile indices, one
// slot kept empty to distinguish full from empty, explicit
// assignment -- never ++/&= -- on volatile indices, which C++20
// deprecates under -Werror). Single producer (core 0 keys / core 1
// MiSTer) / single consumer (core 2 epilogue) per mailbox, but the
// lock makes it safe regardless. Enqueue returns FALSE when full
// (drop-NEW + caller counts it) -- the opposite convention from
// SkelRing's drop-oldest, which is why the mailboxes use this class:
// losing the OLDEST key/MiSTer event to a full mailbox would be
// worse than dropping the newest press.

#include <circle/spinlock.h>

template <class T, size_t N>
class MinimalRing
{
public:
	MinimalRing()
		: m_Lock(IRQ_LEVEL),
		  m_nInPtr(0),
		  m_nOutPtr(0),
		  m_Data{}
	{
		static_assert((N & (N - 1)) == 0, "ring size must be a power of 2");
	}

	bool Enqueue(const T& Item)
	{
		bool bSuccess;
		m_Lock.Acquire();
		bSuccess = EnqueueOne(Item);
		m_Lock.Release();
		return bSuccess;
	}

	bool Dequeue(T& OutItem)
	{
		bool bSuccess = false;
		m_Lock.Acquire();
		if (m_nInPtr != m_nOutPtr)
		{
			OutItem = m_Data[m_nOutPtr];
			m_nOutPtr = m_nOutPtr + 1;
			m_nOutPtr = m_nOutPtr & BufferMask;
			bSuccess = true;
		}
		m_Lock.Release();
		return bSuccess;
	}

private:
	static constexpr size_t BufferMask = N - 1;

	bool EnqueueOne(const T& Item)
	{
		const size_t nNext = (m_nInPtr + 1) & BufferMask;
		if (nNext == m_nOutPtr)
			return false;	// full
		m_Data[m_nInPtr] = Item;
		m_nInPtr = nNext;
		return true;
	}

	CSpinLock m_Lock;
	volatile size_t m_nInPtr;
	volatile size_t m_nOutPtr;
	T m_Data[N];
};

#endif
