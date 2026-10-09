// ============================================================================
//  ChronoStateRing.h — lock-free handoff from the 1 kHz physics thread to the
//  render thread.
//
//  Deliberately free of Unreal types so it can be unit-tested standalone.
//
//  WHY NOT A MUTEX:
//  A mutex is not slow in the average case -- an uncontended pthread mutex is
//  tens of nanoseconds. The problem is the TAIL. If the render thread holds the
//  lock when the physics thread wants it, the physics thread blocks, and a
//  blocked 1 kHz thread has already missed its deadline. Worse, if the holder is
//  descheduled while holding it, the waiter inherits that entire scheduling
//  quantum -- milliseconds. The physics thread must never wait on a thread that
//  can be preempted by the compositor.
//
//  WHY A SEQLOCK:
//  The writer NEVER blocks and never waits. It stamps an odd sequence number,
//  writes, then stamps even. Readers retry if they observe an odd number or a
//  change across the read. All the cost of contention is pushed onto the reader,
//  which is exactly right here: the render thread has 16.6 ms of slack and the
//  physics thread has 1.0 ms.
//
//  WHY A RING AND NOT A SINGLE SLOT:
//  Interpolation needs the two snapshots that BRACKET the render timestamp, so
//  at least the recent history must stay addressable. See the interpolation
//  note in ChronoPhysicsThread.h.
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "chrono_shim.h"

/** Power-of-two so the modulo is a mask. */
template <uint32_t N>
class TChronoStateRing
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

public:
    struct FSlot
    {
        // Even = stable and readable. Odd = a write is in progress.
        // Starts at 0 but Head starts at 0 too, so a reader must always check
        // Head before trusting slot 0.
        std::atomic<uint64_t> Seq{0};
        ChsSnapshot           Data{};
    };

    /** Producer side. Called ONLY from the physics thread. Never blocks. */
    void Publish(const ChsSnapshot& In)
    {
        const uint64_t Index = Head.load(std::memory_order_relaxed);
        FSlot& Slot = Slots[Index & (N - 1)];

        // Odd: readers observing this will retry rather than tear.
        const uint64_t S = Slot.Seq.load(std::memory_order_relaxed);
        Slot.Seq.store(S + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);

        std::memcpy(&Slot.Data, &In, sizeof(ChsSnapshot));

        // Even again: publish. The release fence guarantees the payload write
        // is visible to any thread that later observes this sequence value.
        std::atomic_thread_fence(std::memory_order_release);
        Slot.Seq.store(S + 2, std::memory_order_relaxed);

        // Head last, with release ordering: a reader that sees the new Head is
        // guaranteed to see the slot contents that go with it.
        Head.store(Index + 1, std::memory_order_release);
    }

    /**
     * Consumer side. Safe from any thread.
     * @return true if a coherent copy was obtained.
     *
     * BackFromHead: 0 = newest, 1 = one older, ...
     */
    bool Read(ChsSnapshot& Out, uint32_t BackFromHead = 0) const
    {
        const uint64_t H = Head.load(std::memory_order_acquire);
        if (H == 0 || BackFromHead >= N || BackFromHead >= H)
        {
            return false;   // nothing published yet, or asking beyond history
        }

        const uint64_t Index = H - 1 - BackFromHead;
        const FSlot& Slot = Slots[Index & (N - 1)];

        // Bounded retry. Unbounded spinning here could stall the game thread if
        // the physics thread is producing faster than we can copy; failing and
        // reusing the previous frame's data is strictly better than a hitch.
        for (int Attempt = 0; Attempt < 8; ++Attempt)
        {
            const uint64_t S1 = Slot.Seq.load(std::memory_order_relaxed);
            if (S1 & 1u)
            {
                continue;               // writer is mid-update
            }
            std::atomic_thread_fence(std::memory_order_acquire);

            std::memcpy(&Out, &Slot.Data, sizeof(ChsSnapshot));

            std::atomic_thread_fence(std::memory_order_acquire);
            const uint64_t S2 = Slot.Seq.load(std::memory_order_relaxed);

            if (S1 == S2)
            {
                return true;            // no write intervened: copy is coherent
            }
            // Otherwise the writer lapped us mid-copy; the bytes in Out may be
            // a mix of two snapshots. Discard and retry.
        }
        return false;
    }

    /**
     * Consumer side, by ABSOLUTE index: the Index-th snapshot ever published,
     * 0-based (Count() is one past the newest). For a consumer that must see
     * EVERY sample, in order -- the referee.
     *
     * Read(back) counts back from the head AS IT IS AT THAT CALL, so a loop of
     * Read(back) that the producer publishes into mid-loop shifts by one and
     * skips a sample. Measured (task 5): 135 one-sample gaps in two minutes of
     * editor play, with frames at 28 ms against 256 ms of history -- the
     * drain never fell behind; its indexing moved under it.
     *
     * Returns false if the sample is not published yet, or is gone or going:
     * the OLDEST slot is refused as well, because between the producer
     * finishing a slot and publishing the new head, that slot holds sample
     * Index + N with a stable sequence and a head that does not yet show it.
     */
    bool ReadAt(uint64_t Index, ChsSnapshot& Out) const
    {
        uint64_t H = Head.load(std::memory_order_acquire);
        if (Index >= H || H - Index >= N)
        {
            return false;
        }
        const FSlot& Slot = Slots[Index & (N - 1)];
        for (int Attempt = 0; Attempt < 8; ++Attempt)
        {
            const uint64_t S1 = Slot.Seq.load(std::memory_order_relaxed);
            if (S1 & 1u)
            {
                continue;
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            std::memcpy(&Out, &Slot.Data, sizeof(ChsSnapshot));
            std::atomic_thread_fence(std::memory_order_acquire);
            if (Slot.Seq.load(std::memory_order_relaxed) != S1)
            {
                continue;
            }
            // Coherent -- but still sample Index? Only if the producer has not
            // lapped the ring since the first check.
            H = Head.load(std::memory_order_acquire);
            return H - Index < N;
        }
        return false;
    }

    /** Total snapshots ever published. */
    uint64_t Count() const { return Head.load(std::memory_order_acquire); }

private:
    // Head is written by the physics thread and read by the game thread every
    // frame. Give it its own cache line: sharing one with Slots[0] would make
    // every publish invalidate the reader's copy of the slot it is reading.
    alignas(64) std::atomic<uint64_t> Head{0};
    alignas(64) FSlot Slots[N];
};
