// ============================================================================
//  ring/test_ring.cpp -- the physics -> game snapshot ring, under a real race.
//
//  The plugin's CANONICAL ChronoStateRing.h, compiled on the host against the
//  shim's plain-C header: no Unreal, no Chrono. A producer thread publishes
//  numbered snapshots whose every field encodes their number; a consumer
//  drains them the way UChronoWorldSubsystem does.
//
//  Task 5 found the drain's Read(back) loop skipping one sample whenever the
//  producer published mid-loop (Read(back) counts from the head AS IT IS at
//  each call). ReadAt(absolute index) replaced it. Asserted here:
//    1. keeping up, ReadAt skips NOTHING -- and the old loop does skip;
//    2. falling far behind, ReadAt never returns a sample under the wrong
//       index (it refuses overwritten ones), and
//    3. no copy is ever torn.
// ============================================================================
#include "ChronoStateRing.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

using Ring = TChronoStateRing<256>;

static void make(ChsSnapshot& s, uint64_t i)
{
    std::memset(&s, 0, sizeof s);
    s.step_index = i;
    s.sim_time = double(i);
    s.body_count = 16;
    for (int b = 0; b < 16; ++b) {
        s.bodies[b].position = ChsVec3{ double(i), double(i) + b, -double(i) };
        s.bodies[b].linear_velocity = ChsVec3{ double(i), 0.5 * double(i), 1.0 };
    }
}

static bool coherent(const ChsSnapshot& s)
{
    const double i = double(s.step_index);
    if (s.sim_time != i || s.body_count != 16) return false;
    for (int b = 0; b < 16; ++b)
        if (s.bodies[b].position.x != i || s.bodies[b].position.y != i + b ||
            s.bodies[b].position.z != -i || s.bodies[b].linear_velocity.y != 0.5 * i)
            return false;
    return true;
}

struct Result { uint64_t read = 0, skipped = 0, wrong = 0, torn = 0; };

// period_us: producer pacing; pause_us: consumer sleep between drains;
// absolute: ReadAt (the fix) or the old Read(back) loop.
static Result run(int period_us, int pause_us, bool absolute, int seconds)
{
    static Ring ring;
    ring.~Ring(); new (&ring) Ring();
    std::atomic<bool> stop{false};
    std::thread producer([&] {
        ChsSnapshot s;
        uint64_t i = 0;
        auto next = std::chrono::steady_clock::now();
        while (!stop.load(std::memory_order_relaxed)) {
            make(s, i++);
            ring.Publish(s);
            next += std::chrono::microseconds(period_us);
            while (std::chrono::steady_clock::now() < next) {}
        }
    });

    Result r;
    uint64_t next_index = 0, last_step = 0;
    bool primed = false;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (std::chrono::steady_clock::now() < end) {
        ChsSnapshot s;
        if (absolute) {
            const uint64_t published = ring.Count();
            uint64_t first = next_index;
            if (published > first + 255) first = published - 255;
            next_index = published;
            for (uint64_t i = first; i < published; ++i) {
                if (!ring.ReadAt(i, s)) continue;
                if (!coherent(s)) { ++r.torn; continue; }
                if (s.step_index != i) { ++r.wrong; continue; }
                if (primed && s.step_index > last_step + 1) r.skipped += s.step_index - last_step - 1;
                if (!primed || s.step_index > last_step) { last_step = s.step_index; primed = true; ++r.read; }
            }
        } else {                                          /* the old drain */
            ChsSnapshot head;
            if (ring.Read(head, 0)) {
                const uint64_t unjudged = head.step_index > last_step ? head.step_index - last_step : 0;
                const int oldest = int(unjudged < 256 ? unjudged : 256) - 1;
                for (int back = oldest; back >= 0; --back) {
                    if (!ring.Read(s, uint32_t(back))) continue;
                    if (!coherent(s)) { ++r.torn; continue; }
                    if (primed && s.step_index <= last_step) continue;
                    if (primed && s.step_index > last_step + 1) r.skipped += s.step_index - last_step - 1;
                    last_step = s.step_index; primed = true; ++r.read;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(pause_us));
    }
    stop.store(true);
    producer.join();
    return r;
}

int main()
{
    int fails = 0;
    auto check = [&](bool c, const char* what) {
        std::printf("  %s %s\n", c ? "ok  " : "FAIL", what);
        if (!c) ++fails;
    };
    std::printf("snapshot ring, 256 slots, %zu-byte snapshots\n\n", sizeof(ChsSnapshot));

    /* Keeping up, and racing hard: a snapshot every 20 us, drained every 2 ms
     * (~100 new per drain, well inside the 255 readable), so the producer
     * publishes during most drains. */
    const Result a = run(20, 2000, true, 3);
    const Result o = run(20, 2000, false, 3);
    std::printf("  keeping up   ReadAt:     %llu read, %llu skipped, %llu wrong, %llu torn\n",
                (unsigned long long)a.read, (unsigned long long)a.skipped,
                (unsigned long long)a.wrong, (unsigned long long)a.torn);
    std::printf("  keeping up   Read(back): %llu read, %llu skipped, %llu torn  (the old drain)\n",
                (unsigned long long)o.read, (unsigned long long)o.skipped,
                (unsigned long long)o.torn);
    check(a.skipped == 0, "ReadAt skips nothing while the consumer keeps up");
    check(o.skipped > 0, "the old Read(back) drain does skip (the bug reproduces)");

    /* Falling behind: drained every 20 ms against 5 ms of history. ReadAt
     * must refuse what is gone, never return it under the wrong index. */
    const Result b = run(20, 20000, true, 3);
    std::printf("  falling behind ReadAt:   %llu read, %llu skipped, %llu wrong, %llu torn\n",
                (unsigned long long)b.read, (unsigned long long)b.skipped,
                (unsigned long long)b.wrong, (unsigned long long)b.torn);
    check(b.skipped > 0, "falling behind loses samples (and says so by the gap)");
    check(a.wrong == 0 && b.wrong == 0, "never a sample under the wrong index");
    check(a.torn == 0 && b.torn == 0 && o.torn == 0, "never a torn copy");

    std::printf("\n%s\n", fails ? "RING TEST FAILED" : "RING TEST PASSED");
    return fails ? 1 : 0;
}
