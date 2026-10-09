// ===========================================================================
//  Q1: what one steady-state tyre-table lookup costs at 1 kHz.
//
//  A 5-D table -- slip angle x slip ratio x load x camber x speed -- holding
//  six outputs per node (Fx Fy Fz Mx My Mz), read by 5-D multilinear
//  interpolation: 2^5 = 32 corner nodes per lookup. The table is filled with
//  arbitrary smooth values: the CONTENT does not change the cost, the size
//  and the access pattern do.
//
//  Two access patterns, four tyres a step:
//    smooth  inputs drift the way a car's do between 1 ms steps, so the same
//            few cache lines are reused (the realistic case)
//    random  every lookup lands somewhere new (the worst case: every corner
//            a cache or TLB miss)
//
//  Host-native, no Chrono:  gcc -O2 -march=skylake bench_tyre_table.c -lm
//                           ./a.out [n_alpha n_kappa n_fz n_gamma n_v]
// ===========================================================================
#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

enum { D = 5, OUT = 6 };

typedef struct {
    int n[D];            // nodes per axis
    double lo[D], hi[D]; // axis range
    long stride[D];      // in nodes
    float* v;            // n0*n1*...*n4 nodes, OUT floats each, interleaved
} Table;

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + 1e-9 * ts.tv_nsec;
}

// 5-D multilinear interpolation, clamped at the table edge.
static void lookup(const Table* t, const double x[D], float out[OUT]) {
    long base = 0;
    double f[D];
    for (int d = 0; d < D; ++d) {
        double u = (x[d] - t->lo[d]) / (t->hi[d] - t->lo[d]) * (t->n[d] - 1);
        if (u < 0) u = 0;
        if (u > t->n[d] - 1.000001) u = t->n[d] - 1.000001;
        const int i = (int)u;
        f[d] = u - i;
        base += i * t->stride[d];
    }
    float acc[OUT] = {0};
    for (int c = 0; c < (1 << D); ++c) {
        double w = 1.0;
        long off = base;
        for (int d = 0; d < D; ++d) {
            const int bit = (c >> d) & 1;
            w *= bit ? f[d] : 1.0 - f[d];
            off += bit * t->stride[d];
        }
        const float* p = t->v + off * OUT;
        for (int k = 0; k < OUT; ++k) acc[k] += (float)w * p[k];
    }
    for (int k = 0; k < OUT; ++k) out[k] = acc[k];
}

int main(int argc, char** argv) {
    // Default grid: see CLAUDE.md, Q1 item 5.
    Table t = {.n = {41, 31, 7, 5, 8},
               .lo = {-0.35, -0.5, 1000, -0.07, 0.0},
               .hi = {+0.35, +0.5, 8000, +0.07, 15.0}};
    for (int d = 0; d < D && d + 1 < argc; ++d) t.n[d] = atoi(argv[d + 1]);
    long nodes = 1;
    for (int d = D - 1; d >= 0; --d) { t.stride[d] = nodes; nodes *= t.n[d]; }
    t.v = malloc(sizeof(float) * OUT * nodes);
    if (!t.v) return 1;
    for (long i = 0; i < nodes * OUT; ++i) t.v[i] = (float)sin(0.001 * i);

    printf("grid %d x %d x %d x %d x %d = %ld nodes, %d outputs: %.2f MB as float\n",
           t.n[0], t.n[1], t.n[2], t.n[3], t.n[4], nodes, OUT,
           sizeof(float) * OUT * nodes / 1e6);

    const int steps = 2000000;          // 2000 s at 1 kHz, 4 tyres a step
    float out[OUT];
    double sink = 0;
    unsigned s = 12345;
    for (int mode = 0; mode < 2; ++mode) {
        double x[4][D];
        for (int w = 0; w < 4; ++w)
            for (int d = 0; d < D; ++d) x[w][d] = 0.5 * (t.lo[d] + t.hi[d]);
        const double t0 = now_s();
        for (int i = 0; i < steps; ++i)
            for (int w = 0; w < 4; ++w) {
                for (int d = 0; d < D; ++d) {
                    s = s * 1103515245u + 12345u;
                    const double r = ((s >> 8) & 0xffff) / 65535.0;
                    if (mode == 0) {   // smooth: a small random walk, reflected
                        x[w][d] += (r - 0.5) * 1e-3 * (t.hi[d] - t.lo[d]);
                        if (x[w][d] < t.lo[d]) x[w][d] = 2 * t.lo[d] - x[w][d];
                        if (x[w][d] > t.hi[d]) x[w][d] = 2 * t.hi[d] - x[w][d];
                    } else {           // random: anywhere
                        x[w][d] = t.lo[d] + r * (t.hi[d] - t.lo[d]);
                    }
                }
                lookup(&t, x[w], out);
                sink += out[0];
            }
        const double dt = now_s() - t0;
        printf("  %-6s %7.1f ns per lookup, %6.2f us per 1 ms step (4 tyres) = %.2f%% of the budget\n",
               mode == 0 ? "smooth" : "random", 1e9 * dt / (4.0 * steps),
               1e6 * dt / steps, 100.0 * (1e6 * dt / steps) / 1000.0);
    }
    printf("  (checksum %g)\n", sink);
    free(t.v);
    return 0;
}
