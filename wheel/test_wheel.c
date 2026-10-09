/* ===========================================================================
 *  test_wheel — Milestone 5 bring-up harness for the G923.
 *
 *    probe    enumerate every evdev node, report which one looks like a wheel
 *             and which kernel driver claimed it
 *    monitor  live normalised axes + RAW button codes (use this to confirm or
 *             correct the shifter gate table against real hardware)
 *    decode   offline self-test of the event decoder: no device, no root
 *    ffb      constant-force sweep; confirms direction and measures ioctl cost
 *    loop     1 kHz poll+FFB loop, the shape the physics thread will run
 * ======================================================================== */
#define _GNU_SOURCE
#include "wheel_device.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <linux/input.h>

static double now_us(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec / 1e3;
}

/* Which kernel module claimed the device -- lg4ff and hid-logitech-hidpp offer
 * different FF capability sets, so this is the first thing to know. */
static void print_driver(const char* devpath) {
    const char* base = strrchr(devpath, '/'); base = base ? base + 1 : devpath;
    char link[320], target[320];
    snprintf(link, sizeof link, "/sys/class/input/%.32s/device/device/driver", base);
    ssize_t n = readlink(link, target, sizeof target - 1);
    if (n <= 0) { printf("    driver     : (unknown)\n"); return; }
    target[n] = '\0';
    const char* d = strrchr(target, '/');
    printf("    driver     : %s\n", d ? d + 1 : target);
}

static void print_axis(const char* label, const WheelAxisCaps* a) {
    if (a->code < 0) { printf("    %-9s: absent\n", label); return; }
    printf("    %-9s: code %d  range %d..%d  fuzz %d flat %d%s\n",
           label, a->code, a->min, a->max, a->fuzz, a->flat,
           a->inverted ? "  (inverted: released = max)" : "");
}

static void print_caps(const WheelCaps* c) {
    printf("  %s  \"%s\"\n", c->path, c->name[0] ? c->name : "(unnamed)");
    printf("    id         : %04x:%04x v%04x   %s\n",
           c->vendor, c->product, c->version, c->writable ? "rw" : "ro (no FFB)");
    print_driver(c->path);
    print_axis("steer",    &c->steer);
    print_axis("throttle", &c->throttle);
    print_axis("brake",    &c->brake);
    print_axis("clutch",   &c->clutch);
    printf("    ff         : %s%s%s%s%s  slots=%d\n",
           c->has_ff ? "" : "NONE",
           c->ff_constant ? "CONSTANT " : "", c->ff_spring ? "SPRING " : "",
           c->ff_damper ? "DAMPER " : "", c->ff_autocenter ? "AUTOCENTER " : "",
           c->ff_slots);
    printf("    gates      : %d of 7 shifter buttons present\n", c->gate_count);
    if (c->range_deg >= 0) printf("    range      : %d deg (lg4ff sysfs)\n", c->range_deg);
}

/* ---- probe -------------------------------------------------------------- */

static int do_probe(void) {
    printf("=== every evdev node ===\n");
    DIR* dir = opendir("/dev/input");
    if (!dir) { perror("opendir"); return 1; }
    struct dirent* de;
    char names[64][64]; int n = 0;
    while ((de = readdir(dir)) && n < 64)
        if (strncmp(de->d_name, "event", 5) == 0)
            snprintf(names[n++], 64, "/dev/input/%.32s", de->d_name);
    closedir(dir);

    for (int i = 0; i < n; ++i) {
        WheelDevice* d; char err[256];
        if (wheel_open(&d, names[i], err, sizeof err) != 0) {
            printf("  %s  -- %s\n", names[i], err); continue;
        }
        const WheelCaps* c = wheel_caps(d);
        if (c->steer.code < 0)
            printf("  %s  \"%s\"  (no ABS_X -- not a wheel)\n",
                   c->path, c->name[0] ? c->name : "?");
        else
            print_caps(c);
        wheel_close(d);
    }

    printf("\n=== auto-selected ===\n");
    WheelDevice* d; char err[256];
    if (wheel_open(&d, NULL, err, sizeof err) != 0) {
        printf("  NO WHEEL: %s\n", err);
        return 2;
    }
    print_caps(wheel_caps(d));
    wheel_close(d);
    return 0;
}

/* ---- monitor ------------------------------------------------------------ */

static int do_monitor(int seconds) {
    WheelDevice* d; char err[256];
    if (wheel_open(&d, NULL, err, sizeof err) != 0) { printf("no wheel: %s\n", err); return 2; }
    printf("monitoring %s for %ds -- turn the wheel, press pedals, move the shifter\n",
           wheel_caps(d)->path, seconds);
    printf("  %-8s %-8s %-8s %-8s %-5s %s\n",
           "steer", "throt", "brake", "clutch", "gear", "gates");
    const double t_end = now_us() + seconds * 1e6;
    uint64_t last_seq = 0;
    while (now_us() < t_end) {
        WheelState s;
        if (wheel_poll(d, &s) < 0) { printf("device lost\n"); break; }
        if (s.seq != last_seq) {
            last_seq = s.seq;
            printf("\r  %+8.4f %8.3f %8.3f %8.3f %5d  0x%02x",
                   s.steer, s.throttle, s.brake, s.clutch, s.gear, s.gates);
            fflush(stdout);
        }
        struct timespec ts = {0, 2000000};   /* 2 ms; this is a human-rate view */
        nanosleep(&ts, NULL);
    }
    printf("\n");
    wheel_close(d);
    return 0;
}

/* ---- decode self-test --------------------------------------------------- */

/* Capabilities matching a G29/G923 in its usual PS/PC layout: 16-bit steering,
 * 8-bit pedals that read MAXIMUM when released, seven shifter gates. */
static WheelCaps synth_caps(void) {
    WheelCaps c; memset(&c, 0, sizeof c);
    snprintf(c.path, sizeof c.path, "(synthetic)");
    snprintf(c.name, sizeof c.name, "Synthetic G923");
    c.vendor = 0x046d; c.product = 0xc24f; c.writable = 0; c.range_deg = -1;
    c.steer    = (WheelAxisCaps){ ABS_X,  0, 65535, 0, 0, 0 };
    c.throttle = (WheelAxisCaps){ ABS_Y,  0,   255, 0, 0, 1 };
    c.brake    = (WheelAxisCaps){ ABS_Z,  0,   255, 0, 0, 1 };
    c.clutch   = (WheelAxisCaps){ ABS_RZ, 0,   255, 0, 0, 1 };
    c.gate_count = 7;
    return c;
}

static int g_fail = 0;
static void chk(const char* what, double got, double want, double tol) {
    int ok = fabs(got - want) <= tol;
    if (!ok) ++g_fail;
    printf("  [%s] %-38s got %+9.5f  want %+9.5f\n", ok ? "PASS" : "FAIL", what, got, want);
}
static void chk_i(const char* what, long got, long want) {
    int ok = got == want;
    if (!ok) ++g_fail;
    printf("  [%s] %-38s got %+9ld  want %+9ld\n", ok ? "PASS" : "FAIL", what, got, want);
}

static void emit(int fd, uint16_t type, uint16_t code, int32_t val) {
    struct input_event e; memset(&e, 0, sizeof e);
    e.type = type; e.code = code; e.value = val;
    if (write(fd, &e, sizeof e) != (ssize_t)sizeof e) perror("write");
}
static void syn(int fd) { emit(fd, EV_SYN, SYN_REPORT, 0); }

static int do_decode(void) {
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sv) < 0) { perror("socketpair"); return 1; }

    WheelCaps caps = synth_caps();
    WheelDevice* d;
    if (wheel_open_raw(&d, sv[0], &caps) != 0) { printf("open_raw failed\n"); return 1; }
    const int w = sv[1];
    WheelState s;

    printf("=== decoder self-test (no hardware, no root) ===\n");

    printf("-- steering axis, 16-bit, centre-referenced\n");
    emit(w, EV_ABS, ABS_X, 32767); syn(w); wheel_poll(d, &s);
    chk("centre (raw 32767) -> 0", s.steer, 0.0, 1e-4);
    emit(w, EV_ABS, ABS_X, 0); syn(w); wheel_poll(d, &s);
    chk("full anticlockwise (raw 0) -> -1", s.steer, -1.0, 1e-9);
    emit(w, EV_ABS, ABS_X, 65535); syn(w); wheel_poll(d, &s);
    chk("full clockwise (raw 65535) -> +1", s.steer, +1.0, 1e-9);

    printf("-- pedals, 8-bit, INVERTED (released reads max)\n");
    emit(w, EV_ABS, ABS_Y, 255); emit(w, EV_ABS, ABS_Z, 255);
    emit(w, EV_ABS, ABS_RZ, 255); syn(w); wheel_poll(d, &s);
    chk("throttle released -> 0", s.throttle, 0.0, 1e-9);
    chk("brake released -> 0",    s.brake,    0.0, 1e-9);
    chk("clutch released -> 0",   s.clutch,   0.0, 1e-9);
    emit(w, EV_ABS, ABS_Y, 0); emit(w, EV_ABS, ABS_Z, 128); syn(w); wheel_poll(d, &s);
    chk("throttle floored -> 1",  s.throttle, 1.0, 1e-9);
    chk("brake half (raw 128)",   s.brake, 1.0 - 128.0/255.0, 1e-9);

    printf("-- H-pattern gates -> gear\n");
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY1, 1); syn(w); wheel_poll(d, &s);
    chk_i("gate 1 -> gear 1", s.gear, 1);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY1, 0); syn(w); wheel_poll(d, &s);
    chk_i("gate released -> neutral", s.gear, 0);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY5, 1); syn(w); wheel_poll(d, &s);
    chk_i("gate 5 -> gear 5", s.gear, 5);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY5, 0);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY6, 1); syn(w); wheel_poll(d, &s);
    chk_i("gate 6 -> NEUTRAL (Innova is 5-speed)", s.gear, 0);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY6, 0);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY7, 1); syn(w); wheel_poll(d, &s);
    chk_i("gate 7 -> reverse", s.gear, -1);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY7, 0); syn(w); wheel_poll(d, &s);

    printf("-- shift transition: gear must pass through neutral\n");
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY2, 1); syn(w); wheel_poll(d, &s);
    chk_i("in 2nd", s.gear, 2);
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY2, 0); syn(w);   /* frame A: stick between gates */
    emit(w, EV_KEY, BTN_TRIGGER_HAPPY3, 1); syn(w);   /* frame B: 3rd engaged */
    wheel_poll(d, &s);
    chk_i("after both frames drained -> 3rd", s.gear, 3);

    printf("-- D-pad left/right -> glance direction (task 17; G923 codes: task 12)\n");
    emit(w, EV_ABS, ABS_X, 32767); syn(w); wheel_poll(d, &s);
    emit(w, EV_ABS, ABS_HAT0X, -1); syn(w); wheel_poll(d, &s);
    chk_i("D-pad left (ABS_HAT0X -1) -> -1", s.dpad_x, -1);
    chk("... and the rim does not move", s.steer, 0.0, 1e-4);
    emit(w, EV_ABS, ABS_HAT0X, 1); syn(w); wheel_poll(d, &s);
    chk_i("D-pad right (+1) -> +1", s.dpad_x, 1);
    emit(w, EV_ABS, ABS_HAT0X, 0); syn(w); wheel_poll(d, &s);
    chk_i("D-pad released -> 0", s.dpad_x, 0);

    printf("-- frame accounting\n");
    uint64_t before = s.seq;
    for (int i = 0; i < 10; ++i) { emit(w, EV_ABS, ABS_X, 30000 + i); syn(w); }
    int frames = wheel_poll(d, &s);
    chk_i("wheel_poll returns 10 frames", frames, 10);
    chk_i("seq advanced by 10", (long)(s.seq - before), 10);

    printf("-- empty queue is cheap (this runs every 1 ms on the RT thread)\n");
    const int N = 100000;
    double t0 = now_us();
    for (int i = 0; i < N; ++i) wheel_poll(d, &s);
    double per = (now_us() - t0) / N;
    printf("  [INFO] empty wheel_poll: %.3f us/call (%d calls)\n", per, N);
    if (per > 20.0) { printf("  [FAIL] too expensive for a 1 kHz RT thread\n"); ++g_fail; }
    else            printf("  [PASS] well inside a 1000 us budget\n");

    wheel_close(d);
    close(w);
    printf("\n%s (%d failures)\n", g_fail ? "DECODE TEST FAILED" : "DECODE TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}

/* ---- ffb ---------------------------------------------------------------- */

static int do_ffb(int seconds, double peak_nm) {
    WheelDevice* d; char err[256];
    if (wheel_open(&d, NULL, err, sizeof err) != 0) { printf("no wheel: %s\n", err); return 2; }
    if (wheel_ffb_init(d, peak_nm, err, sizeof err) != 0) { printf("no FFB: %s\n", err); wheel_close(d); return 3; }

    printf("FFB sweep on %s, peak %.2f Nm, %ds.\n", wheel_caps(d)->path, peak_nm, seconds);
    printf("DIRECTION CHECK: for the first half the wheel should pull CLOCKWISE\n"
           "(to the driver's right). If it pulls the other way, the sign convention\n"
           "in wheel_ffb_set() is inverted for this device -- negate it there.\n");

    wheel_ffb_set_rate(d, 4, 64);
    const double t0 = now_us(), tend = t0 + seconds * 1e6;
    while (now_us() < tend) {
        double t = (now_us() - t0) / 1e6;
        /* Slow triangle: unambiguous to feel, unlike a fast sine. */
        double phase = fmod(t / 4.0, 1.0);
        double u = phase < 0.5 ? (4.0 * phase - 1.0) : (3.0 - 4.0 * phase);
        wheel_ffb_set(d, u * peak_nm * 0.6);
        struct timespec ts = {0, 1000000}; nanosleep(&ts, NULL);
    }
    wheel_ffb_set(d, 0.0);

    uint64_t calls; double us;
    wheel_ffb_stats(d, &calls, &us);
    printf("EVIOCSFF: %llu calls, %.2f us mean, %.1f Hz effective\n",
           (unsigned long long)calls, calls ? us / calls : 0.0,
           calls / (double)seconds);
    wheel_ffb_stop(d);
    wheel_close(d);
    return 0;
}

/* ---- 1 kHz loop --------------------------------------------------------- */

static int do_loop(int seconds, double peak_nm) {
    WheelDevice* d; char err[256];
    int synthetic = 0, sv[2] = {-1, -1};

    if (wheel_open(&d, NULL, err, sizeof err) != 0) {
        printf("no wheel (%s)\n-> running the loop against a synthetic fd so the\n"
               "   POLL cost is still measured; FFB is skipped.\n\n", err);
        if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sv) < 0) return 1;
        WheelCaps c = synth_caps();
        if (wheel_open_raw(&d, sv[0], &c) != 0) return 1;
        synthetic = 1;
    } else if (wheel_ffb_init(d, peak_nm, err, sizeof err) != 0) {
        printf("wheel found but no FFB: %s\n", err);
        synthetic = 1;
    }
    if (!synthetic) wheel_ffb_set_rate(d, 4, 64);

    /* Same absolute-deadline + hybrid-spin structure as ChronoPhysicsThread:
     * this measures the input layer in the exact shape it will run in. */
    const long PeriodNs = 1000000L, SpinNs = 200000L;
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    double max_late = 0, sum_late = 0, max_work = 0, sum_work = 0;
    long steps = 0, target = seconds * 1000L;

    for (long i = 0; i < target; ++i) {
        next.tv_nsec += PeriodNs;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; ++next.tv_sec; }
        struct timespec early = next;
        early.tv_nsec -= SpinNs;
        if (early.tv_nsec < 0) { early.tv_nsec += 1000000000L; --early.tv_sec; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &early, NULL);
        struct timespec nw;
        do { clock_gettime(CLOCK_MONOTONIC, &nw); __builtin_ia32_pause(); }
        while ((nw.tv_sec - next.tv_sec) * 1e6 + (nw.tv_nsec - next.tv_nsec) / 1e3 < 0.0);

        double late = (nw.tv_sec - next.tv_sec) * 1e6 + (nw.tv_nsec - next.tv_nsec) / 1e3;
        double w0 = now_us();

        WheelState s; wheel_poll(d, &s);
        /* Placeholder torque: a centring spring. Milestone 5 phase 2 replaces
         * this with the tyre self-aligning moment out of Chrono. */
        if (!synthetic) wheel_ffb_set(d, -s.steer * peak_nm * 0.5);

        double work = now_us() - w0;
        sum_late += late; sum_work += work; ++steps;
        if (late > max_late) max_late = late;
        if (work > max_work) max_work = work;
    }

    printf("1 kHz input loop, %ld steps%s:\n", steps, synthetic ? " (synthetic fd)" : "");
    printf("  wakeup lateness : mean %.2f us   max %.2f us\n", sum_late / steps, max_late);
    printf("  input work      : mean %.2f us   max %.2f us\n", sum_work / steps, max_work);
    if (!synthetic) {
        uint64_t calls; double us; wheel_ffb_stats(d, &calls, &us);
        printf("  EVIOCSFF        : %llu calls, %.2f us mean\n",
               (unsigned long long)calls, calls ? us / calls : 0.0);
        wheel_ffb_stop(d);
    }
    wheel_close(d);
    if (sv[1] >= 0) close(sv[1]);
    return 0;
}

int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "probe";
    int arg = argc > 2 ? atoi(argv[2]) : 0;
    if (strcmp(mode, "probe")   == 0) return do_probe();
    if (strcmp(mode, "monitor") == 0) return do_monitor(arg ? arg : 20);
    if (strcmp(mode, "decode")  == 0) return do_decode();
    if (strcmp(mode, "ffb")     == 0) return do_ffb(arg ? arg : 8, 2.3);
    if (strcmp(mode, "loop")    == 0) return do_loop(arg ? arg : 5, 2.3);
    fprintf(stderr, "usage: %s probe|monitor [s]|decode|ffb [s]|loop [s]\n", argv[0]);
    return 64;
}
