/* wheel_device.cpp — implementation. See wheel_device.h for the design notes.
 *
 * The body is C, but the file is .cpp so that UnrealBuildTool compiles it as
 * part of the ChronoBridge module without any special handling. The header's
 * extern "C" block covers the linkage, so the standalone harness in wheel/ --
 * which builds this same file with gcc -- links against identical symbols.
 *
 * This is the CANONICAL copy. wheel/run.sh locates it by walking up the tree,
 * the same way ChronoBridge.Build.cs locates chrono-sdk; there is deliberately
 * no second copy to drift out of sync. */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE   /* strcasestr; UE may already define it */
#endif
#include "wheel_device.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/input.h>

/* ---- evdev bitmap helpers ---------------------------------------------- *
 * EVIOCGBIT returns a packed bit array of "which codes does this device
 * support". There is no libc wrapper; every evdev consumer rewrites these. */
#define WD_LONG_BITS   (sizeof(unsigned long) * 8)
#define WD_NLONGS(n)   (((n) + WD_LONG_BITS - 1) / WD_LONG_BITS)
#define WD_TESTBIT(a, b) (((a)[(b) / WD_LONG_BITS] >> ((b) % WD_LONG_BITS)) & 1UL)

/* ---- what we know about specific Logitech wheels ------------------------ *
 * Used ONLY for (a) ranking candidates during auto-discovery and (b) the
 * pedal-inversion default. Capabilities are always re-queried from the kernel;
 * nothing functional is derived from the product ID. */
typedef struct { uint16_t pid; const char* label; int pedals_inverted; } KnownWheel;
static const KnownWheel kKnown[] = {
    { 0xc24f, "G29 (also G923-PS compatibility mode)", 1 },
    { 0xc260, "G29 (alt)",                              1 },
    { 0xc262, "G920",                                   1 },
    { 0xc266, "G923 PS/PC (no in-tree FF driver)",      1 },
    { 0xc26e, "G923 Xbox/PC (hid-logitech-hidpp)",      1 },
    { 0xc29b, "G27",                                    1 },
    { 0xc294, "Driving Force",                          1 },
    { 0xc298, "Driving Force Pro",                      1 },
    { 0xc29a, "Driving Force GT",                       1 },
};
#define NKNOWN ((int)(sizeof(kKnown) / sizeof(kKnown[0])))

/* ---- H-pattern shifter gates -------------------------------------------
 * The G29/G923 shifter is not a separate USB device: it plugs into the wheel
 * base and its seven gates surface as buttons on the wheel's own evdev node.
 *
 * BTN_TRIGGER_HAPPY1..7 is the mapping reported for the G29 shifter, and is
 * the DEFAULT here -- but it is unverified on this hardware, because no wheel
 * has ever been attached to this machine. `test_wheel monitor` prints every
 * button code it sees precisely so this table can be confirmed or corrected in
 * one pass once the wheel arrives. Do not treat it as measured. */
static const int kGateCodes[7] = {
    BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2, BTN_TRIGGER_HAPPY3,
    BTN_TRIGGER_HAPPY4, BTN_TRIGGER_HAPPY5, BTN_TRIGGER_HAPPY6,
    BTN_TRIGGER_HAPPY7,
};

/* Gate index -> gear. The Innova is a FIVE-speed: gate 6 exists on the
 * shifter and is deliberately left unmapped (CLAUDE.md, Milestone 5 notes).
 * Selecting it yields neutral, which is also what a real mis-shift feels
 * like -- better than silently engaging a gear the car does not have. */
static int gate_to_gear(int gate) {
    switch (gate) {
        case 0: case 1: case 2: case 3: case 4: return gate + 1;  /* 1..5 */
        case 5: return 0;    /* 6th gate: unmapped, Innova has no 6th gear */
        case 6: return -1;   /* reverse */
        default: return 0;
    }
}

struct WheelDevice {
    int        fd;
    WheelCaps  caps;
    WheelState state;

    /* force feedback */
    int        ff_id;            /* effect slot the kernel gave us, -1 = none */
    double     ff_peak_nm;
    int        ff_divisor, ff_deadband;
    int        ff_call_n;        /* counts wheel_ffb_set calls for decimation */
    int        ff_last_level;    /* last magnitude actually transmitted       */
    uint64_t   ff_calls;         /* EVIOCSFF ioctls issued                    */
    double     ff_total_us;
};

/* ---- axis handling ------------------------------------------------------ */

static void probe_axis(int fd, const unsigned long* absbits, int code,
                       int inverted_default, WheelAxisCaps* out) {
    out->code = -1;
    if (code < 0 || !WD_TESTBIT(absbits, code)) return;

    struct input_absinfo ai;
    if (ioctl(fd, EVIOCGABS(code), &ai) < 0) return;

    out->code = code;
    out->min  = ai.minimum;
    out->max  = ai.maximum;
    out->flat = ai.flat;
    out->fuzz = ai.fuzz;
    out->inverted = inverted_default;
}

/* Raw -> 0..1, honouring the axis's own declared range rather than a
 * hardcoded 0..255 or 0..65535: the G29 reports 16-bit steering and 8-bit
 * pedals, the G920 differs again, and "combined pedals" mode changes it once
 * more. Reading EVIOCGABS is the only way to be right in all of them. */
static double axis_unit(const WheelAxisCaps* a, int32_t raw) {
    if (a->code < 0 || a->max <= a->min) return 0.0;
    if (raw < a->min) raw = a->min;
    if (raw > a->max) raw = a->max;
    double u = (double)(raw - a->min) / (double)(a->max - a->min);
    return a->inverted ? 1.0 - u : u;
}

/* ---- sysfs (lg4ff rotation range) --------------------------------------- */

static int sysfs_range_path(const char* devpath, char* out, size_t n) {
    const char* base = strrchr(devpath, '/');
    base = base ? base + 1 : devpath;                  /* "eventN" */
    /* The HID device sits two levels above the input node; older kernels put
     * the attribute one level up. Try both rather than guess. */
    const char* forms[] = { "/sys/class/input/%s/device/device/range",
                            "/sys/class/input/%s/device/range" };
    for (int i = 0; i < 2; ++i) {
        snprintf(out, n, forms[i], base);
        if (access(out, F_OK) == 0) return 0;
    }
    out[0] = '\0';
    return -1;
}

int wheel_set_range(WheelDevice* d, int degrees) {
    char p[256];
    if (sysfs_range_path(d->caps.path, p, sizeof p) != 0) return -ENOENT;
    FILE* f = fopen(p, "w");
    if (!f) return -errno;
    fprintf(f, "%d\n", degrees);
    int rc = fclose(f) == 0 ? 0 : -errno;
    if (rc == 0) d->caps.range_deg = degrees;
    return rc;
}

/* ---- capability probe --------------------------------------------------- */

static int fill_caps(int fd, const char* path, int writable, WheelCaps* c) {
    memset(c, 0, sizeof *c);
    snprintf(c->path, sizeof c->path, "%s", path);
    c->writable = writable;
    c->range_deg = -1;

    if (ioctl(fd, EVIOCGNAME(sizeof c->name), c->name) < 0) c->name[0] = '\0';

    struct input_id id;
    if (ioctl(fd, EVIOCGID, &id) == 0) {
        c->vendor = id.vendor; c->product = id.product; c->version = id.version;
    }

    unsigned long absbits[WD_NLONGS(ABS_CNT)]; memset(absbits, 0, sizeof absbits);
    unsigned long keybits[WD_NLONGS(KEY_CNT)]; memset(keybits, 0, sizeof keybits);
    unsigned long ffbits [WD_NLONGS(FF_CNT)];  memset(ffbits,  0, sizeof ffbits);
    ioctl(fd, EVIOCGBIT(EV_ABS, sizeof absbits), absbits);
    ioctl(fd, EVIOCGBIT(EV_KEY, sizeof keybits), keybits);
    ioctl(fd, EVIOCGBIT(EV_FF,  sizeof ffbits),  ffbits);

    /* Pedal inversion: on every Logitech wheel a RELEASED pedal reads at the
     * axis maximum. For an unknown device we cannot know, so fall back to the
     * resting value -- if the axis sits in its top third at open() time, treat
     * it as inverted. (Fooled by a pedal held down during startup; that is why
     * known devices use the table instead.) */
    int inv = 1, known = 0;
    for (int i = 0; i < NKNOWN; ++i)
        if (kKnown[i].pid == c->product && c->vendor == 0x046d) {
            inv = kKnown[i].pedals_inverted; known = 1; break;
        }

    probe_axis(fd, absbits, ABS_X,  0, &c->steer);   /* steering never inverts */
    probe_axis(fd, absbits, ABS_Y,  inv, &c->throttle);
    probe_axis(fd, absbits, ABS_Z,  inv, &c->brake);
    probe_axis(fd, absbits, ABS_RZ, inv, &c->clutch);

    if (!known) {
        const WheelAxisCaps* pedals[3] = { &c->throttle, &c->brake, &c->clutch };
        WheelAxisCaps* wr[3] = { &c->throttle, &c->brake, &c->clutch };
        for (int i = 0; i < 3; ++i) {
            if (pedals[i]->code < 0) continue;
            struct input_absinfo ai;
            if (ioctl(fd, EVIOCGABS(pedals[i]->code), &ai) < 0) continue;
            double u = (double)(ai.value - ai.minimum) /
                       (double)(ai.maximum - ai.minimum + 1);
            wr[i]->inverted = (u > 0.66) ? 1 : 0;
        }
    }

    c->has_ff        = 0;
    for (int i = 0; i < FF_CNT; ++i) if (WD_TESTBIT(ffbits, i)) { c->has_ff = 1; break; }
    c->ff_constant   = WD_TESTBIT(ffbits, FF_CONSTANT)   ? 1 : 0;
    c->ff_spring     = WD_TESTBIT(ffbits, FF_SPRING)     ? 1 : 0;
    c->ff_damper     = WD_TESTBIT(ffbits, FF_DAMPER)     ? 1 : 0;
    c->ff_autocenter = WD_TESTBIT(ffbits, FF_AUTOCENTER) ? 1 : 0;
    if (ioctl(fd, EVIOCGEFFECTS, &c->ff_slots) < 0) c->ff_slots = 0;

    c->gate_count = 0;
    for (int i = 0; i < 7; ++i) if (WD_TESTBIT(keybits, kGateCodes[i])) ++c->gate_count;

    char p[256];
    if (sysfs_range_path(path, p, sizeof p) == 0) {
        FILE* f = fopen(p, "r");
        if (f) { if (fscanf(f, "%d", &c->range_deg) != 1) c->range_deg = -1; fclose(f); }
    }
    return 0;
}

/* ---- discovery ---------------------------------------------------------- */

/* Ranks a candidate node. A machine can have a dozen evdev nodes; the wheel is
 * the one that is Logitech, has an X axis, and ideally carries FF_CONSTANT.
 * Returning a score rather than a boolean means a G923 that binds without FF
 * still wins over a keyboard instead of being rejected outright. */
static int score_device(const WheelCaps* c) {
    if (c->steer.code < 0) return -1;          /* no steering axis: not a wheel */
    int s = 20;
    if (c->vendor == 0x046d) s += 100;
    for (int i = 0; i < NKNOWN; ++i) if (kKnown[i].pid == c->product) { s += 50; break; }
    if (c->ff_constant) s += 40;
    if (c->gate_count)  s += 15;
    if (strcasestr(c->name, "G923") || strcasestr(c->name, "G29") ||
        strcasestr(c->name, "G920") || strcasestr(c->name, "Driving Force")) s += 10;
    return s;
}

static int open_node(const char* path, int* writable) {
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd >= 0) { *writable = 1; return fd; }
    fd = open(path, O_RDONLY | O_NONBLOCK);   /* probe still works read-only */
    if (fd >= 0) { *writable = 0; return fd; }
    return -1;
}

int wheel_open(WheelDevice** out, const char* path, char* err, size_t errsz) {
    *out = NULL;
    char best_path[64] = {0};
    int  best_score = -1;

    if (path) {
        snprintf(best_path, sizeof best_path, "%s", path);
    } else {
        DIR* dir = opendir("/dev/input");
        if (!dir) { snprintf(err, errsz, "opendir /dev/input: %s", strerror(errno)); return -errno; }
        struct dirent* de;
        while ((de = readdir(dir))) {
            if (strncmp(de->d_name, "event", 5) != 0) continue;
            char p[64]; snprintf(p, sizeof p, "/dev/input/%.32s", de->d_name);
            int w = 0, fd = open_node(p, &w);
            if (fd < 0) continue;
            WheelCaps c; fill_caps(fd, p, w, &c);
            close(fd);
            int s = score_device(&c);
            if (s > best_score) { best_score = s; snprintf(best_path, sizeof best_path, "%s", p); }
        }
        closedir(dir);
        if (best_score < 0) {
            snprintf(err, errsz,
                     "no device with an ABS_X axis found under /dev/input "
                     "(is the wheel plugged in? are you in the 'input' group?)");
            return -ENODEV;
        }
    }

    int writable = 0;
    int fd = open_node(best_path, &writable);
    if (fd < 0) { snprintf(err, errsz, "open %s: %s", best_path, strerror(errno)); return -errno; }

    WheelDevice* d = static_cast<WheelDevice*>(calloc(1, sizeof *d));
    if (!d) { close(fd); snprintf(err, errsz, "out of memory"); return -ENOMEM; }
    d->fd = fd;
    d->ff_id = -1;
    d->ff_divisor = 4;      /* 1 kHz in -> 250 Hz on the wire */
    d->ff_deadband = 64;    /* out of 32767 */
    d->ff_last_level = INT32_MIN;   /* force the first update through */
    fill_caps(fd, best_path, writable, &d->caps);

    /* Seed the state from the CURRENT axis positions. Without this the first
     * frames report a centred wheel and released pedals no matter where they
     * actually are, because evdev only sends an event when something CHANGES. */
    struct input_absinfo ai;
    const WheelAxisCaps* ax[4] = { &d->caps.steer, &d->caps.throttle,
                                   &d->caps.brake, &d->caps.clutch };
    double* dst[4] = { &d->state.steer, &d->state.throttle,
                       &d->state.brake, &d->state.clutch };
    for (int i = 0; i < 4; ++i) {
        if (ax[i]->code < 0) continue;
        if (ioctl(fd, EVIOCGABS(ax[i]->code), &ai) < 0) continue;
        double u = axis_unit(ax[i], ai.value);
        *dst[i] = (i == 0) ? (2.0 * u - 1.0) : u;
    }
    *out = d;
    return 0;
}

int wheel_open_raw(WheelDevice** out, int fd, const WheelCaps* caps) {
    WheelDevice* d = static_cast<WheelDevice*>(calloc(1, sizeof *d));
    if (!d) return -ENOMEM;
    d->fd = fd;
    d->ff_id = -1;
    d->ff_divisor = 4;
    d->ff_deadband = 64;
    d->ff_last_level = INT32_MIN;
    d->caps = *caps;
    *out = d;
    return 0;
}

void wheel_close(WheelDevice* d) {
    if (!d) return;
    if (d->ff_id >= 0) wheel_ffb_stop(d);
    if (d->fd >= 0) close(d->fd);
    free(d);
}

const WheelCaps* wheel_caps(const WheelDevice* d) { return &d->caps; }

/* ---- event pump --------------------------------------------------------- */

int wheel_poll(WheelDevice* d, WheelState* out) {
    struct input_event ev[64];
    int frames = 0;

    for (;;) {
        ssize_t n = read(d->fd, ev, sizeof ev);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;  /* nothing queued */
            if (errno == EINTR) continue;
            return -errno;                                       /* unplugged */
        }
        if (n == 0) break;

        for (size_t i = 0; i < (size_t)n / sizeof ev[0]; ++i) {
            const struct input_event* e = &ev[i];
            if (e->type == EV_ABS) {
                if (e->code == d->caps.steer.code)
                    d->state.steer = 2.0 * axis_unit(&d->caps.steer, e->value) - 1.0;
                else if (e->code == d->caps.throttle.code)
                    d->state.throttle = axis_unit(&d->caps.throttle, e->value);
                else if (e->code == d->caps.brake.code)
                    d->state.brake = axis_unit(&d->caps.brake, e->value);
                else if (e->code == d->caps.clutch.code)
                    d->state.clutch = axis_unit(&d->caps.clutch, e->value);
                else if (e->code == ABS_HAT0X)
                    d->state.dpad_x = e->value < 0 ? -1 : (e->value > 0 ? 1 : 0);
            } else if (e->type == EV_KEY) {
                for (int g = 0; g < 7; ++g) {
                    if (e->code != kGateCodes[g]) continue;
                    if (e->value) d->state.gates |=  (1u << g);
                    else          d->state.gates &= ~(1u << g);
                }
            } else if (e->type == EV_SYN && e->code == SYN_REPORT) {
                /* Gear is resolved per frame, not per button event: during a
                 * shift the old gate releases and the new one closes, and
                 * between those two events the stick is genuinely in neutral.
                 * Taking the lowest set bit gives a deterministic answer if
                 * two gates ever report closed at once. */
                int gear = 0;
                for (int g = 0; g < 7; ++g)
                    if (d->state.gates & (1u << g)) { gear = gate_to_gear(g); break; }
                d->state.gear = gear;
                ++d->state.seq;
                ++frames;
            }
        }
    }
    if (out) *out = d->state;
    return frames;
}

/* ---- force feedback ----------------------------------------------------- */

int wheel_ffb_init(WheelDevice* d, double peak_torque_nm, char* err, size_t errsz) {
    if (!d->caps.writable) {
        snprintf(err, errsz, "%s opened read-only; FFB needs O_RDWR "
                 "(add yourself to the 'input' group)", d->caps.path);
        return -EACCES;
    }
    if (!d->caps.ff_constant) {
        snprintf(err, errsz,
                 "device '%s' (%04x:%04x) exposes no FF_CONSTANT. It has bound to "
                 "a driver without force feedback -- check that hid-logitech or "
                 "hid-logitech-hidpp claimed it (dmesg | grep -i logitech).",
                 d->caps.name, d->caps.vendor, d->caps.product);
        return -ENOTSUP;
    }

    d->ff_peak_nm = peak_torque_nm > 0.0 ? peak_torque_nm : 2.3;

    /* One constant-force effect, uploaded once and thereafter only MUTATED.
     *
     * direction = 0x4000 is the sim-racing convention: evdev encodes direction
     * as an angle (0x0000 = "from north", 0x4000 = "from east"), but for a
     * single-axis wheel it is far cleaner to pin the axis and carry the sign in
     * `level`, which is a signed 16-bit value. Re-uploading with a NEGATIVE
     * level then means "the other way" without touching the direction field.
     *
     * replay.length = 0 means infinite: start it once here, and every later
     * update is a bare magnitude rewrite with no restart, no envelope
     * retrigger, and no gap in the force. */
    struct ff_effect eff;
    memset(&eff, 0, sizeof eff);
    eff.type = FF_CONSTANT;
    eff.id   = -1;                    /* -1 asks the kernel to allocate a slot */
    eff.direction = 0x4000;
    eff.u.constant.level = 0;
    eff.replay.length = 0;
    eff.replay.delay  = 0;

    if (ioctl(d->fd, EVIOCSFF, &eff) < 0) {
        snprintf(err, errsz, "EVIOCSFF upload: %s", strerror(errno));
        return -errno;
    }
    d->ff_id = eff.id;

    /* Full gain, and autocentre OFF. The wheel's built-in autocentre spring is
     * a fixed synthetic force; leaving it on would sum with the simulated
     * self-aligning torque and mask exactly the cue the test is meant to teach
     * -- the wheel going light as the front tyres saturate. */
    struct input_event ie;
    memset(&ie, 0, sizeof ie);
    ie.type = EV_FF; ie.code = FF_GAIN; ie.value = 0xFFFF;
    if (write(d->fd, &ie, sizeof ie) < 0) { /* non-fatal */ }
    if (d->caps.ff_autocenter) {
        ie.code = FF_AUTOCENTER; ie.value = 0;
        if (write(d->fd, &ie, sizeof ie) < 0) { /* non-fatal */ }
    }

    /* Start it. value = 1 plays once; with an infinite length that means
     * "play until stopped". */
    ie.type = EV_FF; ie.code = (uint16_t)d->ff_id; ie.value = 1;
    if (write(d->fd, &ie, sizeof ie) < 0) {
        snprintf(err, errsz, "EV_FF play: %s", strerror(errno));
        return -errno;
    }
    return 0;
}

void wheel_ffb_set_rate(WheelDevice* d, int divisor, int deadband) {
    d->ff_divisor  = divisor  > 0 ? divisor  : 1;
    d->ff_deadband = deadband >= 0 ? deadband : 0;
}

int wheel_ffb_set(WheelDevice* d, double torque_nm) {
    if (d->ff_id < 0) return -EINVAL;

    /* SOFT clip, not hard clip. The G923 saturates around 2.3 Nm while a real
     * unassisted rack can demand far more; a hard clip flattens every event
     * above the limit into the same sensation, so understeer at 2.4 Nm and a
     * kerb strike at 8 Nm feel identical. Compressing the top 20% keeps the
     * gradient -- the driver can still tell "more" from "much more". */
    double n = torque_nm / d->ff_peak_nm;
    const double knee = 0.8;
    double a = fabs(n);
    if (a > knee) {
        a = knee + (1.0 - knee) * tanh((a - knee) / (1.0 - knee));
        n = copysign(a, n);
    }

    int level = (int)lrint(n * 32767.0);
    if (level >  32767) level =  32767;
    if (level < -32767) level = -32767;

    /* Decimate. Counting calls rather than consulting a clock keeps this
     * allocation- and syscall-free on the steps where nothing is sent. */
    if (++d->ff_call_n < d->ff_divisor) return 0;
    d->ff_call_n = 0;

    if (d->ff_last_level != INT32_MIN &&
        abs(level - d->ff_last_level) < d->ff_deadband) return 0;

    struct ff_effect eff;
    memset(&eff, 0, sizeof eff);
    eff.type = FF_CONSTANT;
    eff.id   = d->ff_id;              /* same id => update in place, no restart */
    eff.direction = 0x4000;
    eff.u.constant.level = (int16_t)level;
    eff.replay.length = 0;

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    int rc = ioctl(d->fd, EVIOCSFF, &eff);
    clock_gettime(CLOCK_MONOTONIC, &t1);

    d->ff_calls++;
    d->ff_total_us += (double)(t1.tv_sec - t0.tv_sec) * 1e6 +
                      (double)(t1.tv_nsec - t0.tv_nsec) / 1e3;
    if (rc < 0) return -errno;
    d->ff_last_level = level;
    return 1;
}

int wheel_ffb_stop(WheelDevice* d) {
    if (d->ff_id < 0) return 0;
    struct input_event ie;
    memset(&ie, 0, sizeof ie);
    ie.type = EV_FF; ie.code = (uint16_t)d->ff_id; ie.value = 0;   /* stop */
    ssize_t w = write(d->fd, &ie, sizeof ie);
    (void)w;
    int id = d->ff_id;
    d->ff_id = -1;
    /* Release the slot. Skipping this leaks an effect in the driver, and the
     * wheel only has a handful (EVIOCGEFFECTS), so repeated restarts of the
     * editor would eventually fail to upload. */
    if (ioctl(d->fd, EVIOCRMFF, id) < 0) return -errno;
    return 0;
}

void wheel_ffb_stats(const WheelDevice* d, uint64_t* calls, double* total_us) {
    if (calls)    *calls    = d->ff_calls;
    if (total_us) *total_us = d->ff_total_us;
}

#undef WD_LONG_BITS
#undef WD_NLONGS
#undef WD_TESTBIT
