/* ===========================================================================
 *  virtual_g923 — a uinput device that impersonates a G923 well enough to
 *  bring up and regression-test the whole input + force-feedback path with no
 *  hardware attached.
 *
 *  WHY THIS EXISTS: no wheel has ever been connected to this machine, and the
 *  parts of Milestone 5 that cannot be checked by the offline decoder test are
 *  exactly the parts that involve the kernel -- EVIOCSFF upload, in-place
 *  effect mutation, effect-slot lifetime, and the real syscall cost of an FFB
 *  update. uinput reproduces all of those faithfully because it is the same
 *  input core, the same ff-core, and the same ioctls; only the USB transport
 *  is missing.
 *
 *  WHAT IT DOES NOT PROVE: real axis ranges, the true shifter button codes,
 *  the sign of a positive constant force, lg4ff's sysfs `range` attribute, or
 *  USB transfer latency. Those need the wheel.
 *
 *  Needs write access to /dev/uinput -- see scripts/04-uinput-access.sh.
 * ======================================================================== */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/input.h>
#include <linux/uinput.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

static const int kGates[7] = {
    BTN_TRIGGER_HAPPY1, BTN_TRIGGER_HAPPY2, BTN_TRIGGER_HAPPY3,
    BTN_TRIGGER_HAPPY4, BTN_TRIGGER_HAPPY5, BTN_TRIGGER_HAPPY6,
    BTN_TRIGGER_HAPPY7,
};

static void emit(int fd, uint16_t t, uint16_t c, int32_t v) {
    struct input_event e; memset(&e, 0, sizeof e);
    e.type = t; e.code = c; e.value = v;
    if (write(fd, &e, sizeof e) != (ssize_t)sizeof e) perror("uinput write");
}

static void abs_setup(int fd, uint16_t code, int32_t min, int32_t max, int32_t val) {
    struct uinput_abs_setup a; memset(&a, 0, sizeof a);
    a.code = code;
    a.absinfo.minimum = min; a.absinfo.maximum = max;
    a.absinfo.value = val;   a.absinfo.flat = 0; a.absinfo.fuzz = 0;
    if (ioctl(fd, UI_ABS_SETUP, &a) < 0) perror("UI_ABS_SETUP");
}

static double now_s(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

int main(int argc, char** argv) {
    const int seconds = argc > 1 ? atoi(argv[1]) : 30;
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    int fd = open("/dev/uinput", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr,
            "open /dev/uinput: %s\n"
            "Run scripts/04-uinput-access.sh once (needs sudo) to load the module\n"
            "and grant the 'input' group access.\n", strerror(errno));
        return 1;
    }

    ioctl(fd, UI_SET_EVBIT, EV_ABS);
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_EVBIT, EV_FF);
    ioctl(fd, UI_SET_EVBIT, EV_SYN);

    /* Same axis layout and ranges a G29/G923 reports in PS/PC mode: 16-bit
     * steering, 8-bit pedals. The pedal ranges are declared 0..255 here and the
     * device drives them INVERTED (255 = released), which is what makes the
     * inversion path in wheel_device.c testable. */
    ioctl(fd, UI_SET_ABSBIT, ABS_X);
    ioctl(fd, UI_SET_ABSBIT, ABS_Y);
    ioctl(fd, UI_SET_ABSBIT, ABS_Z);
    ioctl(fd, UI_SET_ABSBIT, ABS_RZ);
    for (int i = 0; i < 7; ++i) ioctl(fd, UI_SET_KEYBIT, kGates[i]);

    /* Advertise the lg4ff capability set specifically -- CONSTANT and
     * AUTOCENTER only, no SPRING or DAMPER. If the client works against this
     * it works against the narrower of the two plausible bind paths. */
    ioctl(fd, UI_SET_FFBIT, FF_CONSTANT);
    ioctl(fd, UI_SET_FFBIT, FF_AUTOCENTER);
    ioctl(fd, UI_SET_FFBIT, FF_GAIN);

    abs_setup(fd, ABS_X,  0, 65535, 32767);
    abs_setup(fd, ABS_Y,  0,   255,   255);
    abs_setup(fd, ABS_Z,  0,   255,   255);
    abs_setup(fd, ABS_RZ, 0,   255,   255);

    struct uinput_setup us; memset(&us, 0, sizeof us);
    us.id.bustype = BUS_USB;
    us.id.vendor  = 0x046d;
    us.id.product = 0xc24f;      /* G29 / G923-PS compatibility mode */
    us.id.version = 0x0111;
    us.ff_effects_max = 4;       /* mirrors what lg4ff exposes */
    snprintf(us.name, sizeof us.name, "Logitech G923 Racing Wheel (virtual)");

    if (ioctl(fd, UI_DEV_SETUP, &us) < 0) { perror("UI_DEV_SETUP"); return 1; }
    if (ioctl(fd, UI_DEV_CREATE) < 0)     { perror("UI_DEV_CREATE"); return 1; }

    char sys[256] = {0};
    if (ioctl(fd, UI_GET_SYSNAME(sizeof sys), sys) < 0) snprintf(sys, sizeof sys, "?");
    printf("virtual G923 up as %s (046d:c24f, 4 FF slots). Running %ds; Ctrl-C to stop.\n",
           sys, seconds);
    printf("%-10s %-8s %s\n", "t(s)", "event", "detail");
    fflush(stdout);

    const double t0 = now_s();
    int gear_gate = -1;
    int16_t last_level = 0;
    long uploads = 0;

    while (!g_stop && now_s() - t0 < seconds) {
        /* ---- service force-feedback requests from the input core ---------
         * A real driver receives effects through its ff->upload callback.
         * uinput forwards them to userspace as EV_UINPUT events which must be
         * answered with BEGIN/END ioctls; failing to answer makes the client's
         * EVIOCSFF block until it times out. */
        struct input_event ev;
        while (read(fd, &ev, sizeof ev) == (ssize_t)sizeof ev) {
            const double t = now_s() - t0;
            if (ev.type == EV_UINPUT && ev.code == UI_FF_UPLOAD) {
                struct uinput_ff_upload up; memset(&up, 0, sizeof up);
                up.request_id = ev.value;
                if (ioctl(fd, UI_BEGIN_FF_UPLOAD, &up) < 0) { perror("BEGIN_FF_UPLOAD"); continue; }
                up.retval = 0;
                if (ioctl(fd, UI_END_FF_UPLOAD, &up) < 0) perror("END_FF_UPLOAD");
                ++uploads;
                if (up.effect.type == FF_CONSTANT) {
                    last_level = up.effect.u.constant.level;
                    printf("%-10.3f %-8s id=%d level=%+6d dir=0x%04x len=%u\n",
                           t, "UPLOAD", up.effect.id, last_level,
                           up.effect.direction, up.effect.replay.length);
                } else {
                    printf("%-10.3f %-8s id=%d type=%u\n", t, "UPLOAD",
                           up.effect.id, up.effect.type);
                }
                fflush(stdout);
            } else if (ev.type == EV_UINPUT && ev.code == UI_FF_ERASE) {
                struct uinput_ff_erase er; memset(&er, 0, sizeof er);
                er.request_id = ev.value;
                if (ioctl(fd, UI_BEGIN_FF_ERASE, &er) == 0) {
                    er.retval = 0;
                    ioctl(fd, UI_END_FF_ERASE, &er);
                    printf("%-10.3f %-8s id=%u\n", t, "ERASE", er.effect_id);
                    fflush(stdout);
                }
            } else if (ev.type == EV_FF) {
                const char* what = ev.code == FF_GAIN       ? "GAIN"
                                 : ev.code == FF_AUTOCENTER ? "AUTOCENT" : "PLAY";
                printf("%-10.3f %-8s code=%u value=%d\n", t, what, ev.code, ev.value);
                fflush(stdout);
            }
        }

        /* ---- drive the axes so `monitor` has something to display -------- */
        const double t = now_s() - t0;
        emit(fd, EV_ABS, ABS_X, (int32_t)(32767.0 + 32000.0 * sin(t * 0.7)));
        emit(fd, EV_ABS, ABS_Y, (int32_t)(255.0 * (0.5 + 0.5 * cos(t * 1.1))));
        emit(fd, EV_ABS, ABS_Z, 255);
        emit(fd, EV_ABS, ABS_RZ, 255);

        /* Walk the H-pattern once every two seconds, including the 6th gate
         * (which must decode to neutral) and reverse. */
        int want = ((int)(t / 2.0)) % 7;
        if (want != gear_gate) {
            if (gear_gate >= 0) emit(fd, EV_KEY, kGates[gear_gate], 0);
            emit(fd, EV_KEY, kGates[want], 1);
            gear_gate = want;
        }
        emit(fd, EV_SYN, SYN_REPORT, 0);

        struct timespec ts = {0, 2000000};   /* 500 Hz, like a real wheel's HID rate */
        nanosleep(&ts, NULL);
    }

    printf("\nserviced %ld FF uploads, last constant level %+d\n", uploads, last_level);
    ioctl(fd, UI_DEV_DESTROY);
    close(fd);
    return 0;
}
