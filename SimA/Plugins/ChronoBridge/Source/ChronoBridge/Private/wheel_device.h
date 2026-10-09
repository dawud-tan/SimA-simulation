/* ===========================================================================
 *  wheel_device.h — Logitech G923 (and G29/G920) evdev + force-feedback layer.
 *
 *  PURE C11 + POSIX + Linux uapi. No Chrono, no Eigen, no Unreal, no C++.
 *
 *  WHY THIS IS NOT IN THE SHIM:
 *  The shim exists for exactly one reason -- to stop libstdc++ types and
 *  AVX2-widened Eigen structs from crossing into Unreal's libc++ world. evdev
 *  has neither problem: <linux/input.h> is a kernel uapi header describing a
 *  fixed C ABI that every compiler on this platform agrees on. Routing the
 *  wheel through the shim would add a hop and buy nothing. The shim gains only
 *  the *vehicle* entry points (driver inputs in, steering torque out); the
 *  device itself is spoken to directly from the Unreal side.
 *
 *  WHAT THE KERNEL ON THIS MACHINE ACTUALLY SUPPORTS (probed, not assumed):
 *    hid-logitech      (lg4ff) : ...C24F... -> G29. FF via ff-memless.
 *                                Capability set: FF_CONSTANT + FF_AUTOCENTER.
 *    hid-logitech-hidpp        : C262 (G920), C26E (G923 Xbox/PC).
 *                                Native HID++ FF: CONSTANT, SPRING, DAMPER,
 *                                AUTOCENTER -- a *different* capability set.
 *    Neither module lists C266 (the G923 PlayStation/PC SKU). That wheel is
 *    expected to enumerate in its G29 compatibility mode (C24F) and land on
 *    lg4ff; if it instead appears as a bare C266 HID gamepad it will have
 *    axes but NO force feedback, and wheel_ffb_init() will say so plainly.
 *
 *  CONSEQUENCE FOR THE DESIGN: capabilities are QUERIED AT RUNTIME, never
 *  assumed from the product ID. Two of the three plausible bind paths above
 *  offer different effect types; code that hardcodes one of them breaks on
 *  the other two.
 * ======================================================================== */
#ifndef WHEEL_DEVICE_H
#define WHEEL_DEVICE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Normalised driver state ------------------------------------------- */

/* SIGN CONVENTION -- fix it here, once, and never re-argue it.
 *
 *   steer : -1 = wheel rotated fully ANTICLOCKWISE (driver's left)
 *           +1 = wheel rotated fully CLOCKWISE     (driver's right)
 *   This is the DEVICE convention: evdev ABS_X increases clockwise.
 *
 *   Chrono's DriverInputs::m_steering is the opposite: positive steers LEFT
 *   (ISO 8855, +Y left). The negation happens at ONE place -- the shim call
 *   in ChronoWheelInput.cpp -- and nowhere else. CLAUDE.md's warning about
 *   the handedness flip applies verbatim: get this wrong and the car steers
 *   away from the wheel.
 */
typedef struct {
    double   steer;      /* -1 .. +1, clockwise positive                    */
    double   throttle;   /*  0 .. 1                                         */
    double   brake;      /*  0 .. 1                                         */
    double   clutch;     /*  0 .. 1                                         */
    int32_t  gear;       /* -1 reverse, 0 neutral, 1..5 forward             */
    uint32_t gates;      /* raw shifter-gate bitmask, for rebinding         */
    int32_t  dpad_x;     /* D-pad: -1 left, 0, +1 right. ABS_HAT0X, the HID
                            hat a Logitech wheel reports under Linux -- an
                            UNVERIFIED default for the G923, like the gate
                            codes (task 12). Drives the glance (task 17).  */
    uint64_t seq;        /* ++ per SYN_REPORT; lets a consumer detect churn */
} WheelState;

/* --- Capability report ------------------------------------------------- */

typedef struct {
    int      code;          /* evdev ABS_* code, or -1 if the axis is absent */
    int32_t  min, max;      /* from EVIOCGABS -- ranges differ per SKU/mode   */
    int32_t  flat, fuzz;    /* kernel-declared deadzone / noise              */
    int      inverted;      /* 1 => raw maximum means "released"             */
} WheelAxisCaps;

typedef struct {
    char     path[64];      /* /dev/input/eventN                            */
    char     name[128];     /* EVIOCGNAME                                   */
    uint16_t vendor, product, version;
    int      writable;      /* opened O_RDWR? required for any FFB          */

    WheelAxisCaps steer, throttle, brake, clutch;

    int      has_ff;            /* EV_FF present at all                     */
    int      ff_constant;       /* FF_CONSTANT  -- the one we actually need */
    int      ff_spring;         /* FF_SPRING    -- hidpp only               */
    int      ff_damper;         /* FF_DAMPER    -- hidpp only               */
    int      ff_autocenter;     /* FF_AUTOCENTER                            */
    int      ff_slots;          /* EVIOCGEFFECTS: simultaneous effects      */

    int      gate_count;        /* shifter gates detected                   */
    int      range_deg;         /* lg4ff sysfs `range`, or -1 if unavailable */
} WheelCaps;

/* --- Lifetime ---------------------------------------------------------- */

typedef struct WheelDevice WheelDevice;

/* Opens the wheel. `path` may be NULL to auto-discover by scanning
 * /dev/input/event*. Returns 0 on success; on failure fills `err` and returns
 * a negative errno-ish code. Tries O_RDWR first (needed for FFB) and falls
 * back to O_RDONLY so that a read-only probe still works. */
int  wheel_open(WheelDevice** out, const char* path, char* err, size_t errsz);
void wheel_close(WheelDevice* d);

const WheelCaps* wheel_caps(const WheelDevice* d);

/* Drains every pending event without blocking. The fd is O_NONBLOCK, so this
 * returns promptly even with nothing queued -- safe to call at 1 kHz.
 * Returns the number of complete SYN_REPORT frames consumed, or <0 on a fatal
 * read error (device unplugged). `out` always holds the newest state. */
int  wheel_poll(WheelDevice* d, WheelState* out);


/* TESTING HOOK -- wraps an already-open fd (a pipe, a socketpair, a uinput
 * clone) with a caller-supplied capability set. This exists so the event
 * decoder -- axis normalisation, pedal inversion, gate->gear resolution -- can
 * be exercised with no hardware and no root, which is the only way any of it
 * could be verified on this machine: no wheel has ever been attached to it.
 * Takes ownership of `fd`. Not used on any production path. */
int  wheel_open_raw(WheelDevice** out, int fd, const WheelCaps* caps);

/* --- Force feedback ---------------------------------------------------- */

/* Uploads one FF_CONSTANT effect and starts it with infinite replay, disables
 * the wheel's own autocentre spring, and sets gain to full. After this,
 * wheel_ffb_set() only has to rewrite the magnitude.
 * Returns 0, or <0 with `err` filled (e.g. no FF_CONSTANT on this bind path). */
int  wheel_ffb_init(WheelDevice* d, double peak_torque_nm, char* err, size_t errsz);

/* Sets the commanded steering-wheel torque in newton-metres. Positive = the
 * wheel is pushed CLOCKWISE, matching WheelState::steer. Values beyond the
 * device peak are soft-clipped, not wrapped.
 *
 * Cheap by design: the value is quantised to the device's 16-bit scale and the
 * EVIOCSFF ioctl is SKIPPED when the quantised level has not moved past the
 * deadband. See wheel_ffb_set_rate() for why that matters.
 * Returns 1 if an ioctl was issued, 0 if suppressed, <0 on error. */
int  wheel_ffb_set(WheelDevice* d, double torque_nm);

/* Rate limiting. `divisor` = issue at most one ioctl every N calls (default 4:
 * called at 1 kHz that is 250 Hz on the wire). `deadband` = minimum change in
 * quantised units, out of 32767, to bother sending (default 64 ~ 0.2%).
 *
 * WHY: the physics thread produces a new torque every 1 ms, but each EVIOCSFF
 * becomes a USB interrupt-OUT transfer. A full-speed HID endpoint polls on a
 * 1-4 ms interval, so commanding faster than it can drain just queues work in
 * the driver. Compute at 1 kHz, transmit at 250 Hz. */
void wheel_ffb_set_rate(WheelDevice* d, int divisor, int deadband);

int  wheel_ffb_stop(WheelDevice* d);

/* Number of EVIOCSFF calls actually issued, and their total cost in
 * microseconds -- this is what decides whether FFB may share the RT thread. */
void wheel_ffb_stats(const WheelDevice* d, uint64_t* out_calls, double* out_total_us);

/* lg4ff exposes the wheel's mechanical rotation range in sysfs. Setting it to
 * the vehicle's real lock-to-lock keeps the mapping 1:1 instead of scaled.
 * Returns 0 on success, <0 if the attribute is not present (hidpp path). */
int  wheel_set_range(WheelDevice* d, int degrees);

#ifdef __cplusplus
}
#endif
#endif /* WHEEL_DEVICE_H */
