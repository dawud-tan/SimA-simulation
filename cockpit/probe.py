#!/usr/bin/env python3
"""Measures task 9's cockpit frames. Three checks, one per subcommand:

  wheel   A -ChronoLookWheel frame: the steering wheel seen down its column
          from the driver's side. The vehicle logs (WheelProbe) the input,
          the rim angle it commands -- input x the pinion's full-lock angle,
          clockwise as the driver sees it -- and where the rim's centre and
          its UNTURNED 12 and 3 o'clock land on screen. This walks the rim's
          circle, finds the pale stripe by its brightness, and reports its
          angle clockwise from 12. A sign error puts it 245 deg off at steer
          0.2; any scale error but a tiny one, tens of degrees.

  seat    The driver's forward view with the HUD (-ChronoShotUI): no panel
          over any mirror on screen (the HUD measures that itself and logs
          "HUD over mirrors"), and no void -- no black just below the
          horizon, where the ground ran out before the scenery existed, and
          none in any mirror's render target.

  glance  A -ChronoShotUI frame of driver/scripts/glance.txt (task 17): the
          view turned to the left door mirror. The vehicle logs the glance
          at the shot (GlanceProbe); the HUD's "HUD over mirrors" line says
          where that glass landed. Fully turned, the glass must be centred
          and no panel may cover it. The rays from the eye to the glass are
          the ones `mirror/run.sh render` already found unobstructed: a
          glance turns the head, it does not move the eye.

  frames  The rendered frame time from the subsystem's 5 s health lines,
          after the session start, against the 40 ms the session start needs;
          and its split as `stat unit` makes it -- game, render and RHI
          thread, and the GPU. The slowest of the four sets the frame.

  gpu     One frame's GPU time pass by pass (-ChronoProfileGPU runs UE's
          ProfileGPU): every pass over 0.3 ms, inclusive, nested.

usage: probe.py wheel  --log SimA.log --shot PNG
       probe.py glance --log SimA.log
       probe.py seat   --log SimA.log --dumps DIR --shot PNG
       probe.py frames --log SimA.log
       probe.py gpu    --log SimA.log
exit status 0 only if the check passes.
"""
import argparse
import importlib.util
import math
import os
import re
import sys

# The PNG decoder is mirror/probe.py's (pure Python; no PIL on this machine).
# Loaded by path: both files are called probe.py.
_spec = importlib.util.spec_from_file_location(
    "mirror_probe", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mirror", "probe.py"))
_mirror_probe = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mirror_probe)
read_png = _mirror_probe.read_png

TOL_WHEEL_DEG = 3.0       # the stripe is one 15 deg segment of 24
TOL_GLANCE_PX = 4.0       # glass box centre from the frame centre
DARK = 16                 # max channel below this is black
MAX_DARK_FRACTION = 0.01
FRAME_LIMIT_MS = 40.0     # the session start's steady-frame threshold


def luminance(img, x, y):
    w, h, ch, px = img
    xi, yi = int(round(x)), int(round(y))
    if not (0 <= xi < w and 0 <= yi < h):
        return None
    i = (yi * w + xi) * ch
    return 0.2126 * px[i] + 0.7152 * px[i + 1] + 0.0722 * px[i + 2]


def dark_fraction(img, y0=None, y1=None):
    w, h, ch, px = img
    y0, y1 = (0 if y0 is None else y0), (h if y1 is None else y1)
    dark = 0
    for y in range(y0, y1):
        row = y * w * ch
        for x in range(w):
            i = row + x * ch
            if max(px[i], px[i + 1], px[i + 2]) < DARK:
                dark += 1
    return dark / float(w * (y1 - y0))


def last_line(log, tag, before_shot=False):
    found = None
    for line in open(log, errors="replace"):
        if before_shot and "Taking screenshot" in line:
            break                      # the state the shot was taken in
        if tag in line:
            found = line
    return found


def hud_line(log):
    """The HUD's mirror report for the frame the shot took, or failing that
    the last one before it. A report is otherwise logged only when a mirror
    goes on or off screen or under a panel, which can be mid-glance."""
    return last_line(log, "HUD over mirrors at the shot") or \
        last_line(log, "HUD over mirrors", before_shot=True)


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def check_wheel(args):
    line = last_line(args.log, "WheelProbe")
    if not line:
        print("FAIL  no WheelProbe line in %s" % args.log)
        return 1
    kv = dict(re.findall(r"(\w+)=([-\d.,]+)", line))
    pt = lambda k: tuple(float(v) for v in kv[k].split(","))
    steer, full, expect = float(kv["steer"]), float(kv["full_lock_deg"]), float(kv["expect_deg"])
    c, t12, t3 = pt("centre"), pt("twelve"), pt("three")
    if kv.get("onscreen") != "1":
        print("FAIL  the wheel is not on screen")
        return 1
    u12 = (t12[0] - c[0], t12[1] - c[1])
    u3 = (t3[0] - c[0], t3[1] - c[1])
    r = math.hypot(*u12)
    u12 = (u12[0] / r, u12[1] / r)
    u3 = (u3[0] / math.hypot(*u3), u3[1] / math.hypot(*u3))
    # Screen y points down: 3 o'clock is clockwise of 12 on screen exactly
    # when this is positive -- the view is the driver's, not the dashboard's.
    handed = u12[0] * u3[1] - u12[1] * u3[0]
    print("wheel view: centre (%.1f, %.1f) px, rim radius %.1f px, 3 o'clock %s of 12 on screen"
          % (c[0], c[1], r, "clockwise" if handed > 0 else "ANTICLOCKWISE"))
    if handed <= 0:
        print("FAIL  the view is not from the driver's side")
        return 1

    img = read_png(args.shot)
    # Brightness round the rim's centreline, every half degree, clockwise
    # from 12. Three radii well inside the tube (its half-thickness is
    # 0.09 of the radius) so the samples are rim and nothing behind it.
    step = 0.5
    prof = []
    for k in range(int(360 / step)):
        a = math.radians(k * step)
        d = (math.cos(a) * u12[0] + math.sin(a) * u3[0], math.cos(a) * u12[1] + math.sin(a) * u3[1])
        vals = [luminance(img, c[0] + d[0] * r * f, c[1] + d[1] * r * f) for f in (0.96, 1.0, 1.04)]
        if any(v is None for v in vals):
            print("FAIL  the rim runs off the frame at %.1f deg" % (k * step))
            return 1
        prof.append(sum(vals) / 3.0)
    base = sorted(prof)[len(prof) // 2]
    peak = max(prof)
    if peak < 3.0 * max(base, 1.0):
        print("FAIL  no stripe: brightest %.1f against a rim of %.1f" % (peak, base))
        return 1
    cut = 0.5 * (base + peak)
    on = [prof[k] > cut for k in range(len(prof))]
    # One contiguous arc, wherever it wraps: count the rising edges.
    rises = sum(1 for k in range(len(on)) if on[k] and not on[k - 1])
    if rises != 1:
        print("FAIL  the bright part of the rim is %d arcs, not one stripe" % rises)
        return 1
    angles = [k * step for k in range(len(on)) if on[k]]
    sx = sum(math.sin(math.radians(a)) for a in angles)
    cx = sum(math.cos(math.radians(a)) for a in angles)
    found = math.degrees(math.atan2(sx, cx))
    width = len(angles) * step
    err = wrap(found - expect)
    ok = abs(err) <= TOL_WHEEL_DEG
    print("stripe: %.1f deg wide, brightness %.0f against a rim of %.0f" % (width, peak, base))
    print("%s  steer %+.4f x %.1f deg = expected %+.2f deg clockwise; found %+.2f; error %+.2f deg"
          % ("ok   " if ok else "FAIL ", steer, full, wrap(expect), found, err))
    print("WHEEL MEASURED CORRECT" if ok else "WHEEL MEASUREMENT FAILED")
    return 0 if ok else 1


def check_seat(args):
    rc = 0
    line = hud_line(args.log)
    if not line:
        print("FAIL  no 'HUD over mirrors' line in %s" % args.log)
        return 1
    seen = re.findall(r" (\w+) at \(([-\d]+),([-\d]+)\)-\(([-\d]+),([-\d]+)\) ([\d.]+)% covered", line)
    off = re.findall(r" (\w+) off-screen", line)
    print("HUD against the mirrors in the forward view:")
    for name, x0, y0, x1, y1, pct in seen:
        ok = float(pct) == 0.0
        rc |= 0 if ok else 1
        print("  %s  %-15s glass (%s,%s)-(%s,%s) px, %s%% under a panel"
              % ("ok  " if ok else "FAIL", name, x0, y0, x1, y1, pct))
    for name in off:
        print("  --    %-15s off screen (a head turn away)" % name)
    if "MirrorInterior" not in [s[0] for s in seen]:
        print("  FAIL  the interior mirror is not in the forward view")
        rc = 1

    print("the void (max channel < %d):" % DARK)
    for name in sorted(os.listdir(args.dumps)):
        if name.endswith(".png"):
            f = dark_fraction(read_png(os.path.join(args.dumps, name)))
            ok = f < MAX_DARK_FRACTION
            rc |= 0 if ok else 1
            print("  %s  %-22s %.2f%% black" % ("ok  " if ok else "FAIL", name, 100.0 * f))
    # The view is level (the car sits at -0.05 deg, task 14), so the horizon
    # is the centre row. The ground ran out just below it: 129 m ahead is
    # 0.6 deg down, the patch's side 20 m off at the frame's edge ~3 deg.
    img = read_png(args.shot)
    h = img[1]
    y0, y1 = h // 2 + 1, h // 2 + 1 + int(round(40 * h / 720.0))
    f = dark_fraction(img, y0, y1)
    ok = f < MAX_DARK_FRACTION
    rc |= 0 if ok else 1
    print("  %s  rows %d-%d below the horizon %.2f%% black" % ("ok  " if ok else "FAIL", y0, y1 - 1, 100.0 * f))
    print("SEAT VIEW MEASURED CORRECT" if rc == 0 else "SEAT VIEW MEASUREMENT FAILED")
    return rc


def check_glance(args):
    line = last_line(args.log, "GlanceProbe")
    hud = hud_line(args.log)
    if not line or not hud:
        print("FAIL  no %s line in %s" % ("GlanceProbe" if not line else "'HUD over mirrors'", args.log))
        return 1
    m = re.search(r"glance=([-\d.]+) eased=([-\d.]+) toward=(\w+)", line)
    g, e, toward = float(m.group(1)), float(m.group(2)), m.group(3)
    vw = re.search(r"panels, (\d+)x(\d+)\)", hud)
    cx, cy = float(vw.group(1)) / 2.0, float(vw.group(2)) / 2.0
    print("glance %+.4f, eased %+.4f, toward %s" % (g, e, toward))
    rc = 0
    if abs(e) < 0.9999 or toward == "ahead":
        print("FAIL  the head is not fully turned at the shot")
        return 1
    box = re.search(r" %s at \(([-\d]+),([-\d]+)\)-\(([-\d]+),([-\d]+)\) ([\d.]+)%% covered"
                    % toward, hud)
    if not box:
        print("FAIL  %s is not on screen while glancing at it" % toward)
        return 1
    x0, y0, x1, y1, pct = (float(v) for v in box.groups())
    off = ((0.5 * (x0 + x1) - cx) ** 2 + (0.5 * (y0 + y1) - cy) ** 2) ** 0.5
    for ok, text in ((off <= TOL_GLANCE_PX,
                      "%s glass (%.0f,%.0f)-(%.0f,%.0f) px, centre %.1f px from the frame's"
                      % (toward, x0, y0, x1, y1, off)),
                     (pct == 0.0, "%.1f%% of it under a HUD panel" % pct)):
        rc |= 0 if ok else 1
        print("%s  %s" % ("ok  " if ok else "FAIL", text))
    print("GLANCE MEASURED CORRECT" if rc == 0 else "GLANCE MEASUREMENT FAILED")
    return rc


def check_frames(args):
    started, windows, shot = False, [], False
    for line in open(args.log, errors="replace"):
        if "Session started" in line:
            started = True
            continue
        if "Taking screenshot" in line:
            shot = True
        m = re.search(r"frame ([\d.]+)/([\d.]+) ms(?: \(mean/max\) = game ([\d.]+) "
                      r"render ([\d.]+) rhi ([\d.]+) gpu ([\d.]+))?", line)
        if started and m and not shot:
            windows.append(tuple(float(v) if v else None for v in m.groups()))
    # The first window began before the session start and holds its frames.
    windows = windows[1:]
    if not windows:
        print("FAIL  no health lines after the session start")
        return 1
    means = [w[0] for w in windows]
    mean = sum(means) / len(means)
    worst = max(w[1] for w in windows)
    ok = max(means) < FRAME_LIMIT_MS
    print("%d windows of 5 s after the session start: frame mean %.2f ms "
          "(windows %.2f-%.2f), max %.2f ms"
          % (len(windows), mean, min(means), max(means), worst))
    if all(w[2] is not None for w in windows):
        split = [sum(w[k] for w in windows) / len(windows) for k in range(2, 6)]
        names = ("game", "render", "rhi", "gpu")
        print("split, mean: game %.2f  render %.2f  rhi %.2f  gpu %.2f ms -> %s-bound"
              % tuple(split + [names[split.index(max(split))]]))
    print("%s  every window's mean under the session start's %.0f ms"
          % ("ok  " if ok else "FAIL", FRAME_LIMIT_MS))
    return 0 if ok else 1


def check_gpu(args):
    lines = open(args.log, errors="replace").read().split("\n")
    start = [i for i, l in enumerate(lines) if "GPU Profile for Frame" in l and "Graphics" in l]
    if not start:
        print("FAIL  no ProfileGPU output -- was -ChronoProfileGPU passed?")
        return 1
    # Each row: exclusive ... ms, inclusive ... ms, then the pass name,
    # indented three columns per level.
    row = re.compile(r"\u2502\s*([\d.]+) ms \u2503.*\u2502\s*([\d.]+) ms \u2503 (\s*)(.*?)\s*\u2503\s*$")
    print("one frame on the GPU, passes over 0.3 ms (inclusive ms):")
    for line in lines[start[0] + 1:]:
        if "GPU Profile for Frame" in line:
            break
        m = row.search(line)
        if m and float(m.group(2)) >= 0.3 and len(m.group(3)) // 3 <= 8:
            print("  %7.3f  %s%s" % (float(m.group(2)), "  " * (len(m.group(3)) // 3), m.group(4)[:90]))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("check", choices=("wheel", "seat", "glance", "frames", "gpu"))
    ap.add_argument("--log", required=True)
    ap.add_argument("--shot")
    ap.add_argument("--dumps")
    args = ap.parse_args()
    return {"wheel": check_wheel, "seat": check_seat, "glance": check_glance,
            "frames": check_frames,
            "gpu": check_gpu}[args.check](args)


if __name__ == "__main__":
    sys.exit(main())
