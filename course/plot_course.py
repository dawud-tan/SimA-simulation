#!/usr/bin/env python3
# ============================================================================
#  plot_course.py — ASCII top-down view of simA_course.json.
#
#  Exists because the alternative is launching the editor, and CLAUDE.md is
#  explicit that a headless UE run takes over twenty minutes to reach its first
#  Tick. A layout mistake -- a bay on the wrong side, a gate facing the wrong
#  way, an exercise overlapping the next one -- is obvious in a schematic and
#  invisible in a JSON diff.
#
#  Chrono frame: X forward (rightwards here), Y LEFT (upwards here). So the
#  picture is what a driver would see looking down at the course from above
#  with the start on the left. Indonesia drives on the left, so kerbside
#  exercises appear at the TOP.
# ============================================================================

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = json.load(open(os.path.join(HERE, "simA_course.json")))

COLS = int(sys.argv[1]) if len(sys.argv) > 1 else 118

xs, ys = [], []
for c in DOC["cones"]:
    xs.append(c["x"]); ys.append(c["y"])
for l in DOC["lines"]:
    xs += [l["x0"], l["x1"]]; ys += [l["y0"], l["y1"]]
for g in DOC["gates"]:
    xs += [g["x0"], g["x1"]]; ys += [g["y0"], g["y1"]]
for z in DOC["zones"]:
    for px, py in z["poly"]:
        xs.append(px); ys.append(py)
# The start pose is upstream of every gate, so leaving it out of the extents
# silently cropped it off the left edge -- the plot looked fine and was missing
# the one element that says where the car begins.
xs.append(DOC["start"]["x"]); ys.append(DOC["start"]["y"])

x0, x1 = min(xs) - 2.0, max(xs) + 2.0
y0, y1 = min(ys) - 1.0, max(ys) + 1.0
ROWS = 30

sx = (COLS - 1) / (x1 - x0)
sy = (ROWS - 1) / (y1 - y0)

grid = [[" "] * COLS for _ in range(ROWS)]


def put(x, y, ch, over=False):
    """Y is inverted: +Y (left of the car) draws upward."""
    c = int(round((x - x0) * sx))
    r = ROWS - 1 - int(round((y - y0) * sy))
    if 0 <= r < ROWS and 0 <= c < COLS:
        if over or grid[r][c] == " ":
            grid[r][c] = ch


def seg(xa, ya, xb, yb, ch):
    n = max(2, int(max(abs(xb - xa) * sx, abs(yb - ya) * sy) * 2) + 1)
    for i in range(n + 1):
        t = i / n
        put(xa + (xb - xa) * t, ya + (yb - ya) * t, ch)


for l in DOC["lines"]:
    seg(l["x0"], l["y0"], l["x1"], l["y1"],
        {"kerb": "=", "bay_edge": ":", "bay_end": "_",
         "aisle_edge": "-"}.get(l["kind"], "."))

for z in DOC["zones"]:
    p = z["poly"]
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        seg(a[0], a[1], b[0], b[1], "#")

for g in DOC["gates"]:
    seg(g["x0"], g["y0"], g["x1"], g["y1"], "|")

for c in DOC["cones"]:
    put(c["x"], c["y"], "o", over=True)

for p in DOC["patches"]:
    if p["kind"] == "ramp":
        for i in range(12):
            t = i / 11.0
            put(p["x0"] + (p["x1"] - p["x0"]) * t, 0.0, "/", over=True)

s = DOC["start"]
put(s["x"], s["y"], "S", over=True)

print(f"SIM A course, {DOC['meta']['course_length_m']:.1f} m along +X")
print(f"x {x0:.1f} .. {x1:.1f} m   y {y0:.1f} .. {y1:.1f} m   "
      f"({(x1-x0)/COLS:.2f} m/col, {(y1-y0)/ROWS:.2f} m/row -- aspect is NOT 1:1)")
print("+" + "-" * COLS + "+")
for r in grid:
    print("|" + "".join(r) + "|")
print("+" + "-" * COLS + "+")
print("S start   o cone   | gate   = kerb   # zone   : bay edge   "
      "- aisle edge   / ramp")
print("up = +Y = the driver's LEFT = kerbside (Indonesia drives on the left)")

print("\nfeasibility (derived minimum vs configured):")
d, c = DOC["derived_minima"], DOC["configured"]
# The sense of each bound is NOT uniform, and treating it as though it were is
# how this table first reported a 12% hill against a 42% traction limit as
# "IMPOSSIBLE". Three of these are floors the course must clear; the hill grade
# is a CEILING the course must stay under. Carry the direction explicitly.
rows = [
    ("slalom cone spacing",  "min", d["slalom_cone_spacing"],  c["slalom_cone_spacing"]),
    ("parallel slot length", "min", d["parallel_slot_length"], c["parallel_slot_length"]),
    ("bay aisle width",      "min", d["bay_aisle_width"],      c["bay_aisle_width"]),
    ("hill grade (traction)","max", d["hill_grade_traction_limit"], c["hill_grade"]),
]
print(f"  {'dimension':<23} {'bound':>5} {'limit':>8} {'set':>8} {'margin':>8}")
for name, sense, lim, cf in rows:
    if sense == "min":
        ok, margin = cf >= lim, cf - lim
    else:
        ok, margin = cf <= lim, lim - cf
    print(f"  {name:<23} {sense:>5} {lim:>8.3f} {cf:>8.3f} {margin:>+8.3f}  "
          f"{'ok' if ok else 'OUT OF RANGE'}")
