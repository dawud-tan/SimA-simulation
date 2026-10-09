#!/usr/bin/env python3
"""Measures a -ChronoMirrorProbe run: are the markers where the geometry says?

UChronoMirrorComponent (with -ChronoMirrorProbe) floats two coloured markers
behind the car, each on the ray from the reflected eye through a chosen,
off-centre point of the glass. By the law of reflection the driver sees that
marker AT that glass point, so two predictions follow, and the component logs
both from the frame it dumps:

  * in the mirror's render target, the marker sits at a known (u, v);
  * in the screenshot, its reflection sits ON the glass, at the glass point
    projected through the player's camera (only for the mirror the
    -ChronoLookMirror view is aimed at).

This finds each marker's blob in the dumped target and in the screenshot and
reports the error in pixels. One number per marker covers the frustum, the
capture's orientation, the clip plane, the glass's UV swap and both flips --
which a wrong answer in any of them moves by tens to hundreds of pixels.

Pure Python: no PIL on this machine, so PNG is decoded here (8-bit RGB/RGBA,
non-interlaced -- what UE's ExportRenderTarget and HighResShot write).

usage: probe.py --log SimA.log --dumps DIR [--shot PNG]
exit status 0 only if every marker is found and within tolerance.
"""
import argparse
import os
import re
import struct
import sys
import zlib

# Tolerances in pixels. A correct mirror puts an unlit marker's centroid
# within a pixel or two of the prediction; any of the faults this exists to
# catch moves it by a large fraction of the image (a wrong U flip moves the
# magenta marker by 55% of the target width, ~280 px).
TOL_TARGET_PX = 3.0
TOL_SCREEN_PX = 6.0
MIN_BLOB_PX = 12          # fewer pixels than this is not a marker


def read_png(path):
    data = open(path, "rb").read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("%s: not a PNG" % path)
    pos, idat, width = 8, b"", None
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        if kind == b"IHDR":
            width, height, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            if depth != 8 or ctype not in (2, 6) or interlace:
                raise ValueError("%s: only 8-bit RGB/RGBA non-interlaced PNG "
                                 "(got depth %d type %d interlace %d)"
                                 % (path, depth, ctype, interlace))
            channels = 3 if ctype == 2 else 4
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            break
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = width * channels
    pixels = bytearray(height * stride)
    prev = bytearray(stride)
    src = 0
    for y in range(height):
        ftype = raw[src]
        line = bytearray(raw[src + 1:src + 1 + stride])
        src += 1 + stride
        for i in range(stride):
            a = line[i - channels] if i >= channels else 0
            b = prev[i]
            c = prev[i - channels] if i >= channels else 0
            if ftype == 1:
                line[i] = (line[i] + a) & 0xFF
            elif ftype == 2:
                line[i] = (line[i] + b) & 0xFF
            elif ftype == 3:
                line[i] = (line[i] + ((a + b) >> 1)) & 0xFF
            elif ftype == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pred = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pred) & 0xFF
        pixels[y * stride:(y + 1) * stride] = line
        prev = line
    return width, height, channels, pixels


def is_colour(name, r, g, b):
    """Hue tests loose enough to survive tonemapping and filtering, strict
    enough that nothing else in the scene passes: the course is grey road,
    white lines, orange cones, a blue car and a black sky."""
    if name == "magenta":
        return r >= 90 and b >= 90 and g <= 0.5 * min(r, b)
    if name == "green":
        return g >= 90 and r <= 0.5 * g and b <= 0.5 * g
    raise ValueError(name)


def find_blob(img, colour, box=None):
    """Largest 4-connected blob of `colour`, optionally only inside `box`
    (x0, y0, x1, y1); returns (cx, cy, npix) with the centroid in continuous
    image coordinates (pixel i spans [i, i+1)), or None."""
    w, h, ch, px = img
    x_lo, y_lo, x_hi, y_hi = 0, 0, w, h
    if box is not None:
        x_lo, y_lo = max(0, int(box[0])), max(0, int(box[1]))
        x_hi, y_hi = min(w, int(box[2]) + 1), min(h, int(box[3]) + 1)
    mask = bytearray(w * h)
    for y in range(y_lo, y_hi):
        row = y * w * ch
        for x in range(x_lo, x_hi):
            i = row + x * ch
            if is_colour(colour, px[i], px[i + 1], px[i + 2]):
                mask[y * w + x] = 1
    best = None
    for start in range(w * h):
        if mask[start] != 1:
            continue
        mask[start] = 2
        stack, members = [start], []
        while stack:
            k = stack.pop()
            members.append(k)
            x, y = k % w, k // w
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h and mask[ny * w + nx] == 1:
                    mask[ny * w + nx] = 2
                    stack.append(ny * w + nx)
        if best is None or len(members) > len(best):
            best = members
    if not best or len(best) < MIN_BLOB_PX:
        return None
    cx = sum(k % w for k in best) / len(best) + 0.5
    cy = sum(k // w for k in best) / len(best) + 0.5
    return cx, cy, len(best)


def parse_log(path):
    probes, look, ndc = [], None, []
    pat = re.compile(
        r"MirrorProbe mirror=(\S+) marker=(\S+) target_uv=([-\d.]+),([-\d.]+) "
        r"target_size=(\d+),(\d+) screen=([-\d.]+),([-\d.]+) onscreen=(\d) "
        r"viewport=(\d+),(\d+) glass_box=([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)")
    for line in open(path, errors="replace"):
        m = pat.search(line)
        if m:
            g = m.groups()
            probes.append(dict(mirror=g[0], marker=g[1],
                               u=float(g[2]), v=float(g[3]),
                               tw=int(g[4]), th=int(g[5]),
                               sx=float(g[6]), sy=float(g[7]),
                               onscreen=g[8] == "1",
                               vw=int(g[9]), vh=int(g[10]),
                               box=tuple(float(v) for v in g[11:15])))
            continue
        m = re.search(r"MirrorLook index=(\d+) mirror=(\S+)", line)
        if m:
            look = m.group(2)
            continue
        m = re.search(r"Mirror '(\w+)': corner NDC .*\| worst error ([\d.eE+-]+)"
                      r"\s+glass ndc z ([\d.eE+-]+)\s+(.*)$", line)
        if m:
            ndc.append((m.group(1), float(m.group(2)), float(m.group(3)),
                        m.group(4).strip()))
    return probes, look, ndc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log", required=True)
    ap.add_argument("--dumps", required=True)
    ap.add_argument("--shot")
    a = ap.parse_args()

    probes, look, ndc = parse_log(a.log)
    failures = 0

    print("runtime frustum check (CheckCornersLandOnNdcEdges):")
    if not ndc:
        print("  FAIL  no 'corner NDC' lines in the log")
        failures += 1
    for name, err, glass_z, verdict in ndc:
        # Corners on their own NDC corners AND the glass clipped by the near
        # plane (reversed-Z > 1). The verdict string encodes both.
        good = verdict == "FRUSTUM IS THE APERTURE"
        failures += 0 if good else 1
        print("  %s  %-15s worst error %.2e  glass ndc z %.5f  %s"
              % ("ok  " if good else "FAIL", name, err, glass_z, verdict))

    if not probes:
        print("FAIL  no MirrorProbe lines in %s -- was -ChronoMirrorProbe passed?" % a.log)
        return 1

    print("\nmarkers in each render target (pixels; prediction from the glass geometry):")
    targets = {}
    for p in probes:
        path = os.path.join(a.dumps, p["mirror"] + ".png")
        if p["mirror"] not in targets:
            targets[p["mirror"]] = read_png(path) if os.path.exists(path) else None
        img = targets[p["mirror"]]
        ex, ey = p["u"] * p["tw"], p["v"] * p["th"]
        if img is None:
            print("  FAIL  %-15s %-7s no dump at %s" % (p["mirror"], p["marker"], path))
            failures += 1
            continue
        if (img[0], img[1]) != (p["tw"], p["th"]):
            ex, ey = p["u"] * img[0], p["v"] * img[1]
        blob = find_blob(img, p["marker"])
        if blob is None:
            print("  FAIL  %-15s %-7s expected (%6.1f, %5.1f)  NOT FOUND"
                  % (p["mirror"], p["marker"], ex, ey))
            failures += 1
            continue
        err = ((blob[0] - ex) ** 2 + (blob[1] - ey) ** 2) ** 0.5
        good = err <= TOL_TARGET_PX
        failures += 0 if good else 1
        print("  %s  %-15s %-7s expected (%6.1f, %5.1f)  found (%6.1f, %5.1f)  "
              "error %5.2f px  [%d px blob]"
              % ("ok  " if good else "FAIL", p["mirror"], p["marker"], ex, ey,
                 blob[0], blob[1], err, blob[2]))

    if a.shot:
        print("\nmarkers on the glass in the screenshot (the mirror being looked at):")
        if look is None:
            print("  FAIL  no 'MirrorLook' line -- was -ChronoLookMirror passed?")
            failures += 1
        else:
            shot = read_png(a.shot)
            for p in (q for q in probes if q["mirror"] == look):
                kx = shot[0] / max(p["vw"], 1)
                ky = shot[1] / max(p["vh"], 1)
                sx, sy = p["sx"] * kx, p["sy"] * ky
                if not p["onscreen"] or p["box"][2] < 0:
                    print("  FAIL  %-15s %-7s glass is off screen" % (look, p["marker"]))
                    failures += 1
                    continue
                # Only ON THIS GLASS: another mirror's glass can share the
                # frame and carries its own markers in the same colours.
                box = (p["box"][0] * kx - 2, p["box"][1] * ky - 2,
                       p["box"][2] * kx + 2, p["box"][3] * ky + 2)
                blob = find_blob(shot, p["marker"], box)
                if blob is None:
                    print("  FAIL  %-15s %-7s expected (%6.1f, %6.1f)  NOT FOUND"
                          % (look, p["marker"], sx, sy))
                    failures += 1
                    continue
                err = ((blob[0] - sx) ** 2 + (blob[1] - sy) ** 2) ** 0.5
                good = err <= TOL_SCREEN_PX
                failures += 0 if good else 1
                print("  %s  %-15s %-7s expected (%6.1f, %6.1f)  found (%6.1f, %6.1f)  "
                      "error %5.2f px  [%d px blob]"
                      % ("ok  " if good else "FAIL", look, p["marker"], sx, sy,
                         blob[0], blob[1], err, blob[2]))

    print("\n%s" % ("PROBE PASSED" if failures == 0 else "PROBE FAILED (%d)" % failures))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
