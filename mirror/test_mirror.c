/* ===========================================================================
 *  test_mirror.c — offline self-test for the planar-mirror geometry.
 *
 *  No GPU, no editor, no content. Compiles the SAME mirror_math.cpp the plugin
 *  ships.
 *
 *  The central assertion is not about the matrix ENTRIES, which are easy to
 *  get plausibly wrong, but about what the matrix DOES: the mirror's own four
 *  corners must land exactly on the edges of normalised device coordinates. If
 *  they do, the frustum is the mirror aperture and nothing outside it is being
 *  rendered. That is the whole efficiency argument for an off-axis projection,
 *  and it is checkable without drawing a pixel.
 * ======================================================================== */

#define _GNU_SOURCE
#include "mirror_math.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;

static void ok(int cond, const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    if (cond) { ++g_pass; printf("  ok   "); } else { ++g_fail; printf("  FAIL "); }
    vprintf(fmt, ap); printf("\n"); va_end(ap);
}
static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }
static MmVec3 V(double x, double y, double z) { MmVec3 v = { x, y, z }; return v; }
static MmVec3 add(MmVec3 a, MmVec3 b) { return V(a.x + b.x, a.y + b.y, a.z + b.z); }
static MmVec3 sub(MmVec3 a, MmVec3 b) { return V(a.x - b.x, a.y - b.y, a.z - b.z); }
static MmVec3 scl(MmVec3 a, double s) { return V(a.x * s, a.y * s, a.z * s); }
static double dot(MmVec3 a, MmVec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static MmVec3 crs(MmVec3 a, MmVec3 b)
{
    return V(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static MmVec3 unit(MmVec3 a) { return scl(a, 1.0 / sqrt(dot(a, a))); }

static void test_reflection(void)
{
    printf("reflection\n");

    const MmVec3 p0 = V(0, 0, 0);
    const MmVec3 n  = V(0, 1, 0);          /* plane y = 0 */

    const MmVec3 p  = V(3, 2, -1);
    const MmVec3 r  = mm_reflect_point(p, p0, n);
    ok(near(r.x, 3, 1e-12) && near(r.y, -2, 1e-12) && near(r.z, -1, 1e-12),
       "point reflects across y=0 (%.3f, %.3f, %.3f)", r.x, r.y, r.z);

    /* Reflection is an INVOLUTION. If this ever fails the mirror will drift
     * further from the truth every frame it is recomputed. */
    const MmVec3 rr = mm_reflect_point(r, p0, n);
    ok(near(rr.x, p.x, 1e-12) && near(rr.y, p.y, 1e-12) && near(rr.z, p.z, 1e-12),
       "reflecting twice is the identity");

    const MmVec3 on = V(5, 0, 7);
    const MmVec3 ron = mm_reflect_point(on, p0, n);
    ok(near(ron.y, 0, 1e-12) && near(ron.x, 5, 1e-12),
       "a point ON the plane is its own reflection");

    ok(near(mm_plane_distance(p, p0, n), 2.0, 1e-12) &&
       near(mm_plane_distance(r, p0, n), -2.0, 1e-12),
       "distance is preserved and its sign flips");

    /* An oblique plane not through the origin -- the case a real door mirror
     * actually presents. */
    const MmVec3 q0 = V(1, 1, 0);
    const double s = 1.0 / sqrt(2.0);
    const MmVec3 qn = V(s, s, 0);
    const MmVec3 q  = V(4, 1, 2);
    const MmVec3 qr = mm_reflect_point(q, q0, qn);
    ok(near(mm_plane_distance(q, q0, qn), -mm_plane_distance(qr, q0, qn), 1e-12),
       "oblique plane off the origin: distance still flips");

    const MmVec3 d = mm_reflect_dir(V(1, -1, 0), V(0, 1, 0));
    ok(near(d.x, 1, 1e-12) && near(d.y, 1, 1e-12),
       "a direction reflects without regard to plane position");
}

static void test_projection(void)
{
    printf("\noff-axis projection\n");

    /* A mirror 0.4 x 0.2 m, centred 1.5 m ahead of the origin, facing back
     * along -X toward a viewer on the -X side.
     *
     * Corners are VIEWER-relative. Chrono is X forward, Y LEFT, so a viewer
     * looking along +X has their left hand at +Y: the lower-LEFT corner is at
     * y = +0.2, not -0.2. Ordering them by world axis instead is the mistake
     * that makes cross(pb-pa, pc-pa) face away from the eye, and the library
     * now returns -2 rather than silently rendering the mirror inside out. */
    const MmVec3 pa = V(1.5,  0.2, -0.1);      /* lower-left  (viewer)  */
    const MmVec3 pb = V(1.5, -0.2, -0.1);      /* lower-right (viewer)  */
    const MmVec3 pc = V(1.5,  0.2,  0.1);      /* upper-left  (viewer)  */
    const MmVec3 pd = V(1.5, -0.2,  0.1);      /* upper-right, implied  */

    /* THE OFF-AXIS CASE: the eye is well off the mirror's axis, laterally and
     * vertically, which is exactly a door mirror seen from the driver's seat. */
    const MmVec3 eye = V(-0.3, 0.9, 0.35);

    double m[16];
    memset(m, 0, sizeof m);
    const int rc = mm_offaxis_projection(eye, pa, pb, pc, 0.05, 500.0, m);
    ok(rc == 0, "matrix built for an off-axis eye (rc=%d)", rc);

    MmVec3 ndc;
    const int prc = mm_project_point(m, pa, &ndc);
    ok(prc == 0 && near(ndc.x, -1.0, 1e-9) && near(ndc.y, -1.0, 1e-9),
       "lower-left corner -> NDC (-1,-1)  got (%.6f, %.6f)", ndc.x, ndc.y);
    mm_project_point(m, pb, &ndc);
    ok(near(ndc.x, 1.0, 1e-9) && near(ndc.y, -1.0, 1e-9),
       "lower-right corner -> NDC (+1,-1)  got (%.6f, %.6f)", ndc.x, ndc.y);
    mm_project_point(m, pc, &ndc);
    ok(near(ndc.x, -1.0, 1e-9) && near(ndc.y, 1.0, 1e-9),
       "upper-left corner -> NDC (-1,+1)  got (%.6f, %.6f)", ndc.x, ndc.y);
    mm_project_point(m, pd, &ndc);
    ok(near(ndc.x, 1.0, 1e-9) && near(ndc.y, 1.0, 1e-9),
       "upper-right corner -> NDC (+1,+1)  got (%.6f, %.6f)", ndc.x, ndc.y);

    /* The mirror centre must land dead centre. A frustum that got the corners
     * right by luck and the middle wrong would be a skew, not a projection. */
    const MmVec3 centre = V(1.5, 0.0, 0.0);
    mm_project_point(m, centre, &ndc);
    ok(near(ndc.x, 0.0, 1e-9) && near(ndc.y, 0.0, 1e-9),
       "mirror centre -> NDC (0,0)  got (%.6f, %.6f)", ndc.x, ndc.y);

    /* A point well outside the mirror must fall outside the unit square --
     * i.e. it is genuinely clipped, not merely squashed toward the edge. */
    mm_project_point(m, V(1.5, 1.0, 0.0), &ndc);
    ok(fabs(ndc.x) > 1.0, "a point beyond the mirror edge is outside NDC (%.3f)",
       ndc.x);

    /* On-axis: the frustum should come out symmetric. Same code path, so this
     * is the sanity check that the off-axis terms vanish when they should. */
    /* Symmetry has to be checked on the EXTENTS, not on the composed matrix:
     * once P is multiplied by the basis rotation and the eye translation, the
     * skew is spread through the whole thing. Checking ms[8]/ms[9] was reading
     * unrelated entries and passing for the wrong reason. */
    double l, r, b2, t;
    const MmVec3 axis_eye = V(0.0, 0.0, 0.0);   /* on the mirror's axis */
    const int rcs = mm_offaxis_extents(axis_eye, pa, pb, pc, 0.05, &l, &r, &b2, &t);
    ok(rcs == 0 && near(l, -r, 1e-12) && near(b2, -t, 1e-12),
       "on-axis eye -> symmetric frustum (l=%.6f r=%.6f b=%.6f t=%.6f)",
       l, r, b2, t);

    double l2, r2, b3, t2;
    mm_offaxis_extents(eye, pa, pb, pc, 0.05, &l2, &r2, &b3, &t2);
    ok(fabs(l2 + r2) > 1e-6,
       "off-axis eye -> ASYMMETRIC frustum (l=%.6f r=%.6f)", l2, r2);

    /* Depth ordering must survive. Reversed near/far is a classic transposition
     * symptom and would render the mirror inside-out. */
    MmVec3 n1, n2;
    mm_project_point(m, V(1.5, 0.0, 0.0), &n1);
    mm_project_point(m, V(6.0, 0.0, 0.0), &n2);
    ok(n2.z > n1.z, "further geometry has greater NDC depth (%.4f > %.4f)",
       n2.z, n1.z);
}

/* THE TEST THAT WAS MISSING.
 *
 * Every earlier projection test fed mm_offaxis_projection an eye that was
 * already on the correct side, so it never exercised the thing the component
 * actually does: reflect the driver's eye first, then build a frustum for the
 * REFLECTED one. That is the whole mirror pipeline, and it is where the
 * winding bug lived -- three mirrors reported rc = -2 on every frame of every
 * run while all 24 assertions passed. */
static void test_full_mirror_pipeline(void)
{
    printf("\nfull mirror pipeline (reflect, then project)\n");

    /* A mirror at the origin, glass facing +X toward a driver at +X. Its width
     * axis is +Y and its height axis is +Z, so normal = right x up = +X. */
    const MmVec3 centre = V(0, 0, 0);
    const MmVec3 normal = V(1, 0, 0);
    const MmVec3 right  = V(0, 1, 0);
    const MmVec3 up     = V(0, 0, 1);

    MmVec3 pa, pb, pc;
    mm_mirror_corners(centre, normal, right, up, 0.125, 0.03, &pa, &pb, &pc);

    /* The driver, off-axis in both directions as a door mirror really is. */
    const MmVec3 eye  = V(0.6, 0.35, 0.12);
    const MmVec3 refl = mm_reflect_point(eye, centre, normal);

    ok(refl.x < 0.0 && near(refl.y, eye.y, 1e-12),
       "reflected eye crosses the plane (x %.3f -> %.3f)", eye.x, refl.x);

    /* THE REGRESSION. Corners wound toward the DRIVER would return -2 here,
     * because the reflected eye is on the far side by construction. */
    double m[16];
    memset(m, 0, sizeof m);
    const int rc = mm_offaxis_projection(refl, pa, pb, pc, 0.05, 500.0, m);
    ok(rc == 0, "projection builds for the REFLECTED eye (rc=%d, was -2)", rc);

    /* And it must still be a correct frustum: the glass maps to NDC edges. */
    MmVec3 n1, n2, n3;
    mm_project_point(m, pa, &n1);
    mm_project_point(m, pb, &n2);
    mm_project_point(m, pc, &n3);
    ok(near(n1.x, -1.0, 1e-9) && near(n1.y, -1.0, 1e-9) &&
       near(n2.x,  1.0, 1e-9) && near(n2.y, -1.0, 1e-9) &&
       near(n3.x, -1.0, 1e-9) && near(n3.y,  1.0, 1e-9),
       "glass corners still map to the NDC edges");

    /* Wound the other way -- the original mistake -- must still be caught. */
    double bad[16];
    ok(mm_offaxis_projection(refl, pb, pa, pc, 0.05, 500.0, bad) == -2,
       "the OLD winding is still reported as -2, not silently accepted");

    /* A mirror facing the other way must work identically. MirrorRight did not
     * warn in the real run purely because of its orientation, which is exactly
     * the kind of asymmetry that hides a winding bug. */
    const MmVec3 n2v = V(-1, 0, 0);
    MmVec3 qa, qb, qc;
    mm_mirror_corners(centre, n2v, V(0, -1, 0), up, 0.125, 0.03, &qa, &qb, &qc);
    const MmVec3 eye2  = V(-0.6, 0.35, 0.12);
    const MmVec3 refl2 = mm_reflect_point(eye2, centre, n2v);
    ok(mm_offaxis_projection(refl2, qa, qb, qc, 0.05, 500.0, m) == 0,
       "a mirror facing -X works the same way");
}

/* THE MATRIX THE COMPONENT SHIPS.
 *
 * Everything above tests the OpenGL-convention construction, which the
 * component never rendered with directly: it took the extents, composed an
 * Unreal matrix by hand, and rendered that from a look-at rotation. The
 * composition had no test and it was wrong -- in the editor the glass corners
 * landed 1.8 to 10.8 NDC units off the edges while all 29 assertions here
 * passed. mm_ue_mirror_capture is now the whole of it, so this tests what
 * actually renders.
 *
 * Numbers are Unreal centimetres, shaped like the Innova's right-hand door
 * mirror seen from the right-hand driver's seat: close, low, and off the
 * glass's axis in both directions. */
static void test_unreal_capture(void)
{
    printf("\nthe Unreal capture (what the component renders)\n");

    const MmVec3 eye    = V(-55.0, 38.0, 115.0);
    const MmVec3 centre = V(-10.0, 95.0, 105.0);
    const MmVec3 aim    = V(-1200.0, 260.0, 50.0);
    const double hw = 8.0, hh = 4.5;
    const double past = 0.5;    /* near plane this far beyond the glass */
    const double kn = 5.0;      /* an absolute near plane for Kooima's extents */

    /* Aimed the way AChronoVehicleActor aims it: the normal bisects the
     * directions to the eye and to the point the mirror should show. */
    const MmVec3 normal = unit(add(unit(sub(eye, centre)), unit(sub(aim, centre))));

    MmUeCapture cap;
    const int rc = mm_ue_mirror_capture(eye, centre, normal, V(0, 0, 1),
                                        hw, hh, past, &cap);
    ok(rc == 0, "capture built for an off-axis driver (rc=%d)", rc);

    /* The basis must be a rotation Unreal can hold -- X x Y = Z -- or
     * FRotationMatrix::MakeFromXZ(forward, up) will not reproduce `right`, and
     * the matrix would be rendered in axes it was not built for. */
    ok(near(dot(crs(cap.forward, cap.right), cap.up), 1.0, 1e-12) &&
       near(dot(cap.forward, cap.right), 0.0, 1e-12) &&
       near(dot(cap.forward, cap.up), 0.0, 1e-12),
       "view basis is a proper rotation: forward x right = up");

    const MmVec3 R = scl(cap.right, hw);
    const MmVec3 U = scl(cap.up, hh);
    struct { MmVec3 p; double x, y; const char* name; } corner[4] = {
        { add(add(centre, R), U),  1.0,  1.0, "+right +up" },
        { add(sub(centre, R), U), -1.0,  1.0, "-right +up" },
        { sub(add(centre, R), U),  1.0, -1.0, "+right -up" },
        { sub(sub(centre, R), U), -1.0, -1.0, "-right -up" },
    };
    /* Not just |ndc| = 1: each corner on ITS edge. A mirrored or rotated
     * frustum also puts four corners on the edges, just the wrong four. */
    double worst = 0.0;
    for (int i = 0; i < 4; ++i) {
        double x = 9, y = 9;
        mm_ue_project(&cap, corner[i].p, &x, &y, NULL);
        worst = fmax(worst, fmax(fabs(x - corner[i].x), fabs(y - corner[i].y)));
    }
    ok(worst < 1e-9, "each glass corner lands on its own NDC corner (worst %.1e)",
       worst);

    double x, y;
    mm_ue_project(&cap, centre, &x, &y, NULL);
    ok(near(x, 0.0, 1e-9) && near(y, 0.0, 1e-9),
       "glass centre -> NDC (0,0)  got (%.2e, %.2e)", x, y);

    /* Affine across the WHOLE glass, not just at the corners: the image plane
     * is parallel to the glass, so any glass point maps by scale and offset.
     * That is what lets the target sit on the glass's UVs with no warp. */
    mm_ue_project(&cap, sub(add(centre, scl(R, 0.3)), scl(U, 0.7)), &x, &y, NULL);
    ok(near(x, 0.3, 1e-9) && near(y, -0.7, 1e-9),
       "interior glass point (0.3, -0.7) -> NDC (0.3, -0.7)  got (%.6f, %.6f)",
       x, y);

    /* THE MIRROR'S JOB, stated as geometry. The bisector law makes the ray
     * from the eye to the glass centre reflect straight at `aim` -- so `aim`
     * must appear dead centre in the image. This ties the aiming in the
     * vehicle actor to the projection here: either one wrong, and it moves. */
    mm_ue_project(&cap, aim, &x, &y, NULL);
    ok(near(x, 0.0, 1e-9) && near(y, 0.0, 1e-9),
       "the aimed-at point appears dead centre  got (%.2e, %.2e)", x, y);

    /* And for an arbitrary world point: find where the driver sees it on the
     * glass (where the segment from the reflected eye to it crosses the
     * plane), check the law of reflection holds there, and check the image
     * puts the point exactly where that glass point is. */
    const MmVec3 q = V(-800.0, 330.0, 20.0);
    const MmVec3 qd = sub(q, cap.position);
    const MmVec3 p = add(cap.position,
                         scl(qd, dot(sub(centre, cap.position), cap.forward) /
                                 dot(qd, cap.forward)));
    const MmVec3 bounced = mm_reflect_dir(unit(sub(p, eye)), cap.forward);
    const MmVec3 to_q = unit(sub(q, p));
    ok(near(dot(bounced, to_q), 1.0, 1e-12),
       "law of reflection: eye -> glass point -> world point");
    mm_ue_project(&cap, q, &x, &y, NULL);
    ok(near(x, dot(sub(p, centre), cap.right) / hw, 1e-9) &&
       near(y, dot(sub(p, centre), cap.up) / hh, 1e-9),
       "a world point appears exactly where the driver sees it on the glass");

    /* THE ROOT CAUSE, pinned. Against Kooima's extents for the same mirror,
     * the width matches but the HORIZONTAL off-centre term flips sign while
     * the vertical one does not -- Kooima's basis is left-handed. So a single
     * skew sign shared by both axes, which is what bFlipOffAxisSkew was, can
     * never be right for both. */
    MmVec3 pa, pb, pc;
    mm_mirror_corners(centre, cap.forward, cap.right, cap.up, hw, hh, &pa, &pb, &pc);
    double kl, kr, kb, kt;
    mm_offaxis_extents(cap.position, pa, pb, pc, kn, &kl, &kr, &kb, &kt);
    ok(near(cap.proj[0], 2.0 * kn / (kr - kl), 1e-12) &&
       near(cap.proj[5], 2.0 * kn / (kt - kb), 1e-12),
       "same frustum size as the Kooima construction");
    ok(fabs(cap.proj[8]) > 0.1 &&
       near(cap.proj[8],  (kr + kl) / (kr - kl), 1e-12) &&
       near(cap.proj[9], -(kt + kb) / (kt - kb), 1e-12),
       "x off-centre term is Kooima's NEGATED, y is not (%.4f vs %.4f)",
       cap.proj[8], -(kr + kl) / (kr - kl));

    ok(cap.proj[11] == 1.0 && cap.proj[14] == cap.nearz &&
       cap.proj[10] == 0.0 && cap.proj[15] == 0.0,
       "reversed-Z infinite-far layout ([2][3]=1, [3][2]=near)");

    /* THE NEAR PLANE IS THE MIRROR'S CLIP PLANE. Unreal's own capture clip
     * plane was inert (r.AllowGlobalClipPlane is 0), and with the frustum
     * finally equal to the aperture, each two-sided glass filled its own
     * capture: three 100% black targets. Reversed-Z: ndc z > 1 is nearer than
     * the near plane, i.e. clipped. */
    const double depth = dot(sub(eye, centre), cap.forward);
    ok(near(cap.nearz, depth + past, 1e-12),
       "near plane sits %.2f past the glass (glass depth %.3f, near %.3f)",
       past, depth, cap.nearz);
    double z = 0.0;
    mm_ue_project(&cap, centre, &x, &y, &z);
    ok(z > 1.0, "the glass itself is clipped (ndc z %.5f > 1)", z);
    mm_ue_project(&cap, add(cap.position, scl(sub(centre, cap.position), 0.5)),
                  &x, &y, &z);
    ok(z > 1.0, "a point between the reflected eye and the glass is clipped "
                "(ndc z %.4f)", z);
    mm_ue_project(&cap, add(centre, scl(cap.forward, 2.0 * past)), &x, &y, &z);
    ok(z > 0.0 && z < 1.0, "a point just past the near plane is kept "
                           "(ndc z %.5f)", z);
    mm_ue_project(&cap, aim, &x, &y, &z);
    ok(z > 0.0 && z < 1.0, "the aimed-at point is kept (ndc z %.6f)", z);

    /* Behind the reflected eye there is no image at all. */
    ok(mm_ue_project(&cap, sub(cap.position, cap.forward), &x, &y, NULL) == -1,
       "a point behind the reflected eye is refused");
}

static void test_unreal_capture_rejects(void)
{
    printf("\nthe Unreal capture: rejected inputs\n");

    const MmVec3 centre = V(0, 0, 0);
    const MmVec3 normal = V(1, 0, 0);
    const MmVec3 up     = V(0, 0, 1);
    MmUeCapture cap, before;
    memset(&cap, 0xAB, sizeof cap);
    memcpy(&before, &cap, sizeof cap);

    ok(mm_ue_mirror_capture(V(-40, 5, 3), centre, normal, up, 8, 4, 5, &cap) == -2,
       "an eye behind the glass is reported as -2");
    ok(memcmp(&cap, &before, sizeof cap) == 0, "and the output is left untouched");
    ok(mm_ue_mirror_capture(V(0, 30, 3), centre, normal, up, 8, 4, 5, &cap) == -1,
       "an eye in the glass plane is rejected, not approximated");
    ok(mm_ue_mirror_capture(V(40, 5, 3), centre, normal, normal, 8, 4, 5, &cap) == -1,
       "`up` parallel to the normal is rejected");
    ok(mm_ue_mirror_capture(V(40, 5, 3), centre, normal, up, 0, 4, 5, &cap) == -1 &&
       mm_ue_mirror_capture(V(40, 5, 3), centre, normal, up, 8, 4, 0, &cap) == -1,
       "zero glass size, or a near plane not past the glass, is rejected");
    ok(memcmp(&cap, &before, sizeof cap) == 0, "no failure wrote to the output");

    /* A world-up `up` on a glass pitched 20 degrees is accepted: the part
     * along the normal is removed, which is what MakeFromXZ does too. */
    const MmVec3 pitched = V(cos(20.0 * M_PI / 180.0), 0, -sin(20.0 * M_PI / 180.0));
    ok(mm_ue_mirror_capture(V(40, 5, 3), centre, pitched, up, 8, 4, 5, &cap) == 0 &&
       near(dot(cap.up, pitched), 0.0, 1e-12),
       "a non-orthogonal `up` is orthogonalised, not rejected");
}

static void test_rejects(void)
{
    printf("\nrejected inputs\n");

    const MmVec3 pa = V(1.5,  0.2, -0.1);
    const MmVec3 pb = V(1.5, -0.2, -0.1);
    const MmVec3 pc = V(1.5,  0.2,  0.1);
    const MmVec3 pd = V(1.5, -0.2,  0.1);
    double m[16];

    ok(mm_offaxis_projection(V(0,0,0), pa, pa, pc, 0.05, 500.0, m) == -1,
       "degenerate rectangle (pa == pb) rejected");
    ok(mm_offaxis_projection(V(0,0,0), pa, pb, pc, -1.0, 500.0, m) == -1,
       "negative near plane rejected");
    ok(mm_offaxis_projection(V(0,0,0), pa, pb, pc, 100.0, 10.0, m) == -1,
       "far <= near rejected");
    /* An eye IN the mirror plane has no frustum at all. Returning a matrix of
     * infinities would be worse than refusing. */
    ok(mm_offaxis_projection(V(1.5, 0.0, 0.0), pa, pb, pc, 0.05, 500.0, m) == -1,
       "an eye in the mirror plane is rejected, not approximated");
    /* Corners in world-axis order rather than viewer order. This is THE likely
     * caller mistake and it gets its own return code. */
    ok(mm_offaxis_projection(V(-0.3, 0.9, 0.35), pb, pa, pd, 0.05, 500.0, m) == -2,
       "corners wound away from the eye are reported as -2, not -1");
    /* Same winding, eye behind the mirror: also -2. */
    ok(mm_offaxis_projection(V(3.0, 0.0, 0.0), pa, pb, pc, 0.05, 500.0, m) == -2,
       "an eye behind the mirror is reported as -2");
}

static void test_realistic(void)
{
    printf("\na real mirror, in Chrono metres\n");

    /* Interior rear-view mirror: ~0.25 x 0.06 m, roughly 0.6 m ahead of the
     * driver's eye and a little above it. Chrono frame, X forward, Y left. */
    const MmVec3 eye = V(0.0, 0.0, 0.0);
    const MmVec3 pa = V(0.60,  0.125, 0.02);   /* viewer's lower-left  */
    const MmVec3 pb = V(0.60, -0.125, 0.02);   /* viewer's lower-right */
    const MmVec3 pc = V(0.60,  0.125, 0.08);   /* viewer's upper-left  */

    double hh, hv;
    mm_subtended_angles(eye, pa, pb, pc, &hh, &hv);
    printf("       subtends %.1f deg horizontally, %.1f deg vertically\n",
           2.0 * hh * 180.0 / M_PI, 2.0 * hv * 180.0 / M_PI);
    ok(2.0 * hh * 180.0 / M_PI > 10.0 && 2.0 * hh * 180.0 / M_PI < 40.0,
       "an interior mirror subtends a plausible angle");

    /* And the pixel argument. A symmetric FOV wide enough to contain an
     * off-axis door mirror wastes most of its render target; this is the
     * number that justifies the extra matrix maths on an RX 550. */
    const MmVec3 door_eye = V(0.0, 0.75, 0.0);
    const MmVec3 da = V(0.85, 1.15, -0.05);
    const MmVec3 db = V(0.85, 0.95, -0.05);
    const MmVec3 dc = V(0.85, 1.15,  0.07);
    double dhh, dhv;
    mm_subtended_angles(door_eye, da, db, dc, &dhh, &dhv);
    /* Half-angle from the eye axis out to the far edge of the mirror. */
    const double off = atan2(1.15 - 0.75, 0.85);
    const double sym_needed = 2.0 * off * 180.0 / M_PI;
    const double actual = 2.0 * dhh * 180.0 / M_PI;
    printf("       door mirror: subtends %.1f deg, but a SYMMETRIC frustum\n"
           "       containing it needs %.1f deg -- %.1fx the horizontal pixels\n",
           actual, sym_needed, sym_needed / actual);
    ok(sym_needed > actual,
       "off-axis projection is strictly tighter than a symmetric one");
}

int main(void)
{
    test_reflection();
    test_projection();
    test_full_mirror_pipeline();
    test_unreal_capture();
    test_unreal_capture_rejects();
    test_rejects();
    test_realistic();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
