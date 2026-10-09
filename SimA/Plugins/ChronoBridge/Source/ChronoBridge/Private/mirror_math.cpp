/* mirror_math.cpp — see mirror_math.h. Written as C, compiled as C++ by the UE
 * module and as C by the host harness. */

#include "mirror_math.h"

#include <math.h>
#include <string.h>

static MmVec3 v_sub(MmVec3 a, MmVec3 b) { MmVec3 r = { a.x-b.x, a.y-b.y, a.z-b.z }; return r; }
static double v_dot(MmVec3 a, MmVec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static MmVec3 v_scale(MmVec3 a, double s) { MmVec3 r = { a.x*s, a.y*s, a.z*s }; return r; }
static MmVec3 v_cross(MmVec3 a, MmVec3 b)
{
    MmVec3 r = { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
    return r;
}
static double v_len(MmVec3 a) { return sqrt(v_dot(a, a)); }
static MmVec3 v_norm(MmVec3 a)
{
    const double l = v_len(a);
    return l > 1e-18 ? v_scale(a, 1.0 / l) : a;
}

double mm_plane_distance(MmVec3 p, MmVec3 p0, MmVec3 n)
{
    return v_dot(v_sub(p, p0), n);
}

MmVec3 mm_reflect_point(MmVec3 p, MmVec3 p0, MmVec3 n)
{
    /* p' = p - 2 d n, with d the signed distance. An involution: applying it
     * twice is the identity, which is the first thing the tests check. */
    return v_sub(p, v_scale(n, 2.0 * mm_plane_distance(p, p0, n)));
}

MmVec3 mm_reflect_dir(MmVec3 v, MmVec3 n)
{
    return v_sub(v, v_scale(n, 2.0 * v_dot(v, n)));
}

void mm_mirror_corners(MmVec3 centre, MmVec3 normal, MmVec3 right, MmVec3 up,
                       double half_w, double half_h,
                       MmVec3* out_pa, MmVec3* out_pb, MmVec3* out_pc)
{
    if (!out_pa || !out_pb || !out_pc) return;

    /* Wound so cross(pb-pa, pc-pa) comes out ANTI-parallel to `normal`, i.e.
     * pointing away from the driver and toward the reflected eye. Taking pa on
     * the +right edge and pb on the -right edge is what achieves that:
     *     vr = pb - pa = -2*half_w*right
     *     vu = pc - pa = +2*half_h*up
     *     cross(vr, vu) = -(right x up) = -normal
     * See the header for why the reflected eye, and not the driver, is the one
     * that matters. */
    const MmVec3 r_off = v_scale(right, half_w);
    const MmVec3 u_off = v_scale(up, half_h);
    (void)normal;   /* the sign is carried by the ordering, not by a dot */

    out_pa->x = centre.x + r_off.x - u_off.x;
    out_pa->y = centre.y + r_off.y - u_off.y;
    out_pa->z = centre.z + r_off.z - u_off.z;

    out_pb->x = centre.x - r_off.x - u_off.x;
    out_pb->y = centre.y - r_off.y - u_off.y;
    out_pb->z = centre.z - r_off.z - u_off.z;

    out_pc->x = centre.x + r_off.x + u_off.x;
    out_pc->y = centre.y + r_off.y + u_off.y;
    out_pc->z = centre.z + r_off.z + u_off.z;
}

/* Shared by mm_offaxis_extents and mm_offaxis_projection so the two can never
 * disagree about the frustum. */
static int offaxis_core(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc, double nearz,
                        MmVec3* out_vr, MmVec3* out_vu, MmVec3* out_vn,
                        double* out_l, double* out_r, double* out_b, double* out_t)
{
    MmVec3 vr = v_sub(pb, pa);
    MmVec3 vu = v_sub(pc, pa);
    if (v_len(vr) < 1e-12 || v_len(vu) < 1e-12) return -1;
    vr = v_norm(vr);
    vu = v_norm(vu);
    MmVec3 vn = v_norm(v_cross(vr, vu));

    const MmVec3 va = v_sub(pa, eye);
    const MmVec3 vb = v_sub(pb, eye);
    const MmVec3 vc = v_sub(pc, eye);

    const double d = -v_dot(va, vn);
    if (fabs(d) < 1e-9) return -1;      /* eye in the plane: undefined */
    /* vn must face the eye. If it does not, the caller wound the corners in
     * world-axis order rather than viewer order, or put the eye behind the
     * mirror. Distinguished from -1 because the fixes differ. */
    if (d < 0.0) return -2;

    const double s = nearz / d;
    const double l = v_dot(vr, va) * s;
    const double r = v_dot(vr, vb) * s;
    const double b = v_dot(vu, va) * s;
    const double t = v_dot(vu, vc) * s;
    if (!(r > l) || !(t > b)) return -1;

    if (out_vr) *out_vr = vr;
    if (out_vu) *out_vu = vu;
    if (out_vn) *out_vn = vn;
    *out_l = l; *out_r = r; *out_b = b; *out_t = t;
    return 0;
}

int mm_offaxis_extents(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc, double nearz,
                       double* out_l, double* out_r, double* out_b, double* out_t)
{
    if (!out_l || !out_r || !out_b || !out_t || !(nearz > 0.0)) return -1;
    return offaxis_core(eye, pa, pb, pc, nearz, 0, 0, 0,
                        out_l, out_r, out_b, out_t);
}

int mm_offaxis_projection(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc,
                          double nearz, double farz, double* out)
{
    if (!out || !(nearz > 0.0) || !(farz > nearz)) return -1;

    MmVec3 vr, vu, vn;
    double l, r, b, t;
    const int rc = offaxis_core(eye, pa, pb, pc, nearz, &vr, &vu, &vn, &l, &r, &b, &t);
    if (rc != 0) return rc;

    /* P = frustum(l,r,b,t,n,f) * M_basis * T(-eye), all column-major. */
    double P[16];
    memset(P, 0, sizeof P);
    P[0]  = (2.0 * nearz) / (r - l);
    P[5]  = (2.0 * nearz) / (t - b);
    P[8]  = (r + l) / (r - l);
    P[9]  = (t + b) / (t - b);
    P[10] = -(farz + nearz) / (farz - nearz);
    P[11] = -1.0;
    P[14] = -(2.0 * farz * nearz) / (farz - nearz);

    /* Rotation taking world axes onto the mirror basis. Rows are vr, vu, vn,
     * so in column-major storage they land in the strided slots below. */
    double M[16];
    memset(M, 0, sizeof M);
    M[0] = vr.x; M[4] = vr.y; M[8]  = vr.z;
    M[1] = vu.x; M[5] = vu.y; M[9]  = vu.z;
    M[2] = vn.x; M[6] = vn.y; M[10] = vn.z;
    M[15] = 1.0;

    /* Translation to the eye. */
    double T[16];
    memset(T, 0, sizeof T);
    T[0] = T[5] = T[10] = T[15] = 1.0;
    T[12] = -eye.x; T[13] = -eye.y; T[14] = -eye.z;

    /* out = P * M * T */
    double MT[16];
    for (int c = 0; c < 4; ++c)
        for (int r2 = 0; r2 < 4; ++r2) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += M[k * 4 + r2] * T[c * 4 + k];
            MT[c * 4 + r2] = sum;
        }
    for (int c = 0; c < 4; ++c)
        for (int r2 = 0; r2 < 4; ++r2) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) sum += P[k * 4 + r2] * MT[c * 4 + k];
            out[c * 4 + r2] = sum;
        }
    return 0;
}

int mm_project_point(const double* m, MmVec3 p, MmVec3* out_ndc)
{
    if (!m || !out_ndc) return -1;
    const double v[4] = { p.x, p.y, p.z, 1.0 };
    double o[4];
    for (int r = 0; r < 4; ++r) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) sum += m[k * 4 + r] * v[k];
        o[r] = sum;
    }
    if (o[3] <= 1e-12) return -1;
    out_ndc->x = o[0] / o[3];
    out_ndc->y = o[1] / o[3];
    out_ndc->z = o[2] / o[3];
    return 0;
}

int mm_ue_mirror_capture(MmVec3 eye, MmVec3 centre, MmVec3 normal, MmVec3 up,
                         double half_w, double half_h, double near_past_glass,
                         MmUeCapture* out)
{
    if (!out || !(half_w > 0.0) || !(half_h > 0.0) || !(near_past_glass > 0.0))
        return -1;
    if (v_len(normal) < 1e-12) return -1;

    /* An orthonormal basis, Unreal-handed: right = up x forward. */
    const MmVec3 fwd = v_norm(normal);
    const MmVec3 up_perp = v_sub(up, v_scale(fwd, v_dot(up, fwd)));
    if (v_len(up_perp) < 1e-9) return -1;      /* up parallel to the normal */
    const MmVec3 u = v_norm(up_perp);
    const MmVec3 r = v_cross(u, fwd);

    /* The virtual eye, and the glass's depth in front of it. By construction
     * that depth is the driver's own distance in front of the glass. */
    const MmVec3 eref = mm_reflect_point(eye, centre, fwd);
    const double d = v_dot(v_sub(centre, eref), fwd);
    if (fabs(d) < 1e-9) return -1;             /* eye in the glass plane */
    if (d < 0.0) return -2;                    /* eye behind the glass */

    /* The glass in the capture's own view coordinates. All four corners sit
     * at depth d -- the image plane is parallel to the glass -- so the
     * extents are plain ratios and need no corner ordering at all. */
    const MmVec3 c = v_sub(centre, eref);
    const double cx = v_dot(c, r);
    const double cy = v_dot(c, u);
    const double l = (cx - half_w) / d, rt = (cx + half_w) / d;
    const double b = (cy - half_h) / d, t  = (cy + half_h) / d;

    /* The near plane sits just past the glass. Every glass point is at depth
     * d, so this one plane clips the glass itself and everything between the
     * reflected eye and it: the mirror's clip plane, exact, and free. */
    const double nearz = d + near_past_glass;

    /* Reversed-Z, infinite far, off-centre. As a row vector,
     *     ndc_x = (2 x/z - (r + l)) / (r - l)     -1 at x/z = l, +1 at r
     * and the same in y. The off-centre terms take the textbook sign because
     * l, r, b, t were measured in THIS view's axes -- which is the whole fix. */
    memset(out->proj, 0, sizeof out->proj);
    out->proj[0]  = 2.0 / (rt - l);
    out->proj[5]  = 2.0 / (t - b);
    out->proj[8]  = -(rt + l) / (rt - l);
    out->proj[9]  = -(t + b) / (t - b);
    out->proj[11] = 1.0;
    out->proj[14] = nearz;

    out->nearz    = nearz;
    out->position = eref;
    out->forward  = fwd;
    out->right    = r;
    out->up       = u;
    return 0;
}

int mm_ue_project(const MmUeCapture* cap, MmVec3 p,
                  double* out_ndc_x, double* out_ndc_y, double* out_ndc_z)
{
    if (!cap || !out_ndc_x || !out_ndc_y) return -1;

    /* Unreal's view space for a capture: x right, y up, z forward. */
    const MmVec3 rel = v_sub(p, cap->position);
    const double v[4] = { v_dot(rel, cap->right), v_dot(rel, cap->up),
                          v_dot(rel, cap->forward), 1.0 };
    double o[4];
    for (int col = 0; col < 4; ++col) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) sum += v[k] * cap->proj[4 * k + col];
        o[col] = sum;
    }
    if (o[3] <= 1e-12) return -1;
    *out_ndc_x = o[0] / o[3];
    *out_ndc_y = o[1] / o[3];
    if (out_ndc_z) *out_ndc_z = o[2] / o[3];
    return 0;
}

void mm_subtended_angles(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc,
                         double* out_half_h, double* out_half_v)
{
    MmVec3 vr = v_norm(v_sub(pb, pa));
    MmVec3 vu = v_norm(v_sub(pc, pa));
    MmVec3 vn = v_norm(v_cross(vr, vu));
    const MmVec3 va = v_sub(pa, eye);
    const double d = fabs(v_dot(va, vn));
    if (out_half_h) *out_half_h = atan2(v_len(v_sub(pb, pa)) * 0.5, d);
    if (out_half_v) *out_half_v = atan2(v_len(v_sub(pc, pa)) * 0.5, d);
}
