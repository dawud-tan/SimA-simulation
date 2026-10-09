/* ===========================================================================
 *  mirror_math.h — exact planar-mirror geometry.
 *
 *  PURE C11. No Unreal. Same reasoning as driver_source and referee: this is
 *  arithmetic, and arithmetic can be pinned down by a host compiler in a
 *  second instead of by launching an editor that takes twenty minutes to reach
 *  its first Tick.
 *
 *  WHY PLANAR REFLECTION AND NOT RAY TRACING:
 *  Not a preference. Step 0 established that neither GPU in this machine
 *  exposes VK_KHR_ray_query or VK_KHR_ray_tracing_pipeline -- the RX 550 is
 *  Polaris/GCN4 and AMD hardware RT starts at RDNA 2. Only llvmpipe (CPU) has
 *  it, which is not a renderer.
 *
 *  That constraint costs nothing here. For a FLAT mirror, planar reflection is
 *  not an approximation of the right answer -- it IS the right answer. The set
 *  of rays leaving a flat mirror toward the eye is exactly the set of rays
 *  reaching the reflected eye position, so rendering from the reflected eye is
 *  exact. Ray tracing would compute the same image more slowly. The
 *  approximation only appears for curved mirrors, and the SIM A exercises are
 *  performed with flat glass.
 *
 *  THREE PARTS, AND ALL THREE ARE REQUIRED:
 *
 *  1. THE VIRTUAL EYE. E' = E - 2((E-P0).N)N, the reflection of the eye across
 *     the mirror plane.
 *
 *  2. THE OBLIQUE CLIP PLANE. Everything BEHIND the mirror in real space sits
 *     in FRONT of the virtual camera. Without a clip plane at the mirror
 *     surface, the inside of the car's own bodywork renders into the mirror.
 *     UE exposes this as bEnableClipPlane / ClipPlaneBase / ClipPlaneNormal.
 *
 *  3. THE OFF-AXIS FRUSTUM. The eye is almost never on the mirror's central
 *     axis -- a door mirror is metres to one side of the driver's head -- so
 *     the correct frustum is the asymmetric pyramid from E' through the mirror
 *     rectangle. A symmetric FOV wide enough to contain that pyramid renders a
 *     large majority of pixels that the mirror never shows. On an RX 550
 *     driving three mirrors that waste is the difference between affordable
 *     and not.
 *
 *  MATRIX CONVENTION: these functions emit a COLUMN-MAJOR, column-vector
 *  matrix (v' = M*v), i.e. the OpenGL convention, mapping the view volume to
 *  NDC in [-1,1] on all three axes. Unreal's FMatrix is ROW-major with
 *  row-vector multiplication and uses reversed-Z, so the UE side must convert.
 *  It is stated here, and tested here, precisely because a silently transposed
 *  projection matrix produces a plausible-looking wrong image.
 * ======================================================================== */
#ifndef MIRROR_MATH_H
#define MIRROR_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double x, y, z; } MmVec3;

/** Reflects a POINT across the plane through `p0` with unit normal `n`. */
MmVec3 mm_reflect_point(MmVec3 p, MmVec3 p0, MmVec3 n);

/** Reflects a DIRECTION across a plane of unit normal `n`. Independent of the
 *  plane's position -- a direction has none. */
MmVec3 mm_reflect_dir(MmVec3 v, MmVec3 n);

/** Signed distance from `p` to the plane; positive on the side `n` points to. */
double mm_plane_distance(MmVec3 p, MmVec3 p0, MmVec3 n);

/**
 * Generalised (off-axis) perspective projection, after Kooima.
 *
 * The mirror rectangle is given by three of its corners, which fixes both its
 * plane and its orientation. WINDING MATTERS AND IS VIEWER-RELATIVE:
 *
 *   @param pa  lower-left   AS SEEN BY THE VIEWER
 *   @param pb  lower-right  AS SEEN BY THE VIEWER
 *   @param pc  upper-left   AS SEEN BY THE VIEWER
 *
 * so that cross(pb-pa, pc-pa) points OUT of the mirror, back toward the eye.
 * "Left" here is the viewer's left, which in Chrono's frame (X forward,
 * Y LEFT) means a viewer looking along +X has their left at +Y -- so a mirror
 * ahead of them has pa at LARGER y than pb.
 *
 * Supplying them in world-axis order instead of viewer order is the obvious
 * mistake and it is not a small one: the resulting normal faces away, every
 * frustum extent comes out negated, and the mirror renders inside out. It is
 * reported as a distinct return code rather than folded into the generic
 * failure, because the two need different fixes.
 * @param eye   the VIRTUAL eye, i.e. already reflected
 * @param nearz near plane distance, > 0
 * @param farz  far plane distance, > nearz
 * @param out   16 doubles, column-major
 *
 * @return  0 on success.
 *         -1 degenerate corners, bad near/far, or an eye lying IN the mirror
 *            plane -- where the frustum is undefined rather than merely
 *            extreme, and a matrix full of infinities would be worse.
 *         -2 the corners are wound away from the eye. Either the winding is
 *            wrong or the eye is behind the mirror.
 *
 * On any failure `out` is left UNTOUCHED. Callers must check: an earlier
 * version of the test suite read an uninitialised matrix and reported a pass.
 */
int mm_offaxis_projection(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc,
                          double nearz, double farz, double* out);

/**
 * Builds the three corners of a mirror, wound for the REFLECTED eye.
 *
 * THIS IS THE SUBTLETY THAT GOT IT WRONG THE FIRST TIME, and it is why the
 * winding lives here rather than in the caller.
 *
 * mm_offaxis_projection needs cross(pb-pa, pc-pa) to point at the eye it is
 * given. For a mirror, the eye it is given is the REFLECTED one -- the virtual
 * camera sits behind the glass. But the reflected eye is on the OPPOSITE side
 * of the plane from the driver, by construction. So corners wound to face the
 * driver -- the intuitive choice, and the one the component originally made --
 * face exactly the wrong way, and every mirror reported rc = -2 forever.
 *
 * Wound this way, `normal` is the glass's outward normal (pointing at the
 * DRIVER), and the corners come back facing the other way, at the reflected
 * eye. Callers pass the mirror's own outward normal and get corners that work.
 *
 * A useful side effect: this ordering also swaps which edge is image-left,
 * which is the lateral inversion a mirror is supposed to have.
 *
 * @param centre  centre of the glass, world space
 * @param normal  outward normal, unit, pointing at the driver
 * @param right   unit vector along the glass's width axis
 * @param up      unit vector along the glass's height axis
 */
void mm_mirror_corners(MmVec3 centre, MmVec3 normal, MmVec3 right, MmVec3 up,
                       double half_w, double half_h,
                       MmVec3* out_pa, MmVec3* out_pb, MmVec3* out_pc);

/** The frustum extents at the near plane, before composition.
 *
 *  Exposed because they are the only place the off-axis-ness is directly
 *  visible: once P is composed with the basis rotation and the eye
 *  translation, the skew terms are mixed through the whole matrix and cannot
 *  be read off. A test that checked the composed matrix for symmetry was
 *  reading the wrong entries and passing for the wrong reason.
 *
 *  Also worth logging: |l| far from |r| means a mirror the driver is looking
 *  at from a steep angle, which is a placement question.
 *
 *  Same return codes as mm_offaxis_projection. */
int mm_offaxis_extents(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc,
                       double nearz,
                       double* out_l, double* out_r, double* out_b, double* out_t);

/** Multiplies a point by a column-major 4x4 and does the perspective divide.
 *  Returns 0, or -1 if the point is on or behind the eye plane (w <= 0), where
 *  NDC is meaningless. Exists so tests can assert what the matrix DOES rather
 *  than what its entries are. */
int mm_project_point(const double* m, MmVec3 p, MmVec3* out_ndc);

/** Horizontal and vertical half-angles subtended by the mirror at the eye.
 *  Only meaningful for reporting -- the projection above is exact and does not
 *  use these -- but a mirror subtending a tiny angle is a design mistake worth
 *  seeing in a log. Angles in radians. */
void mm_subtended_angles(MmVec3 eye, MmVec3 pa, MmVec3 pb, MmVec3 pc,
                         double* out_half_h, double* out_half_v);

/* ---------------------------------------------------------------------------
 *  THE CAPTURE UNREAL ACTUALLY RENDERS.
 *
 *  Everything above is the textbook construction (Kooima) in OpenGL
 *  conventions, and it is correct. UChronoMirrorComponent cannot use its
 *  matrix, though: an Unreal scene capture builds its OWN view matrix from the
 *  capture component's rotation -- inverse rotation, then the x=z, y=x, z=y
 *  axis swap (SceneCaptureRendering.cpp, UE 5.8.2) -- and takes only the
 *  projection from the caller. So the projection has to describe the frustum
 *  in the axes of THAT view, and the view has to be a rotation Unreal can hold.
 *
 *  The second condition is the trap the first version of the component fell
 *  into. Kooima's screen basis for a mirror is (vr, vu, vn) with
 *  vr = pb - pa = -right (see mm_mirror_corners): a reflection flips
 *  handedness, and no rotation can reproduce that basis. The component took
 *  Kooima's extents, rendered them from a look-at rotation, and put the glass
 *  corners 1.8 to 10.8 NDC units off the edges. No choice of skew sign could
 *  fix it, because the basis was wrong, not the sign.
 *
 *  So this measures the extents in the basis the capture really has:
 *
 *      forward = normal           capture local +X, view +Z
 *      up      = up               capture local +Z, view +Y
 *      right   = up x normal      capture local +Y, view +X -- Unreal's own
 *                                 Y = Z x X, which is exactly what
 *                                 FRotationMatrix::MakeFromXZ(forward, up)
 *                                 produces
 *
 *  and both off-centre terms then take the textbook sign.
 *
 *  Forward along the glass normal is also what makes the IMAGE right, not just
 *  its corners: the image plane is then parallel to the glass, so every glass
 *  point reaches the target by a pure scale and offset. The target therefore
 *  maps onto the glass's own UVs exactly across the whole surface. A look-at
 *  basis that happened to put the four corners in the right place would still
 *  warp everything between them.
 *
 *  THE SAME PARALLELISM MAKES THE NEAR PLANE THE MIRROR'S CLIP PLANE. Every
 *  point of the glass is at the same view depth, so a near plane just past it
 *  removes the glass and everything between the reflected eye and the glass --
 *  exactly the job of the "oblique clip plane" in part 2 above. Unreal's own
 *  capture clip plane (bEnableClipPlane) turned out to be INERT here: it needs
 *  r.AllowGlobalClipPlane, which defaults to 0, is read-only at runtime, and
 *  costs a full shader recompile plus ~15% base-pass time to turn on. With it
 *  inert, each mirror's two-sided glass -- lying exactly on the aperture --
 *  filled its own capture with its own black render target, a feedback loop
 *  measured as a 100% black target on all three mirrors.
 * ------------------------------------------------------------------------ */
typedef struct MmUeCapture {
    MmVec3 position;   /* the REFLECTED eye                                  */
    MmVec3 forward;    /* capture local +X: the glass normal                 */
    MmVec3 right;      /* capture local +Y = up x forward: image +x (u = 1)  */
    MmVec3 up;         /* capture local +Z: image +y (v = 0, the top row)    */
    double nearz;      /* the near plane: glass depth + near_past_glass      */
    /* FMatrix layout: proj[4*row + col], row-vector convention (v' = v * M),
     * reversed-Z with an infinite far plane -- TReversedZPerspectiveMatrix's
     * pattern, [2][3] = 1 and [3][2] = near, plus the off-centre terms in
     * [2][0] and [2][1]. Copy it into FMatrix row by row, untransposed. */
    double proj[16];
} MmUeCapture;

/**
 * The capture for one flat mirror, in Unreal's conventions: where to put it,
 * how to orient it, and the projection to give it.
 *
 * @param eye     the DRIVER's eye. This reflects it; do not pass it reflected.
 * @param centre  centre of the glass
 * @param normal  outward glass normal, pointing at the driver (need not be unit)
 * @param up      the glass's height axis; its component along `normal` is
 *                removed, so a world-up vector is an acceptable argument
 * @param half_w  half-width, along up x normal, > 0
 * @param half_h  half-height, along up, > 0
 * @param near_past_glass
 *                how far BEYOND the glass the near plane sits, > 0, in the
 *                same units. The near plane is then the mirror's clip plane
 *                (see above): it must be past the glass, or the glass --
 *                which lies exactly on the aperture -- fills the image.
 *
 * What the material's UV flags have to match: the glass edge at
 * +(up x normal) lands on target u = 1, and the edge at +up on target v = 0,
 * because an Unreal render target's row 0 is the top of the image.
 *
 * @return 0 on success.
 *         -1 degenerate size, near_past_glass <= 0, `up` parallel to
 *            `normal`, or an eye IN the glass plane, where there is no
 *            frustum at all.
 *         -2 the eye is BEHIND the glass: that mirror shows it nothing.
 *         On any failure `out` is left untouched, as elsewhere in this file.
 */
int mm_ue_mirror_capture(MmVec3 eye, MmVec3 centre, MmVec3 normal, MmVec3 up,
                         double half_w, double half_h, double near_past_glass,
                         MmUeCapture* out);

/**
 * Projects a world point through a capture exactly as Unreal would: into the
 * capture's view (x right, y up, z forward), times `proj` as a row vector,
 * then the perspective divide. Writes NDC x, y and -- if `out_ndc_z` is not
 * NULL -- the reversed-Z depth, which is > 1 for a point nearer than the near
 * plane, i.e. one that is clipped.
 *
 * Returns 0, or -1 if the point is on or behind the capture's eye plane.
 * Exists so tests can assert what the SHIPPED matrix does -- the same reason
 * mm_project_point exists for the OpenGL one.
 */
int mm_ue_project(const MmUeCapture* cap, MmVec3 p,
                  double* out_ndc_x, double* out_ndc_y, double* out_ndc_z);

#ifdef __cplusplus
}
#endif

#endif /* MIRROR_MATH_H */
