# ============================================================================
#  21-make-mirror-material.py — creates /Game/Materials/M_Mirror.
#
#  Run it the same two ways as 20-make-level.py:
#
#    /WIN_D/protek/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
#        /WIN_D/protek/SimA-simulation/SimA/SimA.uproject \
#        -run=pythonscript -script=".../scripts/21-make-mirror-material.py"
#
#  WHY A SCRIPT AND NOT RUNTIME CODE:
#  UMaterialInstanceDynamic can retarget parameters at runtime but cannot
#  author a material graph -- that needs the material editor, which only exists
#  in the editor. So the graph is built here, once, as a real asset, and the
#  mirror component makes a dynamic instance of it and points the texture
#  parameter at its own render target.
#
#  WHY THE GRAPH IS UNLIT:
#  A texture sample into Emissive, shading model UNLIT.
#
#  Unlit is correct, not lazy. The render target already contains a fully lit
#  image of the scene -- it came out of a real scene capture. Feeding it into
#  Base Color would light it a SECOND time, from the lights near the glass, and
#  a mirror would go dark in shadow and bloom in sunlight while the world it
#  reflects did not.
#
#  There is no screen-space UV math either, and that is a property of the
#  off-axis projection in mirror_math.h: the capture frustum is exactly the
#  mirror rectangle, so the render target maps 1:1 onto the glass. UE's own
#  UPlanarReflectionComponent needs a screen-space projection precisely because
#  its frustum is NOT the mirror.
#
#  ---------------------------------------------------------------------------
#  THREE THINGS THIS SCRIPT DOES THAT THE FIRST VERSION DID NOT, all of them
#  reasons no reflection was ever seen:
#
#  1. TWO-SIDED. The glass is an /Engine/BasicShapes/Plane, and a Plane's
#     normal is its local +Z. The mirror component pitches it +90 degrees to
#     stand it up, and in Unreal a +90 pitch maps +Z to -X -- so the visible
#     face points AWAY from the driver and a single-sided material culls it.
#     The driver sees through the glass, which reads as "the mirror is black".
#     Two-sided removes the dependency on getting that rotation right at all.
#
#  2. THE FLIP PARAMETERS ACTUALLY EXIST. ChronoMirrorComponent has always set
#     a scalar named "FlipU" every tick. This graph had no such parameter and
#     no UV math whatsoever, so bFlipMirrorU was a dead flag that could never
#     have done anything. FlipU and FlipV are real now.
#
#  3. A DIAGNOSTIC DEFAULT TEXTURE. The sampler defaults to /Engine's "Bad"
#     texture rather than to nothing. That is what makes an unbound parameter
#     distinguishable from a black capture: one shows BAD, the other shows
#     black. Three render attempts were spent on a black rectangle that could
#     have been either.
#
#  IDEMPOTENT BY CONTENT, NOT BY EXISTENCE. The first version returned early
#  if the asset existed, so editing this file could never change the material
#  that was already on disk -- the bugs above were unfixable without deleting
#  the asset by hand. It now rebuilds the graph in place, which keeps the asset
#  path and every reference to it intact.
# ============================================================================

import unreal

PKG_PATH = "/Game/Materials"
ASSET_NAME = "M_Mirror"
FULL = "%s/%s" % (PKG_PATH, ASSET_NAME)

# The names the C++ side sets. Must match kMirrorTextureParam / kFlipU /
# kFlipV in ChronoMirrorComponent.cpp -- a mismatch is silent, and shows up as
# a mirror stuck on the parameter's default.
TEX_PARAM = "MirrorTexture"
FLIP_U_PARAM = "FlipU"
FLIP_V_PARAM = "FlipV"
SWAP_PARAM = "SwapUV"
UVDEBUG_PARAM = "UVDebug"

# Recognisable on sight and shipped with the engine. See point 3 above.
#
# A LIST, and loaded with unreal.load_asset rather than
# EditorAssetLibrary.load_asset, because the latter goes through the asset
# registry and most of /Engine is not scanned: "/Engine/EngineResources/Bad"
# is a real asset on disk that the registry lookup nonetheless fails to find.
# load_asset calls LoadObject and does not care.
DEFAULT_TEX_CANDIDATES = (
    "/Engine/EngineMaterials/DefaultCalibrationColor.DefaultCalibrationColor",
    "/Engine/EngineMaterials/DefaultCalibrationGrayscale.DefaultCalibrationGrayscale",
    "/Engine/EngineResources/Bad.Bad",
    "/Engine/EngineResources/DefaultTexture.DefaultTexture",
)


def log(msg):
    print("[mirror-material] %s" % msg)
    unreal.log("[mirror-material] %s" % msg)


def build_graph(mat):
    """Lays down the whole graph. Assumes the material has no expressions."""
    mel = unreal.MaterialEditingLibrary

    # Unlit: the capture is already lit. See the header comment.
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    # See point 1 in the header. This is what makes the glass visible at all.
    mat.set_editor_property("two_sided", True)

    # ---- UV: swap, then flip each axis -----------------------------------
    #
    #     a, b = (v, u) if SwapUV else (u, v)
    #     a    = 1 - a  if FlipU
    #     b    = 1 - b  if FlipV
    #
    # SWAP IS NOT COSMETIC and it is why two flags were never going to be
    # enough. MEASURED from a dumped render target against the same frame's
    # glass: the left mirror's dark bodywork blob occupies the target's LEFT
    # 66% across its full height (dark centroid u=0.321, v=0.476), and appears
    # on the glass across its full WIDTH in the bottom 66%. That is a 90
    # degree rotation, and no combination of two axis flips produces one.
    #
    # The cause is the glass mesh. The component stands an /Engine Plane up
    # with a +90 pitch, which maps the plane's local X to the mirror's HEIGHT
    # axis and its local Y to the WIDTH axis -- so the plane's U runs down the
    # glass and its V runs across it, transposed against the render target.
    #
    # With swap the three flags span all eight orientations, so one render
    # settles the mapping instead of one render per guess. Same handling as
    # the Milestone 5 force-feedback sign, for the same reason: this is a
    # convention, and conventions are read off a picture, not derived.
    uv = mel.create_material_expression(
        mat, unreal.MaterialExpressionTextureCoordinate, -1500, 0)

    u = mel.create_material_expression(
        mat, unreal.MaterialExpressionComponentMask, -1300, -120)
    u.set_editor_property("r", True)
    u.set_editor_property("g", False)
    u.set_editor_property("b", False)
    u.set_editor_property("a", False)

    v = mel.create_material_expression(
        mat, unreal.MaterialExpressionComponentMask, -1300, 120)
    v.set_editor_property("r", False)
    v.set_editor_property("g", True)
    v.set_editor_property("b", False)
    v.set_editor_property("a", False)

    mel.connect_material_expressions(uv, "", u, "")
    mel.connect_material_expressions(uv, "", v, "")

    # ---- the swap, ahead of the flips ------------------------------------
    swap = mel.create_material_expression(
        mat, unreal.MaterialExpressionScalarParameter, -1300, -300)
    swap.set_editor_property("parameter_name", SWAP_PARAM)
    swap.set_editor_property("default_value", 0.0)

    a = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -1080, -120)
    mel.connect_material_expressions(u, "", a, "A")
    mel.connect_material_expressions(v, "", a, "B")
    mel.connect_material_expressions(swap, "", a, "Alpha")

    b = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -1080, 120)
    mel.connect_material_expressions(v, "", b, "A")
    mel.connect_material_expressions(u, "", b, "B")
    mel.connect_material_expressions(swap, "", b, "Alpha")

    one_minus_u = mel.create_material_expression(
        mat, unreal.MaterialExpressionOneMinus, -700, -60)
    mel.connect_material_expressions(a, "", one_minus_u, "")

    one_minus_v = mel.create_material_expression(
        mat, unreal.MaterialExpressionOneMinus, -700, 180)
    mel.connect_material_expressions(b, "", one_minus_v, "")

    flip_u = mel.create_material_expression(
        mat, unreal.MaterialExpressionScalarParameter, -700, -200)
    flip_u.set_editor_property("parameter_name", FLIP_U_PARAM)
    flip_u.set_editor_property("default_value", 0.0)

    flip_v = mel.create_material_expression(
        mat, unreal.MaterialExpressionScalarParameter, -700, 300)
    flip_v.set_editor_property("parameter_name", FLIP_V_PARAM)
    flip_v.set_editor_property("default_value", 0.0)

    lerp_u = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -500, -120)
    mel.connect_material_expressions(a, "", lerp_u, "A")
    mel.connect_material_expressions(one_minus_u, "", lerp_u, "B")
    mel.connect_material_expressions(flip_u, "", lerp_u, "Alpha")

    lerp_v = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -500, 120)
    mel.connect_material_expressions(b, "", lerp_v, "A")
    mel.connect_material_expressions(one_minus_v, "", lerp_v, "B")
    mel.connect_material_expressions(flip_v, "", lerp_v, "Alpha")

    uv_out = mel.create_material_expression(
        mat, unreal.MaterialExpressionAppendVector, -320, 0)
    mel.connect_material_expressions(lerp_u, "", uv_out, "A")
    mel.connect_material_expressions(lerp_v, "", uv_out, "B")

    # ---- the sample ------------------------------------------------------
    tex = mel.create_material_expression(
        mat, unreal.MaterialExpressionTextureSampleParameter2D, -140, 0)
    tex.set_editor_property("parameter_name", TEX_PARAM)

    fallback = None
    for path in DEFAULT_TEX_CANDIDATES:
        try:
            fallback = unreal.load_asset(path)
        except Exception:
            fallback = None
        if fallback:
            tex.set_editor_property("texture", fallback)
            log("sampler default set to %s -- a mirror showing this has an "
                "UNBOUND texture parameter, which is a different fault from a "
                "mirror showing black." % path)
            break
    if not fallback:
        log("WARNING: no default texture loaded; the sampler has no default, "
            "so an unbound parameter will be indistinguishable from a black "
            "capture.")

    mel.connect_material_expressions(uv_out, "", tex, "UVs")

    # ---- UVDebug: emit the sampled coordinate as colour -------------------
    #
    # With UVDebug = 1 the glass shows float3(a, b, 0) instead of the
    # reflection: red rises with the sampled U, green with the sampled V. So
    # the corner that renders BLACK is target (0,0), and one screenshot reads
    # the whole mapping off directly.
    #
    # This exists because the alternative is guessing. There are eight
    # orientations reachable by swap + two flips, the glass is a quad seen
    # obliquely in every diagnostic view, and judging "is that flipped?" from
    # a tilted parallelogram of grey ground is exactly the kind of eyeballing
    # that left two signs unverified for a whole milestone.
    dbg = mel.create_material_expression(
        mat, unreal.MaterialExpressionScalarParameter, -140, 320)
    dbg.set_editor_property("parameter_name", UVDEBUG_PARAM)
    dbg.set_editor_property("default_value", 0.0)

    uv_colour = mel.create_material_expression(
        mat, unreal.MaterialExpressionAppendVector, -140, 200)
    mel.connect_material_expressions(uv_out, "", uv_colour, "A")
    zero = mel.create_material_expression(
        mat, unreal.MaterialExpressionConstant, -320, 260)
    zero.set_editor_property("r", 0.0)
    mel.connect_material_expressions(zero, "", uv_colour, "B")

    pick = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, 60, 0)
    mel.connect_material_expressions(tex, "RGB", pick, "A")
    mel.connect_material_expressions(uv_colour, "", pick, "B")
    mel.connect_material_expressions(dbg, "", pick, "Alpha")

    mel.connect_material_property(pick, "",
                                  unreal.MaterialProperty.MP_EMISSIVE_COLOR)


def main():
    mel = unreal.MaterialEditingLibrary

    if unreal.EditorAssetLibrary.does_asset_exist(FULL):
        mat = unreal.EditorAssetLibrary.load_asset(FULL)
        if not mat:
            log("%s exists but would not load" % FULL)
            return
        # Rebuild in place rather than skipping. See the header: skipping is
        # why two dead bugs survived in this asset across three render
        # attempts. Deleting and recreating would break every reference.
        mel.delete_all_material_expressions(mat)
        log("rebuilding the graph of the existing %s" % FULL)
    else:
        factory = unreal.MaterialFactoryNew()
        tools = unreal.AssetToolsHelpers.get_asset_tools()
        mat = tools.create_asset(ASSET_NAME, PKG_PATH, unreal.Material, factory)
        if not mat:
            log("could not create the material asset")
            return
        log("created %s" % FULL)

    build_graph(mat)

    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_asset(FULL)

    log("%s: two-sided unlit, texture parameter '%s', '%s' then flips "
        "'%s'/'%s'" % (FULL, TEX_PARAM, SWAP_PARAM, FLIP_U_PARAM, FLIP_V_PARAM))
    log("The mirror component makes a dynamic instance of this and points "
        "'%s' at its own render target." % TEX_PARAM)


main()
