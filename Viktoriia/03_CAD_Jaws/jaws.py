"""V3 - Jaws and pad plates for the gripper, part B (group 10, Ovdiienko).

Built with CadQuery to the frozen jaw interface (handover section 4), in the gripper
frame of Elias' CAD (z = 0 underside of the housing top plate, X = grip axis), at the
grip position d = 2 mm - so the STEP files drop straight into gripper_assembly.step.

    pip install cadquery
    python jaws.py            -> STEP files + renders in this folder

All interface dimensions come from Elias/02_CAD_Body/gripper_params.py.
"""
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Elias", "02_CAD_Body"))
import gripper_params as P  # noqa: E402

# ------------------------------------------------------------------ interface (frozen, handover section 4)
JAW_T, JAW_W = P.JAW_T, P.JAW_W                     # 8 x 25 mm, EN AW-7075 T6
Z_TOP, Z_BOT = P.JAW_Z_TOP, P.JAW_Z_BOTTOM           # -28.5 ... -98   (69.5 mm)
X_IN = P.PAD_X0 + P.PAD_T + P.D_GRIP                 # 38  inner face at the grip position (|x|)
X_OUT = X_IN + JAW_T                                 # 46  outer face, sits on the holder
DOWEL_D, DOWEL_DEPTH, DOWEL_Y = 4.0, 6.5, P.DOWEL_Y  # 2 x O4 H7, 6.5 deep, y = +-5.5
DOWEL_Z = Z_TOP - 3.5                                # -32
PLUNGER_D, PLUNGER_DEPTH = 5.2, 6.0                  # GN 617-5 pin O5 -> O5.2, 6 deep
PLUNGER_Z = Z_TOP - 10.5                             # -39
POCKET_D, POCKET_W = 3.0, 23.0
POCKET_Z = P.PAD_Z                                   # -95 ... -60 (31.5 ... 66.5 below the top)
POCKET_R = 1.0                                       # corner radius: milled pocket, removes the FEM notch
SCREW_Z = (Z_TOP - 39.0, Z_TOP - 59.0)               # -67.5, -87.5, y = 0
SCREW_HOLE = 3.4                                     # M3 clearance, through
CBORE_D, CBORE_DEPTH = 6.5, 1.0                      # seat for ISO 4762 M3x6 (BOM pos. 25): 2 mm thread engagement
EDGE = 0.5                                           # break the outer edges, except the top face (dowel wall >= 1.5)

# ------------------------------------------------------------------ pad plate (bought to drawing)
PLATE_T, NBR_T = P.PAD_PLATE_T, P.PAD_T              # steel 3 + NBR 70 Shore A 3 (vulcanised)
PLATE_CLEAR = 0.1                                    # per side in the pocket
PLATE_R = 1.5                                        # > pocket radius, so the plate sits flat in the corners
NBR_INSET = 0.5                                      # rubber 0.5 mm smaller than the plate all round


def jaw_left():
    """Left jaw (part side -X): outer face at x = -46, inner face at x = -38."""
    length = Z_TOP - Z_BOT
    body = (cq.Workplane("XY")
            .box(JAW_T, JAW_W, length)
            .translate((-(X_IN + JAW_T / 2), 0, (Z_TOP + Z_BOT) / 2))
            .edges("not >Z").chamfer(EDGE))                # top edges sharp (deburred): a chamfer there leaves 1.0 mm over the dowels
    # pad pocket on the inner face
    pocket = (cq.Workplane("XY")
              .box(POCKET_D, POCKET_W, POCKET_Z[1] - POCKET_Z[0])
              .edges("|X").fillet(POCKET_R)
              .translate((-(X_IN + POCKET_D / 2), 0, (POCKET_Z[0] + POCKET_Z[1]) / 2)))
    body = body.cut(pocket)
    # dowel holes and indexing-plunger hole from the outer face
    outer = cq.Workplane("YZ", origin=(-X_OUT, 0, 0))
    for y in (-DOWEL_Y, DOWEL_Y):
        body = body.cut(outer.center(y, DOWEL_Z).circle(DOWEL_D / 2).extrude(DOWEL_DEPTH))
    body = body.cut(outer.center(0, PLUNGER_Z).circle(PLUNGER_D / 2).extrude(PLUNGER_DEPTH))
    # pad-plate screws: through holes + shallow counterbore on the outer face
    for z in SCREW_Z:
        body = body.cut(outer.center(0, z).circle(SCREW_HOLE / 2).extrude(JAW_T))
        body = body.cut(outer.center(0, z).circle(CBORE_D / 2).extrude(CBORE_DEPTH))
    return body


def pad_left():
    """Pad plate (steel, 2 x M3 threads) + vulcanised NBR layer, in the left pocket."""
    h = POCKET_Z[1] - POCKET_Z[0] - 2 * PLATE_CLEAR
    w = POCKET_W - 2 * PLATE_CLEAR
    zc = (POCKET_Z[0] + POCKET_Z[1]) / 2
    plate = (cq.Workplane("XY").box(PLATE_T, w, h).edges("|X").fillet(PLATE_R)
             .translate((-(X_IN + POCKET_D - PLATE_T / 2), 0, zc)))
    xp = -(X_IN + POCKET_D)                                       # back face of the plate (pocket floor)
    for z in SCREW_Z:                                             # M3 tapping drill 2.5, thread through
        plate = plate.cut(cq.Workplane("YZ", origin=(xp, 0, 0)).center(0, z).circle(1.25).extrude(PLATE_T))
    nbr = (cq.Workplane("XY").box(NBR_T, w - 2 * NBR_INSET, h - 2 * NBR_INSET)
           .edges("|X").fillet(PLATE_R - NBR_INSET)
           .translate((-(X_IN - NBR_T / 2), 0, zc)))
    return plate, nbr


def mirror_x(wp):
    return wp.mirror("YZ")


def build():
    jaw_l = jaw_left()
    plate_l, nbr_l = pad_left()
    parts = {
        "jaw_L": jaw_l, "jaw_R": mirror_x(jaw_l),
        "pad_plate_L": plate_l, "pad_plate_R": mirror_x(plate_l),
        "pad_NBR_L": nbr_l, "pad_NBR_R": mirror_x(nbr_l),
    }
    return parts


if __name__ == "__main__":
    parts = build()
    for name, wp in parts.items():
        cq.exporters.export(wp, os.path.join(HERE, f"{name}.step"))
    asm = cq.Assembly(name="jaws_V3")
    colors = {"jaw": cq.Color(0.75, 0.77, 0.80), "pad_plate": cq.Color(0.35, 0.35, 0.38), "pad_NBR": cq.Color(0.1, 0.1, 0.1)}
    for name, wp in parts.items():
        asm.add(wp, name=name, color=colors[name.rsplit("_", 1)[0]])
    asm.save(os.path.join(HERE, "jaws_V3_assembly.step"))
    rho = {"jaw": 2.81e-3, "pad_plate": 7.85e-3, "pad_NBR": 1.3e-3}   # g/mm^3
    for name, wp in parts.items():
        v = wp.val().Volume()
        print(f"{name:12s} volume {v:8.0f} mm3  mass {v * rho[name.rsplit('_', 1)[0]]:6.1f} g")
