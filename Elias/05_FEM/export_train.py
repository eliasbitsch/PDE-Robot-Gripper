"""Export the jaw train of the right side (slide + jaw holder + mock jaw) as STEP for the FEM.

Run with FreeCAD: fc.sh export_train.py  ->  train.step (three solids, order: slide, holder, jaw)
The jaw is the CAD placeholder until Viktoriia's jaw (V3) replaces it; only its load path matters here.
"""
import os
import sys

import Part

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))   # fc.sh passes the script path, no __file__
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402
import parts.core as C  # noqa: E402

SIDE = 1
slide = C.slide(SIDE)
holder = C.holder(SIDE)
jaw, _, _ = C.jaw(SIDE)
Part.makeCompound([slide, holder, jaw]).exportStep(os.path.join(HERE, "train.step"))

# Carriage contact patch on the slide (fixed support in the FEM): carriage body footprint at d = 0
xc = SIDE * P.CARR_X0
print("train.step: slide %.0f, holder %.0f, jaw %.0f mm3" % (slide.Volume, holder.Volume, jaw.Volume))
print("carriage patch: x %.1f ... %.1f, y %.1f ... %.1f, z = %.1f"
      % (xc - P.CARRIAGE_L_BODY / 2, xc + P.CARRIAGE_L_BODY / 2, P.RAIL_Y - P.CARRIAGE_W / 2,
         P.RAIL_Y + P.CARRIAGE_W / 2, P.SLIDE_Z_TOP))
print("pad centre: x %.1f, y 0, z %.1f" % (SIDE * P.PAD_X0, sum(P.PAD_Z) / 2))
