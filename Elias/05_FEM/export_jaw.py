"""Export the right jaw (side = +1) as STEP for the FEM. Run with FreeCAD: fc.sh export_jaw.py"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))   # fc.sh passes the script path, no __file__
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import parts.core as C  # noqa: E402

jaw, _, _ = C.jaw(1)
jaw.exportStep(os.path.join(HERE, "jaw.step"))
bb = jaw.BoundBox
print("jaw.step", round(jaw.Volume, 1), "mm3", (bb.XMin, bb.XMax), (bb.YMin, bb.YMax), (bb.ZMin, bb.ZMax))
