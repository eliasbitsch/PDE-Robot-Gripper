"""Viktoriia's jaw V3 into the FEM models: jaw_v3.step (jaw alone) and train_v3.step (slide + holder + V3 jaw).

python v3_import.py
Her parts sit in the gripper frame at the grip position d = 2 mm, the FEM models at the closed stop d = 0:
shift -2 mm in x. Source: Viktoriia/03_CAD_Jaws/jaw_R.step (right jaw).
"""
import os

import gmsh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "Viktoriia", "03_CAD_Jaws", "jaw_R.step")
D_GRIP = 2.0


def _start():
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)


def build():
    _start()
    v = gmsh.model.occ.importShapes(SRC)
    gmsh.model.occ.translate(v, -D_GRIP, 0, 0)
    gmsh.model.occ.synchronize()
    gmsh.write(os.path.join(HERE, "jaw_v3.step"))
    gmsh.finalize()

    _start()
    t = gmsh.model.occ.importShapes(os.path.join(HERE, "train.step"))
    mock = [e for e in t if 35.0 < gmsh.model.occ.getBoundingBox(*e)[0] < 37.0]      # mock jaw starts at x = 36
    assert len(mock) == 1, mock
    gmsh.model.occ.remove(mock, recursive=True)
    v = gmsh.model.occ.importShapes(SRC)
    gmsh.model.occ.translate(v, -D_GRIP, 0, 0)
    gmsh.model.occ.synchronize()
    gmsh.write(os.path.join(HERE, "train_v3.step"))
    out = [(gmsh.model.occ.getBoundingBox(*e), gmsh.model.occ.getMass(*e)) for e in gmsh.model.getEntities(3)]
    gmsh.finalize()
    return out


if __name__ == "__main__":
    for bb, vol in build():
        print([round(c, 1) for c in bb], round(vol))
