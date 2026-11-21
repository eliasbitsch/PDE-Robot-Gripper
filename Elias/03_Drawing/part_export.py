"""E3, part drawing data (FreeCAD): views of Viktoriia's right jaw V3 as 2D line data.

fc.sh part_export.py  ->  out/jaw_views.json
Views: inner face (seen from -x), section A-A in the mid plane y = 0 (seen from -y, part y >= 0 kept), outer face
(seen from +x). Visible and hidden edges, hatched cut faces; model frame of Viktoriia/03_CAD_Jaws (grip position).
"""
import json
import os
import sys

import FreeCAD as App
import Part
import TechDraw

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))
SRC = os.path.join(HERE, "..", "..", "Viktoriia", "03_CAD_Jaws", "jaw_R.step")
V = App.Vector
VIEWS = {"inner": V(-1, 0, 0), "A-A": V(0, -1, 0), "outer": V(1, 0, 0)}


def poly(shape):
    if shape is None or shape.isNull():
        return []
    out = []
    for e in shape.Edges:
        try:
            out.append([[round(p.x, 4), round(p.y, 4)] for p in e.discretize(Deflection=0.01)])
        except Exception:
            pass
    return out


def affine(d):
    import numpy as np
    P3 = [V(0, 0, 0), V(100, 0, 0), V(0, 100, 0), V(0, 0, 100)]
    P2 = []
    for p in P3:
        es = [e for s in TechDraw.projectEx(Part.makeSphere(1.0, p), d) if s and not s.isNull() for e in s.Edges]
        bb = Part.makeCompound(es).BoundBox
        P2.append([0.5 * (bb.XMin + bb.XMax), 0.5 * (bb.YMin + bb.YMax)])
    A = np.array([[p.x, p.y, p.z, 1.0] for p in P3])
    return np.linalg.lstsq(A, np.array(P2), rcond=None)[0].tolist()


def main():
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    jaw = Part.read(SRC)
    bb = jaw.BoundBox
    half = Part.makeBox(200, 200, 300, V(-100, 0, -200))
    cut = jaw.common(half)
    faces = []
    for f in cut.Faces:
        if abs(f.BoundBox.YMin) < 1e-4 and abs(f.BoundBox.YMax) < 1e-4:
            faces.append([[[round(p.x, 4), round(p.z, 4)] for p in w.discretize(Deflection=0.01)]
                          for w in [f.OuterWire] + [w for w in f.Wires if not w.isSame(f.OuterWire)]])
    res = {"bbox": [bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax], "faces_AA": faces, "views": {}}
    for n, d in VIEWS.items():
        r = TechDraw.projectEx(cut if n == "A-A" else jaw, d)
        res["views"][n] = dict(M=affine(d), visible=poly(r[0]) + poly(r[1]) + poly(r[3]), hidden=poly(r[5]) + poly(r[6]) + poly(r[8]))
        print(n, len(res["views"][n]["visible"]), len(res["views"][n]["hidden"]))
    json.dump(res, open(os.path.join(HERE, "out", "jaw_views.json"), "w"))
    print("bbox", [round(v, 2) for v in res["bbox"]], "faces", len(faces))


main()
