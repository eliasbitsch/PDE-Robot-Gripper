"""E3, step 1 (FreeCAD): views of the gripper assembly as 2D line data for the drawing sheet.

fc.sh drawing_export.py  ->  out/views.json

Views as on the sheet (first-angle projection, gripper hanging as on the robot: flange up):
  A-A    full section in the plane y = 0 (front view, seen from -y): own parts and bearings cut and hatched;
         shafts, screws, pins, keys and the bought drive units (motor, gearbox, brake, encoder) shown uncut (ISO 128-44)
  top    seen from +z, placed below the section
  left   seen from -x, placed right of the section
  iso    isometric view
Every view holds visible and hidden edges as polylines in model mm, the hatched cut faces (section only) and one
anchor point per BOM position for the balloons. The model is the grip position of 02_CAD_Body/out/gripper.FCStd.
"""
import json
import os
import sys

import FreeCAD as App
import Part
import TechDraw

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))
CAD = os.path.join(HERE, "..", "02_CAD_Body")
OUT = os.path.join(HERE, "out")
V = App.Vector
VIEWS = {"A-A": V(0, -1, 0), "top": V(0, 0, 1), "left": V(-1, 0, 0), "iso": V(1, -1, 1)}
UNCUT = ("Screw", "Dowel", "Washer", "Setscrew", "Key", "Pinion_Shaft", "Gearbox", "Motor", "Brake", "Encoder",
         "Plunger", "Circlip", "Cable_Gland")
DEFL = 0.05


def norm(n):
    return n.replace("-", "_")


def polylines(shape):
    out = []
    if shape is None or shape.isNull():
        return out
    for e in shape.Edges:
        try:
            pts = e.discretize(Deflection=DEFL)
        except Exception:
            continue
        out.append([[round(p.x, 3), round(p.y, 3)] for p in pts])
    return out


def affine(d):
    """2D projection used by TechDraw for direction d, found with small marker spheres."""
    import numpy as np
    P3 = [V(0, 0, 0), V(100, 0, 0), V(0, 100, 0), V(0, 0, 100)]
    P2 = []
    for p in P3:
        r = TechDraw.projectEx(Part.makeSphere(1.0, p), d)
        es = [e for s in r if s and not s.isNull() for e in s.Edges]
        bb = Part.makeCompound(es).BoundBox
        P2.append([0.5 * (bb.XMin + bb.XMax), 0.5 * (bb.YMin + bb.YMax)])
    A = np.array([[p.x, p.y, p.z, 1.0] for p in P3])
    M = np.linalg.lstsq(A, np.array(P2), rcond=None)[0]           # 4 x 2
    return M.tolist()


def main():
    os.makedirs(OUT, exist_ok=True)
    doc = App.openDocument(os.path.join(CAD, "out", "gripper.FCStd"))
    pos = {norm(k): v for k, v in json.load(open(os.path.join(CAD, "out", "bom_map.json"))).items()}
    parts = {}
    for o in doc.Objects:
        if not hasattr(o, "Shape") or o.Shape.isNull() or getattr(o, "Role", "") in ("part", "table"):
            continue
        if o.Shape.Volume <= 0:
            continue
        parts[norm(o.Name)] = o.Shape.copy()
    print(len(parts), "parts")
    half = Part.makeBox(1000, 1000, 1000, V(-500, 0, -500))          # keeps y >= 0
    cut, faces = {}, []
    for n, s in parts.items():
        if n.startswith(UNCUT):
            if s.BoundBox.Center.y >= -1e-6 or s.BoundBox.YMax > 0 and n.startswith("Pinion_Shaft"):
                cut[n] = s
            continue
        if s.BoundBox.YMax <= 0:
            continue
        c = s.common(half)
        if c.isNull() or c.Volume <= 1e-6:
            continue
        cut[n] = c
        for f in c.Faces:
            if abs(f.BoundBox.YMin) < 1e-4 and abs(f.BoundBox.YMax) < 1e-4:
                loops = [[[round(p.x, 3), round(p.z, 3)] for p in w.discretize(Deflection=DEFL)] for w in [f.OuterWire] +
                         [w for w in f.Wires if not w.isSame(f.OuterWire)]]
                faces.append(dict(part=n, pos=pos.get(n), loops=loops))
    res = {"faces_AA": faces, "views": {}}
    for name, d in VIEWS.items():
        src = cut if name == "A-A" else parts
        r = TechDraw.projectEx(Part.makeCompound(list(src.values())), d)
        M = affine(d)
        anchors = {}
        for n, s in src.items():
            p = pos.get(n)
            if p is None:
                continue
            sol = s.Solids
            c = (sum((so.CenterOfMass * so.Volume for so in sol), V(0, 0, 0)) * (1 / sum(so.Volume for so in sol))
                 if sol and s.Volume > 1 else s.BoundBox.Center)
            anchors.setdefault(str(p), []).append([n, [c.x, c.y, c.z]])
        res["views"][name] = dict(dir=[d.x, d.y, d.z], M=M, visible=polylines(r[0]) + polylines(r[3]),       # no tangent (smooth) edges
                                  hidden=polylines(r[5]) + polylines(r[6]) + polylines(r[8]), anchors=anchors)
        print(name, len(res["views"][name]["visible"]), "visible", len(res["views"][name]["hidden"]), "hidden", len(anchors), "positions")
    json.dump(res, open(os.path.join(OUT, "views.json"), "w"))
    print("faces in A-A:", len(faces))


main()
