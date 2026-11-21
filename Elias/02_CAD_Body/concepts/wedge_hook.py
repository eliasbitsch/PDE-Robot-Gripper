"""Rough concept: wedge-hook gripper (only to compare the size with the rack-and-pinion versions).

fc.sh concepts/wedge_hook.py  ->  out_concepts/wedge_hook/*.stl, wedge_hook.step, wedge_hook.json (mass, bounding box)

A piston (bore 20) on the gripper axis moves the wedge vertically; its flanks (30 deg) drive both jaw slides along x
in a guide slot of the housing: wedge down = open, up = close, 10 mm per jaw need 17.3 mm of piston stroke. The jaws
are Viktoriia's V3 jaws in the grip position, part B as in the main build. Not detailed: T-slot hooks, seals, ports,
spring, sensors. Piston force for 2 x 44.5 N with eta 0.7: 73 N, 2.3 bar on the bore 20.
"""
import json
import math
import os
import sys

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))
ROOT = os.path.join(HERE, "..") if os.path.basename(HERE) == "concepts" else HERE
sys.path.insert(0, ROOT)
import gripper_params as P  # noqa: E402

V = App.Vector
OUT = os.path.join(ROOT, "out_concepts", "wedge_hook")
ALPHA = math.radians(30.0)
D = P.D_GRIP                                   # modelled at the grip position
S_PISTON = D / math.tan(ALPHA)                 # wedge travel from the closed stop (down)
X_IN0 = 10.0                                   # slide inner end at d = 0
Z_SL = (-26.0, -10.0)                          # slide height, guided in the housing
Z_C = -15.0                                    # flank contact height
SL_W = 30.0                                    # slide width (y)
HX, HY = 62.0, 26.0                            # housing half size
BORE, PISTON_T = 20.0, 8.0
DENS = {"Al": 2.70e-6, "steel": 7.85e-6}   # pads counted as steel (plate 3 + NBR 3): a few grams too much


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def wedge(dz):
    """Trapezoid in xz, flanks widen upwards (x = X_IN0 + (z - Z_C) tan a at d = 0), extruded over y."""
    zb, zt = -20.0, 8.0
    xb, xt = X_IN0 + (zb - Z_C) * math.tan(ALPHA), X_IN0 + (zt - Z_C) * math.tan(ALPHA)
    pts = [V(-xb, -10, zb), V(xb, -10, zb), V(xt, -10, zt), V(-xt, -10, zt), V(-xb, -10, zb)]
    s = Part.Face(Part.makePolygon(pts)).extrude(V(0, 20, 0))
    s.translate(V(0, 0, dz))
    return s


def slide(side, d):
    """Jaw slide: guided body inside the housing, inner end on the wedge flank, lug down to the jaw (like the holder)."""
    x0 = X_IN0 + d
    body = box(x0, 56.0 + d, -SL_W / 2, SL_W / 2, Z_SL[0], Z_SL[1])
    lug = box(46.0 + d, 56.0 + d, -P.HOLDER_W / 2, P.HOLDER_W / 2, -47.0, Z_SL[0] + 0.01)
    s = body.fuse(lug)
    # flank face: cut what lies inside the wedge envelope (inclined inner end)
    s = s.cut(wedge(-d / math.tan(ALPHA)).common(box(-80, 80, -SL_W, SL_W, -60, 60)))
    if side < 0:
        s = s.mirror(V(0, 0, 0), V(1, 0, 0))
    return s


def housing():
    """Block with the guide slot for the slides, the wedge pocket and the cylinder bore; open at the bottom."""
    z0, z1 = Z_SL[0], 22.0
    s = box(-HX, HX, -HY, HY, z0, z1)
    s = s.cut(box(-HX - 1, HX + 1, -SL_W / 2 - 0.2, SL_W / 2 + 0.2, z0 - 1, Z_SL[1] + 0.2))        # guide slot
    s = s.cut(box(-26, 26, -10.5, 10.5, z0 - 1, 9.0))                                            # wedge travel
    s = s.cut(Part.makeCylinder(BORE / 2, z1 - 2 - (-6.0), V(0, 0, -6.0)))                      # cylinder bore
    return s.removeSplitter()


def cap():
    """Cylinder cap = robot flange ISO 9409-1-40 (plate 6 + spigot), M6 from the inside like the pneumatic riser."""
    z1 = 22.0
    s = Part.makeCylinder(P.ADAPTER_D / 2, 6.0, V(0, 0, z1)).common(box(-HX, HX, -HY, HY, z1, z1 + 6))
    s = s.fuse(Part.makeCylinder(P.ISO_SPIGOT_D / 2, P.ISO_SPIGOT_H, V(0, 0, z1 + 6)))
    s = s.fuse(box(-HX, HX, -HY, HY, z1, z1 + 3))                                               # cover over the block
    return s


def piston(dz):
    zt = 8.0 + dz
    s = Part.makeCylinder(BORE / 2 - 0.2, PISTON_T, V(0, 0, zt + 5.0))
    s = s.fuse(Part.makeCylinder(4.0, 6.0, V(0, 0, zt - 0.5)))                                  # rod into the wedge
    return s


def main():
    os.makedirs(OUT, exist_ok=True)
    dz = -S_PISTON
    jaws = {}
    v3 = os.path.join(ROOT, "..", "..", "Viktoriia", "03_CAD_Jaws")
    for side, tag in ((-1, "L"), (1, "R")):
        jaws[f"Jaw_{tag}"] = Part.read(os.path.join(v3, f"jaw_{tag}.step"))
        jaws[f"PadPlate_{tag}"] = Part.read(os.path.join(v3, f"pad_plate_{tag}.step"))
        jaws[f"Pad_{tag}"] = Part.read(os.path.join(v3, f"pad_NBR_{tag}.step"))
    parts = {"Housing": (housing(), "Al"), "Cap_flange": (cap(), "Al"), "Wedge": (wedge(dz), "steel"),
             "Piston": (piston(dz), "Al"), "Slide_L": (slide(-1, D), "steel"), "Slide_R": (slide(1, D), "steel")}
    for k, s in jaws.items():
        parts[k] = (s, "Al" if k.startswith("Jaw") else "steel")
    mass = {k: s.Volume * DENS[m] for k, (s, m) in parts.items()}
    bb = None
    for k, (s, _) in parts.items():
        bb = s.BoundBox if bb is None else bb.united(s.BoundBox)
        import Mesh
        Mesh.Mesh(s.tessellate(0.1)).write(os.path.join(OUT, f"{k}.stl"))
    hits = []
    names = list(parts)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = parts[names[i]][0], parts[names[j]][0]
            if a.BoundBox.intersect(b.BoundBox):
                v = a.common(b).Volume
                if v > 1.0 and {names[i], names[j]} != {"Wedge", "Piston"}:   # rod sits in the wedge
                    hits.append((names[i], names[j], round(v, 1)))
    flange_z = 22.0 + 6.0
    tcp_z = P.PART_OFFSET[2] + 17.5
    res = dict(mass_kg=sum(mass.values()), masses=mass, size=[bb.XLength, bb.YLength, bb.ZLength],
               housing=[2 * HX, 2 * HY, flange_z - Z_SL[0]], flange_to_tcp=flange_z - tcp_z, overlaps=hits,
               piston_stroke=S_PISTON, piston_force=2 * P.F_GRIP_SET * math.tan(ALPHA) / 0.7)
    json.dump(res, open(os.path.join(OUT, "wedge_hook.json"), "w"), indent=1)
    import Import
    doc = App.newDocument("wedge")
    objs = []
    for k, (s, _) in parts.items():
        o = doc.addObject("Part::Feature", k)
        o.Shape = s
        objs.append(o)
    Import.export(objs, os.path.join(OUT, "wedge_hook.step"))
    print(f"mass {res['mass_kg']:.3f} kg, size {[round(v) for v in res['size']]}, housing {res['housing']}, "
          f"flange to TCP {res['flange_to_tcp']:.1f} mm, piston stroke {S_PISTON:.1f} mm, force {res['piston_force']:.0f} N, overlaps {hits}")


main()
