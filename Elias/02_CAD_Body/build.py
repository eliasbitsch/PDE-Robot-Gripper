"""Build the gripper assembly headless: fc.sh build.py"""
import itertools
import json
import math
import os
import sys

import FreeCAD as App
import Part
from FreeCAD import Vector as V

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))  # fc.sh exec() has no __file__
sys.path.insert(0, HERE)
import gripper_params as P  # noqa: E402
from parts import core as C  # noqa: E402
from parts import demo_seq  # noqa: E402
from parts import fasteners as F  # noqa: E402

OUT = os.path.join(HERE, "out")
VENDOR = os.path.join(HERE, "vendor")
DENSITY = {"EN AW-6082": 2.70e-6, "EN AW-7075": 2.81e-6, "steel": 7.85e-6, "NBR": 1.20e-6}  # kg/mm3
VENDOR_MASS = {"Gearbox": 0.30, "Motor": 0.20, "Brake": 0.15}  # kg, Nanotec datasheets
TOL = 0.05  # mm3, below this an overlap counts as touching


def vendor(name, rot, base):
    s = Part.read(os.path.join(VENDOR, name))
    s.Placement = App.Placement(base, rot)
    return s


def build_static(gear_pin, gear_rack):
    gb_face = P.GEAR_FLANGE_Z + 21.0        # STEP: output flange at local x = 21
    gb_in = gb_face + 23.7                  # STEP: motor-side face at local x = -23.7
    motor_base = gb_in + 5.5                # STEP: motor front face at local z = 5.5
    motor_rear = motor_base + 24.5          # STEP: rear boss at local z = -24.5
    brake_base = motor_rear + 9.7           # STEP: brake mounting face at local x = -9.7
    brake_top = brake_base + 19.5
    tower_top = brake_top + 2.0
    parts = {
        "Housing": (C.housing_body(), "EN AW-6082", 0),
        "Cover": (C.cover(), "EN AW-6082", 0),
        "Rail": (C.rail(), "steel", 0),
        "Tower": (C.tower(tower_top), "EN AW-6082", 0),
        "Adapter_ISO9409": (C.iso_adapter(tower_top + P.LID_T), "EN AW-6082", 0),
        "Gearbox": (vendor("nanotec_gp42_s1_srs.step", App.Rotation(V(0, 1, 0), 90), V(0, 0, gb_face + 0.0) + V(0, 0, 0)), "vendor", 0),
        "Motor": (vendor("nanotec_st4118s1404b.step",
                         App.Rotation(V(0, 0, 1), 90).multiply(App.Rotation(V(1, 0, 0), 180)), V(0, 0, motor_base)), "vendor", 0),
        "Brake": (vendor("nanotec_bwa_035_5.step", App.Rotation(V(0, 1, 0), -90), V(0, 0, brake_base)), "vendor", 0),
        "Pinion": (C.pinion(gear_pin), "steel", "rot"),
    }
    for side, tag in ((-1, "L"), (1, "R")):
        parts[f"Carriage_{tag}"] = (C.carriage(side * P.CARR_X0), "steel", side)
        parts[f"Slide_{tag}"] = (C.slide(side), "EN AW-6082", side)
        parts[f"Rack_{tag}"] = (C.rack(gear_rack, side), "steel", side)
        parts[f"Holder_{tag}"] = (C.holder(side), "EN AW-6082", side)
        jb, plate, rubber = C.jaw(side)
        parts[f"Jaw_{tag}"] = (jb, "EN AW-7075", side)
        parts[f"PadPlate_{tag}"] = (plate, "steel", side)
        parts[f"Pad_{tag}"] = (rubber, "NBR", side)
        _, _, xj1, _, _ = C.jaw_x(side)
        for i, (y, z) in enumerate(C.pad_screws(side), 1):
            parts[f"Screw_ISO4762-M3x8_Pad{tag}{i}"] = (F.iso4762_x(3, 8, xj1 + side * 0.5, y, z, -side), "steel", side)
            parts[f"Washer_ISO7089-3_Pad{tag}{i}"] = (F.iso7089_x(3, xj1, y, z, side), "steel", side)
        parts[f"Plunger_GN617-5-A_{tag}"] = (C.plunger(side), "steel", side)
        for i, y in enumerate((-P.DOWEL_Y, P.DOWEL_Y), 1):
            parts[f"Dowel_ISO8734-4x12_{tag}{i}"] = (C.dowel(side, y), "steel", side)
        for i, (x, y) in enumerate(C.holder_screws(side), 1):
            parts[f"Screw_ISO4762-M4x25_Holder{tag}{i}"] = (F.iso4762(4, 25, x, y, P.HOLDER_Z_LOW + 4.5), "steel", side)
        for i, (x, y) in enumerate(C.carriage_holes(side * P.CARR_X0), 1):
            parts[f"Screw_ISO4762-M3x8_Carriage{tag}{i}"] = (F.iso4762(3, 8, x, y, P.SLIDE_Z_TOP - P.SLIDE_T + 3.2), "steel", side)
    for i, x in enumerate(C.rail_hole_x(), 1):
        parts[f"Screw_ISO4762-M3x8_Rail{i}"] = (F.iso4762(3, 8, x, P.RAIL_Y, -P.RAIL_H + 4.5), "steel", 0)
    for i, (x, y) in enumerate(P.TOWER_BOSS, 1):
        parts[f"Screw_ISO4762-M4x20_Tower{i}"] = (F.iso4762(4, 20, x, y, P.PLATE_T + P.TOWER_BOSS_H, up=False), "steel", 0)
    for i, (x, y) in enumerate(C.cover_screws(), 1):
        parts[f"Screw_ISO4762-M4x10_Cover{i}"] = (F.iso4762(4, 10, x, y, P.POCKET_Z - P.COVER_T), "steel", 0)
    for i, a in enumerate((45, 135, 225, 315), 1):
        x, y = 18 * math.cos(math.radians(a)), 18 * math.sin(math.radians(a))
        parts[f"Screw_ISO4762-M4x16_Gearbox{i}"] = (F.iso4762(4, 16, x, y, P.GEAR_SCREW_CBORE), "steel", 0)
    z_ad = tower_top + P.LID_T
    for i, (x, y) in enumerate(C.adapter_screws(), 1):
        parts[f"Screw_ISO10642-M5x16_Adapter{i}"] = (F.iso10642(5, 16, x, y, z_ad + P.ADAPTER_T), "steel", 0)
    return parts, {"tower_top": tower_top, "flange_z": z_ad + P.ADAPTER_T}


def scene_parts():
    """Part B on a table, in the gripper frame at grip height."""
    b = Part.read(os.path.join(HERE, "..", "..", "3D-Daten der Bauteile-20260904", "Bauteil_B.STEP"))
    b.translate(V(*P.PART_OFFSET))
    z_table = b.BoundBox.ZMin
    table = C.box(-110, 110, -80, 80, z_table - 10, z_table)
    return {"Part_B": b, "Table": table}


def at(parts, d, theta0):
    """Shapes at opening travel d (0 = closed stop). Pinion turns +d/r for opening."""
    out = {}
    for name, (shape, mat, mov) in parts.items():
        s = shape.copy()
        if mov == "rot":
            s.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(theta0 + d / (P.PINION_D / 2)))
        elif mov:
            s.translate(V(mov * d, 0, 0))
        out[name] = s
    return out


def overlap(a, b):
    if not a.BoundBox.intersect(b.BoundBox):
        return 0.0
    try:
        return a.common(b).Volume
    except Part.OCCError:
        return -1.0


def phase_pinion(parts):
    """Turn the pinion and shift the right rack so the teeth mesh without overlap."""
    pin = parts["Pinion"][0]
    rl, rr = parts["Rack_L"][0], parts["Rack_R"][0]

    def rot(a):
        s = pin.copy()
        s.rotate(V(0, 0, 0), V(0, 0, 1), a)
        return s

    best = min((overlap(rot(a / 4), rl), a / 4) for a in range(0, 72))
    theta = best[1]
    best_r = (1e9, 0.0)
    for i in range(0, 32):
        dx = i * math.pi / 32
        s = rr.copy()
        s.translate(V(dx, 0, 0))
        best_r = min(best_r, (overlap(rot(theta), s), dx))
    rr2 = rr.copy()
    rr2.translate(V(best_r[1], 0, 0))
    parts["Rack_R"] = (rr2, parts["Rack_R"][1], parts["Rack_R"][2])
    return math.radians(theta), best[0], best_r[0], best_r[1]


SKIP = {frozenset(p) for p in [("Gearbox", "Motor"), ("Motor", "Brake"), ("Gearbox", "Brake"),
                                ("Gearbox", "Pinion")]}
SKIP |= {frozenset(("Gearbox", f"Screw_ISO4762-M4x16_Gearbox{i}")) for i in range(1, 5)}  # thread depth in the vendor model unknown


def interference(shapes):
    hits = []
    for a, b in itertools.combinations(shapes, 2):
        if frozenset((a, b)) in SKIP:
            continue
        v = overlap(shapes[a], shapes[b])
        if v > TOL or v < 0:
            hits.append((a, b, round(v, 2)))
    return hits


def main():
    os.makedirs(OUT, exist_ok=True)
    doc = App.newDocument("Gripper")
    gear_pin, gear_rack = C.gears(doc)
    parts, info = build_static(gear_pin, gear_rack)
    theta0, ov_l, ov_r, dx_r = phase_pinion(parts)
    print(f"phase: pinion {math.degrees(theta0):.1f} deg, rack R shift {dx_r:.2f} mm, residual overlap L {ov_l:.3f} R {ov_r:.3f} mm3")

    scene = scene_parts()
    report = {"positions": {}}
    for label, d in (("closed", 0.0), ("grip", P.D_GRIP), ("open", P.D_OPEN)):
        shapes = at(parts, d, theta0)
        if label != "closed":                       # the closed stop is only reached without a part
            shapes.update(scene)
        hits = interference(shapes)
        report["positions"][label] = hits
        print(f"{label:6s} d={d:4.1f}  pads {2 * (P.PAD_X0 + d):5.1f} mm  interference: {hits or 'none'}")

    seq_hits, n = [], 0
    t = 0.0
    while t <= demo_seq.DURATION:
        d, zg, zp = demo_seq.state(t, P.D_GRIP)
        g = at(parts, d, theta0)
        for sh in g.values():
            sh.translate(V(0, 0, zg))
        part = scene["Part_B"].copy()
        part.translate(V(0, 0, zp))
        for obj, other in (("Part_B", part), ("Table", scene["Table"])):
            for name, sh in g.items():
                v = overlap(other, sh)
                if v > TOL or v < 0:
                    seq_hits.append((round(t, 2), obj, name, round(v, 2)))
        n += 1
        t += 0.05
    report["sequence"] = seq_hits
    print(f"demo sequence: {n} steps, collisions part/table vs gripper: {seq_hits[:6] or 'none'}")

    masses = {n: (VENDOR_MASS.get(n, 0.0) if m == "vendor" else s.Volume * DENSITY[m]) for n, (s, m, _) in parts.items()}
    mass = sum(masses.values())
    print("  " + ", ".join(f"{n} {1000 * v:.0f} g" for n, v in sorted(masses.items(), key=lambda kv: -kv[1])))
    print(f"mass {mass:.3f} kg (own parts from volume, Nanotec from datasheets)")
    print(f"grip force: needed {P.F_GRIP_OILY:.1f} N per jaw (oily, mu {P.MU_OILY}), available {P.F_JAW_MAX:.1f} N "
          f"(x{P.F_JAW_MAX / P.F_GRIP_OILY:.2f}), brake {P.T_PINION_BRAKE:.2f} N*m at the pinion")
    print(f"flange face z = {info['flange_z']:.1f}, cover underside z = {P.POCKET_Z - P.COVER_T:.1f}, "
          f"build height {info['flange_z'] - (P.POCKET_Z - P.COVER_T):.1f} mm above the jaws")

    shapes = at(parts, P.D_GRIP, theta0)
    objs = []
    for name, (_, mat, mov) in parts.items():
        o = doc.addObject("Part::Feature", name)
        o.Shape = shapes[name]
        o.addProperty("App::PropertyString", "Material", "Gripper").Material = mat
        o.addProperty("App::PropertyString", "Role", "Gripper").Role = "pinion" if mov == "rot" else ("slide" if mov else "static")
        o.addProperty("App::PropertyInteger", "Side", "Gripper").Side = mov if isinstance(mov, int) else 0
        o.addProperty("App::PropertyPlacement", "BasePlacement", "Gripper").BasePlacement = o.Placement
        objs.append(o)
    for name, sh in scene.items():
        o = doc.addObject("Part::Feature", name)
        o.Shape = sh
        o.addProperty("App::PropertyString", "Material", "Gripper").Material = "PA 6" if name == "Part_B" else "table"
        o.addProperty("App::PropertyString", "Role", "Gripper").Role = "part" if name == "Part_B" else "table"
        o.addProperty("App::PropertyInteger", "Side", "Gripper").Side = 0
        o.addProperty("App::PropertyPlacement", "BasePlacement", "Gripper").BasePlacement = o.Placement
    doc.recompute()
    doc.saveAs(os.path.join(OUT, "gripper.FCStd"))

    import Import
    step = os.path.join(OUT, "gripper.step")
    Import.export(objs, step)
    back = Part.read(step)
    n_model = sum(len(o.Shape.Solids) for o in objs)
    print(f"STEP round trip: {len(back.Solids)} solids in file, {n_model} in model")

    import Mesh
    stl_dir = os.path.join(OUT, "stl")
    os.makedirs(stl_dir, exist_ok=True)
    rest = at(parts, 0.0, theta0)
    for name, s in rest.items():
        Mesh.Mesh(s.tessellate(0.05)).write(os.path.join(stl_dir, f"{name}.stl"))
    kin = {"theta0_rad": theta0, "pinion_r_mm": P.PINION_D / 2, "d_grip": P.D_GRIP, "d_open": P.D_OPEN,
           "moving": {n: m for n, (_, _, m) in parts.items() if m}, "material": {n: m for n, (_, m, _) in parts.items()}}
    with open(os.path.join(OUT, "kinematics.json"), "w") as f:
        json.dump(kin, f, indent=1)
    report["mass_kg"] = mass
    report["mass_parts_kg"] = masses
    with open(os.path.join(OUT, "checks.json"), "w") as f:
        json.dump(report, f, indent=1)


main()
