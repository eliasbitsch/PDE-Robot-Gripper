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
from parts import drive as D  # noqa: E402

OUT = os.path.join(HERE, "out")
VENDOR = os.path.join(HERE, "vendor")
DENSITY = {"EN AW-6082": 2.70e-6, "EN AW-7075": 2.81e-6, "steel": 7.85e-6, "NBR": 1.20e-6}  # kg/mm3
VENDOR_MASS = {"Gearbox": 0.27, "Motor": 0.20, "Brake": 0.15, "Cable_Gland_Lapp_SKINTOP_M16": 0.01}  # kg, datasheets
TOL = 0.05  # mm3, below this an overlap counts as touching


def vendor(name, rot, base):
    s = Part.read(os.path.join(VENDOR, name))
    s.Placement = App.Placement(base, rot)
    return s


def brake(x, za):
    """Brake with its leads turned towards the window; the lead stubs of the STEP end at the hood wall."""
    s = vendor("nanotec_bwa_035_5.step", App.Rotation(V(1, 0, 0), 60), V(x, 0, za))
    return s.common(C.box(x - 15, x + 25, -30, P.HOOD_Y - P.HOOD_WALL - 0.5, za - 25, za + 25))


def build_static(gear_pin, gear_racks, bev):
    xp, za = P.PINION_X, P.DRIVE_Z
    xf = D.gearbox_flange_x()                 # gearbox output flange face, output pointing -X
    gb_t = xf + 21.0                          # STEP: flange at local x = 21; turned 180 deg about Z
    gb_in = gb_t + 23.7                       # STEP: motor side at local x = -23.7
    motor_t = gb_in + 5.5                     # STEP: motor front at local z = 5.5
    brake_t = motor_t + 24.5 + 9.7            # STEP: rear boss local z = -24.5, brake face local x = -9.7
    parts = {
        "Housing": (C.housing_body(), "EN AW-6082", 0),
        "Cover": (C.cover(), "EN AW-6082", 0),
        "Rail": (C.rail(), "steel", 0),
        "Hood": (D.hood(), "EN AW-6082", 0),
        "Adapter_ISO9409": (C.iso_adapter(P.HOOD_TOP), "EN AW-6082", 0),
        "Bracket": (D.bracket(), "EN AW-6082", 0),
        "Gearbox": (vendor("nanotec_gp42_s1_srs.step", App.Rotation(V(0, 0, 1), 180), V(gb_t, 0, za)), "vendor", 0),
        "Motor": (vendor("nanotec_st4118s1404b.step", App.Rotation(V(0, 1, 0), -90), V(motor_t, 0, za)), "vendor", 0),   # connector +Y into the window
        "Brake": (brake(brake_t, za), "vendor", 0),
        "Pinion_Shaft": (D.pinion_shaft(gear_pin), "steel", "rot"),
        "Bevel_Wheel": (D.wheel(bev), "steel", "rot"),
        "Bevel_Pinion": (D.bevel_pinion(bev), "steel", "rotx"),
        "Bearing_608_lower": (D.bearing(xp, 0, P.BEARING_LOWER_Z), "steel", 0),
        "Bearing_608_upper": (D.bearing(xp, 0, P.BEARING_UPPER_Z), "steel", 0),
        "Setscrew_ISO4029-M4x4_Wheel": (F.iso4029_x(4, 4, xp - P.BEVEL_HUB_D / 2, 0, P.BEARING_UPPER_Z + P.BEARING[2] + 3, 1), "steel", "rot"),
        "Window_Plate": (D.window_plate(), "EN AW-6082", 0),
        "Cable_Gland_Lapp_SKINTOP_M16": (D.gland(), "vendor", 0),
    }
    for i, (x, z) in enumerate(D.window_screws(), 1):
        parts[f"Screw_ISO4762-M3x6_Window{i}"] = (F.oriented(F.iso4762(3, 6, 0, 0, 0), V(x, P.HOOD_Y + P.WINDOW_PLATE_T, z), V(0, -1, 0)), "steel", 0)
    for i, (x, y) in enumerate(D.bracket_screws(), 1):
        parts[f"Screw_ISO4762-M4x12_Bracket{i}"] = (F.iso4762(4, 12, x, y, P.PLATE_T + 6, up=False), "steel", 0)
    for i, (y, dz) in enumerate(D.gearbox_screws(), 1):
        parts[f"Screw_ISO4762-M4x12_Gearbox{i}"] = (F.oriented(F.iso4762(4, 12, 0, 0, 0), V(xf - P.BRACKET_T + 4, y, za + dz), V(1, 0, 0)), "steel", 0)
    for i, (x, y) in enumerate(D.hood_screws(), 1):
        parts[f"Screw_ISO4762-M4x12_Hood{i}"] = (F.iso4762(4, 12, x, y, P.PLATE_T + 6, up=False), "steel", 0)
    for side, tag in ((-1, "L"), (1, "R")):
        parts[f"Carriage_{tag}"] = (C.carriage(side * P.CARR_X0), "steel", side)
        parts[f"Slide_{tag}"] = (C.slide(side), "EN AW-6082", side)
        parts[f"Rack_{tag}"] = (C.rack(gear_racks[P.RACK_TEETH_SIDE[side]], side), "steel", side)
        parts[f"Holder_{tag}"] = (C.holder(side), "EN AW-6082", side)
        jb, plate, rubber = C.jaw(side)
        parts[f"Jaw_{tag}"] = (jb, "EN AW-7075", side)
        parts[f"PadPlate_{tag}"] = (plate, "steel", side)
        parts[f"Pad_{tag}"] = (rubber, "NBR", side)
        _, _, xj1, _, _ = C.jaw_x(side)
        for i, (y, z) in enumerate(C.pad_screws(side), 1):
            parts[f"Screw_ISO4762-M3x8_Pad{tag}{i}"] = (F.iso4762_x(3, 8, xj1 + side * 0.5, y, z, -side), "steel", side)  # tip at the rubber
            parts[f"Washer_ISO7089-3_Pad{tag}{i}"] = (F.iso7089_x(3, xj1, y, z, side), "steel", side)
        parts[f"Plunger_GN617-5-A_{tag}"] = (C.plunger(side), "steel", side)
        for i, y in enumerate((-P.DOWEL_Y, P.DOWEL_Y), 1):
            parts[f"Dowel_ISO8734-4x12_{tag}{i}"] = (C.dowel(side, y), "steel", side)
        for i, (x, y) in enumerate(C.holder_screws(side), 1):
            parts[f"Screw_ISO4762-M4x25_Holder{tag}{i}"] = (F.iso4762(4, 25, x, y, P.HOLDER_Z_LOW + 4.5), "steel", side)
        parts[f"Dowel_ISO8734-3x14_Rack{tag}"] = (C.rack_dowel(side), "steel", side)
        for i, (x, y) in enumerate(C.rack_screws(side), 1):
            parts[f"Screw_ISO4762-M3x10_Rack{tag}{i}"] = (F.iso4762(3, 10, x, y, P.SLIDE_Z_TOP - P.SLIDE_T + 3.2), "steel", side)
        for i, (x, y) in enumerate(C.carriage_holes(side * P.CARR_X0), 1):
            parts[f"Screw_ISO4762-M3x8_Carriage{tag}{i}"] = (F.iso4762(3, 8, x, y, P.SLIDE_Z_TOP - P.SLIDE_T + P.SLIDE_CBORE_CARR), "steel", side)
    for i, x in enumerate(C.rail_hole_x(), 1):
        parts[f"Screw_ISO4762-M3x8_Rail{i}"] = (F.iso4762(3, 8, x, P.RAIL_Y, P.RAIL_BASE_Z - P.RAIL_H + P.RAIL_CBORE_DEPTH), "steel", 0)
    for i, (x, y) in enumerate(C.cover_screws(), 1):
        parts[f"Screw_ISO4762-M3x10_Cover{i}"] = (F.iso4762(3, 10, x, y, P.POCKET_Z - P.COVER_T), "steel", 0)
    z_ad = P.HOOD_TOP
    for i, (x, y) in enumerate(C.adapter_screws(), 1):
        parts[f"Screw_ISO10642-M4x16_Adapter{i}"] = (F.iso10642(4, 16, x, y, z_ad + P.ADAPTER_T), "steel", 0)
    return parts, {"flange_z": z_ad + P.ADAPTER_T}


def scene_parts():
    """Part B on a table, in the gripper frame at grip height."""
    b = Part.read(os.path.join(HERE, "..", "..", "3D-Daten der Bauteile-20260904", "Bauteil_B.STEP"))
    b.translate(V(*P.PART_OFFSET))
    z_table = b.BoundBox.ZMin
    table = C.box(-110, 110, -80, 80, z_table - 10, z_table)
    return {"Part_B": b, "Table": table}


def at(parts, d, theta0):
    """Shapes at opening travel d (0 = closed stop). Pinion shaft turns +d/r for opening, bevel pinion with it."""
    out = {}
    for name, (shape, mat, mov) in parts.items():
        s = shape.copy()
        if mov == "rot":
            s.rotate(V(P.PINION_X, 0, 0), V(0, 0, 1), math.degrees(theta0 + d / (P.PINION_D / 2)))
        elif mov == "rotx":
            s.rotate(V(0, 0, P.DRIVE_Z), V(1, 0, 0), math.degrees(theta0 + d / (P.PINION_D / 2)))
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
    pin = parts["Pinion_Shaft"][0]
    rl, rr = parts["Rack_L"][0], parts["Rack_R"][0]

    def rot(a):
        s = pin.copy()
        s.rotate(V(P.PINION_X, 0, 0), V(0, 0, 1), a)
        return s

    best = min((overlap(rot(a / 4), rl), a / 4) for a in range(0, 72))
    theta = best[1]
    best_r = (1e9, 0.0)
    for i in range(0, 96):
        dx = i * math.pi / 96
        s = rr.copy()
        s.translate(V(dx, 0, 0))
        best_r = min(best_r, (overlap(rot(theta), s), dx))
    rr2 = rr.copy()
    rr2.translate(V(best_r[1], 0, 0))
    parts["Rack_R"] = (rr2, parts["Rack_R"][1], parts["Rack_R"][2])
    # bevel pair: turn the bevel pinion about its own axis until it meshes with the wheel at theta
    wh = parts["Bevel_Wheel"][0].copy()
    wh.rotate(V(P.PINION_X, 0, 0), V(0, 0, 1), theta)
    bp = parts["Bevel_Pinion"][0]

    def rotb(a):
        s = bp.copy()
        s.rotate(V(0, 0, P.DRIVE_Z), V(1, 0, 0), a)
        return s
    best_b = min((overlap(rotb(a / 4), wh), a / 4) for a in range(0, 72))
    parts["Bevel_Pinion"] = (rotb(best_b[1] - theta), parts["Bevel_Pinion"][1], "rotx")   # at() adds theta again
    print(f"bevel phase {best_b[1]:.1f} deg, residual overlap {best_b[0]:.3f} mm3")
    return math.radians(theta), best[0], best_r[0], best_r[1]


SKIP = {frozenset(p) for p in [("Gearbox", "Motor"), ("Motor", "Brake"), ("Gearbox", "Brake"), ("Gearbox", "Bevel_Pinion")]}
SKIP |= {frozenset(("Gearbox", f"Screw_ISO4762-M4x12_Gearbox{i}")) for i in range(1, 5)}  # thread depth in the vendor model unknown


def wall_check(parts):
    """Smallest wall around every hole of every own machined part."""
    own = [n for n, (_, m, _) in parts.items() if m in ("EN AW-6082", "EN AW-7075") or n.startswith("PadPlate")]
    thin = []
    for name in own:
        s = parts[name][0]
        faces = s.Faces
        edges_of = [set(e.hashCode() for e in f.Edges) | set(v.hashCode() for v in f.Vertexes) for f in faces]
        for i, f in enumerate(faces):
            su = f.Surface
            if not isinstance(su, Part.Cylinder) or su.Radius > 6.5:
                continue
            u0, u1, v0, v1 = f.ParameterRange
            p = f.valueAt((u0 + u1) / 2, (v0 + v1) / 2)
            n = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
            to_axis = su.Center + su.Axis * (p - su.Center).dot(su.Axis) - p
            if to_axis.dot(n) <= 0:          # boss, not a hole
                continue
            if (u1 - u0) <= math.pi / 2 + 0.05 and all(not isinstance(g.Surface, Part.Cylinder) or g.Surface.Radius != su.Radius
                                                       for j, g in enumerate(faces) if j != i and edges_of[i] & edges_of[j]):
                continue                     # quarter cylinder = pocket corner fillet, not a hole
            near = f.BoundBox
            near.enlarge(P.WALL_MIN)                 # in place
            best = 1e9
            # a wall that is cut into or touched becomes a neighbour of the hole: catch it here
            for j, g in enumerate(faces):
                if j == i or not (edges_of[i] & edges_of[j]):
                    continue
                gs = g.Surface
                if isinstance(gs, Part.Plane) and abs(abs(gs.Axis.dot(su.Axis)) - 1) < 1e-6:
                    continue                 # entry face or counterbore floor
                if isinstance(gs, (Part.Cylinder, Part.Cone)) and (gs.Center - su.Center).cross(su.Axis).Length < 1e-3:
                    continue                 # coaxial: counterbore, thread core, countersink
                shared = [e for e in f.Edges if any(e.isSame(e2) for e2 in g.Edges)]
                if not shared:
                    best = 0.0               # touches only at a point
                    continue
                pt = shared[0].valueAt(sum(shared[0].ParameterRange) / 2)
                nf = f.normalAt(*f.Surface.parameter(pt))
                ng = g.normalAt(*g.Surface.parameter(pt))
                if nf.dot(ng) < 0.98:        # smooth continuation (slot side) is fine, anything else is a breakout
                    best = 0.0
            own_radii = {round(g.Surface.Radius, 3) for g in faces if isinstance(g.Surface, Part.Cylinder)
                         and abs(abs(g.Surface.Axis.dot(su.Axis)) - 1) < 1e-6
                         and (g.Surface.Center - su.Center).cross(su.Axis).Length <= 2 * P.RACK_SLOT + 1e-3}
            for j, g in enumerate(faces):
                if j == i or edges_of[i] & edges_of[j]:
                    continue
                gs = g.Surface
                if isinstance(gs, Part.Cylinder) and abs(abs(gs.Axis.dot(su.Axis)) - 1) < 1e-6:
                    off = (gs.Center - su.Center).cross(su.Axis).Length
                    if off < 1e-3:
                        continue             # same hole: other half, counterbore or thread core
                    if off <= 2 * P.RACK_SLOT + 1e-3:
                        continue             # other end of the same slot
                if isinstance(gs, Part.Cone) and abs(abs(gs.Axis.dot(su.Axis)) - 1) < 1e-6 \
                        and (gs.Apex - su.Center).cross(su.Axis).Length < 1e-3:
                    continue                 # countersink of the same hole
                if isinstance(gs, Part.Plane) and abs(gs.Axis.dot(su.Axis)) < 1e-6:
                    d_axis = abs((su.Center - gs.Position).dot(gs.Axis))
                    if any(abs(d_axis - r) < 1e-3 for r in own_radii):
                        continue             # straight side of the same slot or slotted counterbore
                if not g.BoundBox.intersect(near):
                    continue
                best = min(best, f.distToShape(g)[0])
            if best < P.WALL_MIN:
                c = f.BoundBox.Center
                thin.append((name, round(su.Radius * 2, 2), round(best, 2), (round(c.x, 1), round(c.y, 1), round(c.z, 1))))
    return thin


BOM_INFO = {
    "Housing": ("Gear housing", "own part, milled", "EN AW-6082 T6"),
    "Cover": ("Bottom cover", "own part, cut + milled", "EN AW-6082 T6"),
    "Hood": ("Drive hood", "own part, milled", "EN AW-6082 T6"),
    "Bracket": ("Drive bracket", "own part, milled", "EN AW-6082 T6"),
    "Pinion_Shaft": ("Pinion shaft m = 1, z = 20", "own part, turned + hobbed", "16MnCr5 case hardened"),
    "Bevel_Wheel": ("Bevel gear m = 1, z = 20, i = 1, bored 8 H7", "Maedler", "C45"),
    "Bevel_Pinion": ("Bevel gear m = 1, z = 20, i = 1, bored 8 H7", "Maedler", "C45"),
    "Bearing_608_lower": ("Deep groove ball bearing 608-2Z", "DIN 625 (Schaeffler / SKF)", "steel"),
    "Bearing_608_upper": ("Deep groove ball bearing 608-2Z", "DIN 625 (Schaeffler / SKF)", "steel"),
    "Adapter_ISO9409": ("Robot adapter ISO 9409-1-40-4-M6", "own part, turned", "EN AW-6082 T6"),
    "Window_Plate": ("Cable window plate", "own part, cut", "EN AW-6082 T6"),
    "Slide": ("Slide", "own part, milled", "EN AW-6082 T6"),
    "Holder": ("Jaw holder", "own part, milled", "EN AW-6082 T6"),
    "Jaw": ("Jaw", "own part, milled", "EN AW-7075 T6"),
    "PadPlate": ("Pad plate, rubber-metal (steel 3 + NBR 70 Sh A 3)", "bought to drawing", "S235 / NBR"),
    "Pad": None,                                   # part of the pad plate
    "Rail": ("Miniature ball rail size 9, L = 110, T1 = 15", "Bosch Rexroth R0445", "steel"),
    "Carriage": ("Ball carriage SLS size 9", "Bosch Rexroth R0444 913 01", "steel"),
    "Rack": ("Rack m = 1, 10 x 12, cut to 56.5", "Maedler", "C45"),
    "Gearbox": ("Planetary gearbox GP42-S1-7-SR", "Nanotec", "-"),
    "Motor": ("Stepper motor ST4118S1404-B", "Nanotec", "-"),
    "Brake": ("Spring-applied brake BRAKE-BWA-0,35-5, 24 V", "Nanotec", "-"),
    "Cable_Gland_Lapp_SKINTOP_M16": ("Cable gland SKINTOP STR-M 16x1.5", "Lapp", "PA"),
    "Plunger_GN617-5-A": ("Indexing plunger GN 617-5-A", "Ganter", "steel"),
}


def bom(parts, masses):
    import re
    rows = {}
    for name in parts:
        if name.split("_")[0] in ("Screw", "Dowel", "Washer", "Setscrew"):
            std = name.split("_")[1]
            m = re.match(r"(ISO\d+)-(.*)", std)
            grade = {"4762": "steel 8.8", "10642": "steel 8.8", "4029": "steel 45H", "7089": "steel 200 HV"}.get(m.group(1)[3:], "steel")
            key = (f"ISO {m.group(1)[3:]} {m.group(2)}", "standard part", grade)
        else:
            info = BOM_INFO.get(re.sub(r"_[LR]$", "", name), (name, "?", "?"))
            if info is None:
                continue
            key = info
        r = rows.setdefault(key, [0, 0.0])
        r[0] += 1
        r[1] += masses.get(name, 0.0)
    order = lambda kv: (kv[0][1] == "standard part", kv[0][1].startswith("own") is False, kv[0][0])
    lines = ["Pos;Qty;Designation;Standard / supplier;Material;Mass each [g]"]
    for i, (k, (q, m)) in enumerate(sorted(rows.items(), key=order), 1):
        lines.append(f"{i};{q};{k[0]};{k[1]};{k[2]};{1000 * m / q:.0f}")
    return lines


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
    gear_pin, gear_racks = C.gears(doc)
    bev = D.bevel_gears(doc)
    parts, info = build_static(gear_pin, gear_racks, bev)
    theta0, ov_l, ov_r, dx_r = phase_pinion(parts)
    parts["Rack_R"] = (C.rack(gear_racks[P.RACK_TEETH_SIDE[1]], 1, shift=dx_r), "steel", 1)   # holes cut after the shift
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

    thin = wall_check(parts)
    report["thin_walls"] = thin
    print(f"wall check (min {P.WALL_MIN} mm around holes): {thin or 'ok'}")

    seq_hits, n = [], 0
    t = 0.0
    while t <= demo_seq.DURATION:
        d, zg, zp = demo_seq.state(t, P.D_GRIP, P.D_OPEN)
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
    m_sum, mom = 0.0, V(0, 0, 0)
    for n, (sh, _, _) in parts.items():
        if masses[n] <= 0:
            continue
        sol = sh.Solids
        c = sum((so.CenterOfMass * so.Volume for so in sol), V(0, 0, 0)) * (1 / sum(so.Volume for so in sol)) if sol else sh.BoundBox.Center
        mom += c * masses[n]
        m_sum += masses[n]
    cog = mom * (1 / m_sum)
    print(f"centre of mass from the flange face: x {cog.x:.1f}, y {cog.y:.1f}, z {cog.z - info['flange_z']:.1f} mm; "
          f"TCP (grip centre) z {P.PART_OFFSET[2] + 17.5 - info['flange_z']:.1f} mm")
    report["cog_from_flange_mm"] = [cog.x, cog.y, cog.z - info["flange_z"]]
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
        o.addProperty("App::PropertyString", "Role", "Gripper").Role = {"rot": "pinion", "rotx": "pinion_x"}.get(mov, "slide" if mov else "static")
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
    lines = bom(parts, masses)
    with open(os.path.join(OUT, "bom.csv"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"BOM: {len(lines) - 1} positions -> out/bom.csv")
    report["mass_kg"] = mass
    report["mass_parts_kg"] = masses
    with open(os.path.join(OUT, "checks.json"), "w") as f:
        json.dump(report, f, indent=1)


main()
