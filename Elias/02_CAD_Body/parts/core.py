"""Own parts and simplified catalogue parts. All shapes in the gripper frame (see gripper_params)."""
import math

import Part
from FreeCAD import Vector as V

import gripper_params as P


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cylz(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0))


def cylx(r, x0, x1, y=0.0, z=0.0):
    x0, x1 = sorted((x0, x1))
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def rounded_box(x0, x1, y0, y1, z0, z1, r):
    b = box(x0, x1, y0, y1, z0, z1)
    return b.makeFillet(r, [e for e in b.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[-1].Point.z) > 1e-6])


def holes_pcd(r_hole, pcd, z0, z1, angles=(45, 135, 225, 315)):
    return [cylz(r_hole, z0, z1, pcd / 2 * math.cos(math.radians(a)), pcd / 2 * math.sin(math.radians(a))) for a in angles]


def cut_all(shape, tools):
    for t in tools:
        shape = shape.cut(t)
    return shape


def rail_hole_x():
    return [-P.RAIL_L / 2 + P.RAIL_T1 + i * P.RAIL_PITCH for i in range(P.RAIL_NB)]


def carriage_holes(xc):
    return [(xc + dx, P.RAIL_Y + dy) for dx in (-P.CARRIAGE_HOLES_E1 / 2, P.CARRIAGE_HOLES_E1 / 2)
            for dy in (-P.CARRIAGE_HOLES_E2 / 2, P.CARRIAGE_HOLES_E2 / 2)]


def jaw_x(side):
    """x of pad face, jaw inner/outer face, holder outer faces at d = 0."""
    xp = side * P.PAD_X0
    xj0 = xp + side * P.PAD_T
    xj1 = xj0 + side * P.JAW_T
    return xp, xj0, xj1, xj1 + side * P.HOLDER_T_TOP, xj1 + side * P.HOLDER_T_LOW


def holder_screws(side):
    _, _, xj1, xh_top, _ = jaw_x(side)
    return [((xj1 + xh_top) / 2, y) for y in (-P.HOLDER_SCREW_Y, P.HOLDER_SCREW_Y)]


def cover_screws():
    hx = P.HOUSING_X / 2
    y0, y1 = P.HOUSING_Y
    return [(x, y) for x in (-hx + P.WALL / 2, hx - P.WALL / 2) for y in (y0 + 15, y1 - 15)]


def chamfer_z_edges(s, z, size):
    """Chamfer the outer horizontal edges lying at height z; keeps the part if OCC refuses."""
    edges = [e for e in s.Edges if all(abs(v.Point.z - z) < 1e-6 for v in e.Vertexes)
             and e.BoundBox.ZLength < 1e-6 and e.Length > 5]
    try:
        return s.makeChamfer(size, edges)
    except Part.OCCError:
        return s


def housing_body():
    hx = P.HOUSING_X / 2
    y0, y1 = P.HOUSING_Y
    s = rounded_box(-hx, hx, y0, y1, P.POCKET_Z, P.PLATE_T, P.HOUSING_R)
    s = chamfer_z_edges(s, P.PLATE_T, P.EDGE_CHAMFER)
    s = s.cut(rounded_box(-hx + P.WALL, hx - P.WALL, y0 + P.WALL, y1 - P.WALL, P.POCKET_Z - 1, 0, 6.0))   # R6: standard d10 cutter
    tools = [cylz(8.0, -1, P.PLATE_T + 1)]                                  # shaft + gearbox seal ring d14.7
    tools += holes_pcd(2.25, 36.0, -1, P.PLATE_T + 1)                       # gearbox M4 through
    tools += holes_pcd(3.75, 36.0, -1, P.GEAR_SCREW_CBORE)                  # flush heads
    tools += [cylz(1.25, -1, 8, x, P.RAIL_Y) for x in rail_hole_x()]        # rail M3 tapped
    tools += [cylz(2.25, -1, P.PLATE_T + 1, x, y) for x, y in P.TOWER_BOSS]  # tower M4 from below
    tools += [cylz(3.75, -1, P.GEAR_SCREW_CBORE, x, y) for x, y in P.TOWER_BOSS]
    tools += [cylz(1.65, P.POCKET_Z - 1, P.POCKET_Z + 14, x, y) for x, y in cover_screws()]   # cover M4 tapped
    return cut_all(s, tools)


def cover():
    hx = P.HOUSING_X / 2
    y0, y1 = P.HOUSING_Y
    z1 = P.POCKET_Z
    s = rounded_box(-hx, hx, y0, y1, z1 - P.COVER_T, z1, P.HOUSING_R)
    s = chamfer_z_edges(s, z1 - P.COVER_T, P.COVER_CHAMFER)
    tools = []
    for sg in (1, -1):
        _, _, xj1, xh_top, _ = jaw_x(sg)
        tools.append(box(xj1 - sg * 1, xh_top + sg * (P.D_OPEN + 1), -P.HOLDER_W / 2 - 1, P.HOLDER_W / 2 + 1, z1 - P.COVER_T - 1, z1 + 1))
    tools.append(cylz(6.0, z1 - P.COVER_T - 1, z1 + 1))                    # access to the pinion set screw
    tools += [cylz(2.25, z1 - P.COVER_T - 1, z1 + 1, x, y) for x, y in cover_screws()]
    return cut_all(s, tools)


def rail():
    x = P.RAIL_L / 2
    y = P.RAIL_Y
    s = box(-x, x, y - P.RAIL_W / 2, y + P.RAIL_W / 2, -P.RAIL_H, 0)
    for xi in rail_hole_x():
        s = s.cut(cylz(P.RAIL_HOLE_D / 2, -P.RAIL_H - 1, 1, xi, y))
        s = s.cut(cylz(P.RAIL_CBORE_D / 2, -P.RAIL_H - 1, -P.RAIL_H + 4.5, xi, y))
    return s


def carriage(xc):
    """Hanging carriage: mounting face at z = -CARRIAGE_H, wraps the rail."""
    y = P.RAIL_Y
    zb, zt = -P.CARRIAGE_H, -P.CARRIAGE_GAP
    hb, hl = P.CARRIAGE_L_BODY / 2, P.CARRIAGE_L / 2
    body = box(xc - hb, xc + hb, y - P.CARRIAGE_W / 2, y + P.CARRIAGE_W / 2, zb, zt)
    seals = box(xc - hl, xc + hl, y - P.CARRIAGE_W / 2 + 0.5, y + P.CARRIAGE_W / 2 - 0.5, zb + 0.5, zt - 0.5)
    s = body.fuse(seals)
    s = s.cut(box(xc - hl - 1, xc + hl + 1, y - P.RAIL_W / 2, y + P.RAIL_W / 2, -P.RAIL_H, zt + 1))
    s = cut_all(s, [cylz(1.25, zb - 1, zb + 3.5, hx, hy) for hx, hy in carriage_holes(xc)])
    return s.removeSplitter()


def slide(side):
    """side = +1 right jaw (rack at -Y), -1 left jaw (rack at +Y); built at d = 0."""
    xc = side * P.CARR_X0
    hl = P.CARRIAGE_L / 2
    z0, z1 = P.SLIDE_Z_TOP - P.SLIDE_T, P.SLIDE_Z_TOP
    y_car0 = P.RAIL_Y - P.CARRIAGE_W / 2 - P.SLIDE_EDGE
    _, _, _, xh_top, _ = jaw_x(side)
    x_outer = side * max(abs(xc) + hl, abs(xh_top))
    s = box(xc - side * hl, x_outer, y_car0, P.RAIL_Y + P.CARRIAGE_W / 2, z0, z1)
    y_top = (P.RACK_H + P.PINION_D / 2 + 0.5) if side < 0 else P.HOLDER_W / 2
    s = s.fuse(box(side * P.SLIDE_TRIM_X, x_outer, y_car0, y_top, z0, z1))
    ry = -side * (P.PINION_D / 2 - 1.0)
    rack_x_out = xc + side * (-P.RACK_REL)
    s = s.fuse(box(side * P.SLIDE_TRIM_X, rack_x_out, ry, ry - side * P.RACK_H, z1, z1 + P.RACK_GAP))   # boss under the rack
    tools = []
    for hx, hy in carriage_holes(xc):
        tools += [cylz(1.7, z0 - 1, z1 + 1, hx, hy), cylz(3.0, z0 - 1, z0 + 3.2, hx, hy)]
    tools += [cylz(1.65, z0 - 1, z1 + 1, hx, hy) for hx, hy in holder_screws(side)]   # holder M4 tapped
    for hx, hy in rack_screws(side):
        tools += [cylz(1.7, z0 - 1, z1 + P.RACK_GAP + 1, hx, hy), cylz(3.0, z0 - 1, z0 + 3.2, hx, hy)]
    return cut_all(s, tools).removeSplitter()


def rack_screws(side):
    """Two M3 from below through the slide into the rack body, behind the teeth."""
    y = -side * (P.PINION_D / 2 + 2.25 + (P.RACK_H - 2.25) / 2)
    return [(side * 17.5, y), (side * 27.0, y)]


def holder(side):
    """Jaw holder under the slide; the jaw rests on its inner face, pins + plunger along X."""
    _, _, xj1, xh_top, xh_low = jaw_x(side)
    w = P.HOLDER_W / 2
    z_cov = P.POCKET_Z - P.COVER_T - 0.5
    s = box(xj1, xh_top, -w, w, z_cov, P.SLIDE_Z_TOP - P.SLIDE_T)
    s = s.fuse(box(xj1, xh_low, -w, w, P.HOLDER_Z_LOW, z_cov))
    tools = [cylx(P.DOWEL_D / 2, xj1 - side, xh_low + side, y, P.DOWEL_Z) for y in (-P.DOWEL_Y, P.DOWEL_Y)]
    tools.append(cylx(4.5, xj1 - side, xh_low + side, 0, P.PLUNGER_Z))     # M10x1 tapped
    for hx, hy in holder_screws(side):
        tools += [cylz(2.25, P.HOLDER_Z_LOW - 1, 0, hx, hy), cylz(4.0, P.HOLDER_Z_LOW - 1, P.HOLDER_Z_LOW + 4.5, hx, hy)]
    return cut_all(s, tools).removeSplitter()


def pad_screws(side):
    zc = (P.PAD_Z[0] + P.PAD_Z[1]) / 2
    return [(0.0, zc - P.PAD_SCREW_DZ), (0.0, zc + P.PAD_SCREW_DZ)]


def jaw(side):
    """EN AW-7075 jaw with a pocket for the bought rubber-metal pad plate and quick-change holes at the top."""
    xp, xj0, xj1, _, _ = jaw_x(side)
    w = P.JAW_W / 2
    s = box(xj0, xj1, -w, w, P.JAW_Z_BOTTOM, P.JAW_Z_TOP)
    x_plate = xj0 + side * P.PAD_PLATE_T
    s = s.cut(box(xj0 - side, x_plate, -w + 1, w - 1, P.PAD_Z[0], P.PAD_Z[1]))
    tools = [cylx(P.DOWEL_D / 2, xj1 - side * 6.5, xj1 + side, y, P.DOWEL_Z) for y in (-P.DOWEL_Y, P.DOWEL_Y)]
    tools.append(cylx(P.PLUNGER_PIN_D / 2 + 0.1, xj1 - side * 6, xj1 + side, 0, P.PLUNGER_Z))
    tools += [cylx(1.7, x_plate - side, xj1 + side, y, z) for y, z in pad_screws(side)]   # M3 clearance
    s = cut_all(s, tools)
    plate = box(xj0, x_plate, -w + 1, w - 1, P.PAD_Z[0], P.PAD_Z[1])
    plate = cut_all(plate, [cylx(1.25, xj0 - side, x_plate + side, y, z) for y, z in pad_screws(side)])  # M3 tapped through, rubber covers it
    rubber = box(xp, xj0, -w + 1.5, w - 1.5, P.PAD_Z[0] + 0.5, P.PAD_Z[1] - 0.5)
    return s.removeSplitter(), plate, rubber


def dowel(side, y):
    _, _, xj1, _, _ = jaw_x(side)
    return cylx(P.DOWEL_D / 2, xj1 - side * 6, xj1 + side * (P.DOWEL_L - 6), y, P.DOWEL_Z)


def plunger(side):
    """GN 617-5-A, simplified from the Ganter dimension table, axis along X, knob outwards."""
    _, _, xj1, _, xh_low = jaw_x(side)
    x_tip = xj1 - side * 5.0
    x_body = xh_low - side * 10.0                    # 10 mm of thread in the holder
    x_body_end = x_body + side * P.PLUNGER_L3
    x_knob = x_body_end + side * 3.0
    x_end = x_tip + side * P.PLUNGER_L1
    s = cylx(P.PLUNGER_PIN_D / 2, x_tip, x_body + side * 0.1)
    s = s.fuse(cylx(4.5, x_body, x_body_end))
    s = s.fuse(cylx(2.5, x_body_end - side * 0.1, x_knob + side * 0.1))
    s = s.fuse(cylx(P.PLUNGER_KNOB_D / 2, x_knob, x_end))
    s.translate(V(0, 0, P.PLUNGER_Z))
    return s.removeSplitter()


def adapter_screws():
    r = P.ADAPTER_SCREW_PCD / 2
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]


def iso_adapter(z0):
    """Round adapter plate to ISO 9409-1-50-4-M6, fixed to the housing lid from the top."""
    s = cylz(P.ADAPTER_D / 2, z0, z0 + P.ADAPTER_T)
    s = chamfer_z_edges(s, z0 + P.ADAPTER_T, 1.0)
    s = s.fuse(cylz(P.ISO_SPIGOT_D / 2, z0 + P.ADAPTER_T, z0 + P.ADAPTER_T + P.ISO_SPIGOT_H))
    tools = holes_pcd(3.3, P.ISO_PCD, z0 - 1, z0 + 20)
    tools += holes_pcd(5.5, P.ISO_PCD, z0 - 1, z0 + 6.5)
    tools.append(cylz(3.0, z0 + 2, z0 + 20, P.ISO_PCD / 2, 0))             # locating pin d6 H7
    for x, y in adapter_screws():
        tools.append(cylz(2.75, z0 - 1, z0 + P.ADAPTER_T + 1, x, y))
        tools.append(Part.makeCone(2.75, 5.2, 2.45, V(x, y, z0 + P.ADAPTER_T - 2.45)))   # ISO 10642 M5 countersink
    return cut_all(s, tools)


def tower(z_top):
    """Motor housing, milled from the solid, open at the bottom, lid with opening for the ISO screws."""
    (x0, x1), (y0, y1) = P.TOWER_X, P.TOWER_Y
    t, r = P.TOWER_WALL, P.TOWER_R
    s = rounded_box(x0, x1, y0, y1, P.PLATE_T, z_top + P.LID_T, r)
    s = chamfer_z_edges(s, z_top + P.LID_T, P.EDGE_CHAMFER)
    s = s.cut(rounded_box(x0 + t, x1 - t, y0 + t, y1 - t, P.PLATE_T - 1, z_top, r - t))
    for x, y in P.TOWER_BOSS:
        s = s.fuse(cylz(5.0, P.PLATE_T, P.PLATE_T + P.TOWER_BOSS_H, x, y))
    tools = [cylz(P.LID_HOLE_D / 2, z_top - 1, z_top + P.LID_T + 1)]
    tools += [cylz(2.1, z_top + P.LID_T - 10, z_top + P.LID_T + 1, x, y) for x, y in adapter_screws()]   # M5 tapped
    tools += [cylz(1.65, P.PLATE_T - 1, P.PLATE_T + 14, x, y) for x, y in P.TOWER_BOSS]                  # M4 tapped from below
    tools.append(box(x1 - t - 1, x1 + 1, -14, 21, 68, z_top - 4))          # connector and brake leads
    tools += [cylx(1.25, x1 - t - 1, x1 + 1, y, z) for y, z in window_screws(z_top)]   # M3 for the window plate
    return cut_all(s, tools).removeSplitter()


def gears(doc):
    """Involute pinion and rack from the Gears workbench, returned in their local frames."""
    from freecad.gears.involutegear import InvoluteGear
    from freecad.gears.involutegearrack import InvoluteGearRack
    g = doc.addObject("Part::FeaturePython", "_pinion_gen")
    InvoluteGear(g)
    g.num_teeth, g.module, g.height = P.Z_RITZEL, P.MODUL, P.PINION_B
    r = doc.addObject("Part::FeaturePython", "_rack_gen")
    InvoluteGearRack(r)
    r.num_teeth, r.module, r.height = P.RACK_TEETH, P.MODUL, P.PINION_B
    r.thickness = P.RACK_H - 2.25 * P.MODUL
    doc.recompute()
    pin, rk = g.Shape.copy(), r.Shape.copy()
    doc.removeObject(g.Name)
    doc.removeObject(r.Name)
    return pin, rk


def pinion(gear_shape):
    s = gear_shape.copy()
    s.translate(V(0, 0, P.RACK_Z0 - s.BoundBox.ZMin))
    s = s.fuse(cylz(P.PINION_HUB_D / 2, P.RACK_Z0 - P.PINION_HUB_L, P.RACK_Z0 + 0.1))
    s = s.cut(cylz(P.PINION_BORE / 2, P.RACK_Z0 - P.PINION_HUB_L - 1, P.RACK_Z0 + P.PINION_B + 1))
    s = s.cut(box(-1.5, 1.5, P.PINION_BORE / 2 - 0.5, P.PINION_BORE / 2 + 1.8, P.RACK_Z0 - P.PINION_HUB_L - 1, P.RACK_Z0 + P.PINION_B + 1))
    s = s.cut(cylx(1.65, -P.PINION_HUB_D / 2 - 1, -P.PINION_BORE / 2 + 0.5, 0, P.SETSCREW_Z))   # M4 tapped
    return s.removeSplitter()


def setscrew():
    """ISO 4029 M4x4 (cup point), radial in the hub, tip on the shaft."""
    s = cylx(1.65, -P.PINION_HUB_D / 2, -P.PINION_BORE / 2, 0, P.SETSCREW_Z)
    return s.cut(cylx(1.0, -P.PINION_HUB_D / 2 - 0.1, -P.PINION_HUB_D / 2 + 2, 0, P.SETSCREW_Z))


def window_screws(z_top):
    x1 = P.TOWER_X[1]
    return [(y, z) for y in (-18.0, 25.0) for z in (64.0, z_top)]


def window_plate(z_top):
    """Cover over the cable window with the cable gland."""
    x1 = P.TOWER_X[1]
    zt = z_top - 4 + 4
    s = box(x1, x1 + P.WINDOW_PLATE_T, -22, 29, 60, zt + 4)
    zc = (68 + z_top - 4) / 2
    tools = [cylx(P.GLAND[1] / 2, x1 - 1, x1 + P.WINDOW_PLATE_T + 1, 3.5, zc)]
    tools += [cylx(1.7, x1 - 1, x1 + P.WINDOW_PLATE_T + 1, y, z) for y, z in window_screws(z_top)]
    return cut_all(s, tools), zc


def gland(zc):
    _, m, dk, l_out = P.GLAND
    x1 = P.TOWER_X[1] + P.WINDOW_PLATE_T
    s = cylx(m / 2, x1 - P.WINDOW_PLATE_T - 6, x1)                                  # thread + lock nut inside
    s = s.fuse(cylx(dk / 2, x1, x1 + 6))                                           # hex body (as cylinder)
    s = s.fuse(cylx(dk / 2 - 2, x1 + 6, x1 + l_out))                                # dome nut
    s = s.cut(cylx(4.0, x1 - 20, x1 + l_out + 1))                                   # cable bore
    s.translate(V(0, 3.5, zc))
    return s


def rack(rack_shape, side):
    """Right jaw rack at -Y with teeth to +Y, left at +Y with teeth to -Y."""
    s = rack_shape.copy()
    bb = s.BoundBox
    probe = box(bb.XMax - 0.3, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax)
    tips_plus_x = s.common(probe).Volume < 0.5 * probe.Volume
    s.rotate(V(0, 0, 0), V(0, 0, 1), -90 if tips_plus_x == (side < 0) else 90)
    bb = s.BoundBox
    pitch = P.PINION_D / 2 + P.BACKLASH_SHIFT - P.MODUL
    dy = (pitch - bb.YMin) if side < 0 else (-pitch - bb.YMax)
    x_out = side * P.CARR_X0 + side * (-P.RACK_REL)
    dx = (x_out - bb.XMax) if side > 0 else (x_out - bb.XMin)
    s.translate(V(dx, dy, P.RACK_Z0 - bb.ZMin))
    return cut_all(s, [cylz(1.25, P.RACK_Z0 - 1, P.RACK_Z0 + 7, hx, hy) for hx, hy in rack_screws(side)])  # M3 tapped
