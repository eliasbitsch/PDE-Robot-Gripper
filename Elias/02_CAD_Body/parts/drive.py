"""Lying drive: bevel gear pair over an offset pinion shaft, hood with the robot adapter on top."""
import math

import Part
from FreeCAD import Vector as V

import gripper_params as P
from parts.core import box, cylz, cylx, cut_all, rounded_box, chamfer_z_edges

XP, ZA = P.PINION_X, P.DRIVE_Z


def bevel_gears(doc):
    from freecad.gears.bevelgear import BevelGear
    g = doc.addObject("Part::FeaturePython", "_bevel_gen")
    BevelGear(g)
    g.num_teeth, g.module, g.height, g.pitch_angle = P.BEVEL_Z, P.BEVEL_M, P.BEVEL_B, 45
    doc.recompute()
    s = g.Shape.copy()
    doc.removeObject(g.Name)
    return s                      # base (large end) at z = 0, apex at z = d/2 for 45 deg


def bearing(x, y, z0, axis="z"):
    """DIN 625 608-2Z, 8 x 22 x 7, as a solid ring with the seal recess."""
    d, D, b = P.BEARING
    if axis == "z":
        s = cylz(D / 2, z0, z0 + b, x, y).cut(cylz(d / 2, z0 - 1, z0 + b + 1, x, y))
        s = s.cut(cylz(D / 2 - 2, z0 + b - 0.3, z0 + b + 1, x, y).cut(cylz(d / 2 + 2, z0 - 1, z0 + b + 2, x, y)))
    return s.removeSplitter()


def pinion_shaft(gear_shape):
    """Pinion and shaft in one piece (own part), teeth below the plate, shaft up to the bevel wheel."""
    s = gear_shape.copy()
    s.translate(V(XP, 0, P.PINION_Z0 - s.BoundBox.ZMin))
    top = P.SHAFT_TOP if P.DRIVE == "electric" else P.BOSS_TOP - 0.5        # pneumatic: only the synchronising pinion
    s = s.fuse(cylz(P.BEARING[0] / 2, P.PINION_Z0 + P.PINION_B - 0.1, top, XP, 0))
    s = s.fuse(cylz(P.COLLAR_D / 2, P.PINION_Z0 + P.PINION_B - 0.1, P.BEARING_LOWER_Z, XP, 0))   # collar under the lower inner ring
    z471 = P.BEARING_UPPER_Z - 0.85
    s = s.cut(cylz(P.BEARING[0] / 2 + 1, z471 - 0.05, z471 + 0.85, XP, 0).cut(cylz(3.8, z471 - 1, z471 + 2, XP, 0)))  # DIN 471 groove
    if P.DRIVE == "electric":
        k0, k1 = key_z()
        s = s.cut(box(XP + P.BEARING[0] / 2 - P.KEY_T1, XP + P.BEARING[0] / 2 + 1, -P.KEY_B / 2, P.KEY_B / 2, k0, k1))   # keyway
    return s.removeSplitter()


def hub_z0():
    return P.BEARING_UPPER_Z + P.BEARING[2] + P.SHIM_T


def key_z():
    return hub_z0() + 0.5, hub_z0() + 0.5 + P.KEY_L


def key():
    """DIN 6885 A 2x2x12 in the shaft (t1 = 1.2) and the wheel hub (t2 = 1.0)."""
    k0, k1 = key_z()
    x0 = XP + P.BEARING[0] / 2 - P.KEY_T1 + 0.05
    return box(x0, x0 + P.KEY_H, -P.KEY_B / 2, P.KEY_B / 2, k0, k1)


def shim():
    """DIN 988 8 x 14 x SHIM_T between the upper inner ring and the wheel hub."""
    z0 = P.BEARING_UPPER_Z + P.BEARING[2]
    return cylz(7.0, z0, z0 + P.SHIM_T, XP, 0).cut(cylz(P.BEARING[0] / 2 + 0.05, z0 - 1, z0 + 1, XP, 0))


def circlip_shaft():
    """DIN 471 8 x 0.8 under the upper inner ring (fixed bearing)."""
    z1 = P.BEARING_UPPER_Z - 0.05
    return cylz(5.9, z1 - 0.8, z1, XP, 0).cut(cylz(3.85, z1 - 2, z1 + 1, XP, 0))


def circlip_bore():
    """DIN 472 22 x 1 above the upper outer ring (fixed bearing)."""
    z0 = P.BEARING_UPPER_Z + P.BEARING[2] + 0.15
    return cylz(11.45, z0, z0 + 1.0, XP, 0).cut(cylz(9.4, z0 - 1, z0 + 2, XP, 0))


def wheel(bev):
    """Bevel wheel on the pinion shaft: large end down, apex on the drive axis."""
    s = bev.copy()
    s.translate(V(XP, 0, ZA - P.BEVEL_Z * P.BEVEL_M / 2 + P.BEVEL_WHEEL_DZ))
    z_hub = hub_z0()
    s = s.fuse(cylz(P.BEVEL_HUB_D / 2, z_hub, ZA - P.BEVEL_Z * P.BEVEL_M / 2 + 0.1, XP, 0))
    s = s.cut(cylz(P.BEARING[0] / 2, z_hub - 1, ZA + 1, XP, 0))
    s = s.cut(box(XP + P.BEARING[0] / 2 - 1, XP + P.BEARING[0] / 2 + P.KEY_T2, -P.KEY_B / 2, P.KEY_B / 2, z_hub - 1, ZA + 1))   # keyway
    s = s.cut(cylx(1.65, XP - P.BEVEL_HUB_D / 2 - 1, XP - P.BEARING[0] / 2 + 0.5, 0, z_hub + 3))   # M4 set screw, axial only
    return s.removeSplitter()


def bevel_pinion(bev):
    """Bevel pinion on the gearbox shaft: axis along X, apex at the pinion shaft axis."""
    r = P.BEVEL_Z * P.BEVEL_M / 2
    s = bev.copy()
    s = s.fuse(cylz(P.BEVEL_HUB_D / 2, -P.BEVEL_HUB_L, 0.1))
    s = s.cut(cylz(P.BEARING[0] / 2, -P.BEVEL_HUB_L - 1, r + 1))
    # keyway for the key of the GP42-S1 SRS shaft: torque by the key, the set screw only holds it axially
    s = s.cut(box(-P.BEARING[0] / 2 - P.KEY_T2, -P.BEARING[0] / 2 + 1, -P.KEY_B / 2, P.KEY_B / 2, -P.BEVEL_HUB_L - 1, r + 1))
    s = s.cut(Part.makeCylinder(1.65, P.BEVEL_HUB_D / 2 + 1, V(0, 0, -P.BEVEL_HUB_L / 2), V(1, 0, 0)))  # M4 set screw (local +x = up)
    s.rotate(V(0, 0, 0), V(0, 1, 0), -90)        # local z -> global -X, local x -> global +Z
    s.translate(V(XP + r, 0, ZA))
    return s.removeSplitter()


def pinion_setscrew():
    """ISO 4029 M4x4 in the bevel pinion hub, in the pinion's frame as built by bevel_pinion()."""
    from parts import fasteners as F
    r = P.BEVEL_Z * P.BEVEL_M / 2
    s = F.iso4029_x(4, 4, P.BEVEL_HUB_D / 2, 0, -P.BEVEL_HUB_L / 2, -1)
    s.rotate(V(0, 0, 0), V(0, 1, 0), -90)
    s.translate(V(XP + r, 0, ZA))
    return s


def gearbox_flange_x():
    return XP + 5.0 + P.GEAR_SHAFT_L    # shaft end 5 mm from the pinion shaft axis, inside the m 1.5 bevel pinion's bore


def bracket_screws():  # M3
    return [(x, y) for x in (-40.0, -10.0) for y in (-P.BRACKET_FOOT_Y, P.BRACKET_FOOT_Y)]


def gearbox_screws():
    r = 18.0
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]   # (y, z - ZA)


def bracket():
    """Drive bracket: wall carries the gearbox output flange, two feet screwed to the top plate."""
    xf = gearbox_flange_x()
    w0, w1 = xf - P.BRACKET_T, xf
    hw = P.BRACKET_FOOT_W / 2
    s = box(w0, w1, -P.BRACKET_FOOT_Y - hw, P.BRACKET_FOOT_Y + hw, P.PLATE_T, ZA + 22)
    for sg in (-1, 1):
        s = s.fuse(box(-45, 0, sg * (P.BRACKET_FOOT_Y - hw), sg * (P.BRACKET_FOOT_Y + hw), P.PLATE_T, P.PLATE_T + 6))
    s = s.removeSplitter()
    tools = [cylx(8.0, w0 - 1, w1 + 1, 0, ZA)]                                         # shaft + seal ring
    for y, dz in gearbox_screws():
        tools += [cylx(2.25, w0 - 1, w1 + 1, y, ZA + dz), cylx(3.75, w0 - 1, w0 + 4, y, ZA + dz)]
    tools += [cylz(1.7, P.PLATE_T - 1, P.PLATE_T + 7, x, y) for x, y in bracket_screws()]    # M3 clearance in the feet
    return cut_all(s, tools)


def hood_screws():
    return [(sx * P.HOOD_SCREW_X, y) for sx in (-1, 1) for y in P.HOOD_SCREW_Y]


WIN_X, WIN_Z = (30.0, 60.0), (30.0, 48.0)


def window_screws():
    return [(x, z) for x in (WIN_X[0] - 5, WIN_X[1] + 5) for z in (WIN_Z[0] - 4, WIN_Z[1] + 4)]


def hood():
    """Flat hood over the drive, milled from the solid; a seat ring on top carries the robot adapter."""
    hx, y0, hy = P.HOUSING_X / 2, -P.HOOD_Y, P.HOOD_Y
    top = P.HOOD_TOP
    s = rounded_box(-hx, hx, y0, hy, P.PLATE_T, top, P.HOOD_R)
    s = chamfer_z_edges(s, top, P.EDGE_CHAMFER)
    s = s.fuse(cylz(P.ADAPTER_D / 2, top - 0.1, top + P.HOOD_SEAT))
    s = s.cut(rounded_box(-hx + P.HOOD_WALL, hx - P.HOOD_WALL, y0 + P.HOOD_WALL, hy - P.HOOD_WALL,
                          P.PLATE_T - 1, top - P.HOOD_TOP_T, P.HOOD_R - P.HOOD_WALL))
    for x, y in hood_screws():
        s = s.fuse(cylz(4.5, P.PLATE_T, P.PLATE_T + 13.5, x, y))   # 2.5 mm above the blind thread
    c = P.CABLE_SIDE
    for x, z in window_screws():                                   # 2 mm bosses: M3 gets 5 mm thread instead of 3
        s = s.fuse(Part.makeCylinder(3.5, 2.5, V(x, c * (hy - P.HOOD_WALL - 2.0), z), V(0, c, 0)))
    s = s.removeSplitter()
    seat = top + P.HOOD_SEAT
    tools = [cylz(P.LID_HOLE_D / 2, top - P.HOOD_TOP_T - 1, seat + 1)]
    r = P.ADAPTER_SCREW_PCD / 2
    tools += [cylz(2.1, seat - 6.5, seat + 1, r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]   # HELICOIL M4 x 1.5 d (STI drill 4.2)
    tools += [cylz(1.65, P.PLATE_T - 1, P.PLATE_T + 11, x, y) for x, y in hood_screws()]          # M4 tapped from below
    tools.append(box(WIN_X[0], WIN_X[1], c * (hy - P.HOOD_WALL - 1), c * (hy + 1), WIN_Z[0], WIN_Z[1]))
    tools += [Part.makeCylinder(1.25, P.HOOD_WALL + 3, V(x, c * (hy - P.HOOD_WALL - 2.5), z), V(0, c, 0)) for x, z in window_screws()]
    return cut_all(s, tools).removeSplitter()


def window_plate():
    hy, c = P.HOOD_Y, P.CABLE_SIDE
    s = box(WIN_X[0] - 10, WIN_X[1] + 10, c * hy, c * (hy + P.WINDOW_PLATE_T), WIN_Z[0] - 9, WIN_Z[1] + 9)
    xc, zc = sum(WIN_X) / 2, sum(WIN_Z) / 2
    tools = [Part.makeCylinder(P.GLAND[1] / 2, 10, V(xc, c * (hy - 1), zc), V(0, c, 0))]
    tools += [Part.makeCylinder(1.7, 10, V(x, c * (hy - 1), z), V(0, c, 0)) for x, z in window_screws()]
    tools += [Part.makeCone(3.46, 1.95, 1.9, V(x, c * (hy + P.WINDOW_PLATE_T + 0.01), z), V(0, -c, 0)) for x, z in window_screws()]   # ISO 10642 M3, flush
    return cut_all(s, tools)


def gland():
    _, m, dk, l_out = P.GLAND
    hy, c = P.HOOD_Y + P.WINDOW_PLATE_T, P.CABLE_SIDE
    xc, zc = sum(WIN_X) / 2, sum(WIN_Z) / 2
    s = Part.makeCylinder(m / 2, P.WINDOW_PLATE_T, V(xc, c * (hy - P.WINDOW_PLATE_T), zc), V(0, c, 0))   # screwed into an M16x1.5 thread in the plate
    s = s.fuse(Part.makeCylinder(dk / 2, 6, V(xc, c * hy, zc), V(0, c, 0)))
    s = s.fuse(Part.makeCylinder(dk / 2 - 2, l_out - 6, V(xc, c * (hy + 6), zc), V(0, c, 0)))
    return s.cut(Part.makeCylinder(4.0, 40, V(xc, c * (hy - 15), zc), V(0, c, 0)))
