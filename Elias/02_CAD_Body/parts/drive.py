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
    g.num_teeth, g.module, g.height, g.pitch_angle = P.BEVEL_Z, P.MODUL, P.BEVEL_B, 45
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
    s = s.fuse(cylz(P.BEARING[0] / 2, P.PINION_Z0 + P.PINION_B - 0.1, P.SHAFT_TOP, XP, 0))
    return s.removeSplitter()


def wheel(bev):
    """Bevel wheel on the pinion shaft: large end down, apex on the drive axis."""
    s = bev.copy()
    s.translate(V(XP, 0, ZA - P.BEVEL_Z * P.MODUL / 2 + P.BEVEL_WHEEL_DZ))
    hub_z0 = P.BEARING_UPPER_Z + P.BEARING[2]
    s = s.fuse(cylz(P.BEVEL_HUB_D / 2, hub_z0, ZA - P.BEVEL_Z * P.MODUL / 2 + 0.1, XP, 0))
    s = s.cut(cylz(P.BEARING[0] / 2, hub_z0 - 1, ZA + 1, XP, 0))
    s = s.cut(cylx(1.65, XP - P.BEVEL_HUB_D / 2 - 1, XP - P.BEARING[0] / 2 + 0.5, 0, hub_z0 + 3))   # M4 set screw
    return s.removeSplitter()


def bevel_pinion(bev):
    """Bevel pinion on the gearbox shaft: axis along X, apex at the pinion shaft axis."""
    r = P.BEVEL_Z * P.MODUL / 2
    s = bev.copy()
    s = s.fuse(cylz(P.BEVEL_HUB_D / 2, -P.BEVEL_HUB_L, 0.1))
    s = s.cut(cylz(P.BEARING[0] / 2, -P.BEVEL_HUB_L - 1, r + 1))
    s = s.cut(Part.makeCylinder(1.65, P.BEVEL_HUB_D / 2 + 1, V(0, 0, -P.BEVEL_HUB_L / 2), V(1, 0, 0)))  # M4 set screw (local +x = up)
    s.rotate(V(0, 0, 0), V(0, 1, 0), -90)        # local z -> global -X, local x -> global +Z
    s.translate(V(XP + r, 0, ZA))
    return s.removeSplitter()


def gearbox_flange_x():
    return XP + P.BEVEL_Z * P.MODUL / 4 + P.GEAR_SHAFT_L    # shaft end reaches the small end of the bevel pinion


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
    x, y = P.HOOD_SCREW_XY
    return [(sx * x, sy * y) for sx in (-1, 1) for sy in (-1, 1)]


WIN_X, WIN_Z = (30.0, 60.0), (30.0, 48.0)


def window_screws():
    return [(x, z) for x in (WIN_X[0] - 5, WIN_X[1] + 5) for z in (WIN_Z[0] - 4, WIN_Z[1] + 4)]


def hood():
    """Flat hood over the drive, milled from the solid; a seat ring on top carries the robot adapter."""
    hx, hy = P.HOUSING_X / 2, P.HOOD_Y
    top = P.HOOD_TOP
    s = rounded_box(-hx, hx, -hy, hy, P.PLATE_T, top, P.HOOD_R)
    s = chamfer_z_edges(s, top, P.EDGE_CHAMFER)
    s = s.fuse(cylz(P.ADAPTER_D / 2, top - 0.1, top + P.HOOD_SEAT))
    s = s.cut(rounded_box(-hx + P.HOOD_WALL, hx - P.HOOD_WALL, -hy + P.HOOD_WALL, hy - P.HOOD_WALL,
                          P.PLATE_T - 1, top - P.HOOD_TOP_T, P.HOOD_R - P.HOOD_WALL))
    for x, y in hood_screws():
        s = s.fuse(cylz(4.5, P.PLATE_T, P.PLATE_T + 13.5, x, y))   # 2.5 mm above the blind thread
    s = s.removeSplitter()
    seat = top + P.HOOD_SEAT
    tools = [cylz(P.LID_HOLE_D / 2, top - P.HOOD_TOP_T - 1, seat + 1)]
    r = P.ADAPTER_SCREW_PCD / 2
    tools += [cylz(1.65, seat - 6.5, seat + 1, r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
    tools += [cylz(1.65, P.PLATE_T - 1, P.PLATE_T + 11, x, y) for x, y in hood_screws()]          # M4 tapped from below
    tools.append(box(WIN_X[0], WIN_X[1], hy - P.HOOD_WALL - 1, hy + 1, WIN_Z[0], WIN_Z[1]))
    tools += [Part.makeCylinder(1.25, P.HOOD_WALL + 1, V(x, hy - P.HOOD_WALL - 0.5, z), V(0, 1, 0)) for x, z in window_screws()]
    return cut_all(s, tools).removeSplitter()


def window_plate():
    hy = P.HOOD_Y
    s = box(WIN_X[0] - 10, WIN_X[1] + 10, hy, hy + P.WINDOW_PLATE_T, WIN_Z[0] - 9, WIN_Z[1] + 9)
    xc, zc = sum(WIN_X) / 2, sum(WIN_Z) / 2
    tools = [Part.makeCylinder(P.GLAND[1] / 2, 10, V(xc, hy - 1, zc), V(0, 1, 0))]
    tools += [Part.makeCylinder(1.7, 10, V(x, hy - 1, z), V(0, 1, 0)) for x, z in window_screws()]
    return cut_all(s, tools)


def gland():
    _, m, dk, l_out = P.GLAND
    hy = P.HOOD_Y + P.WINDOW_PLATE_T
    xc, zc = sum(WIN_X) / 2, sum(WIN_Z) / 2
    s = Part.makeCylinder(m / 2, P.WINDOW_PLATE_T, V(xc, hy - P.WINDOW_PLATE_T, zc), V(0, 1, 0))   # screwed into an M16x1.5 thread in the plate
    s = s.fuse(Part.makeCylinder(dk / 2, 6, V(xc, hy, zc), V(0, 1, 0)))
    s = s.fuse(Part.makeCylinder(dk / 2 - 2, l_out - 6, V(xc, hy + 6, zc), V(0, 1, 0)))
    return s.cut(Part.makeCylinder(4.0, 40, V(xc, hy - 15, zc), V(0, 1, 0)))
