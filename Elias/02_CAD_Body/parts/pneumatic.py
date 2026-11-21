"""Pneumatic drive (GRIPPER_DRIVE=pneumatic): compact cylinder + safety spring in a flat riser on the top plate.

The cylinder (Festo ADN-16-10-A-P-A) lies along x at y = CYL_Y. A driver plate on its rod carries a pin that reaches
through a slot in the top plate into the right rack; the pinion keeps the left jaw in step. Rod out = open. A
compression spring on a guide pin beside the cylinder pushes the driver towards closing: it adds to the regulated
closing force and alone holds the part when the air fails. Everything is built at d = 0 (closed stop).
"""
import math

import FreeCAD as App
import Part

import gripper_params as P
from parts.core import box, cut_all, cylx, cylz, rack_y, rounded_box, chamfer_z_edges, adapter_screws

V = App.Vector
ZC = P.CYL_Z0 + P.CYL_BODY[1] / 2                    # cylinder, driver and spring axis height
X_REAR = P.CYL_FRONT_X - P.CYL_BODY[0]
BRACKET_T, FOOT_L, FOOT_T = 6.0, 16.0, 5.0
X_SEAT = P.DRV_X0 + P.DRV_T + P.D_GRIP + P.SPRING_L_GRIP   # spring seat face (spring length SPRING_L_GRIP at grip)


def riser_top():
    return P.PLATE_T + P.RISER_H


def riser_screws():
    """Four M4 from the top through columns in the riser corners into the plate."""
    # clear of the housing pocket walls, the cover screw bosses (x -78 / 70, y -42.5 / 20) and the bracket foot
    return [(x, y) for x in (-68.0, 65.0) for y in (-35.0, 27.0)]


def bracket_screws():
    """Two M4 from the top through the bracket foot into the plate."""
    xf = X_REAR - BRACKET_T - FOOT_L / 2
    return [(xf, P.CYL_Y - 10.0), (xf, P.CYL_Y + 10.0)]


def cylinder_rear_screws():
    """Four M4 through the bracket into the end cap of the cylinder [pattern to verify on the Festo drawing]."""
    return [(P.CYL_Y + dy, ZC + dz) for dy in (-8.0, 8.0) for dz in (-8.0, 8.0)]


def plate_tools():
    """Cuts in the housing top plate: riser and bracket screws (M4 tapped), slot for the driver pin."""
    ry = rack_y(1)
    t = [cylz(1.65, -1, P.PLATE_T + 1, x, y) for x, y in riser_screws()]                    # through into the pocket
    t += [cylz(1.65, 1.5, P.PLATE_T + 1, x, y) for x, y in bracket_screws()]                # blind, 1.5 mm floor
    t.append(box(P.PIN_X - P.PIN_D / 2 - 1, P.PIN_X + P.D_OPEN + P.PIN_D / 2 + 1, ry - P.PIN_D / 2 - 1,
                 ry + P.PIN_D / 2 + 1, -1, P.PLATE_T + 1))
    return t


def riser():
    """Flat cover box over cylinder, spring and bearing boss. Its top is the robot flange (ISO 9409-1-40-4-M6):
    spigot on top, the four M6 go in from the inside (the riser is open at the bottom until it sits on the housing)."""
    from parts.core import holes_pcd
    x0, x1 = P.RISER_XR
    y0, y1 = P.RISER_Y
    z0, z1 = P.PLATE_T, riser_top()
    w, t = P.RISER_WALL, P.RISER_TOP_T
    s = rounded_box(x0, x1, y0, y1, z0, z1, P.HOUSING_R)
    s = chamfer_z_edges(s, z1, P.EDGE_CHAMFER)
    s = s.cut(rounded_box(x0 + w, x1 - w, y0 + w, y1 - w, z0 - 1, z1 - t, 6.0))
    s = s.fuse(cylz(P.ISO_PCD / 2 + 8.0, z1 - P.ISO_BOSS_T, z1 - t + 0.1))                      # flange boss inside
    for x, y in riser_screws():
        s = s.fuse(cylz(6.5, z0, z1 - 0.1, x, y))
    s = s.fuse(cylx(9.0, X_SEAT, x1 - w + 0.1, P.SPRING_Y, ZC))                              # spring seat
    s = s.fuse(cylz(P.ISO_SPIGOT_D / 2, z1 - 0.1, z1 + P.ISO_SPIGOT_H))                       # centring spigot
    tools = [cylz(2.25, z0 - 1, z1 + 1, x, y) for x, y in riser_screws()]
    tools += [cylz(3.75, z1 - 4.5, z1 + 1, x, y) for x, y in riser_screws()]                 # ISO 4762 heads sunk
    tools += holes_pcd(3.3, P.ISO_PCD, z1 - P.ISO_BOSS_T - 1, z1 + P.ISO_SPIGOT_H + 1)        # M6 clearance
    tools += holes_pcd(5.5, P.ISO_PCD, z1 - P.ISO_BOSS_T - 1, z1 - 2.5)                       # M6 heads from the inside
    tools.append(cylz(3.0, z1 - 6, z1 + P.ISO_SPIGOT_H + 1, P.ISO_PCD / 2, 0))                # locating pin d6 H7
    tools.append(cylx(P.GUIDE_D / 2 + 0.3, X_SEAT - 1, x1 - w - 1.5, P.SPRING_Y, ZC))        # guide bore, blind
    for x in bulkhead_x():
        tools.append(Part.makeCylinder(3.0, 2 * w, V(x, y1 - 1.5 * w, ZC), V(0, 1, 0)))       # bulkhead fittings
    return cut_all(s, tools).removeSplitter()


def bulkhead_x():
    return (X_REAR + 8.0, P.CYL_FRONT_X - 8.0)


def cylinder_body():
    """ADN-16-10 body, square 29, ports M5 on the +y face (towards the inside)."""
    h = P.CYL_BODY[1]
    s = box(X_REAR, P.CYL_FRONT_X, P.CYL_Y - h / 2, P.CYL_Y + h / 2, P.CYL_Z0, P.CYL_Z0 + h)
    try:
        s = s.makeFillet(2.0, [e for e in s.Edges if abs(e.tangentAt(0).x) > 0.99])
    except Part.OCCError:
        pass
    return s


def cylinder_rod():
    return cylx(P.CYL_ROD / 2, P.CYL_FRONT_X - 0.1, P.CYL_FRONT_X + P.CYL_ROD_OUT, P.CYL_Y, ZC)


def port_fittings():
    """Festo QSM-M5-4 push-in fittings on the two cylinder ports."""
    y = P.CYL_Y + P.CYL_BODY[1] / 2
    out = []
    for x in bulkhead_x():
        s = Part.makeCylinder(3.5, 5.0, V(x, y, ZC), V(0, 1, 0))
        s = s.fuse(Part.makeCylinder(5.0, 9.0, V(x, y + 5.0, ZC), V(0, 1, 0)))
        out.append(s)
    return out


def bulkhead_fittings():
    """Festo QSS-4 bulkhead fittings through the +y riser wall: tubes to the valve."""
    y1 = P.RISER_Y[1]
    out = []
    for x in bulkhead_x():
        s = Part.makeCylinder(2.9, P.RISER_WALL + 1.0, V(x, y1 - P.RISER_WALL - 0.5, ZC), V(0, 1, 0))
        s = s.fuse(Part.makeCylinder(5.0, 9.0, V(x, y1, ZC), V(0, 1, 0)))
        s = s.fuse(Part.makeCylinder(5.0, 4.0, V(x, y1 - P.RISER_WALL - 4.0, ZC), V(0, 1, 0)))   # lock nut
        out.append(s)
    return out


def bracket():
    """Angle behind the cylinder: upright plate screwed to the end cap, foot screwed to the top plate."""
    h = P.CYL_BODY[1]
    y0, y1 = P.CYL_Y - h / 2, P.CYL_Y + h / 2
    s = box(X_REAR - BRACKET_T, X_REAR, y0, y1, P.PLATE_T, P.CYL_Z0 + h)
    s = s.fuse(box(X_REAR - BRACKET_T - FOOT_L, X_REAR, y0, y1, P.PLATE_T, P.PLATE_T + FOOT_T))
    tools = [Part.makeCylinder(2.25, BRACKET_T + 2, V(X_REAR - BRACKET_T - 1, y, z), V(1, 0, 0)) for y, z in cylinder_rear_screws()]
    tools += [cylz(2.25, P.PLATE_T - 1, P.PLATE_T + FOOT_T + 1, x, y) for x, y in bracket_screws()]
    return cut_all(s, tools)


def driver():
    """Driver plate on the rod (M6), pin hole down into the rack, guide pin pressed in on the spring side."""
    s = box(P.DRV_X0, P.DRV_X0 + P.DRV_T, P.DRV_Y[0], P.DRV_Y[1], P.DRV_Z[0], P.DRV_Z[1])
    tools = [Part.makeCylinder(2.5, 7.0, V(P.DRV_X0 - 1, P.CYL_Y, ZC), V(1, 0, 0)),                 # rod M6 tapped
             cylz(P.PIN_D / 2, P.DRV_Z[0] - 1, P.DRV_Z[0] + 11, P.PIN_X, rack_y(1)),               # pin, pressed
             Part.makeCylinder(P.GUIDE_D / 2, 9.0, V(P.DRV_X0 + P.DRV_T - 8, P.SPRING_Y, ZC), V(1, 0, 0))]
    return cut_all(s, tools)


def pin_z(rack_top):
    return rack_top - 5.0, P.DRV_Z[0] + 10.0


def driver_pin(rack_top):
    z0, z1 = pin_z(rack_top)
    return cylz(P.PIN_D / 2, z0, z1, P.PIN_X, rack_y(1))


def rack_pin_hole(rack_top):
    z0, _ = pin_z(rack_top)
    return cylz(P.PIN_D / 2, z0 - 0.5, rack_top + 1, P.PIN_X, rack_y(1))


def spring_guide():
    x0 = P.DRV_X0 + P.DRV_T - 8.0
    return cylx(P.GUIDE_D / 2, x0, X_SEAT + 6.0, P.SPRING_Y, ZC)


def spring():
    """Helical compression spring at the grip position (d = D_GRIP), d 1.8 / Dm 12, closed ground ends."""
    x0 = P.DRV_X0 + P.DRV_T + P.D_GRIP
    L = P.SPRING_L_GRIP
    n_total = P.SPRING_N + 2
    pitch = L / n_total
    helix = Part.makeHelix(pitch, L - P.SPRING_D, P.SPRING_DM / 2)
    prof = Part.Wire(Part.makeCircle(P.SPRING_D / 2, V(P.SPRING_DM / 2, 0, 0), V(0, 1, 0)))
    try:
        s = Part.Wire(helix).makePipeShell([prof], True, True)
    except Part.OCCError:
        s = cylz(P.SPRING_DM / 2 + P.SPRING_D / 2, 0, L - P.SPRING_D).cut(cylz(P.SPRING_DM / 2 - P.SPRING_D / 2, -1, L + 1))
    s.translate(V(0, 0, P.SPRING_D / 2))
    s.rotate(V(0, 0, 0), V(0, 1, 0), 90)                    # axis along +x
    s.translate(V(x0, P.SPRING_Y, ZC))
    return s


def riser_screw_z():
    return riser_top() - 4.5
