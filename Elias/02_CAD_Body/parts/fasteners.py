"""Simplified standard fasteners. Shanks use the thread core diameter so tapped holes report as touching."""
import Part
import math

from FreeCAD import Vector as V

# d: (head dia dk, head height k, core radius, socket across flats)
ISO4762 = {3: (5.5, 3.0, 1.25, 2.5), 4: (7.0, 4.0, 1.65, 3.0), 5: (8.5, 5.0, 2.1, 4.0), 6: (10.0, 6.0, 2.5, 5.0)}
ISO10642 = {4: (8.96, 2.48, 1.65, 2.5), 5: (10.4, 2.45, 2.1, 3.0)}


def hex_socket(s, depth, base, direction):
    """Hexagon socket prism, across flats s, starting at base and going depth along direction."""
    r = s / math.sqrt(3)
    pts = [V(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), 0) for a in range(0, 361, 60)]
    prism = Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, depth))
    rot = __import__("FreeCAD").Rotation(V(0, 0, 1), direction)
    prism.Placement = __import__("FreeCAD").Placement(base, rot)
    return prism


def iso4762(d, length, x, y, z_bearing, up=True):
    """Socket head cap screw along Z; bearing face at z_bearing, shank pointing up (+Z) or down."""
    dk, k, rc, s = ISO4762[d]
    sg = 1 if up else -1
    shank = Part.makeCylinder(rc, length, V(x, y, z_bearing), V(0, 0, sg))
    head = Part.makeCylinder(dk / 2, k, V(x, y, z_bearing), V(0, 0, -sg))
    socket = hex_socket(s, k * 0.6, V(x, y, z_bearing - sg * k), V(0, 0, sg))
    return shank.fuse(head).cut(socket).removeSplitter()


def iso10642(d, length, x, y, z_top):
    """Countersunk screw pointing down from a top face at z_top (length includes the head)."""
    dk, k, rc, s = ISO10642[d]
    head = Part.makeCone(dk / 2, rc + 0.65, k, V(x, y, z_top), V(0, 0, -1))
    shank = Part.makeCylinder(rc, length, V(x, y, z_top), V(0, 0, -1))
    socket = hex_socket(s, k * 0.8, V(x, y, z_top), V(0, 0, -1))
    return head.fuse(shank).cut(socket).removeSplitter()


def iso4762_x(d, length, x_bearing, y, z, inward):
    """Cap screw along X: bearing face at x_bearing, shank pointing in direction inward (+1 / -1)."""
    s = iso4762(d, length, 0.0, 0.0, 0.0, up=True)
    s.rotate(V(0, 0, 0), V(0, 1, 0), 90 if inward > 0 else -90)
    s.translate(V(x_bearing, y, z))
    return s


def iso7089_x(d, x_face, y, z, outward):
    """Plain washer on a face at x_face, thickness towards outward (+1 / -1). Only M3 tabulated."""
    d1, d2, h = {3: (3.2, 7.0, 0.5)}[d]
    x0 = min(x_face, x_face + outward * h)
    ring = Part.makeCylinder(d2 / 2, h, V(x0, y, z), V(1, 0, 0))
    return ring.cut(Part.makeCylinder(d1 / 2, h, V(x0, y, z), V(1, 0, 0)))


def iso4029_x(d, length, x_outer, y, z, inward):
    """Hexagon socket set screw with cup point along X, outer face at x_outer. Only M4 tabulated."""
    rc, s, t, dz = {4: (1.65, 2.0, 2.0, 2.2)}[d]
    x0 = min(x_outer, x_outer + inward * length)
    body = Part.makeCylinder(rc, length, V(x0, y, z), V(1, 0, 0))
    body = body.cut(hex_socket(s, t, V(x_outer, y, z), V(inward, 0, 0)))
    tip = x_outer + inward * length
    cup = Part.makeCone(dz / 2, 0.0, 0.6, V(tip, y, z), V(-inward, 0, 0))      # cup point recess
    return body.cut(cup).removeSplitter()
