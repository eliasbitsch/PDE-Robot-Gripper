"""Visual meshes that the SimReady catalogue lacks, written in the same layout as usd2obj.py (obj/<name>/manifest.json).

python derived.py   (after fetch.py and usd2obj.py for warehouse_conveyorBelt_a06)

flat_belt: warehouse_conveyorBelt_a06 is a chain module with legs at one end only; the leg pair is mirrored to the
           other end and the posts are cut at the belt top. Colours by material name (texture means came out grey).
controller: ABB OmniCore C30 (449 x 443 x 170 mm, ABB brochure) on a floor switch cabinet 600 x 400 x 1200 mm that
           holds the gripper drive electronics (02_CAD_Body/parts/cabinet.py). Plain shapes, no logos.
"""
import json
import os

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "..", "vendor", "simready", "obj")
BELT_COL = {"paintedBlue": [0.08, 0.16, 0.45], "scratchedMetal": [0.62, 0.63, 0.64], "ConveyorBelt": [0.07, 0.07, 0.07],
            "rubber": [0.07, 0.07, 0.07], "black": [0.04, 0.04, 0.04], "orange": [0.95, 0.45, 0.05], "decal": [0.85, 0.8, 0.2]}


def write(name, parts):
    """parts: [(material, rgb, trimesh)] -> obj/<name>/partNN.obj + manifest."""
    out = os.path.join(OBJ, name)
    os.makedirs(out, exist_ok=True)
    man, lo, hi = [], np.full(3, 1e9), np.full(3, -1e9)
    for i, (mat, rgb, m) in enumerate(parts):
        fn = f"part{i:02d}.obj"
        m.export(os.path.join(out, fn))
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
        man.append(dict(file=fn, material=mat, rgba=list(map(float, rgb)) + [1.0], faces=len(m.faces)))
    json.dump(dict(source="derived.py", parts=man, bbox=[lo.tolist(), hi.tolist()]), open(os.path.join(out, "manifest.json"), "w"), indent=1)
    print(name, len(parts), "parts", np.round(lo, 3).tolist(), np.round(hi, 3).tolist())


def flat_belt():
    src = os.path.join(OBJ, "warehouse_conveyorBelt_a06")
    man = json.load(open(os.path.join(src, "manifest.json")))
    top = max(trimesh.load(os.path.join(src, p["file"]), force="mesh").bounds[1][2] for p in man["parts"]
              if "ConveyorBelt" in p["material"] or "rubber" in p["material"])
    x0, x1 = man["bbox"][0][0], man["bbox"][1][0]
    parts = []
    for p in man["parts"]:
        m = trimesh.load(os.path.join(src, p["file"]), force="mesh", process=False)
        # the posts reach up for a second tier: cut them at the side rail top
        m = trimesh.intersections.slice_mesh_plane(m, plane_normal=[0, 0, -1], plane_origin=[0, 0, top + 0.025], cap=True)
        c = m.triangles_center
        idx = np.where((c[:, 0] > x1 - 0.2) & (c[:, 2] < top - 0.12))[0] if len(c) else []
        if len(idx):
            leg = m.submesh([idx], append=True)
            leg.vertices[:, 0] = x0 + x1 - leg.vertices[:, 0]
            leg.invert()
            m = trimesh.util.concatenate([m, leg])
        rgb = next((v for k, v in BELT_COL.items() if k in p["material"]), p["rgba"][:3])
        if len(m.faces):
            parts.append((p["material"], rgb, m))
    write("flat_belt", parts)
    return top


def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    return trimesh.creation.box(extents=hi - lo, transform=trimesh.transformations.translation_matrix((lo + hi) / 2))

def cyl(r, h, x, y, z):
    """Cylinder along -y (out of a front face at y), base at y."""
    c = trimesh.creation.cylinder(radius=r, height=h, sections=40)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation([x, y - h / 2, z])
    return c


def quad_obj(path, x0, x1, y, z0, z1):
    """Textured rectangle in the plane y, facing -y, uv (0, 0) at the lower left."""
    open(path, "w").write(f"v {x0} {y} {z0}\nv {x1} {y} {z0}\nv {x1} {y} {z1}\nv {x0} {y} {z1}\n"
                          "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\n")


def font(size, bold=False):
    return ImageFont.truetype("C:/Windows/Fonts/" + ("arialbd.ttf" if bold else "arial.ttf"), max(size, 6))


def noisy(rgb, W, H, grad=0, seed=0):
    rng = np.random.default_rng(seed)
    a = np.array(rgb, np.int16) + np.linspace(grad, -grad, H)[:, None, None] + rng.integers(-3, 4, (H, W, 1))
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def tex_c30(path, w_mm, h_mm, px=4.0):
    """OmniCore C30 front, drawn: logo, vent field, power switch, motors-on button, FlexPendant and I/O ports."""
    W, H = int(w_mm * px), int(h_mm * px)
    im = noisy((52, 54, 58), W, H, 3, 1)
    d = ImageDraw.Draw(im)
    m = lambda v: int(v * px)
    d.rectangle([0, 0, W - 1, H - 1], outline=(95, 97, 101), width=m(2))
    for k in range(13):                                                       # vent field
        y = m(25 + 9.5 * k)
        d.rounded_rectangle([m(12), y, m(165), y + m(5)], radius=m(2), fill=(20, 21, 23))
    d.text((m(185), m(12)), "ABB", font=font(m(22), True), fill=(255, 0, 15))
    d.text((m(186), m(40)), "OmniCore", font=font(m(9)), fill=(225, 225, 225))
    d.text((m(186), m(52)), "C30", font=font(m(7)), fill=(170, 170, 170))
    cx, cy = m(395), m(80)                                                    # main power switch
    d.rectangle([cx - m(30), cy - m(30), cx + m(30), cy + m(30)], fill=(235, 190, 20))
    d.ellipse([cx - m(21), cy - m(21), cx + m(21), cy + m(21)], fill=(200, 20, 20))
    d.rectangle([cx - m(5), cy - m(27), cx + m(5), cy + m(27)], fill=(160, 12, 12))
    d.text((cx - m(2), cy - m(42)), "I", font=font(m(8), True), fill=(230, 230, 230))
    d.text((cx + m(33), cy - m(5)), "0", font=font(m(8), True), fill=(230, 230, 230))
    bx, by = m(305), m(55)                                                    # motors on
    d.ellipse([bx - m(13), by - m(13), bx + m(13), by + m(13)], fill=(190, 190, 190))
    d.ellipse([bx - m(9), by - m(9), bx + m(9), by + m(9)], fill=(250, 250, 245))
    d.text((bx - m(17), by + m(15)), "MOTORS ON", font=font(m(5)), fill=(210, 210, 210))
    fx, fy = m(305), m(118)                                                   # FlexPendant connector
    d.ellipse([fx - m(14), fy - m(14), fx + m(14), fy + m(14)], fill=(155, 157, 160))
    d.ellipse([fx - m(10), fy - m(10), fx + m(10), fy + m(10)], fill=(25, 25, 25))
    for t in range(8):
        a = 2 * np.pi * t / 8
        px_, py_ = fx + m(6 * np.cos(a)), fy + m(6 * np.sin(a))
        d.ellipse([px_ - m(1.2), py_ - m(1.2), px_ + m(1.2), py_ + m(1.2)], fill=(205, 175, 60))
    d.text((fx - m(14), fy + m(16)), "FlexPendant", font=font(m(4.5)), fill=(200, 200, 200))
    for k in range(3):                                                        # Ethernet
        x = m(188 + 21 * k)
        d.rectangle([x, m(100), x + m(15), m(113)], fill=(12, 12, 12), outline=(125, 125, 125), width=2)
        d.rectangle([x + m(5), m(113), x + m(10), m(115)], fill=(12, 12, 12))
    d.text((m(188), m(117)), "MGMT  SERVICE  WAN", font=font(m(4)), fill=(185, 185, 185))
    d.rectangle([m(255), m(103), m(268), m(109)], fill=(12, 12, 12), outline=(160, 160, 160), width=2)   # USB
    d.text((m(255), m(111)), "USB", font=font(m(4)), fill=(185, 185, 185))
    for k, c in enumerate([(40, 220, 70), (40, 220, 70), (240, 180, 20)]):     # status LEDs
        x = m(188 + 14 * k)
        d.ellipse([x, m(72), x + m(5), m(77)], fill=c)
    d.text((m(186), m(79)), "PWR   RUN   ERR", font=font(m(4)), fill=(185, 185, 185))
    d.rectangle([m(330), m(145), m(440), m(160)], fill=(228, 228, 222))       # type plate
    d.text((m(333), m(148)), "OmniCore C30 / IRB 1300-7/1.4", font=font(m(5)), fill=(30, 30, 30))
    im.save(path)


def tex_door(path, w_mm, h_mm, px=1.2):
    """Cabinet door RAL 7035: warning sign, labels, pilot lights, vent grille (all drawn)."""
    W, H = int(w_mm * px), int(h_mm * px)
    im = noisy((214, 215, 210), W, H, 6, 2)
    d = ImageDraw.Draw(im)
    m = lambda v: int(v * px)
    d.rectangle([0, 0, W - 1, H - 1], outline=(165, 165, 160), width=m(3))
    cx, ty, s = W // 2, m(60), m(80)                                          # warning: electrical voltage
    h3 = int(s * 0.87)
    d.polygon([(cx, ty), (cx - s // 2, ty + h3), (cx + s // 2, ty + h3)], fill=(15, 15, 15))
    d.polygon([(cx, ty + m(10)), (cx - s // 2 + m(9), ty + h3 - m(5)), (cx + s // 2 - m(9), ty + h3 - m(5))], fill=(250, 200, 0))
    d.polygon([(cx + m(6), ty + m(24)), (cx - m(8), ty + m(46)), (cx + m(1), ty + m(46)), (cx - m(6), ty + m(64)),
               (cx + m(9), ty + m(38)), (cx, ty + m(38))], fill=(15, 15, 15))
    # label ends above the main switch and E-stop (they start 210 mm from the top)
    d.rectangle([m(80), m(140), W - m(80), m(200)], fill=(246, 246, 241), outline=(80, 80, 80), width=2)
    d.text((m(92), m(147)), "=GR1  Gripper drive", font=font(m(17), True), fill=(20, 20, 20))
    d.text((m(92), m(173)), "24 V DC  |  CL4-E  |  PDE group 10", font=font(m(12)), fill=(50, 50, 50))
    for k, (c, t) in enumerate([((40, 200, 60), "Run"), ((235, 160, 20), "Brake open"), ((210, 30, 30), "Fault")]):
        x = m(120 + 140 * k)
        d.ellipse([x - m(18), m(560), x + m(18), m(596)], fill=(60, 60, 60))
        d.ellipse([x - m(13), m(565), x + m(13), m(591)], fill=c)
        d.text((x - m(32), m(602)), t, font=font(m(12)), fill=(40, 40, 40))
    d.text((m(36), m(305)), "Main switch", font=font(m(11)), fill=(40, 40, 40))
    d.text((W // 2 - m(25), m(330)), "E-STOP", font=font(m(11), True), fill=(40, 40, 40))
    for k in range(10):                                                       # vent grille
        y = H - m(160) + m(12) * k
        d.rounded_rectangle([m(150), y, W - m(150), y + m(6)], radius=2, fill=(115, 117, 115))
    d.rectangle([m(30), H - m(40), m(170), H - m(20)], fill=(242, 242, 237))
    d.text((m(34), H - m(38)), "IP54  EN 60204-1", font=font(m(11)), fill=(40, 40, 40))
    im.save(path)


def controller():
    """Origin at the cabinet's floor centre, front (door, controller front) towards -y."""
    out = os.path.join(OBJ, "controller")
    os.makedirs(out, exist_ok=True)
    W, D, H, PL = 0.6, 0.4, 1.2, 0.1
    dz0, dz1 = PL + 0.012, H - 0.012
    zd = lambda mm_from_top: dz1 - mm_from_top / 1000                       # door texture mm (from top) -> z
    grey, black, white = [0.80, 0.80, 0.78], [0.03, 0.03, 0.03], [0.88, 0.88, 0.86]
    g = {"cabinet": [], "plinth": [], "handle": [], "c30": [], "red": [], "yellow": []}
    g["plinth"].append(box((-W / 2, -D / 2, 0), (W / 2, D / 2, PL)))
    g["cabinet"].append(box((-W / 2, -D / 2 + 0.02, PL), (W / 2, D / 2, H)))
    g["cabinet"].append(box((-W / 2 + 0.012, -D / 2, dz0), (W / 2 - 0.012, -D / 2 + 0.02, dz1)))
    g["handle"].append(box((W / 2 - 0.07, -D / 2 - 0.025, 0.55), (W / 2 - 0.045, -D / 2, 0.73)))
    g["yellow"].append(box((-0.045, -D / 2 - 0.012, zd(320)), (0.045, -D / 2, zd(320) + 0.09)))        # E-stop
    g["red"].append(cyl(0.022, 0.035, 0.0, -D / 2 - 0.012, zd(320) + 0.045))
    g["yellow"].append(box((-W / 2 + 0.04, -D / 2 - 0.01, zd(300)), (-W / 2 + 0.13, -D / 2, zd(300) + 0.09)))  # main switch
    g["red"].append(box((-W / 2 + 0.079, -D / 2 - 0.035, zd(300) + 0.01), (-W / 2 + 0.091, -D / 2 - 0.01, zd(300) + 0.08)))
    cw, cd, ch = 0.449, 0.443, 0.170                                          # OmniCore C30, ABB brochure
    y0, z0 = -D / 2, H
    g["c30"].append(box((-cw / 2, y0 + 0.002, z0), (cw / 2, y0 + cd, z0 + ch)))
    col = {"cabinet": grey, "plinth": black, "handle": black, "c30": white, "red": [0.8, 0.05, 0.05], "yellow": [0.95, 0.75, 0.05]}
    write("controller", [(k, col[k], trimesh.util.concatenate(v)) for k, v in g.items() if v])
    man = json.load(open(os.path.join(out, "manifest.json")))
    tex_c30(os.path.join(out, "c30_front.png"), cw * 1000, ch * 1000)
    quad_obj(os.path.join(out, "c30_front.obj"), -cw / 2, cw / 2, y0 + 0.0015, z0, z0 + ch)
    tex_door(os.path.join(out, "door.png"), (W - 0.024) * 1000, (dz1 - dz0) * 1000)
    quad_obj(os.path.join(out, "door.obj"), -W / 2 + 0.012, W / 2 - 0.012, -D / 2 - 0.0005, dz0, dz1)
    man["parts"] += [dict(file="c30_front.obj", material="c30_front", rgba=[1, 1, 1, 1], faces=2, texture="c30_front.png"),
                     dict(file="door.obj", material="door", rgba=[1, 1, 1, 1], faces=2, texture="door.png")]
    json.dump(man, open(os.path.join(out, "manifest.json"), "w"), indent=1)


if __name__ == "__main__":
    print("belt top", round(flat_belt(), 3))
    controller()
