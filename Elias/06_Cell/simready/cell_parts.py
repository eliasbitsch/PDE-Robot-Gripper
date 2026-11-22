"""Own visual parts of the cell around the robot, in world coordinates (m), same layout as usd2obj.py.

python cell_parts.py   ->  obj/machine, obj/buffer_housing, obj/chain_conveyors, obj/fence_fill

machine: CNC machining centre (finishes slot and bore of part B) that closes the enclosure on the place side and
         takes the two workpiece carrier chains through two inlets; operator side (-y) with sliding door, window,
         HMI on a swing arm, signal tower.
buffer_housing: buffer line housing that closes the pick side; the belt leaves it through a tunnel opening sized
         after ISO 13857 (no reach to the robot), lid window over the queued parts.
chain_conveyors: two rigid chain conveyors as in Abb. 1 (top run into the machine, return underneath, end sprocket).
fence_fill: mesh panels between the buffer housing and the side fences and above the housing.
Collisions stay with the boxes of cell_params / build_model.
"""
import json
import os
import sys

import numpy as np
import trimesh
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import cell_params as CP  # noqa: E402
from derived import OBJ, box, font, noisy  # noqa: E402

MM = 0.001
GREY, ANTH, BLACK = [0.80, 0.81, 0.79], [0.20, 0.21, 0.23], [0.03, 0.03, 0.03]
ALU, GLASS = [0.70, 0.72, 0.74], [0.55, 0.70, 0.80, 0.28]
# chain loop (Abb. 1): sprocket at the robot-side end, carriers fixed to the chain, top run into the machine
SPROCKET_R = 0.15
X_WHEEL = (CP.X_PLACE + CP.CARRIER[0] / 2) * MM
Z_TOP = (CP.H_PLACE - CP.CARRIER[2]) * MM                     # chain top = carrier underside
Z_WHEEL = Z_TOP - SPROCKET_R
X_MACHINE = CP.X_FENCE[0] * MM
X_BUFFER = CP.X_FENCE[1] * MM


def emit(name, groups, textures=()):
    """groups: {material: (rgba, [meshes])}; textures: [(file.obj written already, png, material)]."""
    out = os.path.join(OBJ, name)
    os.makedirs(out, exist_ok=True)
    parts, lo, hi = [], np.full(3, 1e9), np.full(3, -1e9)
    for i, (mat, (rgba, ms)) in enumerate(groups.items()):
        if not ms:
            continue
        m = trimesh.util.concatenate(ms)
        fn = f"part{i:02d}.obj"
        m.export(os.path.join(out, fn))
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
        rgba = list(rgba) + [1.0] * (4 - len(rgba))
        parts.append(dict(file=fn, material=mat, rgba=[float(c) for c in rgba], faces=len(m.faces)))
    for obj, png, mat in textures:
        parts.append(dict(file=obj, material=mat, rgba=[1, 1, 1, 1], faces=2, texture=png))
    json.dump(dict(source="cell_parts.py", parts=parts, bbox=[lo.tolist(), hi.tolist()]),
              open(os.path.join(out, "manifest.json"), "w"), indent=1)
    print(name, len(parts), "parts")
    return out


def quad(path, p0, u, v):
    """Textured rectangle p0, p0+u, p0+u+v, p0+v (normal u x v), uv (0, 0) at p0."""
    p = [np.array(p0, float) + a * np.array(u, float) + b * np.array(v, float) for a, b in ((0, 0), (1, 0), (1, 1), (0, 1))]
    open(path, "w").write("".join(f"v {q[0]:.5f} {q[1]:.5f} {q[2]:.5f}\n" for q in p)
                          + "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nf 1/1 2/2 3/3\nf 1/1 3/3 4/4\n")


def cyl_z(r, z0, z1, x, y, sec=32):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sec)
    c.apply_translation([x, y, (z0 + z1) / 2])
    return c


def cyl_y(r, y0, y1, x, z, sec=48):
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=sec)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1) / 2, z])
    return c


def wall_with_holes(axis, at, t, u_rng, z_rng, holes):
    """Wall plate perpendicular to x (axis 0) or y (axis 1) at 'at' (outer face), thickness t into the solid,
    spanning u (the other horizontal axis) and z, with rectangular holes [(u0, u1, z0, z1)]."""
    us = sorted({u_rng[0], u_rng[1], *[h[0] for h in holes], *[h[1] for h in holes]})
    zs = sorted({z_rng[0], z_rng[1], *[h[2] for h in holes], *[h[3] for h in holes]})
    out = []
    for ua, ub in zip(us[:-1], us[1:]):
        for za, zb in zip(zs[:-1], zs[1:]):
            uc, zc = (ua + ub) / 2, (za + zb) / 2
            if any(h[0] <= uc <= h[1] and h[2] <= zc <= h[3] for h in holes):
                continue
            if axis == 0:
                out.append(box((at - t, ua, za), (at, ub, zb)) if t > 0 else box((at, ua, za), (at - t, ub, zb)))
            else:
                out.append(box((ua, at - t, za), (ub, at, zb)) if t > 0 else box((ua, at, za), (ub, at - t, zb)))
    return out


def tex_label(path, w, h, lines, bg=(238, 238, 232), px=1400):
    W, H = int(w * px), int(h * px)
    im = noisy(bg, W, H, 2, 3)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W - 1, H - 1], outline=(60, 60, 60), width=max(2, W // 200))
    y = int(0.12 * H)
    for txt, size, bold, col in lines:
        d.text((int(0.05 * W), y), txt, font=font(int(size * H), bold), fill=col)
        y += int(size * H * 1.3)
    im.save(path)


def tex_hmi(path, w=0.30, h=0.22, px=2400):
    W, H = int(w * px), int(h * px)
    im = Image.new("RGB", (W, H), (20, 26, 34))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, int(0.13 * H)], fill=(36, 70, 110))
    d.text((int(0.03 * W), int(0.02 * H)), "BZ-2   AUTOMATIK", font=font(int(0.08 * H), True), fill=(240, 240, 240))
    rows = [("Programm", "O2041  Teil B, Nut + Bohrung"), ("Takt", "5,0 s"), ("Spindel", "18 000 1/min"),
            ("Werkzeug", "T03  VHM-Fräser d4"), ("Stückzahl", "1 248"), ("Zuführung", "WT-Kette 1 + 2  bereit")]
    for k, (a, b) in enumerate(rows):
        y = int((0.19 + 0.1 * k) * H)
        d.text((int(0.04 * W), y), a, font=font(int(0.06 * H)), fill=(150, 170, 190))
        d.text((int(0.34 * W), y), b, font=font(int(0.06 * H), True), fill=(225, 235, 240))
    for k, (t, c) in enumerate([("START", (40, 160, 70)), ("STOP", (180, 40, 40)), ("RESET", (70, 90, 120))]):
        x = int((0.04 + 0.32 * k) * W)
        d.rounded_rectangle([x, int(0.82 * H), x + int(0.27 * W), int(0.95 * H)], radius=8, fill=c)
        d.text((x + int(0.06 * W), int(0.845 * H)), t, font=font(int(0.07 * H), True), fill=(255, 255, 255))
    im.save(path)


LOGO = os.path.join(HERE, "..", "vendor", "logos", "logo_fhtw.png")    # FH Technikum Wien, git-ignored like vendor


def logo(out, name, p0, u, _):
    """FHTW logo sticker (white backing) on a wall, width |u|, upright; nothing if the logo file is missing."""
    if not os.path.exists(LOGO):
        return []
    im = Image.open(LOGO).convert("RGB")
    pad = Image.new("RGB", (im.width + 80, im.height + 80), (255, 255, 255))
    pad.paste(im, (40, 40))
    pad.save(os.path.join(out, name + ".png"))
    w = float(np.linalg.norm(u))
    quad(os.path.join(out, name + ".obj"), p0, u, (0, 0, w * pad.height / pad.width))
    return [(name + ".obj", name + ".png", name)]


def machine():
    out = os.path.join(OBJ, "machine")
    os.makedirs(out, exist_ok=True)
    x1, x0 = X_MACHINE, X_MACHINE - 1.5                    # front face (towards the robot) at the enclosure end
    y0, y1, H = -1.1, 1.1, 2.25
    t = 0.02
    g = {"grey": [GREY, []], "anth": [ANTH, []], "black": [BLACK, []], "glass": [GLASS, []], "alu": [ALU, []],
         "dark": [[0.08, 0.08, 0.09], []], "green": [[0.1, 0.8, 0.2], []], "amber": [[1.0, 0.65, 0.05], []],
         "red": [[0.85, 0.05, 0.05], []], "yellow": [[0.95, 0.75, 0.05], []]}
    g["anth"][1].append(box((x0, y0, 0), (x1, y1, 0.15)))                          # plinth
    g["anth"][1].append(box((x0, y0, H - 0.2), (x1 + 0.03, y1, H)))                # roof band
    # front wall with two inlets per chain (top run in, return out underneath)
    holes = []
    for yc in (-CP.DY_CHAIN * MM, CP.DY_CHAIN * MM):
        hw = CP.CARRIER[1] / 2 * MM + 0.05
        holes.append((yc - hw, yc + hw, Z_WHEEL - SPROCKET_R - 0.1, Z_TOP + 0.17))
    g["grey"][1] += wall_with_holes(0, x1, t, (y0, y1), (0.15, H - 0.2), holes)
    for (ua, ub, za, zb) in holes:                                                  # dark tunnel + brush strip
        g["dark"][1].append(box((x1 - 0.35, ua, za), (x1 - 0.3, ub, zb)))
        g["dark"][1] += [box((x1 - 0.3, ua, za), (x1 - t, ua + 0.005, zb)), box((x1 - 0.3, ub - 0.005, za), (x1 - t, ub, zb)),
                         box((x1 - 0.3, ua, zb - 0.005), (x1 - t, ub, zb)), box((x1 - 0.3, ua, za), (x1 - t, ub, za + 0.005))]
        g["anth"][1] += [box((x1, ua - 0.02, za - 0.02), (x1 + 0.01, ub + 0.02, za)), box((x1, ua - 0.02, zb), (x1 + 0.01, ub + 0.02, zb + 0.02)),
                         box((x1, ua - 0.02, za), (x1 + 0.01, ua, zb)), box((x1, ub, za), (x1 + 0.01, ub + 0.02, zb))]
    # operator side (-y): sliding door with a big window, interior visible
    wx0, wx1, wz0, wz1 = x0 + 0.25, x1 - 0.45, 0.85, 1.85
    g["grey"][1] += wall_with_holes(1, y0 + t, t, (x0, x1), (0.15, H - 0.2), [(wx0, wx1, wz0, wz1)])
    g["glass"][1].append(box((wx0, y0 - 0.002, wz0), (wx1, y0 + 0.004, wz1)))
    g["anth"][1] += [box((wx0 - 0.04, y0 - 0.03, wz0 - 0.04), (wx1 + 0.04, y0, wz0)), box((wx0 - 0.04, y0 - 0.03, wz1), (wx1 + 0.04, y0, wz1 + 0.04)),
                     box((wx0 - 0.04, y0 - 0.03, wz0), (wx0, y0, wz1)), box((wx1, y0 - 0.03, wz0), (wx1 + 0.04, y0, wz1))]
    g["alu"][1].append(box((wx1 - 0.12, y0 - 0.07, 1.2), (wx1 - 0.09, y0 - 0.03, 1.5)))          # door handle
    g["alu"][1] += [box((wx1 - 0.12, y0 - 0.07, 1.2), (wx1 - 0.09, y0 - 0.03, 1.22)), box((wx1 - 0.12, y0 - 0.07, 1.48), (wx1 - 0.09, y0 - 0.03, 1.5))]
    # other walls and roof (hollow housing so the window shows the interior)
    g["grey"][1] += [box((x0, y1 - t, 0.15), (x1, y1, H - 0.2)), box((x0, y0, 0.15), (x0 + t, y1, H - 0.2)),
                     box((x0, y0, H - 0.22), (x1, y1, H - 0.2))]
    g["anth"][1].append(box((x0, y0 - 0.006, 0.15), (x1, y0 + 0.03, 0.75)))       # lower front strip, operator side (proud of the wall: no coplanar faces)
    # interior: machine bed, table with a fixture, spindle head on a column, chip guard
    cx = 0.5 * (wx0 + wx1)
    g["anth"][1].append(box((wx0, y0 + 0.25, 0.15), (wx1, y1 - 0.25, 0.8)))
    g["alu"][1].append(box((cx - 0.35, -0.35, 0.8), (cx + 0.35, 0.35, 0.86)))
    g["dark"][1] += [box((cx - 0.06, -0.08, 0.86), (cx + 0.06, 0.08, 0.95))]
    g["grey"][1].append(box((cx - 0.25, 0.45, 0.8), (cx + 0.25, 0.75, 2.0)))       # column
    g["grey"][1].append(box((cx - 0.12, 0.05, 1.35), (cx + 0.12, 0.55, 1.75)))     # spindle head
    g["alu"][1].append(cyl_z(0.045, 1.12, 1.35, cx, 0.2))
    g["dark"][1].append(cyl_z(0.008, 1.03, 1.12, cx, 0.2))
    # HMI on a swing arm at the operator corner, E-stop below
    hx, hy = x1 - 0.25, y0 - 0.35
    g["anth"][1] += [box((hx - 0.03, y0 - 0.02, 1.85), (hx + 0.03, y0, 1.9)), box((hx - 0.02, hy, 1.86), (hx + 0.02, y0, 1.9)),
                     box((hx - 0.02, hy - 0.02, 1.55), (hx + 0.02, hy + 0.02, 1.9))]
    g["anth"][1].append(box((hx - 0.2, hy - 0.06, 1.2), (hx + 0.2, hy, 1.56)))
    hmi_png, hmi_obj = "hmi.png", "hmi.obj"
    tex_hmi(os.path.join(out, hmi_png))
    quad(os.path.join(out, hmi_obj), (hx - 0.17, hy - 0.0605, 1.30), (0.34, 0, 0), (0, 0, 0.24))
    g["yellow"][1].append(box((hx - 0.17, hy - 0.07, 1.215), (hx - 0.11, hy - 0.06, 1.285)))
    g["red"][1].append(cyl_y(0.022, hy - 0.1, hy - 0.07, hx - 0.14, 1.25))
    # signal tower on the roof, operator front corner
    sx, sy = x1 - 0.15, y0 + 0.15
    g["black"][1].append(cyl_z(0.03, H, H + 0.06, sx, sy))
    for k, c in enumerate(("green", "amber", "red")):
        g[c][1].append(cyl_z(0.035, H + 0.06 + 0.07 * k, H + 0.125 + 0.07 * k, sx, sy))
    g["black"][1].append(cyl_z(0.03, H + 0.27, H + 0.29, sx, sy))
    # name plate and warning on the robot-facing wall
    plate_png, plate_obj = "plate.png", "plate.obj"
    tex_label(os.path.join(out, plate_png), 0.9, 0.22, [("BZ-2  Bearbeitungszentrum", 0.22, True, (25, 25, 25)),
                                                        ("Nachbearbeitung Teil B: Nut, Bohrung, Entgraten", 0.13, False, (60, 60, 60)),
                                                        ("Zuführung WT-Ketten 1 + 2   |   Takt 5,0 s", 0.13, False, (60, 60, 60))])
    quad(os.path.join(out, plate_obj), (x1 + 0.002, -0.45, 1.75), (0, 0.9, 0), (0, 0, 0.22))
    tex = [(hmi_obj, hmi_png, "hmi"), (plate_obj, plate_png, "plate")]
    tex += logo(out, "logo1", (x1 + 0.002, -1.16, 1.55), (0, 0.46, 0), 0)          # robot-facing wall, outside the fence (y -0.6)
    tex += logo(out, "logo2", (wx0, y0 - 0.009, 0.3), (0.5, 0, 0), 0)               # operator side, lower strip
    emit("machine", g, tex)


def buffer_housing():
    out = os.path.join(OBJ, "buffer_housing")
    os.makedirs(out, exist_ok=True)
    x0, x1, y0, y1, H, t = X_BUFFER, (CP.BELT_X[1] + 100) * MM, -0.4, 0.4, 1.5, 0.02
    g = {"grey": [GREY, []], "anth": [ANTH, []], "dark": [[0.08, 0.08, 0.09], []], "glass": [GLASS, []],
         "alu": [ALU, []], "black": [BLACK, []], "green": [[0.1, 0.8, 0.2], []], "amber": [[1.0, 0.65, 0.05], []],
         "yellow": [[0.95, 0.75, 0.05], []]}
    zb = CP.H_PICK * MM
    tun = (-CP.BELT_W / 2 * MM - 0.04, CP.BELT_W / 2 * MM + 0.04, zb - 0.1, zb + 0.11)   # 110 mm over the belt: ISO 13857 slot, reach >= 850
    g["anth"][1].append(box((x0, y0, 0), (x1, y1, 0.12)))
    g["grey"][1] += wall_with_holes(0, x0 + t, t, (y0, y1), (0.12, H), [tun])
    # tunnel: the belt passes 850 mm through a hood, so the opening gives no reach to the robot (ISO 13857 tab. 4)
    g["anth"][1] += [box((x0 - 0.0, tun[0] - 0.02, tun[3]), (x0 + 0.85, tun[1] + 0.02, tun[3] + 0.02)),
                     box((x0, tun[0] - 0.02, tun[2]), (x0 + 0.85, tun[0], tun[3])),
                     box((x0, tun[1], tun[2]), (x0 + 0.85, tun[1] + 0.02, tun[3]))]
    g["grey"][1] += [box((x1 - t, y0, 0.12), (x1, y1, H)), box((x0, y1 - t, 0.12), (x1, y1, H)), box((x0, y0, 0.12), (x1, y0 + t, H))]
    # lid with a window over the queue
    lw0, lw1 = x0 + 0.15, x1 - 0.15
    g["grey"][1] += [box((x0, y0, H - t), (lw0, y1, H)), box((lw1, y0, H - t), (x1, y1, H)),
                     box((lw0, y0, H - t), (lw1, -0.25, H)), box((lw0, 0.25, H - t), (lw1, y1, H))]
    g["glass"][1].append(box((lw0, -0.25, H - 0.006), (lw1, 0.25, H)))
    g["anth"][1].append(box((x0 - 0.01, y0 - 0.01, H - 0.05), (x0 + 0.01, y1 + 0.01, H)))
    # signal lamp and label
    g["black"][1].append(cyl_z(0.025, H, H + 0.05, x1 - 0.1, y0 + 0.1))
    g["green"][1].append(cyl_z(0.03, H + 0.05, H + 0.11, x1 - 0.1, y0 + 0.1))
    g["amber"][1].append(cyl_z(0.03, H + 0.11, H + 0.17, x1 - 0.1, y0 + 0.1))
    g["yellow"][1] += [box((x0 + 0.3, y0 - 0.005, 1.25), (x0 + 0.38, y0, 1.33))]
    png, obj = "label.png", "label.obj"
    tex_label(os.path.join(out, png), 0.6, 0.16, [("PS-1  Pufferstrecke", 0.26, True, (25, 25, 25)),
                                                  ("Teil B, PA 6   |   Vereinzelung + Anschlag", 0.16, False, (60, 60, 60))])
    quad(os.path.join(out, obj), (x0 + 0.45, y0 - 0.002, 1.2), (0.6, 0, 0), (0, 0, 0.16))
    # end stop at the pick position (part B face at x = X_PICK - 60 mm)
    g["alu"][1].append(box((CP.X_PICK * MM - 0.06 - 0.02, -0.05, zb), (CP.X_PICK * MM - 0.06, 0.05, zb + 0.03)))
    emit("buffer_housing", g, [(obj, png, "label")] + logo(out, "logo", (x0 + 0.45, y0 - 0.002, 0.8), (0.4, 0, 0), 0))


def chain_conveyors():
    g = {"alu": [ALU, []], "chain": [[0.12, 0.12, 0.13], []], "anth": [ANTH, []], "steel": [[0.55, 0.56, 0.58], []],
         "black": [BLACK, []]}
    xa = X_MACHINE - 0.4                                     # runs on inside the machine
    for yc in (-CP.DY_CHAIN * MM, CP.DY_CHAIN * MM):
        for s in (-1, 1):                                    # two chain strands under the carrier edges
            ys = yc + s * 0.06
            g["chain"][1] += [box((xa, ys - 0.008, Z_TOP - 0.012), (X_WHEEL, ys + 0.008, Z_TOP)),
                              box((xa, ys - 0.008, Z_WHEEL - SPROCKET_R), (X_WHEEL, ys + 0.008, Z_WHEEL - SPROCKET_R + 0.012))]
            g["steel"][1].append(cyl_y(SPROCKET_R - 0.006, ys - 0.006, ys + 0.006, X_WHEEL, Z_WHEEL, 60))
            g["chain"][1].append(cyl_y(SPROCKET_R + 0.001, ys - 0.009, ys - 0.007, X_WHEEL, Z_WHEEL, 60))   # chain on the wheel
            # side profile (aluminium 40 x 80) carrying the strand
            ysp = yc + s * 0.1
            g["alu"][1] += [box((xa, ysp - 0.02, Z_TOP - 0.09), (X_WHEEL - 0.03, ysp + 0.02, Z_TOP - 0.012)),
                            box((xa, ysp - 0.02, Z_WHEEL - SPROCKET_R + 0.012), (X_WHEEL - 0.03, ysp + 0.02, Z_WHEEL - SPROCKET_R + 0.06))]
        g["steel"][1].append(cyl_y(0.012, yc - 0.13, yc + 0.13, X_WHEEL, Z_WHEEL))                 # sprocket shaft
        g["anth"][1].append(box((X_WHEEL - 0.04, yc - 0.15, Z_WHEEL - 0.04), (X_WHEEL + 0.04, yc - 0.12, Z_WHEEL + 0.04)))
        g["anth"][1].append(box((X_WHEEL - 0.04, yc + 0.12, Z_WHEEL - 0.04), (X_WHEEL + 0.04, yc + 0.15, Z_WHEEL + 0.04)))
        for s in (-1, 1):                                    # legs with feet near the sprocket end
            x = X_WHEEL - 0.12
            g["alu"][1].append(box((x - 0.02, yc + s * 0.1 - 0.02, 0.0), (x + 0.02, yc + s * 0.1 + 0.02, Z_TOP - 0.09)))
            g["anth"][1].append(box((x - 0.05, yc + s * 0.1 - 0.05, 0.0), (x + 0.05, yc + s * 0.1 + 0.05, 0.008)))
        g["alu"][1].append(box((X_WHEEL - 0.14, yc - 0.1, 0.35), (X_WHEEL - 0.10, yc + 0.1, 0.39)))  # cross brace
    emit("chain_conveyors", g)


def mesh_panel(g, x0, x1, y, z0, z1, axis=0, pitch=0.05):
    """Black mesh panel (frame 30 mm, wires 3 mm) in a vertical plane: along x at y (axis 0) or along y at x (axis 1)."""
    def b(a0, a1, c0, c1, th=0.015):
        if axis == 0:
            return box((a0, y - th, c0), (a1, y + th, c1))
        return box((y - th, a0, c0), (y + th, a1, c1))
    f = 0.03
    g["black"][1] += [b(x0, x1, z0, z0 + f), b(x0, x1, z1 - f, z1), b(x0, x0 + f, z0, z1), b(x1 - f, x1, z0, z1)]
    for a in np.arange(x0 + pitch, x1 - f, pitch):
        g["black"][1].append(b(a - 0.0015, a + 0.0015, z0, z1, 0.004))
    for c in np.arange(z0 + pitch, z1 - f, pitch):
        g["black"][1].append(b(x0, x1, c - 0.0015, c + 0.0015, 0.004))


def fence_fill():
    g = {"black": [BLACK, []], "yellow": [[0.84, 0.47, 0.0], []]}
    xf, Y, Hf = X_BUFFER + 0.0, (CP.Y_FENCE + 75) * MM, CP.H_FENCE * MM
    for s in (-1, 1):                                        # beside the buffer housing
        a, b_ = sorted((s * 0.4, s * Y))
        mesh_panel(g, a, b_, xf - 0.02, 0.1, Hf, axis=1)
        g["yellow"][1].append(box((xf - 0.05, s * Y - 0.03, 0), (xf + 0.01, s * Y + 0.03, Hf)))
        g["yellow"][1].append(box((xf - 0.08, s * Y - 0.06, 0), (xf + 0.04, s * Y + 0.06, 0.008)))
        g["yellow"][1].append(box((xf - 0.05, s * 0.4 - 0.03, 0), (xf + 0.01, s * 0.4 + 0.03, Hf)))
    mesh_panel(g, -0.4, 0.4, xf - 0.02, 1.53, Hf, axis=1)    # above the housing
    emit("fence_fill", g)


if __name__ == "__main__":
    machine()
    buffer_housing()
    chain_conveyors()
    fence_fill()
