"""GIFs of the wedge-hook concept: exploded view and the mechanism gripping part B.

python concepts/render_wedge.py  (after fc.sh concepts/wedge_hook.py)  ->  out_concepts/wedge_hook/*.gif

All parts are exported at the grip position d = 2; for another opening d the slides and jaws move by +-(d - 2) and
the wedge with the piston by -(d - 2) / tan 30 deg (wedge down = open).
"""
import math
import os
import sys

import imageio.v2 as imageio
import numpy as np
import pyvista as pv

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
import gripper_params as P  # noqa: E402

pv.OFF_SCREEN = True
SRC = os.path.join(ROOT, "out_concepts", "wedge_hook")
PART_B = os.path.join(ROOT, "..", "04_Simulation", "meshes", "part_b.stl")
TAN = math.tan(math.radians(30.0))
COL = {"Housing": "#c4c9ce", "Cap_flange": "#b7bec4", "Wedge": "#2f6f9f", "Piston": "#5a8fb8",
       "Slide_L": "#7b8288", "Slide_R": "#7b8288", "Jaw_L": "#c2410c", "Jaw_R": "#c2410c",
       "PadPlate_L": "#55595d", "PadPlate_R": "#55595d", "Pad_L": "#111111", "Pad_R": "#111111"}
SIDE = {k: (-1 if k.endswith("_L") else 1) for k in COL if k.split("_")[0] in ("Slide", "Jaw", "PadPlate", "Pad")}
M = {n: pv.read(os.path.join(SRC, f"{n}.stl")) for n in COL}
PART = pv.read(PART_B).translate(P.PART_OFFSET)


def pose(d):
    out = {}
    for n, m in M.items():
        if n in SIDE:
            out[n] = m.translate((SIDE[n] * (d - P.D_GRIP), 0, 0), inplace=False)
        elif n in ("Wedge", "Piston"):
            out[n] = m.translate((0, 0, -(d - P.D_GRIP) / TAN), inplace=False)
        else:
            out[n] = m
    return out


def frame(p, shapes, see_through, part=None, label=""):
    p.renderer.clear_actors()
    for n, m in shapes.items():
        op = 0.18 if (see_through and n in ("Housing", "Cap_flange")) else 1.0
        p.add_mesh(m, color=COL[n], opacity=op, smooth_shading=True, specular=0.3)
    if part is not None:
        p.add_mesh(part, color="#e8dcb8", smooth_shading=True)
    if label:
        p.add_text(label, font_size=11, color="black", position="upper_left")


def mechanism(path, n=48):
    """Open -> grip -> hold -> open, two views: iso with the housing see-through, front view (x-z)."""
    p = pv.Plotter(off_screen=True, window_size=(1500, 700), shape=(1, 2))
    p.set_background("#f4f4f2")
    ds = list(np.linspace(P.D_OPEN, P.D_GRIP, n // 2)) + [P.D_GRIP] * 8 + list(np.linspace(P.D_GRIP, P.D_OPEN, n // 2))
    imgs = []
    for d in ds:
        sh = pose(d)
        stroke = (P.D_OPEN - d) / TAN
        txt = f"jaw opening d = {d:4.1f} mm   piston {('up' if d < P.D_OPEN else 'down')} by {stroke:4.1f} mm\n" \
              f"wedge 30 deg: piston up = close, down = open"
        p.subplot(0, 0)
        frame(p, sh, True, PART, txt)
        p.view_vector((0.8, -1.0, 0.55), viewup=(0, 0, 1))
        p.camera.parallel_projection = True
        p.camera.focal_point, p.camera.parallel_scale = (0, 0, -35), 95
        p.subplot(0, 1)
        frame(p, sh, True, PART, "front view (housing see-through)")
        p.view_vector((0, -1, 0), viewup=(0, 0, 1))
        p.camera.parallel_projection = True
        p.camera.focal_point, p.camera.parallel_scale = (0, 0, -35), 95
        imgs.append(p.screenshot(return_img=True))
    p.close()
    imageio.mimsave(path, imgs, duration=0.06, loop=0)


def exploded(path, n=40):
    """Parts move apart along their assembly direction and back."""
    off = {"Cap_flange": (0, 0, 90), "Piston": (0, 0, 60), "Wedge": (0, 0, 30), "Housing": (0, 0, 0),
           "Slide_L": (-55, 0, -10), "Slide_R": (55, 0, -10), "Jaw_L": (-95, 0, -45), "Jaw_R": (95, 0, -45),
           "PadPlate_L": (-75, 0, -45), "PadPlate_R": (75, 0, -45), "Pad_L": (-62, 0, -45), "Pad_R": (62, 0, -45)}
    p = pv.Plotter(off_screen=True, window_size=(1100, 900))
    p.set_background("#f4f4f2")
    ts = list(np.linspace(0, 1, n // 2)) + [1.0] * 10 + list(np.linspace(1, 0, n // 2)) + [0.0] * 6
    imgs = []
    base = pose(P.D_GRIP)
    for t in ts:
        s = 0.5 - 0.5 * math.cos(math.pi * t)
        sh = {k: m.translate(tuple(s * np.array(off[k])), inplace=False) for k, m in base.items()}
        frame(p, sh, False, None, "wedge-hook concept: cap = robot flange, piston, wedge, housing, slides, jaws")
        p.view_vector((0.8, -1.0, 0.45), viewup=(0, 0, 1))
        p.camera.parallel_projection = True
        p.camera.focal_point, p.camera.parallel_scale = (0, 0, -20), 140
        imgs.append(p.screenshot(return_img=True))
    p.close()
    imageio.mimsave(path, imgs, duration=0.07, loop=0)


if __name__ == "__main__":
    mechanism(os.path.join(SRC, "wedge_mechanism.gif"))
    exploded(os.path.join(SRC, "wedge_exploded.gif"))
    print("ok", SRC)
