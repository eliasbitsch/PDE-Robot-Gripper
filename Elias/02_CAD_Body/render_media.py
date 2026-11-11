"""Headless GIFs of the gripper from out/stl + out/kinematics.json (run build.py first).

python render_media.py [cycle|drive|explode|stills ...]   -> shared/handover_viktoriia/images/
"""
import json
import math
import os
import sys

import numpy as np
import pyvista as pv

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_params as P  # noqa: E402
from parts import demo_seq  # noqa: E402

OUT = os.path.join(HERE, "out")
DEST = os.path.join(HERE, "..", "..", "shared", "handover_viktoriia", "images")
PART_B = os.path.join(HERE, "..", "..", "3D-Daten der Bauteile-20260904", "Bauteil_B.STEP")

COLORS = {"EN AW-6082": "#b8bfc7", "EN AW-7075": "#c2410c", "steel": "#737a80", "vendor": "#2a2c2e",
          "NBR": "#151515", "PA 6": "#ede6cc", "table": "#596066"}
SEE_THROUGH = {"Housing", "Cover", "Hood"}
GRIP_GREEN = "#26a540"
FPS = 15


def rot(axis, p, deg):
    """4x4 rotation about an axis through point p."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    x, y, z = axis
    r = np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                  [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                  [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = np.array(p) - r @ np.array(p)
    return m


def shift(v):
    m = np.eye(4)
    m[:3, 3] = v
    return m


class Scene:
    def __init__(self, size=(900, 680), with_part=True, see_through=True, hide=()):
        self.kin = json.load(open(os.path.join(OUT, "kinematics.json")))
        self.pl = pv.Plotter(off_screen=True, window_size=size)
        self.pl.set_background("white")
        self.pl.enable_anti_aliasing("ssaa")
        self.actors = {}
        for name, mat in self.kin["material"].items():
            if any(name.startswith(h) for h in hide):
                continue
            mesh = pv.read(os.path.join(OUT, "stl", f"{name}.stl"))
            op = 0.22 if (see_through and name in SEE_THROUGH) else 1.0
            self.actors[name] = self.pl.add_mesh(mesh, color=COLORS[mat], opacity=op, smooth_shading=False,
                                                 specular=0.3, diffuse=0.85, ambient=0.15)
        self.part = self.table = None
        if with_part and os.path.exists(PART_B):
            b = self._step_mesh(PART_B).translate(P.PART_OFFSET, inplace=False)
            zt = b.bounds[4]
            self.part = self.pl.add_mesh(b, color=COLORS["PA 6"], specular=0.2)
            self.table = self.pl.add_mesh(pv.Box((-110, 110, -80, 80, zt - 10, zt)), color=COLORS["table"])
        self.text = self.pl.add_text(" ", position=(14, size[1] - 34), font_size=11, color="#333333")

    @staticmethod
    def _step_mesh(path):
        import gmsh   # part B is FH data, kept out of git; mesh it on the fly
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(path)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", 3)
        gmsh.model.mesh.generate(2)
        tmp = os.path.join(OUT, "part_b.stl")
        gmsh.write(tmp)
        gmsh.finalize()
        return pv.read(tmp)

    def pose(self, d, zg=0.0, zp=0.0, explode=None):
        """d = travel per jaw from the closed stop, zg = gripper lift, explode(name, side) -> offset."""
        ang = math.degrees(d / self.kin["pinion_r_mm"])
        lift = shift((0, 0, zg))
        grip = abs(d - self.kin["d_grip"]) < 1e-6
        for name, actor in self.actors.items():
            mov = self.kin["moving"].get(name, 0)
            if mov == "rot":
                m = rot((0, 0, 1), (P.PINION_X, 0, 0), ang)
            elif mov == "rotx":
                m = rot((1, 0, 0), (0, 0, P.DRIVE_Z), ang)
            elif mov:
                m = shift((mov * d, 0, 0))
            else:
                m = np.eye(4)
            if explode:
                m = shift(explode(name, mov if mov in (-1, 1) else 0)) @ m
            actor.user_matrix = lift @ m
            if name.startswith("Pad_"):
                actor.prop.color = GRIP_GREEN if grip and not explode else COLORS["NBR"]
        if self.part:
            self.part.user_matrix = shift((0, 0, zp))
            self.part.SetVisibility(explode is None)
            self.table.SetVisibility(explode is None)

    def camera(self, focus, vec, dist, up=(0, 0, 1)):
        v = np.array(vec, float) / np.linalg.norm(vec)
        self.pl.camera_position = [tuple(np.array(focus) + v * dist), focus, up]


def gif(name, scene, frames, draw):
    path = os.path.join(DEST, name)
    scene.pl.open_gif(path, fps=FPS, palettesize=128, subrectangles=True)
    for i in range(frames):
        draw(i)
        scene.pl.write_frame()
    scene.pl.close()
    print(f"{name}: {frames} frames, {os.path.getsize(path) / 1e6:.1f} MB")


def cycle():
    """Pick and place of part B with the housing see-through."""
    s = Scene()
    s.camera((-10, 0, 0), (1, -1.6, 0.55), 680)
    dur, n = demo_seq.DURATION, int(demo_seq.DURATION * FPS / 1.5)

    def draw(i):
        t = i / n * dur
        d, zg, zp = demo_seq.state(t, P.D_GRIP, P.D_OPEN)
        s.pose(d, zg, zp)
        phase = "holding part B" if d <= P.D_GRIP + 1e-6 else ("open" if d >= P.D_OPEN - 1e-6 else "moving")
        s.text.SetInput(f"jaw gap {2 * (P.PAD_X0 + d):5.1f} mm   lift {zg:4.0f} mm   {phase}")
    gif("gripper_cycle.gif", s, n, draw)


def drive():
    """Housing, hood and cover hidden: motor -> bevel pair -> pinion -> racks -> jaws."""
    s = Scene(with_part=False, see_through=False, hide=("Housing", "Hood", "Cover", "Screw_ISO4762-M3x10_Cover",
                                                        "Screw_ISO4762-M4x12_Hood", "Adapter", "Screw_ISO10642",
                                                        "Window", "Cable", "Screw_ISO4762-M3x6_Window"))
    s.camera((-45, -5, -8), (-1, -1.3, 0.8), 300)
    n = 4 * FPS

    def draw(i):
        u = 0.5 - 0.5 * math.cos(2 * math.pi * i / n)
        d = P.D_OPEN - (P.D_OPEN - P.D_GRIP) * u
        s.pose(d)
        s.text.SetInput(f"pinion {math.degrees(d / (P.PINION_D / 2)):5.1f} deg   jaw travel {d:4.1f} mm")
    gif("drive_train.gif", s, n, draw)


STEPS = ["housing as the base", "rail against the locating edge, 5 x M3x8",
         "drive bracket with gearbox, motor, brake and bevel pinion", "hood over the drive, 4 x M4x12 from below",
         "pinion shaft with two 608 bearings, bevel wheel with M4 set screw",
         "carriages, slides, racks (aligned in the slots, 3 mm dowel)", "jaw holders on dowels, 2 x M4x25 each",
         "cover, 4 x M3x10", "jaws onto the dowels, plunger locks; pad plates 2 x M3x8",
         "window plate with cable gland, 4 x M3x6", "ISO 9409 adapter, 4 x M4x16 countersunk"]


def explode():
    """Parts fly in in the assembly order of CAD_PLAN.md section 8."""
    s = Scene(with_part=False)
    s.camera((-10, 0, -35), (1, -1.5, 0.5), 1150)
    steps, per = 10, int(0.8 * FPS)
    hold = FPS
    n = hold + (steps + 1) * per + 2 * hold

    def draw(i):
        tau = max(0.0, (i - hold) / per)

        def off(name, side):
            step, o = demo_seq.explode(name, side)
            f = 1.0 if tau < step else max(0.0, 1.0 - (tau - step))
            f = f * f * (3 - 2 * f)
            return tuple(f * c for c in o)
        s.pose(P.D_GRIP, explode=off)
        k = min(int(tau), steps)
        s.text.SetInput(f"{k}  {STEPS[k]}" if k else STEPS[0])
    gif("assembly.gif", s, n, draw)


def stills():
    for name, kw, cam in [("drive_train.png", {"with_part": False, "see_through": False,
                                               "hide": ("Housing", "Hood", "Cover", "Screw_ISO4762-M3x10_Cover",
                                                        "Screw_ISO4762-M4x12_Hood", "Adapter", "Screw_ISO10642",
                                                        "Window", "Cable", "Screw_ISO4762-M3x6_Window")},
                           ((-45, -5, -8), (-1, -1.3, 0.8), 300))]:
        s = Scene(size=(1200, 900), **kw)
        s.camera(*cam)
        s.pose(P.D_GRIP)
        s.pl.render()
        s.pl.screenshot(os.path.join(DEST, name))
        s.pl.close()
        print(name)


if __name__ == "__main__":
    jobs = {"cycle": cycle, "drive": drive, "explode": explode, "stills": stills}
    for j in sys.argv[1:] or ["drive"]:
        jobs[j]()
