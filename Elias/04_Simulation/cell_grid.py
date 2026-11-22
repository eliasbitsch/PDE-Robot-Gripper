"""A field of identical cells running at once: one model, each cell replays the E4 motion with its own small time
offset, and its scene geoms are copied into one shared scene at its grid position (memory of one cell).

python cell_grid.py [nx] [ny]  ->  figures/cell_grid.mp4
"""
import math
import os
import shutil
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cell_video as C  # noqa: E402

S = C.S
FPS, W, H, SS = 30, 1920, 1080, 2
DX, DY = 6.2, 4.0                  # m grid pitch; the cell spans 5.15 x 2.93 m
T0, DUR = 4.4, 9.0                 # s: start just before the second grip, shot length (real time, no slow motion)
JITTER = 0.35                      # s: start offsets per cell, so the field does not move as one
REC_HZ = 120
FLOOR_MESH = "sr9_"                # hall floor of the single cell, replaced by one floor under the field


def record(t_max):
    """Robot joints, jaws and the mocap parts / carriers of one cell at REC_HZ (kinematic replay of cell_video)."""
    ts = list(np.arange(0.0, t_max, 1.0 / REC_HZ))
    rec = {"qpos": [], "mpos": [], "mquat": []}

    def hook(i, t, m, d):
        rec["qpos"].append(d.qpos.copy())
        rec["mpos"].append(d.mocap_pos.copy())
        rec["mquat"].append(d.mocap_quat.copy())
    C.main(n_cycles=int(t_max // C.TAKT) + 2, clean=True, hook=hook, ts_given=ts)
    return {k: np.array(v) for k, v in rec.items()}


def look(m):
    """Lights for a field instead of one cell: the five film spots become a directional key and fill."""
    keys = [((-0.45, 0.55, -0.70), 0.9), ((0.6, -0.3, -0.75), 0.45)]
    for i in range(m.nlight):
        m.light_active[i] = 0
    for i, (dr, p) in enumerate(keys):
        m.light_active[i] = 1
        m.light_type[i] = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        m.light_dir[i] = np.array(dr) / np.linalg.norm(dr)
        m.light_diffuse[i], m.light_specular[i], m.light_ambient[i] = p, 0.15, 0.0
        m.light_castshadow[i] = 0
    m.vis.headlight.diffuse[:], m.vis.headlight.ambient[:] = 0.30, 0.18


def camera(u, centre, span):
    """u in [0, 1]: from the middle cell close up, crane up and back until the whole field fills the frame."""
    e = u * u * u * (u * (6 * u - 15) + 10)                  # quintic ease, no jerk at either end
    cam = mujoco.MjvCamera()
    cam.lookat[:] = centre + np.array([0.0, 0.0, 1.1 - 0.9 * e])
    cam.distance = 5.0 + e * (0.5 * span - 5.0)
    cam.azimuth = 112 - 8 * e                                # operator side: robots, not machine backs
    cam.elevation = -22 - 26 * e
    return cam


def main(nx=10, ny=10):
    rec = record(T0 + DUR + JITTER + 0.5)
    m, d = C.build(os.path.join(S.B.OUT, "cell.xml"), cut=0, film=True)
    look(m)
    mujoco.mj_kinematics(m, d)                                # light directions reach the scene through data
    ext = m.stat.extent
    span = max(nx * DX, ny * DY)
    m.vis.map.znear, m.vis.map.zfar = 0.05 / ext, 4.0 * span / ext
    m.vis.global_.offwidth, m.vis.global_.offheight = W * SS, H * SS
    floor = {g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
             and m.mesh(m.geom_dataid[g]).name.startswith(FLOOR_MESH)}
    for g in range(m.ngeom):                                  # buffer lid glass out, as in the cell film
        if (m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and 0 < m.geom_rgba[g, 3] < 1
                and m.mesh(m.geom_dataid[g]).name.startswith("sr") and m.geom_pos[g][0] > 0.5):
            m.geom_group[g] = 4
    for g in floor:
        m.geom_group[g] = 4
    n = nx * ny
    offs = [np.array([(i - (nx - 1) / 2) * DX, (j - (ny - 1) / 2) * DY, 0.0]) for j in range(ny) for i in range(nx)]
    rng = np.random.default_rng(7)
    dt = rng.uniform(0.0, JITTER, n)
    mid = int(np.argmin([np.linalg.norm(o) for o in offs]))
    dt[mid] = 0.0
    r = mujoco.Renderer(m, H * SS, W * SS, max_geom=600 * n + 100)
    tmp = mujoco.MjvScene(m, 2000)
    opt = mujoco.MjvOption()
    frames = []
    nf = int(DUR * FPS)
    for f in range(nf):
        t = T0 + f / FPS
        cam = camera(f / (nf - 1), offs[mid], span)
        r.update_scene(d, cam, scene_option=opt)             # camera and lights; the geoms are rebuilt below
        scn = r.scene
        scn.ngeom = 0
        for c in range(n):
            k = min(int(round((t + dt[c]) * REC_HZ)), len(rec["qpos"]) - 1)
            d.qpos[:], d.mocap_pos[:], d.mocap_quat[:] = rec["qpos"][k], rec["mpos"][k], rec["mquat"][k]
            mujoco.mj_kinematics(m, d)
            mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, tmp)
            o = offs[c]
            for j in range(tmp.ngeom):
                g = tmp.geoms[j]
                if g.pos[2] < -1.0:                          # parked parts and carriers
                    continue
                dst = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(dst, g.type, g.size, g.pos + o, g.mat.flatten(), g.rgba)
                dst.dataid, dst.matid, dst.objtype, dst.objid = g.dataid, g.matid, g.objtype, g.objid
                dst.category, dst.emission, dst.specular, dst.shininess = g.category, g.emission, g.specular, g.shininess
                dst.reflectance, dst.texcoord, dst.transparent = g.reflectance, g.texcoord, g.transparent
                scn.ngeom += 1
        fl = scn.geoms[scn.ngeom]                             # one dark floor under the whole field
        mujoco.mjv_initGeom(fl, mujoco.mjtGeom.mjGEOM_BOX, np.array([3 * span, 3 * span, 0.01]),
                            np.array([0.0, 0.0, -0.012]), np.eye(3).flatten(), np.array([0.13, 0.13, 0.14, 1.0]))
        scn.ngeom += 1
        im = Image.fromarray(r.render()).resize((W, H), Image.LANCZOS)
        frames.append(np.asarray(im))
        if f % 30 == 0:
            print(f"frame {f}/{nf}, {scn.ngeom} geoms", flush=True)
    out = os.path.join(HERE, "figures", "cell_grid.mp4")
    cmd = [shutil.which("ffmpeg"), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out]
    subprocess.run(cmd, input=b"".join(np.ascontiguousarray(fr).tobytes() for fr in frames), check=True)
    print(out, len(frames), "frames")


if __name__ == "__main__":
    main(*[int(x) for x in sys.argv[1:3]])
