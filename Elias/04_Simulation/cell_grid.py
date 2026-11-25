"""A field of identical cells running at once: one model, each cell replays the E4 motion at its own phase of the
cycle, and its scene geoms are copied into one shared scene at its grid position (memory of one cell).
Each cell takes one sample of the robustness study (out/robustness_cpu.json, "spec"): part B arrives up to +-5 mm off
centre on the belt and sits where that run left it after the jaws closed (seat_mm). Parts are coloured by their
lateral offset, a legend gives the numbers.

python cell_grid.py [nx] [ny]  ->  figures/cell_grid.mp4
"""
import json
import math
import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cell_video as C  # noqa: E402

S = C.S
FPS, SS = 30, 2
W, H = (int(v) for v in os.environ.get("RES", "1920x1080").split("x"))   # RES=854x480 STILLS=0,90,269 for a review
DX, DY = 6.2, 4.0                  # m grid pitch; the cell spans 5.15 x 2.93 m
T0, DUR = 4.4, 9.0                 # s: start just before the second grip, shot length (real time, no slow motion)
PHASE = C.TAKT                     # s: start offsets spread over a whole cycle, so no two cells move as one
OFF_MAX = 5.0                      # mm, offset range of the study (CAD_PLAN point 0), full red in the colour scale
CREAM, RED = np.array([0.93, 0.9, 0.8]), np.array([0.95, 0.22, 0.08])
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
    keys = [((-0.45, 0.55, -0.70), 1.1), ((0.6, -0.3, -0.75), 0.6)]
    for i in range(m.nlight):
        m.light_active[i] = 0
    for i, (dr, p) in enumerate(keys):
        m.light_active[i] = 1
        m.light_type[i] = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        m.light_dir[i] = np.array(dr) / np.linalg.norm(dr)
        m.light_diffuse[i], m.light_specular[i], m.light_ambient[i] = p, 0.15, 0.0
        m.light_castshadow[i] = 0
    m.vis.headlight.diffuse[:], m.vis.headlight.ambient[:] = 0.38, 0.24


def camera(u, centre, span):
    """u in [0, 1]: from the middle cell close up, crane up and back until the whole field fills the frame."""
    e = u * u * u * (u * (6 * u - 15) + 10)                  # quintic ease, no jerk at either end
    cam = mujoco.MjvCamera()
    cam.lookat[:] = centre + np.array([0.0, 0.0, 1.1 - 0.9 * e])
    cam.distance = 5.0 + e * (0.5 * span - 5.0)
    cam.azimuth = 112 - 8 * e                                # operator side: robots, not machine backs
    cam.elevation = -22 - 26 * e
    return cam


def fmt(v, nd=1):
    return f"{v:.{nd}f}".replace(".", ",")


def legend(dr, F1, F2, seat_max, mid_dev):
    """lower left: colour scale of the lateral offset, the two numbers of the study, live value of the middle cell"""
    k = H / 1080
    x0, y0, bw, bh = 48 * k, H - 230 * k, 420 * k, 22 * k
    dr.rounded_rectangle((x0 - 24 * k, y0 - 70 * k, x0 + bw + 24 * k, H - 36 * k), 14 * k, fill=(12, 12, 14, 175))
    dr.text((x0, y0 - 56 * k), "Querversatz von Bauteil B", font=F1, fill=(240, 240, 240))
    for i in range(int(bw)):
        u = i / (bw - 1)
        dr.line((x0 + i, y0, x0 + i, y0 + bh), fill=tuple(int(255 * v) for v in CREAM + (RED - CREAM) * u))
    dr.text((x0, y0 + bh + 4 * k), "0 mm", font=F2, fill=(200, 200, 200))
    dr.text((x0 + bw - 70 * k, y0 + bh + 4 * k), f"{fmt(OFF_MAX)} mm", font=F2, fill=(200, 200, 200))
    dr.text((x0, y0 + 64 * k), f"auf dem Band: bis ±{fmt(OFF_MAX)} mm", font=F2, fill=(235, 235, 235))
    dr.text((x0, y0 + 96 * k), f"in den Backen: max. {fmt(seat_max, 2)} mm", font=F2, fill=(235, 235, 235))
    if mid_dev is not None:
        dr.text((x0, y0 + 128 * k), f"mittlere Zelle jetzt: {fmt(mid_dev, 2)} mm", font=F2, fill=(255, 200, 120))


def main(nx=10, ny=10):
    rec = record(T0 + DUR + PHASE + 0.5)
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
    dt = rng.uniform(0.0, PHASE, n)
    mid = int(np.argmin([np.linalg.norm(o) for o in offs]))
    dt[mid] = 0.0
    study = [r for r in json.load(open(os.path.join(S.B.OUT, "robustness_cpu.json")))["results"] if r["study"] == "spec"][:n]
    study.sort(key=lambda r: -abs(r["offset"]))
    study.insert(mid, study.pop(0))                           # the close-up cell gets the largest offset
    off_local = [np.array([r["offset"], 0.0, 0.0]) for r in study]                # m, part frame = gripper frame
    seat_local = [np.array(r["seat_mm"]) * S.MM for r in study]
    seat_xy_max = max(math.hypot(r["seat_mm"][0], r["seat_mm"][1]) for r in study)
    vp = {m.body(f"vp{i}").id for i in range(C.N_PART)}
    g_frame, tcp_off = m.body("gripper_frame").id, np.array(S.PART_OFFSET)
    t_seat = {}                                               # (cell, body) -> cell time the jaws reached the part
    font = lambda sz: ImageFont.truetype(os.path.join(os.environ.get("WINDIR", "C:/Windows"), "Fonts", "segoeui.ttf"), sz)
    F1, F2 = font(round(30 * H / 1080)), font(round(24 * H / 1080))
    opt = mujoco.MjvOption()
    tmp = mujoco.MjvScene(m, 5000)
    mujoco.mj_kinematics(m, d)
    mujoco.mjv_updateScene(m, d, opt, None, mujoco.MjvCamera(), mujoco.mjtCatBit.mjCAT_ALL, tmp)
    per_cell = tmp.ngeom + 50                                 # one cell's scene plus headroom for moving parts
    r = mujoco.Renderer(m, H * SS, W * SS, max_geom=per_cell * n + 10)
    stills = [int(v) for v in os.environ["STILLS"].split(",")] if os.environ.get("STILLS") else None
    enc = None if stills else C.Encoder(os.path.join(HERE, "figures", "cell_grid.mp4"), W, H, FPS, crf=20)
    nf = int(DUR * FPS)
    sdir = os.path.join(S.B.OUT, "cell_grid_stills")
    os.makedirs(sdir, exist_ok=True)
    mid_readout = None
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
            tc = t + dt[c]
            # per part of this cell: offset in the part frame and colour; the jaws pull it from the belt offset to
            # its seat while they close (from the moment the gripper reaches it)
            pg, Rg = d.xpos[g_frame], d.xmat[g_frame].reshape(3, 3)
            shift, col, near = {}, {}, None
            for b in vp:
                if d.xpos[b][2] < -1.0:
                    t_seat.pop((c, b), None)
                    continue
                if (c, b) not in t_seat and np.linalg.norm(d.xpos[b] - (pg + Rg @ tcp_off)) < 0.002:
                    t_seat[(c, b)] = tc
                u = C.ramp((tc - t_seat[(c, b)]) / C.E.T_CLOSE) if (c, b) in t_seat else 0.0
                loc = off_local[c] * (1 - u) + seat_local[c] * u
                shift[b] = d.xmat[b].reshape(3, 3) @ loc
                dev = math.hypot(loc[0], loc[1]) / S.MM
                col[b] = np.append(CREAM + (RED - CREAM) * min(dev / OFF_MAX, 1.0), 1.0)
                gap = np.linalg.norm(d.xpos[b] - pg)
                if c == mid and (near is None or gap < near[0]):        # readout: the part nearest the gripper
                    near = (gap, dev)
            if c == mid and near is not None:
                mid_readout = near[1]
            if tmp.ngeom >= tmp.maxgeom:
                raise RuntimeError("cell scene full: raise the source scene size")
            for j in range(tmp.ngeom):
                g = tmp.geoms[j]
                if scn.ngeom >= scn.maxgeom - 1:                 # keep one slot for the floor
                    raise RuntimeError("shared scene full: raise per_cell")
                if g.pos[2] < -1.0:                          # parked parts and carriers
                    continue
                dst = scn.geoms[scn.ngeom]
                b = m.geom_bodyid[g.objid] if g.objtype == mujoco.mjtObj.mjOBJ_GEOM else -1
                mujoco.mjv_initGeom(dst, g.type, g.size, g.pos + o + shift.get(b, 0.0), g.mat.flatten(),
                                    col.get(b, g.rgba))
                dst.dataid, dst.matid, dst.objtype, dst.objid = g.dataid, g.matid, g.objtype, g.objid
                dst.category, dst.emission, dst.specular, dst.shininess = g.category, g.emission, g.specular, g.shininess
                dst.reflectance, dst.texcoord, dst.transparent = g.reflectance, g.texcoord, g.transparent
                scn.ngeom += 1
        fl = scn.geoms[scn.ngeom]                             # one dark floor under the whole field
        mujoco.mjv_initGeom(fl, mujoco.mjtGeom.mjGEOM_BOX, np.array([3 * span, 3 * span, 0.01]),
                            np.array([0.0, 0.0, -0.012]), np.eye(3).flatten(), np.array([0.13, 0.13, 0.14, 1.0]))
        scn.ngeom += 1
        im = Image.fromarray(r.render()).resize((W, H), Image.LANCZOS)
        legend(ImageDraw.Draw(im, "RGBA"), F1, F2, seat_xy_max, mid_readout)
        if stills is None:
            enc.add(np.asarray(im))
        elif f in stills:
            im.save(os.path.join(sdir, f"s_{f:04d}.png"))
        if f % 30 == 0:
            print(f"frame {f}/{nf}, {scn.ngeom} geoms", flush=True)
    if enc:
        enc.close(expected=nf)


if __name__ == "__main__":
    main(*[int(x) for x in sys.argv[1:3]])
