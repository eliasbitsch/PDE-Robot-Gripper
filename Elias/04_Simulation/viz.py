"""Slow-motion multi-view video of a recorded run: cell overview plus three cameras that ride with the gripper,
contact points and contact forces (normal and friction split), clearance checks between the gripper and the part
or the cell, and a HUD with the measured values.

    rec = Recorder(dt=0.002)          # sim seconds between frames
    ... rec(m, d, phase="transfer") inside the sim loop ...
    render_video(rec, "out/x.mp4", slowmo=0.1)
"""
import os
import shutil
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import sim as S

MM = 0.001
PANEL = (640, 400)
ACCENT = (194, 65, 12)
# gripper parts the part must never touch (only the pads may), and cell objects the gripper/robot must clear
GRIPPER_KEEP_OUT = ("Jaw_", "PadPlate_", "Holder_", "Cover", "Housing", "Plunger_", "Screw_ISO4762-M3x8_Pad")
CELL_KEEP_OUT = ("belt", "carrier", "pedestal")
SEE_THROUGH = ("Housing", "Cover", "Hood", "Rail", "Window_Plate", "Bracket", "Carriage_")


class Recorder:
    """Stores the full state at a fixed sim interval, so every frame can be re-evaluated (contacts, forces)."""

    def __init__(self, dt=0.002):
        self.dt, self.frames, self._next = dt, [], 0.0

    def __call__(self, m, d, **info):
        if d.time + 1e-9 < self._next:
            return
        self._next = d.time + self.dt
        self.frames.append(dict(t=d.time, qpos=d.qpos.copy(), qvel=d.qvel.copy(), act=d.act.copy(),
                                ctrl=d.ctrl.copy(), warm=d.qacc_warmstart.copy(), info=info))


def _mesh_name(m, g):
    return m.mesh(m.geom_dataid[g]).name[2:] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_dataid[g] >= 0 else m.geom(g).name


def _geoms(m):
    names = {g: _mesh_name(m, g) for g in range(m.ngeom)}
    grip = [g for g, n in names.items() if n.startswith(GRIPPER_KEEP_OUT) and m.geom_bodyid[g] != m.body("part").id]
    part = [m.geom("part").id, m.geom("part_lug").id]
    cell = [m.geom(n).id for n in CELL_KEEP_OUT]
    gripper_bodies = {m.body(b).id for b in ("gripper_frame", "slide_L", "slide_R")}
    movers = [g for g in range(m.ngeom) if m.geom_bodyid[g] in gripper_bodies and names[g] in ("Housing", "Cover", "Jaw_L", "Jaw_R")]
    robot = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("link") and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    see = [g for g, n in names.items() if n.startswith(SEE_THROUGH)]
    return names, grip, part, cell, movers + robot[3:], see


def clearances(m, d, geoms):
    """Signed distances (m, convex hulls: conservative) of keep-out pairs; negative = penetration."""
    names, grip, part, cell, movers, _ = geoms
    out = []
    ft = np.zeros(6)
    for a in grip:
        for b in part:
            out.append((mujoco.mj_geomDistance(m, d, a, b, 0.05, ft), names[a], names[b]))
    for a in movers:
        for b in cell:
            out.append((mujoco.mj_geomDistance(m, d, a, b, 0.05, ft), names[a] if names[a] else m.body(m.geom_bodyid[a]).name, names[b]))
    return out


def pad_forces(m, d):
    pads = {m.geom("pad_L").id: "L", m.geom("pad_R").id: "R"}
    part = {m.geom("part").id, m.geom("part_lug").id}
    f6, res = np.zeros(6), {"L": [0.0, np.zeros(3)], "R": [0.0, np.zeros(3)]}   # normal sum, friction vector
    others = []
    for i in range(d.ncon):
        c = d.contact[i]
        mujoco.mj_contactForce(m, d, i, f6)
        if c.geom1 in pads and c.geom2 in part or c.geom2 in pads and c.geom1 in part:
            s = pads.get(c.geom1, pads.get(c.geom2))
            res[s][0] += f6[0]
            res[s][1] = res[s][1] + f6[1] * c.frame[3:6] + f6[2] * c.frame[6:9]
        else:
            others.append((m.geom(c.geom1).name or "?", m.geom(c.geom2).name or "?", f6[0]))
    return res, others


def _cam(lookat, forward, distance):
    f = np.asarray(forward, float)
    f /= np.linalg.norm(f)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = np.degrees(np.arctan2(f[1], f[0]))
    cam.elevation = np.degrees(np.arcsin(np.clip(f[2], -1, 1)))
    return cam


def views(m, d):
    """(title, camera, see-through) per panel. Gripper frame columns: x grip axis, y along the jaws, z back to the flange."""
    b = m.body("gripper_frame").id
    R = d.xmat[b].reshape(3, 3)
    x, y, z = R[:, 0], R[:, 1], R[:, 2]
    tcp = d.site("tcp").xpos.copy()
    pad_l = d.geom("pad_L").xpos.copy()
    return [
        ("cell", _cam(0.5 * (np.array([0.0, 0.1, 1.1]) + tcp), (-0.55, -0.75, -0.38), 2.4), False),
        ("front: both jaws, housing see-through", _cam(tcp + 0.02 * z, -y - 0.25 * z, 0.34), True),
        ("pad L close-up: contact forces", _cam(pad_l, -y + 0.35 * x - 0.15 * z, 0.13), True),
        ("back, low: part seat and lug side", _cam(tcp - 0.01 * z, y + 0.35 * x - 0.08 * z, 0.3), True),
    ]


def _font(size):
    for f in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


def render_video(rec, path, slowmo=0.1, fps=30, label="", force_scale=0.15):
    """slowmo: playback speed (0.1 = ten times slower). force_scale: arrow metres per 35 N."""
    m, d = S.load()
    geoms = _geoms(m)
    rgba0 = m.geom_rgba.copy()
    part_vis = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("part").id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    opt = mujoco.MjvOption()
    for f in (mujoco.mjtVisFlag.mjVIS_CONTACTPOINT, mujoco.mjtVisFlag.mjVIS_CONTACTFORCE, mujoco.mjtVisFlag.mjVIS_CONTACTSPLIT):
        opt.flags[f] = 1
    m.vis.map.force = force_scale / 35.0 * m.stat.meanmass
    ms = m.stat.meansize                            # vis.scale is relative to it
    m.vis.scale.contactwidth, m.vis.scale.contactheight, m.vis.scale.forcewidth = 0.004 / ms, 0.0008 / ms, 0.0015 / ms
    m.vis.rgba.contactpoint[:] = (1.0, 0.85, 0.1, 1)
    r = mujoco.Renderer(m, PANEL[1], PANEL[0])
    f_big, f_small = _font(17), _font(14)
    step = max(1, int(round(slowmo / fps / rec.dt)))   # sim seconds per video frame = slowmo / fps
    frames = rec.frames[::step]
    tmp = path + ".frames"
    os.makedirs(tmp, exist_ok=True)
    worst = (1.0, "", "")
    for k, fr in enumerate(frames):
        d.qpos[:], d.qvel[:], d.act[:], d.ctrl[:], d.qacc_warmstart[:] = fr["qpos"], fr["qvel"], fr["act"], fr["ctrl"], fr["warm"]
        mujoco.mj_forward(m, d)
        cl = clearances(m, d, geoms)
        hits = [c for c in cl if c[0] < 0]
        lo = min(cl, key=lambda c: c[0])
        worst = min(worst, lo, key=lambda c: c[0])
        pf, others = pad_forces(m, d)
        canvas = Image.new("RGB", (2 * PANEL[0], 2 * PANEL[1] + 70), (245, 245, 243))
        for i, (title, cam, see) in enumerate(views(m, d)):
            m.geom_rgba[:] = rgba0
            if see:
                m.geom_rgba[geoms[5], 3] = 0.18
                m.geom_rgba[part_vis, 3] = 0.45             # pads and force arrows sit inside the part
            for _, a, b in hits:                       # colliding parts in red
                for g, n in geoms[0].items():
                    if n in (a, b):
                        m.geom_rgba[g] = (0.9, 0.1, 0.1, 1)
            r.update_scene(d, cam, scene_option=opt)
            img = Image.fromarray(r.render())
            _d = ImageDraw.Draw(img)
            _d.rectangle(_d.textbbox((10, 8), title, font=f_small), fill=(255, 255, 255))
            _d.text((10, 8), title, fill=(20, 20, 20), font=f_small)
            canvas.paste(img, ((i % 2) * PANEL[0], (i // 2) * PANEL[1]))
        m.geom_rgba[:] = rgba0
        dr = ImageDraw.Draw(canvas)
        y0 = 2 * PANEL[1] + 8
        info = fr["info"]
        util = np.linalg.norm(pf["L"][1] + pf["R"][1]) / max(1e-6, info.get("mu", S.B.MU_PAD) * (pf["L"][0] + pf["R"][0]))
        dr.text((12, y0), f"{label}   t = {fr['t']:.3f} s   {info.get('phase', '')}   playback {slowmo:g}x", fill=(20, 20, 20), font=f_big)
        line = (f"pad normal L {pf['L'][0]:5.1f} N  R {pf['R'][0]:5.1f} N   friction use (net) {util:4.2f}   "
                f"jaw {info.get('jaw_mm', float('nan')):5.2f} mm   part movement {info.get('slip_mm', 0.0):5.2f} mm   ")
        dr.text((12, y0 + 26), line, fill=(20, 20, 20), font=f_small)
        cl_txt = (f"COLLISION {hits[0][1]} / {hits[0][2]} ({hits[0][0] / MM:.1f} mm)" if hits else
                  f"min clearance {lo[0] / MM:5.1f} mm ({lo[1]} / {lo[2]})")
        dr.text((12, y0 + 46), cl_txt + (f"   other contacts: {len(others)}" if others else ""),
                fill=ACCENT if hits else (60, 60, 60), font=f_small)
        canvas.save(os.path.join(tmp, f"{k:05d}.png"))
    r.close()
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg and path.endswith(".mp4"):
        subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(tmp, "%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", path], check=True)
    else:
        imgs = [Image.open(os.path.join(tmp, f)) for f in sorted(os.listdir(tmp))]
        imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0)
    shutil.rmtree(tmp, ignore_errors=True)
    return dict(path=path, frames=len(frames), min_clearance_mm=worst[0] / MM, min_clearance_pair=(worst[1], worst[2]))
