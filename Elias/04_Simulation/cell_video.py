"""Running cell at the plant takt: parts queue on the buffer line, the robot loads both chains in turn, the carriers
index into the machine. Kinematic replay of the E4 motion (e4_cycle.segments, TimeLaw), no contact physics.

python cell_video.py [n_cycles]  ->  figures/cell_takt.mp4

Takt 5.0 s (Angabe), robot cycle from E4; the robot waits at the pick for the rest of the takt. Chain 2 (+y) is the
E4 path, chain 1 (-y) its mirror image (enclosure symmetric). A6 turns the tool so part B lands square on the
carrier (yaw 180 deg) on both chains; with A6 = 0 it would sit 20.6 deg skewed.
"""
import math
import os
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import e4_cycle as E
import sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "06_Cell", "simready"))
import cell_parts as CPT  # noqa: E402

GP, CP, MM = S.GP, S.CP, S.MM
TAKT = GP.TAKTZEIT
FPS, W, H = 30, 1600, 900
PITCH_BELT, PITCH_CHAIN = 0.16, 0.30
T_INDEX, T_ADVANCE = 1.0, 0.6
RC = CPT.SPROCKET_R + CP.CARRIER[2] / 2 * MM             # carrier centre radius around the sprocket
S_LOAD = CPT.X_WHEEL - CP.X_PLACE * MM
N_PART, N_CARR = 14, 12
NAMES_DE = ["Greifen", "Anheben", "Transfer (beladen)", "Absenken auf Werkstückträger", "Öffnen", "Anheben (leer)",
            "Rückfahrt (leer)", "Absenken zum nächsten Teil"]


def place_q(side, lift, a6):
    q, bad = CP.ik_tool_down((CP.X_PLACE, side * CP.DY_CHAIN, CP.H_PLACE + S.wrist_above_part_bottom() + lift), a6=a6)
    assert not bad, bad
    return np.radians(q)


def transfer(side, a6):
    from scipy.interpolate import PchipInterpolator
    a = math.radians(side * S.VIA_A1)
    qv, bad = CP.ik_tool_down((CP.BASE[0] + S.VIA_R * math.cos(a), CP.BASE[1] + S.VIA_R * math.sin(a), S.VIA_Z))
    qs = np.array([S.q_pick(E.LIFT)[0], np.radians(qv), place_q(side, E.LIFT, a6)])
    vmax = np.radians(E.VMAX)
    seg = [np.max(np.abs(b - a_) / vmax) for a_, b in zip(qs[:-1], qs[1:])]
    return PchipInterpolator(np.concatenate([[0.0], np.cumsum(seg)]) / sum(seg), qs)


def square_a6(m, d, side):
    """A6 offset that puts part B at yaw 180 deg on the carrier."""
    best = None
    for a6 in np.arange(-60.0, 60.01, 0.1):
        S.set_robot(m, d, place_q(side, 0.0, a6))
        R = d.xmat[m.body("gripper_frame").id].reshape(3, 3)
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        err = abs((yaw - 180.0 + 180.0) % 360.0 - 180.0)
        if best is None or err < best[0]:
            best = (err, a6)
    return best[1]


def timeline(m, d, side, a6):
    q0, q1 = S.q_pick()[0], S.q_pick(E.LIFT)[0]
    q2, q3 = place_q(side, E.LIFT, a6), place_q(side, 0.0, a6)
    fwd = transfer(side, a6)
    back = lambda u, nu=0: fwd(1.0 - np.asarray(u), nu) * (-1) ** nu
    segs = [(q0, E.T_CLOSE, GP.D_GRIP), (E.line(q0, q1), E.A_LOADED, GP.D_GRIP), (fwd, E.A_LOADED, GP.D_GRIP),
            (E.line(q2, q3), E.A_LOADED, GP.D_GRIP), (q3, E.T_OPEN, GP.D_OPEN), (E.line(q3, q2), E.A_EMPTY, GP.D_OPEN),
            (back, E.A_EMPTY, GP.D_OPEN), (E.line(q1, q0), E.A_EMPTY, GP.D_OPEN)]
    out, t0 = [], 0.0
    for path, lim, jaw in segs:
        if isinstance(path, np.ndarray):
            out.append((t0, lim, (lambda t, p=path: p), jaw))
            t0 += lim
        else:
            law = S.TimeLaw(path, lim, m=m, d=d, vmax_deg=tuple(E.SPEED_USE * E.VMAX))
            out.append((t0, law.T, (lambda t, p=path, l=law: p(l(t)[0])), jaw))
            t0 += law.T
    return out, t0


def at(tl, t):
    for k, (t0, T, f, jaw) in enumerate(tl):
        if t < t0 + T or k == len(tl) - 1:
            return k, f(min(t - t0, T)), jaw


def ramp(x):
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def carrier_pose(s):
    """Carrier centre and rotation about y on the chain loop; s along the loop, s = 0 where the top run leaves the
    sprocket, top run (s > 0) into the machine, return run underneath."""
    if s >= 0:
        return np.array([CPT.X_WHEEL - s, CPT.Z_WHEEL + RC]), 0.0
    if s >= -math.pi * RC:
        th = -s / RC
        return np.array([CPT.X_WHEEL + RC * math.sin(th), CPT.Z_WHEEL + RC * math.cos(th)]), th
    u = -math.pi * RC - s
    return np.array([CPT.X_WHEEL - u, CPT.Z_WHEEL - RC]), math.pi


def build(m0_path):
    spec = mujoco.MjSpec.from_file(m0_path)
    pb = spec.body("part")
    part_geoms = [(g.type, g.meshname, list(g.pos), list(g.quat), list(g.rgba), list(g.size)) for g in pb.geoms]
    wb = spec.worldbody
    for i in range(N_PART):
        b = wb.add_body(name=f"vp{i}", mocap=True, pos=[0, 0, -5])
        for (ty, mesh, pos, quat, rgba, size) in part_geoms:
            g = b.add_geom(type=ty, pos=pos, quat=quat, rgba=rgba if rgba[3] > 0 else [0.85, 0.82, 0.7, 1], size=size,
                           contype=0, conaffinity=0, group=1)
            if mesh:
                g.meshname = mesh
    cx, cy, cz = CP.CARRIER[0] / 2 * MM, CP.CARRIER[1] / 2 * MM, CP.CARRIER[2] / 2 * MM
    for lane in (0, 1):
        for j in range(N_CARR):
            b = wb.add_body(name=f"wt{lane}_{j}", mocap=True, pos=[0, 0, -5])
            b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[cx, cy, cz], rgba=[0.25, 0.45, 0.7, 1], contype=0, conaffinity=0)
            for sx in (-1, 1):                                     # locating rails of the nest
                b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.006, 0.06, 0.006], pos=[sx * 0.07, 0, cz + 0.006],
                           rgba=[0.75, 0.77, 0.8, 1], contype=0, conaffinity=0)
    m = spec.compile()
    for n in ("carrier", "carrier_1"):
        m.geom_rgba[m.geom(n).id, 3] = 0.0
    # fence bays on the camera side (+y) see-through, as a cut-away
    from simready import layout as SR
    for k, (asset, _, o, _) in enumerate(SR.placements()):
        if asset.startswith("wire_machineGuard") and o[1] > 0:
            for g in range(m.ngeom):
                if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.mesh(m.geom_dataid[g]).name.startswith(f"sr{k}_"):
                    m.geom_rgba[g, 3] = min(m.geom_rgba[g, 3], 0.18)
    return m, mujoco.MjData(m)


def main(n_cycles=4):
    m, d = build(os.path.join(S.B.OUT, "cell.xml"))
    sides = {+1: None, -1: None}
    for side in sides:
        a6 = square_a6(m, d, side)
        tl, T = timeline(m, d, side, a6)
        sides[side] = (tl, T, a6)
        print(f"chain {'2' if side > 0 else '1'}: A6 {a6:+.1f} deg, robot cycle {T:.2f} s")
    S.set_robot(m, d, S.q_pick()[0])
    S.place_part_in_gripper(m, d)
    belt_pos, belt_quat = d.body("part").xpos.copy(), d.body("part").xquat.copy()
    d.qpos[m.joint("part").qposadr[0] + 2] = -5.0
    T_tl = {s: v[1] for s, v in sides.items()}
    rel = {}                                                   # part -> (lane, carrier, offset, quat)
    lane_of = lambda k: 1 if k % 2 == 0 else 0                 # chain 2 (+y, lane 1) first
    side_of = lambda k: +1 if k % 2 == 0 else -1

    def t_release(k):
        tl = sides[side_of(k)][0]
        return k * TAKT + tl[4][0] + tl[4][1]

    def t_index(k):                                            # chain moves once the robot has lifted off
        tl = sides[side_of(k)][0]
        return k * TAKT + tl[5][0] + tl[5][1]

    def idx(lane, t):
        return sum(ramp((t - t_index(k)) / T_INDEX) for k in range(n_cycles + 2) if lane_of(k) == lane)

    def advances(t):
        tl0 = sides[+1][0]
        return sum(ramp((t - (k * TAKT + tl0[1][0] + tl0[1][1])) / T_ADVANCE) for k in range(n_cycles + 2))

    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.0, 1.05)
    cam.distance, cam.azimuth, cam.elevation = 4.3, 292, -30
    f_big = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 30)
    f = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 24)
    frames, n_frames = [], int((n_cycles * TAKT + 0.5) * FPS)
    q_wait = S.q_pick()[0]
    for i in range(n_frames):
        t = i / FPS
        k = int(t // TAKT)
        tc = t - k * TAKT
        side = side_of(k)
        tl, T, _ = sides[side]
        if tc < T:
            seg, q, jaw = at(tl, tc)
            seg_name = NAMES_DE[seg]
        else:
            seg, q, jaw, seg_name = -1, q_wait, GP.D_OPEN, "Warten auf Takt"
        S.set_robot(m, d, q, jaw=jaw * MM)
        gb = m.body("gripper_frame").id
        Rg, pg = d.xmat[gb].reshape(3, 3), d.xpos[gb]
        qg = np.zeros(4)
        mujoco.mju_mat2Quat(qg, Rg.flatten())
        # carriers
        cpos = {}
        for lane, y in ((0, -CP.DY_CHAIN * MM), (1, CP.DY_CHAIN * MM)):
            ix = idx(lane, t)
            for j in range(N_CARR):
                s = S_LOAD + PITCH_CHAIN * (j - 6 + ix)
                mb = m.body(f"wt{lane}_{j}").mocapid[0]
                if -math.pi * RC - 0.5 < s < 0.75:
                    xz, th = carrier_pose(s)
                    d.mocap_pos[mb] = (xz[0], y, xz[1])
                    d.mocap_quat[mb] = (math.cos(th / 2), 0, math.sin(th / 2), 0)
                    cpos[(lane, j)] = np.array([xz[0], y, xz[1]])
                else:
                    d.mocap_pos[mb] = (0, 0, -5)
        # parts: on the belt, in the gripper, on a carrier
        adv = advances(t)
        for n in range(N_PART):
            mb = m.body(f"vp{n}").mocapid[0]
            t_grab = n * TAKT
            if n < n_cycles and t >= t_release(n):
                lane = lane_of(n)
                if n not in rel:
                    j = int(round(6 - idx(lane, t_release(n))))
                    rel[n] = (lane, j, part_place[n][0] - cpos[(lane, j)], part_place[n][1])
                lane, j, off, qq = rel[n]
                if (lane, j) in cpos and cpos[(lane, j)][0] > CPT.X_MACHINE - 0.35:
                    d.mocap_pos[mb], d.mocap_quat[mb] = cpos[(lane, j)] + off, qq
                else:
                    d.mocap_pos[mb] = (0, 0, -5)
            elif n < n_cycles and t >= t_grab:
                p = pg + Rg @ S.PART_OFFSET
                d.mocap_pos[mb], d.mocap_quat[mb] = p, qg
                part_place[n] = (p.copy(), qg.copy())
            else:
                x = belt_pos[0] + PITCH_BELT * (n - adv)
                if x < (CP.BELT_X[1] - 60) * MM:
                    d.mocap_pos[mb] = (x, belt_pos[1], belt_pos[2])
                    d.mocap_quat[mb] = belt_quat
                else:
                    d.mocap_pos[mb] = (0, 0, -5)
        mujoco.mj_forward(m, d)
        r.update_scene(d, cam)
        im = Image.fromarray(r.render())
        done = sum(1 for n in range(n_cycles) if t >= t_release(n))
        hud(im, t, k, tc, T, seg_name, jaw, done, f_big, f)
        frames.append(np.asarray(im))
    out = os.path.join(HERE, "figures", "cell_takt.mp4")
    import shutil
    import subprocess
    cmd = [shutil.which("ffmpeg"), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", out]
    subprocess.run(cmd, input=b"".join(np.ascontiguousarray(fr).tobytes() for fr in frames), check=True)
    print(out, len(frames), "frames")


part_place = {}


def hud(im, t, k, tc, T, seg_name, jaw, done, f_big, f):
    dr = ImageDraw.Draw(im, "RGBA")
    dr.rectangle([20, 20, 640, 250], fill=(255, 255, 255, 215), outline=(60, 60, 60, 255))
    dr.text((36, 30), f"Takt {k + 1}    t = {t:5.2f} s", font=f_big, fill=(20, 20, 20))
    dr.text((36, 74), f"Taktzeit 5,0 s (vorgegeben)   Roboterzyklus {T:.2f} s", font=f, fill=(40, 40, 40))
    dr.text((36, 106), f"Kette {'2' if k % 2 == 0 else '1'}:  {seg_name}", font=f, fill=(40, 40, 40))
    dr.text((36, 138), f"Greifer: {'geschlossen' if jaw < GP.D_OPEN - 0.1 else 'offen'}   Teile übergeben: {done}",
            font=f, fill=(40, 40, 40))
    x0, x1, y = 36, 620, 190
    dr.rectangle([x0, y, x1, y + 28], fill=(225, 225, 225, 255), outline=(90, 90, 90, 255))
    dr.rectangle([x0, y, x0 + (x1 - x0) * min(tc, T) / TAKT, y + 28], fill=(194, 65, 12, 255))
    if tc > T:
        dr.rectangle([x0 + (x1 - x0) * T / TAKT, y, x0 + (x1 - x0) * tc / TAKT, y + 28], fill=(150, 150, 150, 255))
    xr = x0 + (x1 - x0) * T / TAKT
    dr.line([xr, y - 6, xr, y + 34], fill=(20, 20, 20, 255), width=2)
    dr.text((x0, y + 32), "Roboter in Bewegung", font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 16), fill=(194, 65, 12))
    dr.text((xr + 6, y + 32), "Reserve", font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 16), fill=(90, 90, 90))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 4)
