"""Running cell at the plant takt: parts queue on the buffer line, the robot loads both chains in turn, the carriers
index into the machine. Kinematic replay of the E4 motion (e4_cycle.segments, TimeLaw), no contact physics.

python cell_video.py [n_cycles]  ->  figures/cell_takt.mp4

Takt 5.0 s (Angabe), robot cycle from E4 (same poses and paths, sim.q_place / sim.transfer_path, both chains); the
robot waits at the pick for the rest of the takt.
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
NAMES_EN = ["close", "lift", "transfer (loaded)", "lower onto the carrier", "open", "lift (empty)", "return (empty)",
            "lower to the next part"]
NAMES_DE = ["Greifen", "Anheben", "Transfer (beladen)", "Absenken auf Werkstückträger", "Öffnen", "Anheben (leer)",
            "Rückfahrt (leer)", "Absenken zum nächsten Teil"]


def timeline(m, d, side):
    q0, q1 = S.q_pick()[0], S.q_pick(E.LIFT)[0]
    q2, q3 = S.q_place(E.LIFT, side)[0], S.q_place(0.0, side)[0]
    fwd = S.transfer_path(E.LIFT, side)
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


def build(m0_path, cut=+1, film=False):
    spec = mujoco.MjSpec.from_file(m0_path)
    if film:                       # stage look: black surround, spots on the cell instead of the sun
        for tx in spec.textures:
            if tx.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
                tx.rgb1, tx.rgb2 = [0, 0, 0], [0, 0, 0]
        for li in spec.worldbody.lights:
            li.active = False
        for pos, aim, cut_deg, power in (((0.3, -2.2, 4.6), (0.1, 0.0, 0.9), 34, 0.95),   # key, operator side
                                         ((-2.6, 1.4, 4.2), (-1.0, 0.0, 1.0), 30, 0.55),  # machine and chains
                                         ((2.8, 1.2, 4.0), (1.3, 0.0, 1.0), 28, 0.55),    # buffer line
                                         ((-0.3, 2.6, 3.8), (0.0, 0.0, 1.2), 32, 0.35),   # rim from behind
                                         ((-0.8, -3.4, 3.4), (-1.6, -1.2, 1.2), 26, 0.5)):  # machine front, cabinet
            d_ = np.array(aim) - np.array(pos)
            li = spec.worldbody.add_light(pos=list(pos), dir=list(d_ / np.linalg.norm(d_)), cutoff=cut_deg, exponent=8)
            li.diffuse, li.specular, li.ambient = [power] * 3, [0.25] * 3, [0.0] * 3
            li.castshadow = False
    pb = spec.body("part")
    # visual geoms only: the collision boxes (group 3) lie on the part's faces and z-fight with it if made visible
    part_geoms = [(g.type, g.meshname, list(g.pos), list(g.quat), list(g.rgba), list(g.size)) for g in pb.geoms
                  if g.group != 3]
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
    # fence bays on the camera side (cut = sign of y) see-through, as a cut-away
    from simready import layout as SR
    for k, (asset, _, o, _) in enumerate(SR.placements()):
        if asset.startswith("wire_machineGuard") and o[1] * cut > 0:
            for g in range(m.ngeom):
                if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.mesh(m.geom_dataid[g]).name.startswith(f"sr{k}_"):
                    m.geom_rgba[g, 3] = min(m.geom_rgba[g, 3], 0.18)
    return m, mujoco.MjData(m)


def main(n_cycles=4, clean=False, slow=1.0, hook=None, ts_given=None):
    """clean: no HUD, 1920 x 1080, writes figures/cell_clean.mp4 and out/cell_clean_frames.json (per frame: real time,
    segment) for the overlays of the explainer film; slow: video time per real second. hook(i, t, m, d): called with
    the posed model at each real time of ts_given instead of rendering (cell_grid.py)."""
    global W, H
    if clean:
        W, H = 1920, 1080
    # explainer film: no see-through fence bays; transparent geoms are re-sorted every frame and flicker when the camera
    # moves. The wire mesh is open anyway.
    m, d = build(os.path.join(S.B.OUT, "cell.xml"), cut=0 if clean else +1, film=clean)
    # the model spans ~17 m (plant, belt), so MuJoCo's default near/far (0.17 m ... 870 m) and the 35 m shadow box
    # leave too little depth and shadow resolution: coplanar faces z-fight and the shadows shimmer. Fit both to the cell.
    ext = m.stat.extent
    m.vis.map.znear, m.vis.map.zfar = (0.05 if clean else 0.3) / ext, 20.0 / ext
    m.vis.map.shadowclip = 3.5 / ext
    m.vis.quality.shadowsize, m.vis.quality.offsamples = 8192, 8
    SS = 2 if clean else 1                                     # supersampling against shimmer on the wire mesh
    m.vis.global_.offwidth, m.vis.global_.offheight = max(W * SS, m.vis.global_.offwidth), max(H * SS, m.vis.global_.offheight)
    sides = {+1: None, -1: None}
    for side in sides:
        tl, T = timeline(m, d, side)
        sides[side] = (tl, T, None)
        print(f"chain {'2' if side > 0 else '1'}: robot cycle {T:.2f} s")
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

    r = mujoco.Renderer(m, H * SS, W * SS)
    opt = mujoco.MjvOption()
    if clean:
        # no shadows in the film: the wire-mesh fence casts a grid finer than any shadow map, which breaks up into
        # blotches on parts and gripper and flickers while the camera moves; a stronger headlight shades instead
        m.light_castshadow[:] = 0
        m.vis.headlight.diffuse[:] = (0.22, 0.22, 0.22)       # a little fill from the camera, the spots do the rest
        m.vis.headlight.ambient[:] = (0.06, 0.06, 0.06)                                                  # shell parts into group 5, switched off for the cut view
        for g in range(m.ngeom):
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.mesh(m.geom_dataid[g]).name.startswith(HIDE_SHELL):
                m.geom_group[g] = 5
        for g in range(m.ngeom):                               # buffer lid glass out: an open window onto the queue
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and 0 < m.geom_rgba[g, 3] < 1 and                     m.mesh(m.geom_dataid[g]).name.startswith("sr") and m.geom_pos[g][0] > 0.5:
                m.geom_group[g] = 4
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.0, 1.05)
    cam.distance, cam.azimuth, cam.elevation = 4.3, 292, -30
    if clean:                                                  # closer on the robot, room left for the overlays
        cam.lookat[:] = (0.05, 0.0, 1.15)             # operator side (-y): switch cabinet and the FHTW logos
        cam.distance, cam.azimuth = 3.1, 112
    f_big = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 30)
    f = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 24)
    frames = []
    flog = []
    q_wait = S.q_pick()[0]
    if clean:   # (real start, real end, video seconds per real second): the whole cell, then follow part 1 at half speed
        ts = []
        for a, b, sl in FILM_TIME:
            ts += list(np.arange(a, b, 1.0 / (FPS * sl)))
    else:
        ts = [i / FPS / slow for i in range(int((n_cycles * TAKT + 0.5) * FPS * slow))]
    ts = ts_given if ts_given is not None else ts
    look = None

    def robot_state(t):
        k = int(t // TAKT)
        tc = t - k * TAKT
        tl, T, _ = sides[side_of(k)]
        if tc < T:
            seg, q, jaw = at(tl, tc)
            if clean and seg in (0, 4):                        # jaws move over their segment instead of jumping
                u = min(1.0, max(0.0, (tc - tl[seg][0]) / tl[seg][1]))
                if seg == 0:                                   # close: fast, then slow onto the part (two-stage)
                    jaw = GP.D_OPEN + (GP.D_GRIP - GP.D_OPEN) * (1 - (1 - u) ** 3)
                else:
                    jaw = GP.D_GRIP + (GP.D_OPEN - GP.D_GRIP) * u * u * (3 - 2 * u)
            return k, tc, T, seg, q, jaw, NAMES_DE[seg]
        return k, tc, T, -1, q_wait, GP.D_OPEN, "Warten auf Takt"

    d2 = mujoco.MjData(m)
    gfb = m.body("gripper_frame").id

    def grip_point(t):                                         # where the part sits in the gripper, at time t
        _, _, _, _, q, jaw, _ = robot_state(t)
        S.set_robot(m, d2, q, jaw=jaw * MM)
        return d2.xpos[gfb] + d2.xmat[gfb].reshape(3, 3) @ S.PART_OFFSET

    for i, t in enumerate(ts):
        k, tc, T, seg, q, jaw, seg_name = robot_state(t)
        side = side_of(k)
        tl = sides[side][0]
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
        if hook is not None:
            hook(i, t, m, d)
            continue
        if clean:
            grip = d.xpos[m.body("gripper_frame").id] + d.xmat[m.body("gripper_frame").id].reshape(3, 3) @ S.PART_OFFSET
            look = film_camera(cam, t, d.mocap_pos[m.body(f"vp{FOLLOW}").mocapid[0]].copy(), grip, look)
            opt.geomgroup[5] = 0 if SEE_MECH[0] <= t < SEE_MECH[1] else 1
            if os.environ.get("CAMDRY"):                   # print the followed part's path, no rendering
                if i % 15 == 0:
                    print(f"t {t:5.2f} part {np.round(d.mocap_pos[m.body(f'vp{FOLLOW}').mocapid[0]], 3)}")
                continue
        r.update_scene(d, cam, scene_option=opt)
        if clean:
            vec = film_vectors(r.scene, m, d, cam.distance, grip_point, t, jaw)
        im = Image.fromarray(r.render())
        if SS > 1:
            im = im.resize((W, H), Image.LANCZOS)
        done = sum(1 for n in range(n_cycles) if t >= t_release(n))
        if clean:
            flog.append(dict(t=round(float(t), 4), seg=seg, name=NAMES_EN[seg] if seg >= 0 else "waiting for the takt",
                             v=round(vec[0], 3), a=round(vec[1], 3), f=round(vec[2], 1)))
        else:
            hud(im, t, k, tc, T, seg_name, jaw, done, f_big, f)
        frames.append(np.asarray(im))
        if i % 60 == 0:
            print(f"frame {i}/{len(ts)}", flush=True)
    if os.environ.get("CAMDRY") or hook is not None:
        return
    out = os.path.join(HERE, "figures", "cell_clean.mp4" if clean else "cell_takt.mp4")
    if clean:
        import json
        json.dump(dict(fps=FPS, film_time=FILM_TIME, follow=FOLLOW, follow_cycle_start=FOLLOW * TAKT, cycle=T_tl[-1],
                       takt=TAKT, frames=flog),
                  open(os.path.join(S.B.OUT, "cell_clean_frames.json"), "w"))
    import shutil
    import subprocess
    cmd = [shutil.which("ffmpeg"), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", out]
    subprocess.run(cmd, input=b"".join(np.ascontiguousarray(fr).tobytes() for fr in frames), check=True)
    print(out, len(frames), "frames")


part_place = {}

# explainer film: real time ranges and their slow-down (video s per real s); part 3 (fourth cycle, chain 1 on the
# camera side) is followed, so the robot first loads both chains a few times while the camera tours the cell
FILM_TIME = [(0.0, 13.4, 1.0), (13.4, 14.7, 2.0), (14.7, 15.6, 3.0), (15.6, 18.6, 2.0), (18.6, 19.5, 1.0),
             (19.6, 20.7, 3.0), (20.7, 21.6, 1.5)]
FOLLOW, FILM_CYCLES = 3, 5
SEE_MECH = (19.55, 22.0)                            # cut to the next grip with the gripper shell hidden
HIDE_SHELL = ("g_Housing", "g_Cover", "g_Hood", "g_Window", "g_Tower", "g_Lid", "g_Cable_Gland",
              "g_Screw_ISO10642-M3x10_Cover", "g_Screw_ISO10642-M3x8_Window", "g_Screw_ISO4762-M4x12_Hood",
              "g_Screw_ISO10642-M4x16_Adapter")      # screws whose parts are hidden would float
FENCE_IN = ((-1.07, 0.97), 0.52, 2.25)              # x range, |y|, height: the camera never crosses the fence
# camera stations: (real t, look-at point or "part" / "grip", camera offset from it [m], cut before this key)
P = np.array
CAM_KEYS = [
    (0.0, P([0.2, 0.0, 0.9]), P([5.0, -2.9, 4.8]), False),          # the whole cell, high and wide
    (3.5, P([0.0, -0.3, 1.0]), P([2.2, -3.2, 2.2]), False),
    (6.0, P([-0.9, -1.0, 1.35]), P([1.1, -1.1, 0.35]), False),      # ABB OmniCore on the cabinet, FHTW logo on the machine
    (7.5, P([-0.95, -1.0, 1.45]), P([0.9, -0.9, 0.25]), False),
    (9.0, P([1.8, -0.40, 1.28]), P([0.25, -1.0, 0.15]), False),     # BL-1 buffer line sign
    (10.6, P([2.0, 0.0, 1.1]), P([0.35, -0.55, 1.0]), False),      # lid window over the queue end (the hood covers the rest)
    (12.3, P([0.9, 0.0, 1.1]), P([0.4, -0.8, 1.8]), False),         # up over the fence corner, clear of the posts
    (13.0, P([0.75, 0.0, 1.05]), P([-0.2, -0.45, 1.2]), False),     # down into the cell
    (14.0, "part", P([-0.5, -0.35, 0.4]), False),                   # the pick
    (14.9, "part", P([-0.25, -0.28, 0.12]), False),                 # close: the jaws close on the part
    (15.5, "part", P([-0.2, -0.3, 0.1]), False),
    (16.4, "part", ("side", 0.9, 0.3, 0.35), False),               # lift and swing, beside the arm plane
    (17.4, "part", ("side", 0.9, 0.3, 0.3), False),
    (18.1, "part", ("side", 0.8, -0.1, 0.3), False),               # set down on chain 1 (outwards is the machine wall)
    (19.5, "part", P([0.5, -0.22, 0.28]), False),                   # the carrier takes it towards the machine
    (19.55, "grip", P([-0.3, -0.3, 0.15]), True),                   # cut: the next grip, shell hidden
    (20.3, "grip", P([-0.22, -0.28, 0.08]), False),
    (21.6, "grip", P([-0.28, -0.2, 0.18]), False),
]


def _ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


ROBOT_XY = np.array([-0.1, 0.0])                    # robot base axis


def _offset(spec, look):
    """('side', across, out, up): beside the vertical plane through the robot axis and the look point (the arm lies in
    it, so it never crosses the line of sight), a little outwards and up. Otherwise a fixed offset."""
    if isinstance(spec, tuple) and spec[0] == "side":
        rad = look[:2] - ROBOT_XY
        rad = rad / max(np.linalg.norm(rad), 1e-6)
        across = np.array([rad[1], -rad[0]])           # clockwise: operator side at the pick, far side at the place
        return np.array([*(spec[1] * across + spec[2] * rad), spec[3]])
    return spec


def film_camera(cam, t, part, grip, look_prev):
    """Eased moves between the stations; 'part' / 'grip' keys track the moving point. Returns the look-at point."""
    hidden = part[2] < -1.0

    def pt(spec):
        if isinstance(spec, str):
            if spec == "part":
                return look_prev if hidden and look_prev is not None else part + np.array([0, 0, 0.03])
            return grip
        return spec

    k = max(i for i, key in enumerate(CAM_KEYS) if key[0] <= t) if t >= CAM_KEYS[0][0] else 0
    if k == len(CAM_KEYS) - 1 or CAM_KEYS[k + 1][3]:
        look = pt(CAM_KEYS[k][1])
        off = _offset(CAM_KEYS[k][2], look)
    else:
        (ta, la, oa, _), (tb, lb, ob, _) = CAM_KEYS[k], CAM_KEYS[k + 1]
        w = _ease((t - ta) / (tb - ta))
        look = pt(la) * (1 - w) + pt(lb) * w
        oa, ob = _offset(oa, look), _offset(ob, look)
        na, nb = np.linalg.norm(oa), np.linalg.norm(ob)
        dirv = oa / na * (1 - w) + ob / nb * w
        off = dirv / np.linalg.norm(dirv) * (na * (1 - w) + nb * w)   # direction and distance blended separately
    (xa, xb), ya, za = FENCE_IN
    if xa < look[0] < xb and abs(look[1]) < ya:                 # looking into the cell: stay on this side of the fence
        c = look + off
        if c[2] < za:
            c[0], c[1] = min(max(c[0], xa), xb), min(max(c[1], -ya), ya)
            off = c - look
    f = -off / np.linalg.norm(off)
    cam.lookat[:] = look
    cam.distance = float(np.linalg.norm(off))
    cam.azimuth = math.degrees(math.atan2(f[1], f[0]))
    cam.elevation = math.degrees(math.asin(max(-1.0, min(1.0, f[2]))))
    return look


V_RGB, A_RGB, F_RGB = (1.0, 0.8, 0.0, 1), (1.0, 0.42, 0.0, 1), (0.95, 0.05, 0.02, 1)


def _arrow(scn, a, b, width, rgba):
    if scn.ngeom >= scn.maxgeom or np.linalg.norm(b - a) < 1e-4:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).flatten(), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, a, b)
    g.emission = 0.7                                   # full colour regardless of the light
    scn.ngeom += 1


def film_vectors(scn, m, d, dist, grip_point, t, jaw, h=0.01):
    """Velocity (yellow) and acceleration (orange) of the gripped point, central differences of the robot path;
    grip force (red) on both pads while the jaws are closed. Lengths scale with the camera distance so the arrows
    read the same in wide shots and close-ups; the values go to the overlay. Returns (|v| m/s, |a| m/s2, F N)."""
    p0, pm, pp = grip_point(t), grip_point(t - h), grip_point(t + h)
    v = (pp - pm) / (2 * h)
    a = (pp - 2 * p0 + pm) / (h * h)
    w = 0.0035 * dist
    if np.linalg.norm(v) > 0.02:
        _arrow(scn, p0, p0 + v * 0.045 * dist, w, V_RGB)
    if np.linalg.norm(a) > 0.3:
        _arrow(scn, p0, p0 + a * 0.005 * dist, w, A_RGB)
    f = GP.F_GRIP_SET if jaw <= GP.D_GRIP + 0.05 else 0.0
    if f:
        gb = m.body("gripper_frame").id
        x = d.xmat[gb].reshape(3, 3)[:, 0]
        for name, sgn in (("pad_L", -1), ("pad_R", 1)):
            pc = d.geom(name).xpos.copy()
            _arrow(scn, pc + sgn * x * 0.07 * dist, pc + sgn * x * 0.008, w * 1.2, F_RGB)
    return float(np.linalg.norm(v)), float(np.linalg.norm(a)), float(f)


if __name__ == "__main__":
    if "--clean" in sys.argv:                      # explainer film: a tour of the cell following one part, no HUD
        main(FILM_CYCLES, clean=True)
    else:
        main(int(sys.argv[1]) if len(sys.argv) > 1 else 4)
