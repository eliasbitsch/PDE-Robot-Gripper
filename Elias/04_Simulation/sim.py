"""Helpers for the cell model: poses from the closed-form IK, part placement, rendering.

python sim.py   ->  out/pose_pick.png, out/pose_place.png (static check of the flange and the part position)
"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_model as B  # noqa: E402

CP, GP = B.CP, B.GP
MM = 0.001
PART_OFFSET = np.array(GP.PART_OFFSET) * MM          # part B STEP frame in the gripper frame (gripped position)


def load():
    m = mujoco.MjModel.from_xml_path(os.path.join(B.OUT, "cell.xml"))
    return m, mujoco.MjData(m)


def wrist_above_part_bottom():
    """Wrist centre (axis 5) above the part underside with the tool pointing down, mm."""
    part_bottom = GP.PART_OFFSET[2] - 40.0                   # part B STEP z -40 ... 40
    return (B.flange_face_z() - part_bottom) + CP.L_FL


def q_pick(lift=0.0):
    q, bad = CP.ik_tool_down((CP.X_PICK, 0.0, CP.H_PICK + wrist_above_part_bottom() + lift))
    return np.radians(q), bad


def q_place(lift=0.0):
    q, bad = CP.ik_tool_down((CP.X_PLACE, CP.DY_CHAIN, CP.H_PLACE + wrist_above_part_bottom() + lift))
    return np.radians(q), bad


# Transfer through the 1200 mm wide enclosure: a plain joint swing pick -> place takes the forearm 318 mm through
# the fence (clearance.py). One via in the middle of the swing, where the arm points at the fence: wrist pulled in
# to VIA_R at VIA_Z, axis 1 at VIA_A1. Axis 1 keeps turning through it, the arm pulls in and reaches out again on
# the way (via search 21 Nov: fence >= 96 mm, axis margin >= 20 deg, loaded 1.52 s). Two retracted vias with a stop
# at each cost 2.5 s, a cubic spline through them overshot axis 5 to 7 deg from its limit.
VIA_R, VIA_Z, VIA_A1 = 350.0, 1700.0, 90.0


def transfer_vias(lift=60.0):
    """Joint waypoints pick (lifted) -> retracted mid-swing -> place (lifted), rad."""
    import math
    q1, _ = q_pick(lift)
    q2, _ = q_place(lift)
    a = math.radians(VIA_A1)
    q, bad = CP.ik_tool_down((CP.BASE[0] + VIA_R * math.cos(a), CP.BASE[1] + VIA_R * math.sin(a), VIA_Z))
    assert not bad, bad
    return [q1, np.radians(q), q2]


def transfer_path(lift=60.0):
    """q(u), u in 0 ... 1: shape-preserving (PCHIP) interpolation through the vias, knots spaced by the time each
    segment needs at the axis speeds; no overshoot of any axis between the waypoints."""
    from scipy.interpolate import PchipInterpolator
    qs = np.array(transfer_vias(lift))
    vmax = np.radians([249.0, 180.0, 247.0, 500.0, 415.0, 720.0])
    seg = [np.max(np.abs(b - a) / vmax) for a, b in zip(qs[:-1], qs[1:])]
    u = np.concatenate([[0.0], np.cumsum(seg)]) / sum(seg)
    return PchipInterpolator(u, qs)


class TimeLaw:
    """Fastest u(t) along a joint path q(u) with the TCP acceleration (tangential + centripetal) <= a_lim and every
    axis below its speed: forward / backward pass on the squared path speed over the TCP arc length.
    __call__(t) -> (u, du/dt), like trapezoid() for a straight segment."""

    def __init__(self, path, a_lim, n=800, m=None, d=None, vmax_deg=(249, 180, 247, 500, 415, 720)):
        if m is None:
            m, d = load()
        u = np.linspace(0.0, 1.0, n)
        site = m.site("tcp").id
        p = np.zeros((n, 3))
        for i, ui in enumerate(u):
            for k in range(6):
                d.qpos[m.joint(f"A{k + 1}").qposadr[0]] = path(ui)[k]
            mujoco.mj_kinematics(m, d)
            p[i] = d.site_xpos[site]
        ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
        sig = np.concatenate([[0.0], np.cumsum(ds)])
        dsig = np.gradient(sig, u)
        dp = np.gradient(p, sig, axis=0)
        kappa = np.linalg.norm(np.gradient(dp, sig, axis=0), axis=1)
        dq_dsig = np.abs(path(u, 1)) / np.maximum(dsig, 1e-9)[:, None]
        v2 = np.minimum(a_lim / np.maximum(kappa, 1e-9), np.min((np.radians(vmax_deg) / np.maximum(dq_dsig, 1e-12)) ** 2, axis=1))
        v2[0] = v2[-1] = 0.0
        at = lambda i, w: np.sqrt(max(a_lim ** 2 - (kappa[i] * w) ** 2, 0.0))
        for i in range(n - 1):
            v2[i + 1] = min(v2[i + 1], v2[i] + 2 * at(i, v2[i]) * ds[i])
        for i in range(n - 2, -1, -1):
            v2[i] = min(v2[i], v2[i + 1] + 2 * at(i + 1, v2[i + 1]) * ds[i])
        v = np.sqrt(np.maximum(v2, 0.0))
        dt = 2 * ds / np.maximum(v[1:] + v[:-1], 1e-9)
        self.t = np.concatenate([[0.0], np.cumsum(dt)])
        self.T = float(self.t[-1])
        self.u, self.du = u, v / np.maximum(dsig, 1e-9)
        self.v_tcp_max, self.length = float(v.max()), float(sig[-1])

    def __call__(self, t):
        t = min(max(t, 0.0), self.T)
        return float(np.interp(t, self.t, self.u)), float(np.interp(t, self.t, self.du))


def set_robot(m, d, q, jaw=GP.D_OPEN * MM):
    for i in range(6):
        d.qpos[m.joint(f"A{i + 1}").qposadr[0]] = q[i]
        d.ctrl[m.actuator(f"A{i + 1}").id] = q[i]
    for tag in ("L", "R"):
        d.qpos[m.joint(f"jaw_{tag}").qposadr[0]] = jaw
    d.ctrl[m.actuator("jaw").id] = jaw
    mujoco.mj_forward(m, d)


def place_part_in_gripper(m, d):
    """Part B at its gripped position, taken from the gripper frame (independent of the tool's turn about its axis)."""
    b = m.body("gripper_frame").id
    R = d.xmat[b].reshape(3, 3)
    pos = d.xpos[b] + R @ PART_OFFSET
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.flatten())
    adr = m.joint("part").qposadr[0]
    d.qpos[adr:adr + 3] = pos
    d.qpos[adr + 3:adr + 7] = quat
    d.qvel[m.joint("part").dofadr[0]:m.joint("part").dofadr[0] + 6] = 0
    mujoco.mj_forward(m, d)


def render(m, d, path, lookat=None, distance=1.0, azimuth=135, elevation=-20, size=(1200, 800)):
    r = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat if lookat is not None else d.site("tcp").xpos
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    r.update_scene(d, cam)
    from PIL import Image
    Image.fromarray(r.render()).save(path)
    r.close()
    return path


if __name__ == "__main__":
    m, d = load()
    for name, fn in (("pick", q_pick), ("place", q_place)):
        q, bad = fn()
        set_robot(m, d, q)
        place_part_in_gripper(m, d)
        tcp = d.site("tcp").xpos / MM
        print(f"{name}: A1..A6 {np.round(np.degrees(q), 1)}, limits violated: {bad or 'none'}, tcp {np.round(tcp)} mm, "
              f"part bottom z {(d.body('part').xpos[2] / MM) - 40:.0f} mm")
        render(m, d, os.path.join(B.OUT, f"pose_{name}.png"), distance=0.9)
        render(m, d, os.path.join(B.OUT, f"pose_{name}_cell.png"), lookat=np.array([0.0, 0.0, 1.0]), distance=3.6,
               azimuth=120, elevation=-22)
