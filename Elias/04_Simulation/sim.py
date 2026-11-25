"""Helpers for the cell model: poses from the closed-form IK, part placement, rendering.

python sim.py   ->  out/pose_pick.png, out/pose_place.png (static check of the flange and the part position)
"""
import math
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


def q_place(lift=0.0, side=+1, square=True):
    """Place pose on chain 2 (side +1, y = +DY_CHAIN) or chain 1 (side -1). square: A6 turns the tool so part B lands
    square on the carrier (yaw 180 deg, as on the belt turned by the swing); with A6 = 0 it would land skewed by
    180 deg - |A1| (20.6 deg)."""
    w = (CP.X_PLACE, side * CP.DY_CHAIN, CP.H_PLACE + wrist_above_part_bottom() + lift)
    q, bad = CP.ik_tool_down(w)
    if square:
        a6 = q[0] - math.copysign(180.0, q[0])
        q, bad = CP.ik_tool_down(w, a6=a6)
    return np.radians(q), bad


# Transfer through the 1200 mm wide enclosure: a plain joint swing pick -> place takes the forearm 318 mm through
# the fence (clearance.py). One via mid-swing, where the arm points at the fence: wrist pulled in to VIA_R at VIA_Z,
# axis 1 at VIA_A1, passed in a corner path of VIA_ZONE mm (zVIA in RAPID) without a stop. The path between the
# targets is what the ABB controller runs (abb_path.py, review 1.5): straight MoveJ lines in joint space. The old
# via (R 350, Z 1700) only cleared the fence on a PCHIP curve; on the real MoveJ line it cut 4.9 mm into it. Via
# search 21 Nov on the MoveJ path (A1 75 ... 120, R 200 ... 500, Z 1250 ... 1800, zone 100 / 200): fence >= 101 mm
# (100.2 mm with the 200 mm body since review 1.10)
# on both chains, axis margin >= 16 deg.
VIA_R, VIA_Z, VIA_A1, VIA_ZONE = 200.0, 1450.0, 90.0, 200.0


def transfer_vias(lift=60.0, side=+1):
    """Joint targets pick (lifted) -> retracted mid-swing -> place (lifted), rad; chain 1 (side -1) is the mirror
    image (via at A1 = -VIA_A1)."""
    q1, _ = q_pick(lift)
    q2, _ = q_place(lift, side)
    a = math.radians(side * VIA_A1)
    q, bad = CP.ik_tool_down((CP.BASE[0] + VIA_R * math.cos(a), CP.BASE[1] + VIA_R * math.sin(a), VIA_Z))
    assert not bad, bad
    return [q1, np.radians(q), q2]


def tcp_fn(m=None, d=None):
    if m is None:
        m, d = load()
    site, adr = m.site("tcp").id, [m.joint(f"A{i + 1}").qposadr[0] for i in range(6)]

    def tcp(q):
        d.qpos[adr] = q
        mujoco.mj_kinematics(m, d)
        return d.site_xpos[site].copy()
    return tcp


def transfer_path(lift=60.0, side=+1, zone=VIA_ZONE):
    """q(u), u in 0 ... 1: MoveJ pick -> via (corner path, zone mm) -> place, as the ABB controller interpolates."""
    import abb_path
    return abb_path.MoveJPath(transfer_vias(lift, side), [zone], tcp_fn())


class TimeLaw:
    """Fastest u(t) along a joint path q(u) with the TCP acceleration (tangential + centripetal) <= a_lim and every
    axis below its speed: forward / backward pass on the squared path speed over the TCP arc length.
    __call__(t) -> (u, du/dt), like trapezoid() for a straight segment."""

    def __init__(self, path, a_lim, n=800, m=None, d=None, vmax_deg=(249, 180, 247, 500, 415, 720), v_lim=None):
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
        if v_lim is not None:                                       # programmed TCP speed (RAPID speeddata)
            v2 = np.minimum(v2, v_lim ** 2)
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
        self.sig, self.v, self.dsig = sig, v, dsig
        self.acc = np.diff(v) / np.maximum(dt, 1e-12)               # constant tangential acceleration per step
        self.v_tcp_max, self.length = float(v.max()), float(sig[-1])

    def __call__(self, t):
        # constant acceleration inside each step: interpolating u(t) linearly made the speed jump where steps are
        # long (up to 40 ms at the start), a spurious 9 m/s2 in the replay (review 1.5)
        t = min(max(t, 0.0), self.T)
        i = min(int(np.searchsorted(self.t, t, side="right")) - 1, len(self.acc) - 1)
        tau = t - self.t[i]
        s = self.sig[i] + self.v[i] * tau + 0.5 * self.acc[i] * tau * tau
        u = float(np.interp(s, self.sig, self.u))
        return u, float((self.v[i] + self.acc[i] * tau) / max(np.interp(u, self.u, self.dsig), 1e-9))


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
