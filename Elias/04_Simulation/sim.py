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
