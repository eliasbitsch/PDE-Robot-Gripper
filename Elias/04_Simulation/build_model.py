"""MuJoCo model of the cell: ABB IRB 1300 on its pedestal, the gripper on the ISO flange, part B on the buffer belt.

python build_model.py  ->  out/cell.xml (+ decimated robot meshes in meshes/)
Inputs: 06_Cell/cell_params.py (layout, kinematics), 02_CAD_Body/out (gripper STL + kinematics.json from build.py),
meshes/ (robot links and part B from export_meshes.py). Units in the XML: m, kg, deg.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "02_CAD_Body")
sys.path.insert(0, os.path.join(HERE, "..", "06_Cell"))
sys.path.insert(0, CAD)
import cell_params as CP  # noqa: E402
import gripper_params as GP  # noqa: E402
import drive as DRV  # noqa: E402

OUT = os.path.join(HERE, "out")
MESH = os.path.join(HERE, "meshes")
GSTL = os.path.join(CAD, "out", "stl")
KIN = json.load(open(os.path.join(CAD, "out", "kinematics.json")))
MM = 0.001

# Pad shear spring: G (NBR 70 Shore A) ~ 1.5 MPa * 737 mm2 / 3 mm = 3.7e5 N/m per direction, taken 10 x softer
# with armature so it stays stable at the 0.5 ms step; only the equal load sharing matters, not the micrometres.
K_PAD_SHEAR, M_PAD_ARM = 3.7e4, 0.05
C_PAD_SHEAR = 2 * (K_PAD_SHEAR * M_PAD_ARM) ** 0.5
MU_PAD = GP.MU_OILY                       # NBR pad on oily PA 6, the value V2 has to justify
GRIPPER_MASS, SLIDE_MASS = 2.38, 0.27     # kg, from the build (2.915 total incl. the moving groups, rail size 12)
GRIPPER_COM = (0.0, -4.2, 76.0 - 66.0)    # mm in the gripper frame (build: 66 mm below the flange face z = 76)
ROBOT_LINK_MASS = [18.0, 9.0, 6.0, 3.0, 2.5, 0.8, 0.1]   # kg, rough split of the 65 kg arm; gravity compensated anyway
COLORS = {"EN AW-6082": "0.72 0.75 0.78 1", "EN AW-7075": "0.76 0.25 0.05 1", "steel": "0.45 0.48 0.50 1",
          "vendor": "0.16 0.17 0.18 1", "NBR": "0.08 0.08 0.08 1"}


def decimate(src, dst, faces=15000):
    """Robot STEP meshes have up to 500k triangles; MuJoCo only needs a light visual mesh."""
    if os.path.exists(dst) and os.path.getmtime(dst) > os.path.getmtime(src):
        return
    import pyvista as pv
    m = pv.read(src).triangulate().clean()
    if m.n_cells > faces:
        m = m.decimate(1.0 - faces / m.n_cells)
    m.save(dst)


def v(x, s=1.0):
    return " ".join(f"{s * c:.6g}" for c in x)


def flange_face_z():
    import pyvista as pv
    return pv.read(os.path.join(GSTL, "Adapter_ISO9409.stl")).bounds[5] - GP.ISO_SPIGOT_H


def box_of(name):
    import pyvista as pv
    b = pv.read(os.path.join(GSTL, name + ".stl")).bounds
    return np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2]), np.array([b[1] - b[0], b[3] - b[2], b[5] - b[4]]) / 2


def build():
    os.makedirs(OUT, exist_ok=True)
    assets, meshes_used = [], set()
    for i in range(7):
        decimate(os.path.join(MESH, f"robot_link{i}.stl"), os.path.join(MESH, f"robot_link{i}_s.stl"))
        assets.append(f'<mesh name="link{i}" file="{os.path.join(MESH, f"robot_link{i}_s.stl")}" scale="{MM} {MM} {MM}"/>')
    assets.append(f'<mesh name="part_b" file="{os.path.join(MESH, "part_b.stl")}" scale="{MM} {MM} {MM}"/>')
    groups = {0: [], -1: [], 1: []}
    for name, mat in KIN["material"].items():
        mov = KIN["moving"].get(name, 0)
        groups[mov if mov in (-1, 1) else 0].append((name, mat))
        assets.append(f'<mesh name="g_{name}" file="{os.path.join(GSTL, name + ".stl")}" scale="{MM} {MM} {MM}"/>')

    def visual(name, mat):
        return f'<geom type="mesh" mesh="g_{name}" rgba="{COLORS.get(mat, "0.6 0.6 0.6 1")}" contype="0" conaffinity="0" group="1"/>'

    # robot chain: body i sits at its joint centre (home pose), mesh vertices are in the robot frame
    chain = CP.joint_chain()
    centres = [np.zeros(3)] + [np.array(c, float) for _, c in chain]
    fz = flange_face_z()
    flange = np.array(CP.flange_home())
    # gripper frame on the flange: gripper +Z points back to the robot (robot -X at home), flange face at z = fz
    grip_body_pos = (flange - centres[6]) * MM
    pad_geoms, slide_xml = [], {}
    for side in (-1, 1):
        tag = "L" if side == -1 else "R"
        body = [visual(n, m) for n, m in groups[side]]
        c, h = box_of(f"Pad_{tag}")
        # NBR layer: the pad hangs on two tangential springs (y, z), so a rigid part loads both pads alike, as the
        # rubber shear does in reality; contacts alone have no tangential memory and split the load arbitrarily
        spring = f'stiffness="{K_PAD_SHEAR:.0f}" damping="{C_PAD_SHEAR:.0f}" armature="{M_PAD_ARM}"'
        body.append(f'<body name="padbody_{tag}" pos="{v(c, MM)}">'
                    f'<inertial pos="0 0 0" mass="0.005" diaginertia="1e-6 1e-6 1e-6"/>'
                    f'<joint name="pad_{tag}_y" type="slide" axis="0 1 0" {spring}/>'
                    f'<joint name="pad_{tag}_z" type="slide" axis="0 0 1" {spring}/>'
                    f'<geom name="pad_{tag}" type="box" size="{v(h, MM)}" rgba="0.1 0.1 0.1 1" contype="2" '
                    f'conaffinity="0" group="2"/></body>')
        pad_geoms.append(f"pad_{tag}")
        slide_xml[side] = f'''
            <body name="slide_{tag}">
              <inertial pos="{v(c, MM)}" mass="{SLIDE_MASS}" diaginertia="2e-4 2e-4 2e-4"/>
              <joint name="jaw_{tag}" type="slide" axis="{side} 0 0" range="-0.001 {(GP.D_OPEN + 4) * MM:.4f}" damping="20" armature="{DRV.ARMATURE_PER_JAW:.4f}"/>  <!-- rotor, gearbox, brake seen at the jaw -->
              {chr(10).join("              " + b for b in body)}
            </body>'''
    static = "\n".join("              " + visual(n, m) for n, m in groups[0])
    gripper = f'''
          <body name="gripper" pos="{v(grip_body_pos)}" euler="0 -90 0">
            <body name="gripper_frame" pos="0 0 {-fz * MM:.6g}">
              <inertial pos="{v(GRIPPER_COM, MM)}" mass="{GRIPPER_MASS}" diaginertia="0.012 0.010 0.008"/>
              <site name="tcp" pos="0 0 {sum(GP.PAD_Z) / 2 * MM:.6g}" size="0.004"/>
{static}
{slide_xml[-1]}
{slide_xml[1]}
            </body>
          </body>'''

    body_xml = gripper
    for i in range(6, 0, -1):
        ax, _ = chain[i - 1]
        lo, hi = CP.ROBOT["lim"][i - 1]
        body_xml = f'''
      <body name="link{i}" pos="{v(centres[i] - centres[i - 1], MM)}" gravcomp="1">
        <inertial pos="0 0 0" mass="{ROBOT_LINK_MASS[i]}" diaginertia="0.05 0.05 0.05"/>
        <joint name="A{i}" axis="{v(ax)}" range="{lo} {hi}" armature="0.5" damping="50"/>
        <geom type="mesh" mesh="link{i}" pos="{v(-centres[i], MM)}" rgba="0.93 0.95 0.96 1" contype="4" conaffinity="0" group="1"/>
        {body_xml}
      </body>'''

    part_c, part_h = np.array([-12.5, 0.0, 0.0]), np.array([47.5, 50.0, 40.0])   # STEP bounding box of part B, mm
    part_z = CP.H_PICK + part_h[2]                     # resting on the belt
    gx = CP.X_PICK
    # part B in the gripper's grip direction: gripper X maps to cell Y at the pick pose (axis 1 = 0, tool down)
    xml = f'''<mujoco model="cell_irb1300_gripper">
  <compiler angle="degree" inertiafromgeom="false" autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" noslip_iterations="10" gravity="0 0 -9.81"/>
  <visual><global offwidth="1600" offheight="1000"/><quality shadowsize="4096"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.75 0.80 0.86" rgb2="0.96 0.97 0.98" width="256" height="256"/>
    {chr(10).join("    " + a for a in assets)}
  </asset>
  <worldbody>
    <light pos="1.5 -2 4" dir="-0.3 0.4 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="4 4 0.1" rgba="0.85 0.86 0.86 1" contype="1" conaffinity="1"/>
    <geom name="pedestal" type="cylinder" pos="{CP.BASE[0] * MM} 0 {CP.BASE[2] / 2 * MM}" size="0.22 {CP.BASE[2] / 2 * MM}" rgba="0.3 0.32 0.35 1" contype="1" conaffinity="1"/>
    <geom name="belt" type="box" pos="{CP.X_PICK * MM} 0 {(CP.H_PICK - 20) * MM}" size="0.15 0.7 0.02" rgba="0.15 0.15 0.15 1" contype="1" conaffinity="1" friction="0.6"/>
    <geom name="fence_left" type="box" pos="{(CP.X_FENCE[0] + CP.X_FENCE[1]) / 2 * MM} {(CP.Y_FENCE + CP.T_FENCE / 2) * MM} {CP.H_FENCE / 2 * MM}" size="{(CP.X_FENCE[1] - CP.X_FENCE[0]) / 2 * MM} {CP.T_FENCE / 2 * MM} {CP.H_FENCE / 2 * MM}" rgba="0.95 0.8 0.1 0.25" contype="0" conaffinity="0"/>
    <geom name="fence_right" type="box" pos="{(CP.X_FENCE[0] + CP.X_FENCE[1]) / 2 * MM} {-(CP.Y_FENCE + CP.T_FENCE / 2) * MM} {CP.H_FENCE / 2 * MM}" size="{(CP.X_FENCE[1] - CP.X_FENCE[0]) / 2 * MM} {CP.T_FENCE / 2 * MM} {CP.H_FENCE / 2 * MM}" rgba="0.95 0.8 0.1 0.25" contype="0" conaffinity="0"/>
    <geom name="chain_1" type="box" pos="{(CP.CHAIN_X[0] + CP.CHAIN_X[1]) / 2 * MM} {-CP.DY_CHAIN * MM} {(CP.H_PLACE - 45) * MM}" size="{(CP.CHAIN_X[1] - CP.CHAIN_X[0]) / 2 * MM} {CP.CHAIN_W / 2 * MM} 0.015" rgba="0.2 0.2 0.22 1" contype="0" conaffinity="0"/>
    <geom name="chain_2" type="box" pos="{(CP.CHAIN_X[0] + CP.CHAIN_X[1]) / 2 * MM} {CP.DY_CHAIN * MM} {(CP.H_PLACE - 45) * MM}" size="{(CP.CHAIN_X[1] - CP.CHAIN_X[0]) / 2 * MM} {CP.CHAIN_W / 2 * MM} 0.015" rgba="0.2 0.2 0.22 1" contype="0" conaffinity="0"/>
    <geom name="carrier_1" type="box" pos="{CP.X_PLACE * MM} {-CP.DY_CHAIN * MM} {(CP.H_PLACE - 15) * MM}" size="0.12 0.08 0.015" rgba="0.25 0.45 0.7 1" contype="0" conaffinity="0"/>
    <geom name="carrier" type="box" pos="{CP.X_PLACE * MM} {CP.DY_CHAIN * MM} {(CP.H_PLACE - 15) * MM}" size="0.12 0.08 0.015" rgba="0.25 0.45 0.7 1" contype="1" conaffinity="1" friction="0.6"/>
    <body name="robot" pos="{v(CP.BASE, MM)}">
      <geom type="mesh" mesh="link0" rgba="0.93 0.95 0.96 1" contype="4" conaffinity="0" group="1"/>
      {body_xml}
    </body>
    <body name="part" pos="{gx * MM} 0 {part_z * MM}">                <!-- placed from the gripper pose by sim.py -->
      <freejoint name="part"/>
      <inertial pos="{v(part_c, MM)}" mass="{GP.B_MASSE}" diaginertia="{GP.B_MASSE * (0.1 ** 2 + 0.08 ** 2) / 12:.6g} {GP.B_MASSE * (0.095 ** 2 + 0.08 ** 2) / 12:.6g} {GP.B_MASSE * (0.095 ** 2 + 0.1 ** 2) / 12:.6g}"/>
      <geom type="mesh" mesh="part_b" rgba="0.93 0.9 0.8 1" contype="0" conaffinity="0" group="1"/>
      <!-- contact shape: the convex hull would span the lug and touch the left pad, so two boxes (GP part comment):
           body x -35..35 with the flat grip faces, lug x -60..-35 up to z -10 -->
      <geom name="part" type="box" pos="0 0 0" size="0.035 0.05 0.04" contype="1" conaffinity="1" friction="0.6" group="3"/>
      <geom name="part_lug" type="box" pos="-0.0475 0 -0.025" size="0.0125 0.05 0.015" contype="1" conaffinity="1" friction="0.6" group="3"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="pad_L" geom2="part" friction="{MU_PAD} {MU_PAD} 0.005 0.0001 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0005"/>
    <pair geom1="pad_R" geom2="part" friction="{MU_PAD} {MU_PAD} 0.005 0.0001 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0005"/>
  </contact>
  <!-- pinion + two racks: the drive acts on both jaws equally (tendon, 0.5 each), the stiff equality keeps them in step -->
  <tendon><fixed name="jaws"><joint joint="jaw_L" coef="0.5"/><joint joint="jaw_R" coef="0.5"/></fixed></tendon>
  <equality><joint joint1="jaw_R" joint2="jaw_L" polycoef="0 1 0 0 0" solref="0.0005 1" solimp="0.99 0.999 0.0001"/></equality>
  <actuator>
    {chr(10).join(f'    <position name="A{i}" joint="A{i}" kp="20000" kv="400" ctrlrange="{np.radians(CP.ROBOT["lim"][i - 1][0]):.4f} {np.radians(CP.ROBOT["lim"][i - 1][1]):.4f}"/>' for i in range(1, 7))}
    <position name="jaw" tendon="jaws" kp="200000" kv="500" forcerange="-{2 * GP.F_GRIP_SET:.3f} {2 * GP.F_GRIP_SET:.3f}"/>   <!-- drives both jaws: 2 x force per jaw -->
  </actuator>
  <sensor>
    <framepos name="part_pos" objtype="body" objname="part"/>
    <framepos name="tcp_pos" objtype="site" objname="tcp"/>
    <framelinacc name="tcp_acc" objtype="site" objname="tcp"/>
    <actuatorfrc name="jaw_force" actuator="jaw"/>
  </sensor>
</mujoco>
'''
    path = os.path.join(OUT, "cell.xml")
    open(path, "w").write(xml)
    return path


if __name__ == "__main__":
    p = build()
    import mujoco
    m = mujoco.MjModel.from_xml_path(p)
    print(p, f"{m.nbody} bodies, {m.njnt} joints, {m.ngeom} geoms, {m.nmesh} meshes, {m.nu} actuators")
