"""Robot kinematics and cell layout, shared by the FreeCAD cell (cell_view.FCMacro) and MuJoCo (04_Simulation).

Units mm and degrees. Robot frame = base of the robot, STEP frame of ABB = home pose (all axes 0).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

ROBOTS = {
    # a2: axis 2 (x, z); l2: axis 2 -> 3; c: axis 4 above axis 3; l3: axis 3 -> 5 along x; l_fl: axis 5 -> flange face;
    # limits from the ABB data sheets.
    "IRB1300-7/1.4": dict(files=os.path.join(HERE, "vendor", "irb1300",
                                             "IRB1300 Type A_7&12Kg-1400-IP40IP67CR-STD_OC_00_link{}.STEP"),
                          links=list(range(7)), a2=(150.0, 544.0), l2=575.0, c=40.0, l3=675.0, l_fl=98.5,
                          lim=[(-180, 180), (-95, 155), (-210, 65), (-230, 230), (-130, 130), (-400, 400)],
                          payload=7.0, flange="ISO 9409-1-40-4-M6"),
    "IRB1600-6/1.45": dict(files=os.path.join(HERE, "vendor", "irb1600", "IRB1600_X-145_STEP_rev03_j",
                                              "IRB1600_X-145_STEP_joint", "IRB1600_X-145_IRC5_rev2_{}.STEP"),
                           links=["BASE", "LINK1", "LINK2", "LINK3", "LINK4", "LINK5", "LINK6"], a2=(150.0, 486.5),
                           l2=700.0, c=0.0, l3=600.0, l_fl=65.0,
                           lim=[(-180, 180), (-90, 150), (-245, 65), (-200, 200), (-115, 115), (-400, 400)],
                           payload=6.0, flange="ISO 9409-1-40-4-M6"),
}
ROBOT_NAME = "IRB1300-7/1.4"
ROBOT = ROBOTS[ROBOT_NAME]

# Cell layout (Angabe Abb. 1, placement study sim/aufstellung.py)
BASE = (-100.0, 0.0, 500.0)          # pedestal 500 mm, 100 mm towards the workpiece carrier
X_PICK, H_PICK = 900.0, 1000.0       # buffer belt
X_PLACE, H_PLACE, DY_CHAIN = -900.0, 1300.0, 300.0   # workpiece carrier on two chains
H_FENCE = 2100.0
# Enclosure (Abb. 1): 1200 mm wide (y = -600 ... 600, inner faces), 2100 high, closed by the production line on the
# place side and open towards the buffer belt; panels 40 mm (mesh panels on posts)
Y_FENCE, T_FENCE = 600.0, 40.0
X_FENCE = (X_PLACE - 250.0, X_PICK + 150.0)
CHAIN_W, CARRIER = 120.0, (240.0, 160.0, 30.0)
CHAIN_X = (X_PLACE - 800.0, X_PLACE + CARRIER[0] / 2)   # chains come out of the line and end under the carriers (Abb. 1)
BELT_X, BELT_W = (X_PICK - 150.0, X_PICK + 1250.0), 300.0   # buffer belt along x, pick at its inner end, leaves the cell
PLANT_X = (X_FENCE[0] - 700.0, X_FENCE[0])                  # production line closing the enclosure on the place side

A2_X, A2_Z = ROBOT["a2"]
L2, C4, L3X, L_FL = ROBOT["l2"], ROBOT["c"], ROBOT["l3"], ROBOT["l_fl"]


def joint_chain():
    """(axis, centre) of axes 1-6 in the robot frame at the home pose."""
    z3, z4 = A2_Z + L2, A2_Z + L2 + C4
    return [((0, 0, 1), (0, 0, 0)), ((0, 1, 0), (A2_X, 0, A2_Z)), ((0, 1, 0), (A2_X, 0, z3)),
            ((1, 0, 0), (0, 0, z4)), ((0, 1, 0), (A2_X + L3X, 0, z4)), ((1, 0, 0), (0, 0, z4))]


def flange_home():
    """Flange face centre in the robot frame at the home pose; tool axis along +X."""
    return (A2_X + L3X + L_FL, 0.0, A2_Z + L2 + C4)


def ik_tool_down(wrist_world, a6=0.0):
    """Closed form for a tool pointing straight down (spherical wrist, axis-4 offset C4).
    wrist_world: wrist centre (axis 5) in the cell frame. Returns (q in deg, list of violated axes)."""
    w = [wrist_world[i] - BASE[i] for i in range(3)]
    a1 = math.degrees(math.atan2(w[1], w[0]))
    r = math.hypot(w[0], w[1]) - A2_X
    h = w[2] - A2_Z
    l3 = math.hypot(L3X, C4)
    beta = math.atan2(C4, L3X)
    s3 = (L2 ** 2 + l3 ** 2 - (r * r + h * h)) / (2 * L2 * l3)
    if abs(s3) > 1:
        raise ValueError("target out of reach")
    a3p = math.asin(s3)
    a, b = L2 - l3 * math.sin(a3p), l3 * math.cos(a3p)
    a2 = math.degrees(math.atan2(a * r - b * h, a * h + b * r))
    a3 = math.degrees(a3p + beta)
    q = [a1, a2, a3, 0.0, 90.0 - a2 - a3, a6]
    bad = [i + 1 for i, (v, (lo, hi)) in enumerate(zip(q, ROBOT["lim"])) if not lo <= v <= hi]
    return q, bad
