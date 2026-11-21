"""Load cases for the jaw train FEM (slide + jaw holder + mock jaw, one jaw side).

python load_cases.py      prints the table that goes into the report.

The jaw is a mock until Viktoriia's jaw (V3) exists: it only carries the pad load into the holder at the
right place. All forces act at the pad centre, per jaw; signs in the gripper frame (CAD_PLAN.md):
x = grip direction (positive = towards the part for the left jaw), y = along the robot path, z = up.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402

# Emergency stop, ABB product specification IRB 1300 (3HAC070393 rev. M, p. 91, IRB 1300 1.4 m 7 kg):
# category 0 at max speed, arm stretched, max load: stopping angle and time per axis. Constant deceleration:
# alpha = 2 phi / t^2. Axis 1 dominates our transfer; the TCP is at most about 1.0 m from axis 1 (at the pick).
STOP_CAT0 = {1: (55.25, 0.40), 2: (33.39, 0.37), 3: (35.66, 0.27)}       # axis: (deg, s)
ALPHA_STOP = {k: 2 * math.radians(phi) / t ** 2 for k, (phi, t) in STOP_CAT0.items()}   # rad/s^2
R_TCP_A1 = 1.0                                  # m, TCP radius about axis 1 at the pick (largest in the cycle)
A_STOP = ALPHA_STOP[1] * R_TCP_A1               # m/s^2, about 12

R_PINION = P.PINION_D / 2 / 1000                # m
F_BRAKE = P.T_PINION_BRAKE / (2 * R_PINION)     # N per jaw before the brake slips

M_PART = P.B_MASSE
F_HOLD_Z = M_PART * (P.G + P.A_PATH_REAL) / P.N_BACKEN     # friction each jaw carries: weight + real path acceleration
# Strength in the e-stop: ABB gives 82 m/s2 (incl. gravity) as the maximum Cartesian design acceleration in an
# emergency stop at nominal load (spec p. 54) - the value for tool strength (review 21 Nov). The part's inertia,
# m * 82 / 2 per jaw, is put on each pad as a shear load at 45 deg between path direction and vertical; whether the
# part is still held at that level is e_stop.py's subject, not the strength check's.
A_STOP_DESIGN = 82.0
F_STOP_SHEAR = M_PART * A_STOP_DESIGN / P.N_BACKEN
F_STOP_Y = F_STOP_SHEAR / math.sqrt(2)
F_STOP_Z = F_STOP_SHEAR / math.sqrt(2)

# (name, Fx, Fy, Fz, safety factor required against Rp0.2, what it stands for)
LOAD_CASES = [
    ("LC1 setpoint grip", P.F_GRIP_SET, 0.0, 0.0, 2.0, "operating grip force (real path, mu 0.2, S 2)"),
    ("LC2 full current", P.F_JAW_MAX, 0.0, 0.0, 2.0, "motor at full current (current limit set wrong), end stop or part"),
    ("LC3 transport", P.F_GRIP_SET, M_PART * P.A_PATH_REAL / P.N_BACKEN, -F_HOLD_Z, 2.0,
     "LC1 plus the part's weight and path acceleration carried by friction"),
    ("LC4 emergency stop", P.F_JAW_MAX, F_STOP_Y, -F_STOP_Z, 1.5,
     "LC2 plus the part's inertia at 82 m/s2 (ABB e-stop design acceleration, spec p. 54)"),
    ("LC5 brake limit", F_BRAKE, 0.0, 0.0, 1.2,
     "power off, jaw pushed until the brake slips (misuse / collision)"),
]

# Materials (EN 755-2 / EN 573, minimum values)
MATERIALS = {
    "EN AW-6082 T6": {"E": 70000.0, "nu": 0.33, "Rp02": 250.0, "fatigue": 90.0},   # MPa; fatigue: R = -1, 1e7 cycles
    "EN AW-7075 T6": {"E": 71700.0, "nu": 0.33, "Rp02": 460.0, "fatigue": 150.0},
}
PARTS = {"slide": "EN AW-6082 T6", "holder": "EN AW-6082 T6", "jaw": "EN AW-7075 T6"}

# Boundary conditions
BC = [
    "slide: fixed on its contact face with the ball carriage (carriage taken as rigid, steel vs aluminium)",
    "slide - holder and holder - jaw: bonded (screws and dowels preloaded, no gap opening checked here)",
    "load: uniform traction on the pad area of the mock jaw, resultant at the pad centre",
]

# Acceptance criteria
TILT_MAX = 0.1                   # deg, former fixed limit, reported only; the criterion is the grip with the tilt (tilt_check.py)
DEFLECTION_MAX = 0.05            # mm at the pad centre for LC2, reported only: most of it is the slide bending
CYCLES_PER_YEAR = 3.0e6          # 5 s cycle, 2 shifts; LC1 (setpoint) is the pulsating load (0 -> F)


def table():
    rows = ["| Load case | Fx [N] | Fy [N] | Fz [N] | S req. | Meaning |", "|---|---|---|---|---|---|"]
    for name, fx, fy, fz, s, what in LOAD_CASES:
        rows.append(f"| {name} | {fx:.1f} | {fy:.1f} | {fz:.1f} | {s:.1f} | {what} |")
    return "\n".join(rows)


if __name__ == "__main__":
    print(table())
    print()
    for part, mat in PARTS.items():
        m = MATERIALS[mat]
        print(f"{part:11s} {mat}: allowed von Mises {m['Rp02'] / 2:.0f} MPa (S 2), "
              f"fatigue amplitude for LC1 <= {m['fatigue'] / 2:.0f} MPa")
    print(f"\npad tilt: judged by tilt_check.py (grip with the pressure wedge); former limit {TILT_MAX} deg for reference")
    print(f"brake limit per jaw: {F_BRAKE:.0f} N, emergency stop {A_STOP:.1f} m/s^2 (ABB category 0, axis 1 at {R_TCP_A1} m)")
