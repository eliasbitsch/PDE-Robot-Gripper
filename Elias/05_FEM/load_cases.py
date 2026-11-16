"""Load cases for the jaw train FEM (slide + jaw holder + mock jaw, one jaw side).

python load_cases.py      prints the table that goes into the report.

The jaw is a mock until Viktoriia's jaw (V3) exists: it only carries the pad load into the holder at the
right place. All forces act at the pad centre, per jaw; signs in the gripper frame (CAD_PLAN.md):
x = grip direction (positive = towards the part for the left jaw), y = along the robot path, z = up.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402

# Emergency stop deceleration of the IRB 1300 is still open in V2 (Viktoriia); this value is an assumption
# until V2 delivers it. Change it here, everything below follows.
A_STOP = 15.0                                   # m/s^2, assumed (about 1.5 g)

R_PINION = P.PINION_D / 2 / 1000                # m
F_BRAKE = P.T_PINION_BRAKE / (2 * R_PINION)     # N per jaw before the brake slips

M_PART = P.B_MASSE
F_HOLD_Z = M_PART * (P.G + P.A_BAHN) / P.N_BACKEN     # friction each jaw carries: weight + path acceleration
F_STOP_Z = M_PART * (P.G + A_STOP) / P.N_BACKEN
F_STOP_Y = M_PART * A_STOP / P.N_BACKEN

# (name, Fx, Fy, Fz, safety factor required against Rp0.2, what it stands for)
LOAD_CASES = [
    ("LC1 nominal grip", P.F_GRIP_OILY, 0.0, 0.0, 2.0, "required grip force, oily part, mu 0.2, S 2"),
    ("LC2 full current", P.F_JAW_MAX, 0.0, 0.0, 2.0, "motor at full current, end stop or part"),
    ("LC3 transport", P.F_GRIP_OILY, M_PART * P.A_BAHN / P.N_BACKEN, -F_HOLD_Z, 2.0,
     "LC1 plus the part's weight and path acceleration carried by friction"),
    ("LC4 emergency stop", P.F_JAW_MAX, F_STOP_Y, -F_STOP_Z, 1.5,
     "LC2 plus the part decelerating at A_STOP (assumed, V2)"),
    ("LC5 brake limit", F_BRAKE, 0.0, 0.0, 1.2,
     "power off, jaw pushed until the brake slips (misuse / collision)"),
]

# Materials (EN 755-2 / EN 573, minimum values)
MATERIALS = {
    "EN AW-6082 T6": {"E": 70000.0, "nu": 0.33, "Rp02": 250.0, "fatigue": 90.0},   # MPa; fatigue: R = -1, 1e7 cycles
    "EN AW-7075 T6": {"E": 71700.0, "nu": 0.33, "Rp02": 460.0, "fatigue": 150.0},
}
PARTS = {"slide": "EN AW-6082 T6", "holder": "EN AW-6082 T6", "jaw (mock)": "EN AW-7075 T6"}

# Boundary conditions
BC = [
    "slide: fixed on its contact face with the ball carriage (carriage taken as rigid, steel vs aluminium)",
    "slide - holder and holder - jaw: bonded (screws and dowels preloaded, no gap opening checked here)",
    "load: uniform traction on the pad area of the mock jaw, resultant at the pad centre",
]

# Acceptance criteria
DEFLECTION_MAX = 0.05            # mm at the pad centre for LC2: pads stay parallel within about 0.1 deg
CYCLES_PER_YEAR = 3.0e6          # 5 s cycle, 2 shifts; LC1 is the pulsating load (0 -> F)


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
    print(f"\ndeflection at the pad centre, LC2: <= {DEFLECTION_MAX} mm")
    print(f"brake limit per jaw: {F_BRAKE:.0f} N, emergency stop assumed {A_STOP} m/s^2 (update from V2)")
