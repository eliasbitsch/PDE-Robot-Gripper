"""Control cabinet parts for the gripper (D11, CAD_PLAN.md): the gripper BOM covers only what sits on the robot.

python -m parts.cabinet  ->  out/bom_cabinet.csv (also written by build.py)
Load on the 24 V rail: controller + motor ~1 A, 2 A peak while closing, brake 0.33 A (8 W, BKE-0,4-5,0).
"""
import csv
import os

# (designation, standard / supplier, qty, why)
CABINET_BOM = [
    ("Closed-loop stepper controller, 12-58 V, 3 A / 6 A peak, encoder input, brake output, 4 DI / 2 DO, NanoJ",
     "Nanotec CL4-E-1-12", 1, "drives motor, encoder and brake; current limit = grip force (setpoint 44.5 N per jaw, calibrated at commissioning)"),
    ("Power supply 24 V DC, 5 A (120 W), DIN rail", "Mean Well SDR-120-24", 1,
     "about 2.5x the peak load, recharges the buffer"),
    ("Buffer module 24 V, >= 200 ms at 20 A (> 1 s at the 2 A of the gripper), capacitive", "PULS UF20.241", 1,
     "keeps the drive powered on power loss until the brake has closed (04_Simulation/power_fail.py)"),
    ("Circuit breaker 1-pole, C6 A, 230 V AC feed of the power supply", "e.g. ABB S201-C6", 1, "line protection"),
    ("Fuse terminal 24 V with 4 A slow-blow fuse, controller supply", "e.g. Phoenix Contact UT 4-HESI", 1,
     "protects the hybrid cable and the controller"),
    ("Hybrid robot cable, shielded, torsion-rated: motor 4 x 0.5 + brake 2 x 0.5 + encoder 6 x 0.14 mm2, "
     "OD 8-9 mm, 5 m through the dress pack", "robot-rated hybrid cable, e.g. igus chainflex (type to be selected)", 1,
     "fits the SKINTOP M16 on the gripper (clamping range 4.5-10 mm)"),
    ("Control cable 4 x 0.25 mm2, shielded, 3 m: OmniCore DO close / open -> CL4-E, CL4-E DO gripped / fault -> OmniCore",
     "standard", 1, "24 V I/O handshake, no fieldbus option on the robot needed"),
    ("DIN rail terminal blocks 2.5 mm2 (24 V, 0 V, PE, I/O)", "e.g. Phoenix Contact PT 2.5", 10, ""),
]


def write_csv(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, delimiter=";")
        w.writerow(["Pos", "Qty", "Designation", "Standard / supplier", "Purpose"])
        for i, (d, s, q, why) in enumerate(CABINET_BOM, 1):
            w.writerow([i, q, d, s, why])
    return path


if __name__ == "__main__":
    print(write_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out", "bom_cabinet.csv")))
