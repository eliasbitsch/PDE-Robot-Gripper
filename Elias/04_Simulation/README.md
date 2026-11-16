# E4 - MuJoCo cell and grip validation

ABB IRB 1300-7/1.4 on its pedestal, the gripper on the ISO 9409 flange, part B on the buffer belt and the
workpiece carrier. The model is generated from the sources, nothing is drawn by hand.

| File | Purpose |
|---|---|
| `export_meshes.py` | robot links (ABB STEP) and part B (FH STEP) as STL, `fc.sh export_meshes.py`; third-party, so `meshes/` is gitignored |
| `build_model.py` | MJCF from `06_Cell/cell_params.py` (layout, kinematics), the gripper build (`02_CAD_Body/out`) and the meshes |
| `sim.py` | poses from the closed-form IK, part placement in the jaws, rendering; `python sim.py` renders pick and place |
| `validate_grip.py` | grip validation over the real path, `python validate_grip.py` |

**Model.** Robot joints follow the path through stiff position control with gravity compensation (the robot's own
dynamics are not the question here). The two jaws are slide joints driven together through a tendon (pinion and two
racks) by one position actuator whose force limit is 2 × the force per jaw: the current-limited motor. NBR pad on
PA 6 with μ = 0.2 (oily part), elliptic friction cone, noslip solver. Part B as a free body, 0.602 kg, contact shape
from two boxes (body with the flat grip faces, lug) because its convex hull would touch the left pad.

**Path.** Pick on the belt, lift 60 mm in 0.4 s, transfer to the carrier in T_EINWEG = 2.0 s with a trapezoidal
profile in joint space (MoveJ), lower 60 mm. Axis 1 turns about 160°, so the TCP sees up to 5 m/s², not the 2.05 m/s²
of a straight 1.8 m line that the hand calculation assumes. E-stop: in the middle of the transfer the path decelerates
at A_STOP (15 m/s², assumed until V2), the brake locks the drive with the preload kept.

**Results, 16 Nov 2026** (`out/grip_results.json`, `figures/grip_slip.png`). Friction use = |F_t| / (μ F_n) on the
pads, 1.0 means the part is at the friction limit; median over the moving phases and 50 ms peak.

| Case | Force per jaw | friction use median / peak | part movement in the jaws | |
|---|---|---|---|---|
| nominal | 35.7 N | 0.46 / 0.93 | 0.48 mm | held |
| no safety (F / S) | 17.9 N | 0.85 / 1.00 | 0.87 mm | held only just, micro-slip |
| full current | 63.6 N | 0.32 / 0.75 | 0.41 mm | held |
| e-stop at 15 m/s² | 35.7 N | 0.54 / 0.93 | 0.29 mm | held |
| e-stop, full current | 63.6 N | 0.27 / 0.75 | 0.24 mm | held |

- The hand calculation is confirmed in the mean: with S = 2 the friction use should be 0.5; the simulation gives 0.46.
- Without the safety factor the part sits at the friction limit already at rest (0.85, weight alone 0.83) and creeps
  at every acceleration peak: the safety factor is needed.
- Peaks of 0.93 at the start of the transfer come from the real path (axis 1 accelerating while the lift ends,
  5 m/s²) and from the trapezoidal profile's jumps in acceleration. A real ABB path is jerk-limited, so the peaks
  are lower; with full current (63.6 N, available from the motor) the peak drops to 0.75.
- Recommendation for V2: take the path acceleration of the real motion (about 5 m/s²) instead of 2.05 m/s², or set
  the grip force for it: S · m · (g + a) / (2 μ) = 2 · 0.602 · (9.81 + 4.96) / 0.4 ≈ 44 N per jaw, which the motor
  gives (63.6 N).
- Soft contacts creep a few tenths of a millimetre near the friction limit; part movement below 2 mm is counted as
  held, a dropped part ends below the carrier.
