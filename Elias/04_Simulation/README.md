# E4 - MuJoCo cell and grip validation

ABB IRB 1300-7/1.4 on its pedestal, the gripper on the ISO 9409 flange, part B on the buffer belt and the
workpiece carrier. The model is generated from the sources, nothing is drawn by hand.

| File | Purpose |
|---|---|
| `export_meshes.py` | robot links (ABB STEP) and part B (FH STEP) as STL, `fc.sh export_meshes.py`; third-party, so `meshes/` is gitignored |
| `build_model.py` | MJCF from `06_Cell/cell_params.py` (layout, kinematics), the gripper build (`02_CAD_Body/out`) and the meshes |
| `sim.py` | poses from the closed-form IK, part placement in the jaws, rendering; `python sim.py` renders pick and place |
| `validate_grip.py` | grip validation over the real path, `python validate_grip.py` |
| `drive.py` | motor curve (ST4118S at 24 V from R, L, holding torque), reflected drive inertia, fastest move |
| `closing_time.py` | closing time and impact on the part, `python closing_time.py` |
| `viz.py` | slow-motion video with four cameras, contact points and forces, clearance check (convex hulls) |
| `validation_notebook.py` | everything live: `marimo edit validation_notebook.py` |

**Model.** Robot joints follow the path through stiff position control with gravity compensation (the robot's own
dynamics are not the question here). The two jaws are slide joints driven together through a tendon (pinion and two
racks) by one position actuator whose force limit is 2 × the force per jaw: the current-limited motor. Rotor,
gearbox, brake and bevel pair sit on the slides as armature (3.2 kg reflected on the jaw coordinate, `drive.py`). NBR pad on
PA 6 with μ = 0.2 (oily part), elliptic friction cone, noslip solver. Part B as a free body, 0.602 kg, contact shape
from two boxes (body with the flat grip faces, lug) because its convex hull would touch the left pad.

**Path.** Pick on the belt, lift 60 mm in 0.4 s, transfer to the carrier in T_EINWEG = 2.0 s with a trapezoidal
profile in joint space (MoveJ), lower 60 mm. Axis 1 turns about 160°, so the TCP sees up to 5 m/s², not the 2.05 m/s²
of a straight 1.8 m line that the hand calculation assumes. E-stop (category 0): in the middle of the transfer every
axis brakes with its deceleration from the ABB product specification (3HAC070393 rev. M, p. 91: axis 1 55.25° in
0.40 s, i.e. 12.1 rad/s²), which gives 12.3 m/s² at the TCP; the brake locks the drive with the preload kept.

**Results, 16 Nov 2026** (`out/grip_results.json`, `figures/grip_slip.png`), with the drive inertia on the jaws.
Friction use = |vector sum of the pad friction forces| / (μ ΣF_n): the load the friction has to carry, 1.0 = at
the limit; median over the moving phases and 50 ms peak.

| Case | Force per jaw | friction use median / peak | part movement in the jaws | |
|---|---|---|---|---|
| nominal | 35.7 N | 0.42 / 0.85 | 0.81 mm | held |
| no safety (F / S) | 17.9 N | 0.83 / 1.00 | 1.96 mm | held only just, micro-slip |
| full current | 63.6 N | 0.23 / 0.65 | 0.41 mm | held |
| e-stop, ABB category 0 | 35.7 N | 0.41 / 0.64 | 0.45 mm | held |
| e-stop, full current | 63.6 N | 0.23 / 0.65 | 0.25 mm | held |

- The hand calculation is confirmed: with S = 2 it expects 0.5 at its design acceleration; the simulation gives a
  median of 0.42 and scales with 1/F as it should (full current 0.23).
- Without the safety factor the part sits at the friction limit already at rest (weight alone 0.83) and creeps
  at every acceleration peak: the safety factor is needed.
- Peaks of 0.85 come from the real path (axis 1 accelerating while the lift ends, 5 m/s²) and from the
  trapezoidal profile's jumps in acceleration. A real ABB path is jerk-limited, so the peaks are lower.
- Recommendation for V2: take the path acceleration of the real motion (about 5 m/s²) instead of 2.05 m/s², or set
  the grip force for it: S · m · (g + a) / (2 μ) = 2 · 0.602 · (9.81 + 4.96) / 0.4 ≈ 44 N per jaw, which the motor
  gives (63.6 N).
- Per pad, one pad always shows 1.0 (`util_pad_max`): two pads on a rigid part are statically indeterminate, and
  the solver locks in an internal shear (one pad pushes up, the other down) that depends on how the jaws closed. It
  cancels in the net force and does not move the part; it is a property of the rigid model, not a finding.
- Soft contacts creep a few tenths of a millimetre near the friction limit; part movement below 2 mm is counted as
  held, a dropped part ends below the carrier.
- The cycle starts with the jaws 0.3 mm before the part; the closing stroke itself is the next section.

## Closing time (`closing_time.py`, `out/closing_results.json`, `figures/closing.png`)

Requirement from the hand calculation V2: close 8 mm per jaw and have the grip force within 0.3 s. The drive runs
position controlled with the current limit as grip force (35.7 N per jaw). Motor curve from the datasheet's electrical
values at 24 V (phasor sum of back-EMF and winding impedance), 70 % usable in closed loop; the Nanotec curve still
has to be laid over it.

| Profile | contact | grip force within 5 % | peak pad force | motor peak |
|---|---|---|---|---|
| direct, trapezoid 0.25 s | 177 ms at 50 mm/s | 193 ms | 76.7 N (1.9 ×) | 324 rpm, 0.018 of 0.136 N·m |
| direct, trapezoid 0.12 s | 87 ms at 108 mm/s | 99 ms | 148.5 N (3.6 ×) | 675 rpm, 0.030 of 0.136 N·m |
| two-stage: 0.12 s to 1 mm before the part, then 10 mm/s | 176 ms at 9 mm/s | 255 ms | 37.0 N (1.0 ×) | 591 rpm, 0.028 of 0.136 N·m |

- Torque is not the limit: the motor could cover the 8 mm in 45 ms. The motor uses at most a quarter of its torque.
- The limit is the impact. Rotor, gearbox and brake act like 3.2 kg on the jaws, seven times the jaws themselves
  (0.47 kg), and the current limit cannot stop this inertia before the pads hit: driving straight in at 50 mm/s
  gives 77 N, more than the FEM's full-current case (63.6 N); the undamped estimate with a 3 mm NBR pad is even
  higher (`impact_estimate`, no damping, upper bound).
- **Recommendation:** two-stage profile, fast to 1 mm before the part and the last millimetre at 10 mm/s
  (NanoJ program or two position records in the CL4-E). Meets 0.3 s with 45 ms to spare, no force overshoot.
- The MuJoCo contact stiffness is not calibrated to the pad; the peaks are a comparison between the profiles,
  the order of magnitude is backed by the estimate.

## Videos

`viz.py` renders any run in slow motion with four cameras (cell; both jaws with the housing see-through; pad close-up;
back view on the lug side), contact points and forces split into normal and friction, and a clearance check between
the part and the jaws, pad plates, holders, cover and housing, and between gripper / robot and belt, carrier and
pedestal (convex hulls, so conservative). Colliding parts turn red; the HUD shows pad forces, friction use and
part movement. [`figures/videos/closing_*.mp4`](figures/videos) at 0.05 ×, [`figures/videos/cycle_estop.mp4`](figures/videos/cycle_estop.mp4) at 0.25 ×; in the notebook with your own values.
