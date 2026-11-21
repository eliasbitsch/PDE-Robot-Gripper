# E1 - Robot selection and placement

ABB IRB 1300-7/1.4 on a 500 mm pedestal, base 100 mm towards the workpiece carrier.

| File | Content |
|---|---|
| `robot_selection.py` | payload vs load diagram, axis 5/6 inertia and wrist torques; MoveJ time; placement grid; candidates -> `robot_selection.json`, `figures/` |
| `wrist_torque.py` | payload torques about axes 5 and 6 along the real E4 cycle (open-source stand-in for RobotLoad) -> `wrist_torque.json`, `figures/wrist_torque.png` |
| `robot_notebook.py` | `marimo edit robot_notebook.py`: tables, load diagram, placement map, marked datasheet excerpts |
| `mark_datasheets.py` | marks the values we use in the vendor PDFs (yellow value, red frame) -> `<vendor>/marked/` (local, not in the public repo), `datasheet_values.json` (committed: value, page, why) |

**Results (21 Nov 2026):** payload 3.64 kg (gripper 3.036 + part 0.602) at Z = 84 mm, L = 6 mm; highest use is the
wrist torque about axis 4/5 with the tool horizontal (72 %, a bound: this cell never holds the tool horizontal), load
diagram 45 %, Ja5 25 %, payload 52 %. Along the real cycle (`wrist_torque.py`, review 1.14) the payload torque about
axis 5 peaks at 7.0 N m (54 %) when the empty gripper starts the way back at 15 m/s², 3.4 N m with the part, 0.06 N m
at rest (tool down); axis 6 at most 0.5 N m (8 %). ABB gives the 13 / 6.9 N m as a reference, the binding check is
RobotLoad, which we replace by this inverse-dynamics calculation. Transfer through the 1200 mm enclosure (one retracted via, ABB MoveJ with z200 corner path) 1.86 s loaded, 1.33 s empty; full cycle 4.48 of 5 s (E4). The chosen placement keeps
20.6° axis margin (axis 1 at the place position) with the arm 74 % stretched; the lowest feasible pedestal is 200 mm
(12.5°, 92 % stretched). Axis 5 stays below 91° of ±130°.

Other candidates (`CANDIDATES`, data checked 21 Nov 2026 against manufacturer pages): ABB IRB 1600-6/1.45 (1450 mm,
6 kg, 150°/s), KUKA KR 10 R1420 (1420 mm, 10 kg, KR C4 compact; 220°/s only from a distributor sheet), FANUC
M-10iD/12 (1441 mm, 12 kg, 260°/s), UR10e (1300 mm, 12.5 kg, cobot, TCP speed limited). The IRB 1300 is kept: enough
reach and payload, the fastest axis 1 of the ABB options, current OmniCore controller.

Inputs: `02_CAD_Body/out/checks.json` (mass, CoG, inertia from the build), `06_Cell/cell_params.py` (kinematics,
cell layout), product specification 3HAC070393-001 rev. M (in `06_Cell/vendor`, see its SOURCES.md).
