# E1 - Robot selection and placement

ABB IRB 1300-7/1.4 on a 500 mm pedestal, base 100 mm towards the workpiece carrier.

| File | Content |
|---|---|
| `robot_selection.py` | payload vs load diagram, axis 5/6 inertia and wrist torques; MoveJ time; placement grid; candidates -> `robot_selection.json`, `figures/` |
| `robot_notebook.py` | `marimo edit robot_notebook.py`: tables, load diagram, placement map, marked datasheet excerpts |
| `mark_datasheets.py` | marks the values we use in the vendor PDFs (yellow value, red frame) -> `<vendor>/marked/` (local, not in the public repo), `datasheet_values.json` (committed: value, page, why) |

**Results (21 Nov 2026):** payload 3.64 kg (gripper 3.036 + part 0.602) at Z = 84 mm, L = 6 mm; highest use is the
wrist torque about axis 4/5 (72 %), load diagram 45 %, Ja5 25 %, payload 52 %. Transfer through the 1200 mm enclosure (one retracted via, ABB MoveJ with z200 corner path) 1.86 s loaded, 1.33 s empty; full cycle 4.38 of 5 s (E4). The chosen placement keeps
20.6° axis margin (axis 1 at the place position) with the arm 74 % stretched; the lowest feasible pedestal is 200 mm
(12.5°, 92 % stretched). Axis 5 stays below 91° of ±130°.

Inputs: `02_CAD_Body/out/checks.json` (mass, CoG, inertia from the build), `06_Cell/cell_params.py` (kinematics,
cell layout), product specification 3HAC070393-001 rev. M (in `06_Cell/vendor`, see its SOURCES.md).
