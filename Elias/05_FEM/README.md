# E5 - FEM of the jaw

Scripted chain: `jaw.step` -> gmsh (C3D10) -> CalculiX -> matplotlib / PyVista. No GUI tool needed.

| File | Purpose |
|---|---|
| `fem_jaw.py` | mesh, boundary conditions, solve, read results; `python fem_jaw.py` runs the mesh convergence study |
| `fem_notebook.py` | interactive: pick load case, element size, clamp; `marimo edit fem_notebook.py` |
| `analytic.py` | beam-theory cross-check (Euler-Bernoulli + shear); the graded hand calculation is V4 |
| `export_jaw.py` | regenerates `jaw.step` from the CAD parameters (FreeCAD: `fc.sh export_jaw.py`) |
| `figures/` | convergence plot, von Mises and deflection images |

Setup: `pip install gmsh pyvista marimo matplotlib numpy`. CalculiX comes with FreeCAD 1.1
(`bin/ccx.exe`); otherwise set the `CCX` environment variable.

**Model.** Holder face (x = 44, z = -47 ... -28.5) fixed in x only, dowel holes fixed in y and z,
grip force as uniform pressure on the pocket floor (x = 39). EN AW-7075 T6, E = 71.7 GPa, Rp0.2 = 460 MPa.

**Results, 11 Nov 2026, F = 63.6 N (full current):**

| | FEM (158k el.) | beam theory | |
|---|---|---|---|
| bending stress at z = -53.5 | 5.71 MPa | 5.73 MPa | -0.3 %, converged |
| deflection at load centre | 17.0 µm | 11.6 µm | +47 %; rigid clamp in FEM: 15.0 µm |
| peak von Mises | ~20 MPa | - | sharp pocket corner, grows with refinement |

- The stress matches within 0.3 % from 9k elements on. The ≤ 5 % criterion is met on stress.
- The deflection gap is the classic clamp question: beam theory assumes a rigid clamp at the holder
  edge, the real jaw also deforms inside the holder zone and around the holes. The printed check (V5) decides.
- The peak sits on the sharp edge between pocket wall and rim and is a singularity. A milled pocket has a
  corner radius anyway; with R1 in the CAD it converges. Safety is > 20 against Rp0.2 either way.
