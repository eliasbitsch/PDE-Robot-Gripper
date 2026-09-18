# Elias - system & numerical proof

Working area of Bitsch Elias. See [PLAN.md](../PLAN.md), § 5.

| Folder | Package | Result |
|---|---|---|
| `01_Robot` | E1 | Robot selection, placement study (500 mm pedestal + 100 mm base offset), reachability proof |
| `02_CAD_Body` | E2 | Housing, drive unit (servo + self-locking screw), four-bar with bearings, quick-change coupler, robot flange |
| `03_Drawing` | E3 | A3 assembly drawing + BOM |
| `04_Simulation` | E4 | MuJoCo model, path pick -> place, collision log, sequence video |
| `05_FEM` | E5 | Mesh, boundary conditions, mesh convergence, stress and deflection; compared against V4 |

Plus E6: infrastructure (repo, Onshape document, MCP integration, parameter table) in `../shared/`.

**Interface to Viktoriia:** the mounting plane carrier <-> jaw.
Frozen from week 41, changes only together.
