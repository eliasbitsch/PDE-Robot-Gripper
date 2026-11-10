# CAD Plan - 2-finger rack-and-pinion gripper

Status: 10 Nov 2026, proposal. Replaces "Onshape is the leading CAD model" in PLAN.md § 2.

## 1. Decisions

| # | Topic | Decision | Why |
|---|---|---|---|
| D1 | CAD tool | **FreeCAD 1.1, built by Python scripts** (headless `freecadcmd`) | The assignment does not prescribe a tool. Scripts read one parameter file, so a change in sizing is one edit plus a rebuild. STEP is the exchange format to Onshape/Viktoriia. |
| D2 | Mechanism | **One pinion, two opposed racks, one jaw per rack** (half of the Cartesian Hand) | Parallel jaws, constant jaw force over the stroke, one gear mesh to prove. |
| D3 | Standard parts | **Standard or catalogue part wherever one exists**, from large, well-known (preferably German) manufacturers, metric; own parts only where the geometry is specific to part B or to our layout | Shorter BOM, ratings come from datasheets instead of own proofs, cheaper to justify. |
| D4 | Guide | **Miniature ball rail size 9, one rail, two carriages** (Bosch Rexroth: carriage SLS **R0444 913 01**, rail **R0445**, L = 110), rail on a 3 mm land that is also the reference shoulder; C0 = 3150 N, Mt0 = 14.5 N·m, ML0 = 14.0 N·m (catalogue R999001206). Worst case 63.6 N × ≈ 70 mm ≈ 4.5 N·m → S0 ≈ 3 (≈ 2 with the yaw moment added); normal operation (≈ 36 N) S0 ≈ 4 | Catalogue part with rated static moments; size 9 instead of 12 since 10 Nov to slim the housing. One shared rail keeps both jaws parallel by construction. |
| D5 | Gear | **m = 1, pinion z = 20 (d = 20 mm), steel racks, both catalogue** | z = 20 is above the undercut limit (17), no profile shift. 14 mm stroke = 80° pinion rotation. Pinion torque for 14.3 N per jaw: 2 · 14.3 N · 10 mm = **0.29 N·m**. `cad/params.py` already says z = 20; PLAN.md still says z = 16 and gets corrected. |
| D6 | Drive | **Lying drive under a flat hood** (variant 4, decided 10 Nov 2026): Nanotec ST4118S1404-B + GP42-S1-7-SR + BRAKE-BWA-0,35-5 along X; a 1:1 bevel pair (Maedler m = 1, z = 20) turns it onto a pinion shaft (own part, two DIN 625 608-2Z) at x = -56. Racks of 35 and 14 teeth keep jaws, hood and flange centred because both racks always run at the same speed in opposite directions. | Stem 63 mm instead of 124 mm, flange to grip centre 148 mm instead of 208 mm, nothing overhangs; COG 3 mm off the flange axis. GPLE22 rejected: 20 N permissible shaft load against ≈ 30 N from the bevel pinion. Static torque at the pinion 1.27 N·m (63.6 N per jaw), brake 2.5 N·m. |
| D7 | Jaw quick change | **2 dowel pins ISO 8734 + 1 indexing plunger (catalogue, e.g. GN 617) per jaw** on the carriage plate | The assignment requires quick-change jaws ("Greiferzangen ... mit einem Schnellwechselsystem"), not a tool changer. Pins locate, the plunger locks, no tool needed. |
| D8 | Robot interface | **Round adapter Ø76 to ISO 9409-1-40-4-M6** (ABB IRB 1300-7/1.4), fixed to the motor housing lid from the top with 4 × ISO 10642 M4 on PCD 62 | Prescribed by the robot; screw heads stay reachable outside the robot flange. |
| D9 | Visuals | **Blender** (glTF from FreeCAD) for materials, renders and the open/close animation | FreeCAD has no proper material rendering. Same parameters drive the Blender rig. |
| D10 | Animation | Kinematic check in the **FreeCAD Assembly workbench** (rack-pinion joint + "Create Simulation"); the **presentation animation in Blender**; the cell sequence stays in **MuJoCo** | FreeCAD proves the kinematics are right, Blender makes it look good, MuJoCo proves the cell. |

## 2. Parts list (target BOM)

Own parts (milled, drawn):

| Part | Material | Note |
|---|---|---|
| Housing / base plate | EN AW-6082 T6, anodised | carries rail, motor, flange adapter |
| Flange adapter | EN AW-6082 T6 | ISO 9409-1-50-4-M6 pattern + centring |
| Carriage plate x2 | EN AW-6082 T6 | sits on MGN carriage, carries rack and jaw interface (pins + plunger) |
| Jaw x2 | **EN AW-7075 T6** | the part under FEM; contour matched to part B |
| Pad plate x2 (bought, rubber-metal) | steel 3 mm + vulcanised NBR 70 Shore A 3 mm (oil resistant) | sits in a 3 mm pocket in the jaw, 2 × ISO 4762 M3×8 + ISO 7089 washer from the back; replaced without glue. No catalogue standard part found (Ganter GN 351/352 are natural rubber and 10-20 mm tall), ordered from a rubber-metal supplier to our drawing |
| Cover | EN AW-6082 or sheet | keeps chips out of the mesh |

Standard / catalogue parts:

| Part | Standard / source |
|---|---|
| Miniature ball rail size 9 R0445 + 2 carriages SLS R0444 913 01 | Bosch Rexroth |
| Rack m = 1, steel, 10 × 12 mm (pitch line 11 mm), cut to length x2 | Mädler |
| Pinion m = 1, z = 20, d = 20 mm, b = 10 mm, bored to the gearbox shaft | Mädler |
| Stepper ST4118S1404-B + gearbox GP42-S1-5-SR + brake BRAKE-BWA-0,35-5 | Nanotec (supplied assembled) |
| Cap screws | ISO 4762 |
| Washers | ISO 7089 |
| Dowel pins | ISO 8734 |
| Indexing plunger | GN 617 (Ganter) |
| Set screw / clamping on pinion | ISO 4029 or clamping hub from catalogue |
| End stops (rubber buffer) | catalogue |

Vendor STEP files go into `vendor/` and are **gitignored** (the repo is public, vendor licences usually forbid redistribution). `vendor/SOURCES.md` lists where each file comes from so the build can be reproduced.

## 3. Script architecture

```
Elias/02_CAD_Body/
  CAD_PLAN.md          this file
  gripper_params.py    gripper geometry; imports part/cell values from shared params
  parts/               one module per own part -> returns Part.Shape, names every face group it exports
  std/                 standard parts: Fasteners workbench calls, vendor STEP loaders
  vendor/              downloaded catalogue STEP (gitignored)
  build.py             builds the assembly, runs the checks, writes out/
  out/                 FCStd, STEP per part + assembly, glTF, STL (generated, gitignored)
blender/
  render.py            imports glTF, assigns materials by part name, renders stills
  animate.py           open/close animation from the same stroke/angle parameters
```

Rules (from the build123d lessons, `pde-werkzeugkette-stolperfallen`):

- **One parameter source.** No number in a part module that is not in `gripper_params.py`.
- **Each part is its own named solid.** Never fuse parts across part boundaries; the BOM and the STEP depend on it.
- **Re-import the STEP after every build** and compare body count and volume with the model. What is in memory says nothing about what arrives in the file.
- **Parts that touch must overlap or be separate on purpose.** Zero-thickness contact does not fuse.
- Headless for build and checks. GUI only for screenshots and the Assembly simulation.

## 4. Checks that run on every build

1. Interference: pairwise `common()` volume of all parts = 0, at **open, grip and closed** positions
2. Stroke: jaws open ≥ part width + clearance (95 + 2 · 1.5 mm), close ≤ 70 mm minus margin, so the motor still presses at grip
3. Mesh: pinion pitch circle tangent to rack pitch line within 0.01 mm
4. Mass and centre of gravity of the gripper → feed back into payload check (E1) and `M_GREIFER`
5. STEP round trip: body count and volume match
6. Clearance to part B and to the buffer belt at the pick position (jaw tips above the belt)

## 5. Steps

| # | Step | Result | Done when |
|---|---|---|---|
| 0 | Setup | Fasteners + Gears workbenches installed, folder skeleton, `fc.sh` build runs empty | `build.py` writes an empty FCStd |
| 1 | Freeze inputs | Gear, rail, motor, plunger chosen with catalogue numbers; vendor STEP downloaded; jaw interface (carriage plate ↔ jaw) dimensioned in `params.py` | Viktoriia has seen the interface |
| 2 | Layout | 2 layouts (motor axis vertical beside flange vs. motor coaxial under flange with offset pinion) as boxes only, pick one | one layout picked, reason written down |
| 3 | Core | housing, rail, carriages, racks, pinion, motor, flange adapter | checks 1-3 pass |
| 4 | Jaws | carriage plates, quick change, jaws, pads | checks 1-6 pass |
| 5 | Assembly | FreeCAD Assembly with joints (fixed, slider x2, revolute, rack-pinion x2), simulation runs open → closed | screenshot of open / grip / closed |
| 6 | Exports | STEP per part + assembly, glTF, STL for FEM and MuJoCo | round trip passes |
| 7 | Blender | materials, stills, open/close animation | images and MP4 in `out/` |
| 8 | Handover | jaw STEP to FEM (E5), gripper STEP to cell (E7) and MuJoCo (E4), drawing start (E3) | - |

## 6. Timeline (to the interim report on Thu 3 Dec)

PLAN.md M2 "CAD closes" was due 1 Nov and is late; the FEM (M3) depends on the jaw geometry. Order is therefore jaw-first where it matters:

| Dates | Steps |
|---|---|
| 10-12 Nov | 0, 1 |
| 13-17 Nov | 2, 3 |
| 18-20 Nov | 4, 5 (jaw geometry frozen on 20 Nov → FEM can start) |
| 21-25 Nov | 6, FEM of the jaw (E5) against V4 |
| 26-28 Nov | 7, slides |
| 29 Nov - 2 Dec | buffer |
| **3 Dec** | interim report: concept, sizing, CAD, renders, first FEM |

December: drawing + BOM in TechDraw (E3), cell (E7), MuJoCo sequence (E4).

## 7. Open points

1. **Placement orientation on the workpiece carrier** (PLAN § 8.2) - still the only point that can change the grip direction.

## 8. Manufacturing and assembly

**Manufacturing.** One gripper per cell, so all housings are milled from the solid (EN AW-6082 T6): no tooling,
chips are recycled. For a series the motor housing would become an extruded profile or a casting.
Rules the build checks or the layout follows:

- every hole of every own part keeps at least **1.5 mm** wall (`wall_check` in `build.py`)
- inner pocket corners R6 (gear housing), motor housing bore Ø64: standard cutters
- each housing is machined from two sides only (pocket from below, holes from the top)
- gear housing 146 × 95.5 mm, walls 7 mm, outer edges R12; round motor housing Ø72 with a flat connector pod; 1.5 mm chamfer on top edges

**Few fastener variants:** ISO 4762 M3×8, M3×10, M4×16, M4×25, ISO 10642 M4×16, ISO 4029 M4×4,
ISO 7089 3, ISO 8734 4×12. The Rexroth hole pattern fixes M3 at rail and carriages.

**Assembly order** (every screw reachable with a straight hex key at its step):

1. Gear housing upside down: rail against the reference edge, 5 × M3×8 (Rexroth asks for all holes)
2. Nanotec drive (motor + gearbox + brake, supplied assembled) onto the top plate, 4 × M4×16 from below, heads flush
3. Round motor housing over the drive, 3 × M4×16 from below into its bosses, heads flush
4. Pinion with key onto the gearbox shaft, set screw M4×4 from the side
5. Carriages onto the rail, slides onto the carriages (4 × M3×8 each), racks onto the slides (2 × M3×10 each)
6. Jaw holders with pressed-in dowels onto the slides, 2 × M4×25 each
7. Bottom cover, 4 × M3×10
8. Jaws onto the dowels until the plunger snaps in; pad plates 2 × M3×8 + washer each
9. Window plate with cable gland, 4 × M3×8; cables
10. Adapter onto the robot (ISO screws, robot side), gripper onto the adapter with 4 × M4×16 countersunk from the top

Jaw change in the cell: pull the plunger knob, pull the jaw off the dowels, push the new one on.
