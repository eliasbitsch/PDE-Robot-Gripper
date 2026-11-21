# CAD Plan - 2-finger rack-and-pinion gripper

Status: 10 Nov 2026, proposal. Replaces "Onshape is the leading CAD model" in PLAN.md § 2.

## 1. Decisions

| # | Topic | Decision | Why |
|---|---|---|---|
| D1 | CAD tool | **FreeCAD 1.1, built by Python scripts** (headless `freecadcmd`) | The assignment does not prescribe a tool. Scripts read one parameter file, so a change in sizing is one edit plus a rebuild. STEP is the exchange format to Onshape/Viktoriia. |
| D2 | Mechanism | **One pinion, two opposed racks, one jaw per rack** (half of the Cartesian Hand) | Parallel jaws, constant jaw force over the stroke, one gear mesh to prove. |
| D3 | Standard parts | **Standard or catalogue part wherever one exists**, from large, well-known (preferably German) manufacturers, metric; own parts only where the geometry is specific to part B or to our layout | Shorter BOM, ratings come from datasheets instead of own proofs, cheaper to justify. |
| D4 | Guide | **Miniature ball rail size 12, one rail, two carriages** (Bosch Rexroth: carriage SLS long **R0444 213 01**, rail **R0445 203 31**, L = 125), carriages hang under the base plate, rail aligned to a straightedge; C0 = 5630 N, Mt0 = 33.5 N·m, ML0 = 29.2 N·m (catalogue R999001206). V2 § 6: S0 = 8.4 at the 44.5 N setpoint, 7.5 at the calibrated setpoint +5 % with the e-stop, 5.7 at full current plus e-stop (size 9 gave 2.5 ... 3.7). **Hanging arrangement (review 1.9):** Rexroth asks S0 ≥ 12 for overhead arrangements (R999001206 p. 9) because a failed carriage drops the load. Here the slide hangs 2 mm above the bottom cover and is wider than the holder slot: a slide that loses its carriage rests on the cover (≈ 2800 ... 3300 mm²) and its rack stays 7 of 10 mm in mesh - nothing falls out. With this catch the "normal design" row S0 5 ... 8 applies (V2 § 6a); SLS 15 would be needed for ≥ 12. The catch is checked in every build (`carriage_loss_check`) | Catalogue part with rated static moments; changed from size 9 back to size 12 for the tilting moment of the long jaw. One shared rail keeps both jaws parallel by construction. |
| D5 | Gear | **m = 1, pinion z = 20 (d = 20 mm), steel racks, both catalogue** | z = 20 is above the undercut limit (17), no profile shift. 14 mm stroke = 80° pinion rotation. Pinion torque for 14.3 N per jaw: 2 · 14.3 N · 10 mm = **0.29 N·m**. `cad/params.py` already says z = 20; PLAN.md still says z = 16 and gets corrected. |
| D6 | Drive | **Lying drive under a flat hood** (variant 4, decided 10 Nov 2026): Nanotec ST4118S1404-B + GP42-S1-7-SR + BRAKE-BWA-0,35-5 along X; a 1:1 bevel pair (KHK SMSG1.5-20 R/L since 21 Nov, ground spiral teeth, m = 1.5, z = 20) turns it onto a pinion shaft (own part, two DIN 625 608-2Z) at x = -56. Racks of 35 and 14 teeth keep jaws, hood and flange centred because both racks always run at the same speed in opposite directions. | Stem 63 mm instead of 124 mm, flange to grip centre 148 mm instead of 208 mm, nothing overhangs; COG 3 mm off the flange axis. GPLE22 rejected: 20 N permissible shaft load against ≈ 30 N from the bevel pinion. Static torque at the pinion 1.27 N·m (63.6 N per jaw), brake 2.5 N·m. |
| D7 | Jaw quick change | **2 dowel pins ISO 8734 + 1 indexing plunger (catalogue, e.g. GN 617) per jaw** on the carriage plate | The assignment requires quick-change jaws ("Greiferzangen ... mit einem Schnellwechselsystem"), not a tool changer. Pins locate, the plunger locks, no tool needed. |
| D8 | Robot interface | **Round adapter Ø76 to ISO 9409-1-40-4-M6** (ABB IRB 1300-7/1.4), fixed to the motor housing lid from the top with 4 × ISO 10642 M4 on PCD 62 | Prescribed by the robot; screw heads stay reachable outside the robot flange. |
| D9 | Visuals | **Blender** (glTF from FreeCAD) for materials, renders and the open/close animation | FreeCAD has no proper material rendering. Same parameters drive the Blender rig. |
| D10 | Animation | Kinematic check in the **FreeCAD Assembly workbench** (rack-pinion joint + "Create Simulation"); the **presentation animation in Blender**; the cell sequence stays in **MuJoCo** | FreeCAD proves the kinematics are right, Blender makes it look good, MuJoCo proves the cell. |
| D11 | Motor control | **Closed-loop stepper with an external controller** (e.g. Nanotec CL3-E in the cabinet): ST4118 + encoder + BRAKE-BWA + GP42 stay in the hood (decided 15 Nov 2026, replaces the Plug & Drive idea of 12 Nov) | Grip force set by motor current, "part gripped / nothing gripped" from current and following error, no lost steps, fieldbus to the robot. A Plug & Drive (PD2-C/PD2-E) fits only without brake: its only brake (BCD42, front-mounted) adds 45.5 mm and the chain would end at x ≈ 132 against 79 inside the hood. Holding on power failure wins; cost: a thicker cable (motor, encoder, brake) through the dress pack. Encoder type and the extra motor length are still open (open point 2). |

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
| Miniature ball rail size 12 R0445 203 31 + 2 carriages SLS long R0444 213 01 | Bosch Rexroth |
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

0. **Assumption, part position on the buffer belt:** part B arrives within about ±5 mm of its nominal position (the assignment does not say; stops or a centring guide on the belt are assumed). The 10 mm stroke per jaw opens to 86 mm, 8 mm clearance per side over the 70 mm grip faces. Without redesign the stroke can grow to about 12.5 mm (carriage at the rail end, long rack still covering the pinion) if a larger tolerance is required.

1. **Placement orientation on the workpiece carrier** (PLAN § 8.2) - still the only point that can change the grip direction.

2. **D11 drive electronics (chosen 15 Nov 2026, to be confirmed against the CL4-E manual):**
   - Gripper: ST4118S1404-B with long B shaft, BRAKE-BWA-0,35-5, NME2 incremental encoder (5 V) on the brake; body 192 mm.
   - Cabinet: **Nanotec CL4-E-1-12** (12-58 V, 3 A rated / 6 A peak, encoder input, brake output, 4 DI / 2 DO,
     CANopen / Modbus RTU / USB, programmable in NanoJ). Supply 24 V, so the brake runs from the same rail.
   - Robot interface over plain 24 V I/O (no fieldbus option needed on the OmniCore): DI "close", DI "open" to the
     controller; DO "part gripped" (current limit reached and position inside the part window), DO "fault / nothing
     gripped" (closed end stop reached). A NanoJ program on the CL4-E runs the current-limited close move.
   - Cable: one shielded robot-rated hybrid cable (motor 4 + encoder 6 + brake 2 cores, about 8-9 mm), fits the
     existing SKINTOP M16 (clamping range 4.5-10 mm).
   - **Power failure (19 Nov 2026):** the brake alone does not keep the part. Without current the elastic preload opens
     the jaws within ≈ 2 ms (detent ≈ 1.9 N per jaw, the part needs 14.8 N), long before a spring-applied brake engages
     (`04_Simulation/power_fail.py`). Hence: 24 V buffer module **PULS UF20.241** (≥ 200 ms at 20 A, > 1 s at the ≈ 2 A
     of controller + motor + brake) in front of the CL4-E; on undervoltage the NanoJ program closes the brake, waits
     the brake engagement time plus margin, then disables the motor current. The gripper 24 V is a separate,
     non-e-stop-switched supply (the e-stop stops the robot, it must not open the jaws).
   - Shutdown on power loss (NanoJ): undervoltage detected -> motor keeps its current (preload stays) -> brake output off,
     the spring closes the brake -> wait brake engagement time + margin (50-100 ms; not in the Nanotec sheet, measure or
     ask) -> motor current off, the brake now holds the preload.
   - **Cabinet BOM** (`parts/cabinet.py` -> `out/bom_cabinet.csv`, written by every build):

     | Pos | Qty | Part | Type |
     |---|---|---|---|
     | 1 | 1 | Closed-loop stepper controller (encoder input, brake output, NanoJ) | Nanotec CL4-E-1-12 |
     | 2 | 1 | Power supply 24 V DC, 5 A, DIN rail | Mean Well SDR-120-24 |
     | 3 | 1 | Buffer module 24 V, >= 200 ms at 20 A | PULS UF20.241 |
     | 4 | 1 | Circuit breaker C6 A, 230 V AC feed | e.g. ABB S201-C6 |
     | 5 | 1 | Fuse terminal 24 V, 4 A slow (controller supply) | e.g. Phoenix Contact UT 4-HESI |
     | 6 | 1 | Hybrid robot cable motor + brake + encoder, shielded, torsion-rated, 5 m | e.g. igus chainflex (to be selected) |
     | 7 | 1 | Control cable 4 x 0.25 shielded, 3 m (OmniCore I/O <-> CL4-E) | standard |
     | 8 | 10 | DIN rail terminal blocks 2.5 mm2 | e.g. Phoenix Contact PT 2.5 |
   - Still to check: NME2 output level against the CL4-E encoder input, NanoJ sequence, V2 motor sizing unchanged
     (ST4118S stays).
   Candidates (15 Nov 2026): **PD2-C4118L1804-E-08** (CANopen; -E-01 USB/IO), 0.50 N·m holding, 1.8 A, 12-48 V,
   single-turn absolute encoder 1024 CPR, body ≈ 74 mm (ST4118S: 34 mm); or the newer **PD2-E** (up to 0.80 N·m,
   CANopen / Modbus RTU / EtherCAT, PWM output for an external holding brake). Open: where the brake goes - the
   controller sits on the motor's rear end, so the BRAKE-BWA can no longer sit on the rear shaft; the PD2-C page lists
   a brake accessory, its mounting and length are still to be read from the datasheet drawing.
   Datasheets read (vendor/nanotec_pd2, gitignored): PD2-E421M 0.28 N·m, 74.7 mm long but 69.5 mm high (controller
   box on one side), brake output on X4; PD2-C4118L1804 74 mm, 58.7 mm high; the only brake that fits a Plug & Drive
   is the front-mounted BRAKE-BCD42-0,4-5 (45.5 mm between motor and gearbox, needs the motor shaft cut to 10-11 mm).
   Fit check against the lying layout: gearbox input flange at x ≈ 12, hood inside to x ≈ 79, hood top z = 66.
   Without brake the PD2 ends at x ≈ 87 (8 mm too long) and its controller box does not fit in height or width;
   with the BCD42 it ends at x ≈ 132. **The integrated drive does not fit the current hood.** Decision pending
   (see options in the 15 Nov chat: external closed-loop controller, self-locking worm stage, or a larger hood).

## 8. Manufacturing and assembly

**Manufacturing.** One gripper per cell, so all housings are milled from the solid (EN AW-6082 T6): no tooling,
chips are recycled. For a series the motor housing would become an extruded profile or a casting.
Rules the build checks or the layout follows:

- every hole of every own part keeps at least **1.5 mm** wall (`wall_check` in `build.py`)
- inner pocket corners R6, standard d10 cutter; small bosses at the pocket ends carry the countersunk cover screws
- each housing is machined from two sides only (pocket from below, holes from the top)
- housing 192 × 91.5 mm (y -53.5 ... +38), walls 7 mm, outer edges R12; flat hood 192 × 76 mm (y ±38) with R12,
  flush with the housing on the +y side and with the Ø76 robot adapter; 1.5 mm chamfer on the top edges
- visible screw heads are countersunk (ISO 10642) or sit in counterbores, nothing stands proud of the outer faces

**Fastener variants** (as in the model): ISO 4762 M3×8 (rail, carriages, pad plates), M3×10 (racks, bracket into HELICOIL plus M3 × 1.5 d),
M4×12 (gearbox, hood), M4×25 (jaw holders); ISO 10642 M3×8 (window plate), M3×10 (cover), M4×16 (adapter, into
HELICOIL plus M4 × 1.5 d inserts in the hood seat); ISO 4029 M4×4; ISO 7089 3; ISO 8734 3×14 and 4×12;
DIN 6885 A 2×2×8; DIN 471 8; DIN 472 22; DIN 988 8×14.
Thread engagement in aluminium is at least 1.5 × d with an insert or 2 × d without (window plate 1.7 × d on 2 mm bosses). Brake and encoder are fixed with the screws Nanotec specifies
for them (3 × M3 brake to motor, 2 × M2.5 encoder to brake). The Rexroth hole pattern fixes M3 at rail and carriages.

**Assembly order** (every screw reachable with a straight hex key at its step; same text as the FreeCAD demo and
the assembly GIF, kept in `parts/demo_seq.py`):

1. Housing upside down: rail against the reference edge, 5 × M3×8 (Rexroth asks for all holes)
2. Drive bracket with the gearbox (4 × M4×12), motor, brake, encoder and bevel pinion onto the top plate;
   bracket 4 × M3×10 from the top into HELICOIL inserts
3. Pinion shaft with the two 608 bearings from below: the lower one is the floating bearing (inner ring on the
   shaft collar, outer ring slides in the bore), the upper one the fixed bearing (DIN 471 on the shaft, DIN 472 in the
   bore). Shim DIN 988 on the upper inner ring, bevel wheel with key DIN 6885 and the M4×4 set screw (axial only).
   Set the mesh with the shim thickness (0.1 / 0.2 / 0.5) until the flank backlash is right. Before the hood:
   the wheel sits under it.
4. Hood over the drive, 4 × M4×12 from below through the top plate, heads flush
5. Carriages onto the rail, slides onto the carriages (4 × M3×8 each), racks onto the slides
   (2 × M3×10 each in slots, jaws aligned symmetric, then drilled and pinned with a dowel 3×14)
6. Jaw holders with pressed-in dowels onto the slides, 2 × M4×25 each
7. Bottom cover, 4 × M3×10 countersunk
8. Jaws onto the dowels until the plunger snaps in; pad plates 2 × M3×8 + washer each
9. Window plate with cable gland, 4 × M3×8 countersunk; one hybrid cable (motor, encoder, brake)
10. Adapter onto the robot (ISO screws, robot side), gripper onto the adapter with 4 × M4×16 countersunk
    from the top (heads lie outside the robot flange)

Jaw change in the cell: pull the plunger knob, pull the jaw off the dowels, push the new one on. The jaw has one
round hole and one slot along the dowel line, so two dowels locate it without over-constraining it.

**Sealing.** Parts may be oily. A brush seal strip along each holder slot in the cover keeps chips and oil off rail and
racks (bought, cut to length, in the BOM; not modelled because it touches the moving holder).

## 9. Fits and tolerances (for the drawings, E3)

General tolerances ISO 2768-mK. Fits:

| Place | Fit | Why |
|---|---|---|
| 608 outer rings in the housing (lower and upper) | Ø22 H7 | point load on the outer ring; the lower (floating) ring must slide axially |
| 608 inner rings on the pinion shaft | Ø8 k5 | circumferential load on the inner ring |
| Housing shoulders | Ø15, touch the outer rings only | the rotating inner ring must not rub on the housing |
| Retaining ring grooves | DIN 471 / 472 groove dimensions | standard |
| Bevel wheel and bevel pinion bores | Ø8 H7 / shafts h6 (gearbox shaft as supplied) | slide-on fit, torque by the key |
| Keyway shaft / hub | width 2 P9 / 2 JS9 | DIN 6885 tight fit in the shaft, normal fit in the hub |
| Dowels in the jaw holder | Ø4 H7 / m6 | pressed in |
| Dowel holes in the jaw | Ø4 F7 round hole, 4 F7 × 5 slot along the dowel line | quick change without jamming |
| Plunger pin hole in the jaw | Ø5.2 (pin Ø5) | locking only, not locating |
| Rack dowel | Ø3 H7 / m6, drilled and reamed at assembly | after aligning the jaws |
| Rail reference edge | parallelism 0.01 to the rail axis over 125 mm, flatness of the land 0.01 | Rexroth mounting tolerances for size 12 [verify table] |
| Pinion / rack centre distance | ±0.02 via the slots, set at assembly | backlash |
| Robot adapter spigot | ISO 9409-1-40 centring Ø25 H7 / the robot flange | as prescribed by the standard |

## 10. Design review (16 Nov 2026)

Checked against the usual design rules (statically determined location and bearing arrangement, shaft-hub joints
for the load, thread engagement by material, sealing, fits). Found and fixed in the model:

1. Jaw located by two round dowels in two round holes: over-constrained. Now one round hole and one slot.
2. Bevel wheel held by a set screw on a round shaft: unreliable under 1.27 N·m alternating torque. Now key DIN 6885;
   the set screw only holds it axially. Key pressure about 66 MPa (l_eff = 6 mm), fine for C45.
3. Pinion shaft without axial location and housing shoulders touching the rotating inner rings. Now fixed / floating
   bearing with DIN 471 and DIN 472, shaft collar under the lower inner ring, shoulders at Ø15.
4. Bevel mesh without adjustment. Now a DIN 988 shim under the wheel hub.
5. Short thread engagement in aluminium: window plate (1 × d), bracket (1.3 × d), adapter (1.5 × d, carries the whole
   gripper). Now 2 mm bosses (1.7 × d), HELICOIL inserts for bracket and adapter (1.5 × d with insert; a 2 × d thread would come within 1 mm of the pocket edge).
6. Holder slots in the cover open to oil and chips: brush seal strips (BOM).
7. Fits and tolerances missing: section 9.

## 11. Drive chain margins (review 1.7, 21 Nov 2026, `drive_margin.py`, `out/drive_margin.json`)

The setpoint 44.5 N per jaw puts 0.94 N·m on the bevel teeth (driving side, after the losses of bevel and rack).
The m 1 pair KHK SMSG1-20 is rated 0.97 N·m (flank) / 1.17 N·m (root), so it only held at K_A = 1 and with no
scatter of the current setting. Life: one load cycle per grip on the same few teeth, 2.9e7 in 10 years (2 shifts).

| | current set from data sheets (k_T ±10 %, driver ±3 %, η ranges) | calibrated at commissioning (force gauge, ±5 %) |
|---|---|---|
| lowest jaw force (needed 35.9 N: path with twist, S 2) | 35.6 N ✗ | **42.3 N** ✓ |
| highest bevel torque | 1.12 N·m | 1.01 N·m |
| SMSG1-20 (m 1): K_A·T / T_root, K_A·T / T_flank | 1.20, 1.44 ✗ | 1.08, 1.31 ✗ |
| **SMSG1.5-20 (m 1.5, built)** | 0.34, 0.40 ✓ | **0.31, 0.37** ✓ |

Rare overloads against the static capacity (ISO 6336 life factors, induction hardened: flank Z_NT 1.6, root
Y_NT 2.5) with the m 1.5 pair: full current if the current limit is lost 1.48 N·m (use 0.17), direct close in 0.12 s
without the two-stage profile, 160 N impact (`closing_time.py`) 3.36 N·m (0.38), LC5 jaw pushed with the power off until the brake slips 2.47 N·m (0.28, the
m 1 pair was at 1.00). Key DIN 6885 A 2×2×12 in the S45C hub: 32 MPa in operation, 77 MPa at LC5 (static limit
310 MPa).

Consequences: bevel pair m 1.5 (bore 8 as bought, hubs turned to Ø16, wheel faced to mounting distance 19), drive
axis 32.5 → 36 mm, hood 2 mm higher, gearbox shaft reaches 5 mm from the pinion shaft axis so motor, brake and
encoder stay where they were; gripper 3.053 kg. The grip force is calibrated once at commissioning with a grip
force gauge (current setting from the data sheets alone may give 35.6 N). The current limit is now the motor at full
current (60.5 N per jaw), no longer the bevel pair.

## Second version: pneumatic drive (21 Nov 2026, `GRIPPER_DRIVE=pneumatic`, output `out_pneumatic/`)

Checkpoint of the electric version: tag `v1-electric`. Built from the same parameters with a switch (`DRIVE` in
`gripper_params.py`); racks, slides, rail, jaws, pinion with its bearings and the FEM models stay the same.

- Festo ADN-16-10-A-P-A compact cylinder in a flat riser on the top plate, driver plate on the rod with a pin
  (ISO 8734 5 x 27) through a slot in the plate into the right rack; the pinion keeps the left jaw in step.
- Double-acting: air closes with the force set on a pressure regulator (2.9 bar for 44.5 N per jaw, the counterpart
  of the current limit), air opens (108.6 N at 6 bar against 90 N of spring). Spring-only closing was dropped: a
  spring that alone gives 95 N over 10 mm stroke does not fit (block length) or needs a Ø25-32 cylinder.
- Safety spring (d 1.8 x Dm 12 x L0 46, c ~ 5 N/mm) adds 50 N at the grip position and alone holds 25 N per jaw
  when the air fails (22 N needed with S = 1.5 at standstill): fail-safe without check valves.
- The riser top is the robot flange (ISO 9409-1-40-4-M6 machined in, M6 from the inside): no separate adapter.
- Strength case if the regulator fails (full 6 bar): 70.2 N per jaw (electric 63.6 N).

| | electric | pneumatic |
|---|---|---|
| mass | 2.92 kg | 2.08 kg (-29 %) |
| flange face to grip centre (TCP) | 150.5 mm | 123.5 mm (-18 %) |
| housing length | 192 mm | 160 mm |
| height above the top plate | 76 mm (hood + adapter) | 49 mm (riser with flange) |
| bounding box | 3.38 l | 2.80 l (-17 %) |

Still to do for the pneumatic version: V2 cylinder and spring sizing (DIN EN 13906-1, block length, fatigue),
cabinet BOM (valve, regulator, throttles, sensors), E1 payload, MuJoCo closing and air-loss cases, FEM strength
case at 70 N, verify the ADN-16 drawing (body length, end-cap thread pattern).

## Decision 21 Nov 2026: the electric version stays the main design

Compared on the same jaws and part: electric 2.92 kg / flange to TCP 150.5 mm, pneumatic 2.08 kg / 123.5 mm
(`GRIPPER_DRIVE=pneumatic`), wedge hook (rough, `concepts/wedge_hook.py`) about 1.4 kg / 102.5 mm.
The electric drive is kept: grip force set in software (current limit), part detection from current and position
without extra sensors, programmable stroke and speed (two-stage close), no compressed air in the cell, holding on
power failure proven (brake + buffer + brake-first shutdown). The weight is no limit for the IRB 1300 (3.5 kg of
7 kg, wrist torque 70 %). No further slimming: the drive chain (motor, brake, encoder, gearbox, about 165 mm) sets
the length, and the hood, housing and flange close flush on the +y face on purpose; narrowing the hood would only
gain a few percent and break that line. Pneumatic version and wedge-hook concept stay in the repo as the evaluated
alternatives for the variant comparison (V1).
