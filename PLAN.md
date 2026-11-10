# Project Plan - Robot Gripper, Part B

**Course:** Produktdesign und Produktentwicklung, MRE WS 2026/27
**Group:** 10 · **Part:** B (PA 6, Ra 1.6)
**Team:** Bitsch Elias · Ovdiienko Viktoriia
**Course groups:** split (BB-1 / BB-2) - interim report and final submission run **together on BB-2** (decided 19 Oct 2026)
**Status:** 18 Sep 2026 - proposal, not yet agreed

> **Working language is English** - this plan, the repo, folder names and commit
> messages. Two exceptions, both deliberate: the submission folder names under
> `shared/submission/` stay **German**, because the assignment prescribes them
> verbatim; and the language of the submitted report itself still has to be
> confirmed with Saliger (see open point 5).

---

## 1. What this is

A robot picks part B off the buffer belt (1000 mm), travels 1800 mm
horizontally and places it on a workpiece carrier (1300 mm). Cycle time 5 s per
part. The part itself is given and is not modified.

**We deliver the whole robot cell, not just a gripper** (confirmed in the
lecture on 17 Oct): the gripper flanged onto the robot, the robot in the cell
with safety fence, buffer belt, workpiece carrier and part B, plus a motion
and grip analysis of the sequence.

Nine deliverables, listed in § 3, submitted in the folder structure from Fig. 2
of the assignment.

---

## 2. Technical decisions

Six decisions. If Viktoriia sees any of them differently, **now** is the moment.
After the design freeze in week 40 every change costs half the chain.

> **Changed 30 Oct 2026:** the mechanism is **rack and pinion**, modelled on
> one half of the [Cartesian Hand](https://arxiv.org/abs/2609.25696). An
> earlier four-bar concept was dropped: transmission-angle proof, pad scrub
> and a screw drive were a lot of mechanism for two flat faces 70 mm apart.

| Topic | Decision | Why |
|---|---|---|
| **Gripping concept** | 2-finger parallel gripper with jaws | Part B has two flat, parallel side faces 70 mm apart. No form fit needed, friction is enough. |
| **Mechanism** | **Rack and pinion**: one pinion between two racks, each rack carries one jaw | Like one gripper of the Cartesian Hand (see below). Jaws move in a straight line and stay parallel, no scrub, **constant jaw force over the stroke**, one gear mesh to calculate. **Rack and pinion are catalogue parts** (steel, m = 1), screws and pins to DIN/ISO - few own parts, a short BOM. Racks run in a milled dovetail slot in the housing; a bought miniature profile rail is the alternative, decided in V2. |
| **Drive** | **Industrial servo motor**, 24 V, with gearbox and a self-locking stage or holding brake, on the pinion | The cell is industrial: 5 s cycle ≈ 720 grips per hour, millions per year - a hobby servo does not survive that. Force set by the motor current limit, no force sensor. Self-locking stage or brake holds part B on power failure. Candidates with datasheets in V2. |
| **CAD** | **FreeCAD 1.1, built by Python scripts** (changed 10 Nov 2026, was Onshape) | The assignment leaves the tool open. One parameter file drives every part, a sizing change is one edit plus a rebuild. STEP to Onshape for viewing. Details in [`Elias/02_CAD_Body/CAD_PLAN.md`](Elias/02_CAD_Body/CAD_PLAN.md). |
| **Calculation / docs** | **LaTeX** | Formulas, units and cross-references stay consistent; the report is worth 10 points and grows from M1 on. |
| **Simulation** | **MuJoCo** | See § 4. |

### Inspired by the Cartesian Hand - one half of it

[The Cartesian Hand](https://arxiv.org/abs/2609.25696) (Xia et al., Duke
University, 2026; [video](https://www.youtube.com/watch?v=ktNcPPTPsQE)) is a
7-DoF hand built only from prismatic joints. Every axis is a rack and pinion
driven by a hobby servo, guided in printed dovetail rails, the structure FDM-
or SLS-printed.

<img src="shared/references/cartesian-hand.png" width="420" alt="Cartesian Hand, isometric view">

*Cartesian Hand, Fig. 2b from Xia et al. (2026), CC BY 4.0. Source in
[`shared/references/SOURCES.md`](shared/references/SOURCES.md).*

What we take: **one** of its two parallel grippers - one servo, one pinion,
two racks moving in opposite directions, jaws on the racks.

What we leave out: the second (auxiliary) gripper, the axis between the two,
and the four **fingertip slides** along the fingers. Part B only has to be
held, not manipulated in the hand.

It also settles the force question. Their servo (1.39 N·m stall) gives about
170 N per rack; we need 14.3 N per jaw. Nothing about this concept is
force-limited - the task is limiting the force, not raising it.

The Cartesian Hand is the model for the **principle only**. Our graded design
is industrial: milled aluminium housing, EN AW-7075 jaws, steel rack and
pinion from a catalogue, an industrial drive.

### Figures from the preliminary work (to be confirmed, not fixed)

- Part B: 95 × 100 × 80 mm, 528.5 cm³, PA 6 (ρ = 1140 kg/m³) → **0.602 kg**
- Grip faces: ±X, jaw distance **70 mm**, 6200 mm² on the smaller side
- Path acceleration from the cycle-time budget: **2.05 m/s²**
- Required gripping force: **14.3 N per jaw** (µ = 0.5 NBR/PA 6, S = 2.0)
- Starting geometry of the rack and pinion (to be confirmed in V2): module
  **m = 1**, pinion **z = 20** (d = 20 mm, above the undercut limit), face width 8 mm, stroke
  **14 mm per side** → pinion turns **80°** for the full stroke. Pinion
  torque for 14.3 N on both jaws: T = 2 · F · r = 2 · 14.3 N · 10 mm =
  **0.29 N·m**.
- Reference Cartesian Hand: 52 mm gripper stroke, ~60 mm/s, servo 1.39 N·m
  stall, ~170 N per rack, whole 7-axis hand 850 g. Force is not the
  bottleneck - we need 14.3 N.
- Robot: **ABB IRB 1600-6/1.45**
- Most important finding: with the robot centred on the floor the sequence is
  **not executable** - not because of reach, but because of axis 5 (±115°).
  Fix: **500 mm pedestal + 100 mm base offset**. That belongs in the report as
  a finding, not hidden.

### Vacuum was considered and rejected

Physically it would work: a 50 mm cup at −0.6 bar gives ~118 N normal, ~59 N in
shear against the needed 14.3 N. It fails on the assignment, not on the
physics. The assignment prescribes jaws verbatim -

> "Die **Greiferzangen** sind an das jeweilige Bauteil anzupassen und mit einem
> Schnellwechselsystem auszuführen."

and 30 of 100 points hang on a *Greiferzange* and a *Greifmechanismus* that a
suction cup does not have. On top of that, parts can be gripped **from the side
only**, which loads a cup in shear and peeling, its weakest mode; an ejector
needs the compressed air we avoided; and vacuum does not hold on power failure.
This goes into V1 as a reasoned rejection - a comparison that rules out an
obvious option cleanly is worth more than one that never raised it.

---

## 3. What the points are for

100 points total. The distribution is in the introduction slides and it is the
single most important planning input:

| Work package | Points | Who |
|---|---:|---|
| **3D design of the gripper** | **20** | Elias + Viktoriia |
| General (quality of the technical documentation) | 10 | both |
| Robot selection incl. reasoning | 10 | Elias |
| Concept sketch of the gripper | 10 | Viktoriia |
| Calculation and mechanical design of the gripping mechanism | 10 | Viktoriia |
| Analytical calculation of the jaws | 10 | Viktoriia |
| Numerical calculation of the jaws (FEM) | 10 | Elias |
| Assembly drawing including BOM | 10 | Elias |
| Collision check and visualisation of the sequence | 10 | Elias |

**Grading scale:** ≥88 % very good · ≥75 % good · ≥63 % satisfactory · ≥50 % sufficient · <50 % fail

Two conclusions:

- **The CAD counts double.** It gets the most time and both of us carry it.
- **The documentation alone is worth 10 points**, independent of content. That
  is the cheapest grade in the project, which is why it does not get pushed to
  the end.

### The five warnings from the course leader

From the introduction slides, verbatim - the plan is built around them:

| Warning | How the plan absorbs it |
|---|---|
| 1. Do not start designing without calculating | Gripping force (V2) sits in M1, **before** CAD starts in M2. The design freeze separates them cleanly. |
| 2. Do not start the FEM too late | The FEM (E5) sits in M3 and is done **before** the interim report, not after. |
| 3. Do not underestimate the drawings | Drawing + BOM (E3) starts in December, not January. Two full weeks. |
| 4. Do not start the documentation in the last week | Runs from M1 on. Every package is written **when** it is finished. January is merging only. |
| 5. Take the interim report seriously | Its own milestone with a content list and 4 days of buffer. |

---

## 4. Simulation: MuJoCo over Gazebo

Gazebo would be right if we were driving a real ABB controller through ROS 2
and MoveIt - sensor simulation, plugins, a whole cell. We do not need that. The
assignment asks for "visual representation incl. collision check of the working
sequence": kinematics, collision, and a video.

MuJoCo is the better fit:

- **Runs without ROS**, straight from Python - no stack that eats a week of setup
- **Contact forces** are readable. We can show that the part actually does not
  slip under the calculated gripping force instead of asserting it. That is a
  real argument in the report.
- **Headless rendering** in good quality → images for the report and the video
  come out of the same run
- **It already runs.** `sim/aufstellung.py` and `sim/ablauf.py` work, including
  Levenberg-Marquardt IK with configuration continuity.

Against MuJoCo, Gazebo here has only downsides: heavier, ROS-dependent, on
Windows only through WSL, and no added value for what gets submitted.

---

## 5. Who does what

Two halves, each a closed story. Nobody gets leftovers.

### Elias - system & numerical proof

| # | Package | Result | Folder |
|---|---|---|---|
| E1 | **Robot selection & placement** | Selection calculation (reach, cycle time, payload), placement study for pedestal/base offset, reachability proof for all waypoints | `Elias/01_Robot` |
| E2 | **Gripper body** | Housing with dovetail slots for the racks, drive unit (servo + pinion), quick-change coupler, robot flange ISO 9409-1-50-4-M6 | `Elias/02_CAD_Body` |
| E3 | **Assembly drawing & BOM** | A3 drawing, item numbers, BOM split into bought and made parts | `Elias/03_Drawing` |
| E4 | **Motion, grip & collision study** | MuJoCo model of the full cell, path pick → place, grip analysis (contact forces, part does not slip), collision log incl. fence, sequence video | `Elias/04_Simulation` |
| E5 | **FEM of the jaws** | Mesh, boundary conditions, mesh convergence, stress and deflection; compared against Viktoriia's analytical result, **deviation ≤ 5 %** (lecture 17 Oct) | `Elias/05_FEM` |
| E7 | **Cell assembly (CAD)** | Gripper flanged onto the IRB 1600 on its 500 mm pedestal, safety fence, buffer belt, workpiece carrier, part B - one Onshape assembly, the same layout as the simulation | `Elias/06_Cell` |
| E6 | **Infrastructure** | Repo, Onshape document, API integration, parameter table, report template | `shared/` |

### Viktoriia - design & analytical proof

| # | Package | Result | Folder |
|---|---|---|---|
| V1 | **Gripping concept & sketch** | Variant comparison to VDI 2225 - rack and pinion (chosen), **four-bar (Robotiq-type parallel linkage)** as the strongest rejected variant, wedge hook, screw, and vacuum as a reasoned rejection. Dimensioned concept sketch | `Viktoriia/01_Concept` |
| V2 | **Gripping force & drive sizing** | Force balance (weight + acceleration + safety), friction coefficient with a source, surface pressure on PA 6, **gear sizing (tooth root, flank pressure)**, tilting moment on the dovetail guide, holding on power failure (§ 8, point 4), motor selection with margin | `Viktoriia/02_Sizing` |
| V3 | **Jaws & soft pads (CAD)** | Parametric jaws in Onshape, pad pocket, contour matched to part B | `Viktoriia/03_CAD_Jaws` |
| V4 | **Analytical strength proof** | Bending stress and deflection of the jaw by hand, assumptions and sources documented - the reference the FEM has to meet within 5 % | `Viktoriia/04_Analytical` |
| V5 | **3D-printed check** | Print the jaw, measure deflection under a defined load, hold it against Elias' FEM | `Viktoriia/05_Print_Check` |

**Why this cut:** Viktoriia designs the jaws and works them out by hand. Elias
recomputes the same jaws numerically. Two people check the same part on two
independent paths - exactly what the assignment asks for, and a discrepancy
shows up immediately instead of hiding in one pair of hands.

The printed check (V5) sits with Viktoriia for the same reason: whoever ran the
FEM should not confirm it themselves.

The interface between the two halves is a single surface: the **mounting plane
carrier ↔ jaw**. It is fixed in week 41 and not touched afterwards.

### Together

- **Documentation** - each writes their own chapters, mutual review
- **Cross-review** before every milestone: Elias re-checks Viktoriia's
  calculations, Viktoriia checks Elias' model for manufacturability
- **Submission ZIP** in the structure from Fig. 2

---

## 6. The 3D-printing idea

Nobody asks for this - but it is cheap and it lifts the work noticeably.

We print the jaw in PLA/PETG at 1:1, clamp it, hang a defined load on the free
end and **measure the deflection**. Then we recompute the same geometry with the
print material's Young's modulus and compare three numbers: analytical, FEM,
measured.

What it buys:

- The analytical/numerical comparison turns from a box to tick into a real
  result - with a third, physical data point
- It shows immediately whether the clamping in the FEM model was assumed too
  stiff. That is the classic FEM error and we can demonstrate it instead of
  guessing.
- The printed jaw can be shown. A real jaw in your hand beats any rendering.

**For honesty in the report:** we print PLA, we build EN AW-7075. The test
validates the *model* (boundary conditions, load introduction, mesh), not the
part. That is exactly how we write it.

---

## 7. Timeline

### The two hard dates

We are in different course groups (CIS: Viktoriia BB-1 / MRE-3B1, Elias BB-2 /
MRE-3B2) but deliver together. Decided 19 Oct 2026: we both present in **BB-2**,
the **later** dates - one extra week before each:

| | Date | Note |
|---|---|---|
| 🔴 **Interim report** | **Thu 3 Dec 2026, 17:50, on site** | BB-2 instead of BB-1 (26 Nov) |
| 🔴 **Final review** | **Fri 22 Jan 2027, 17:50, on site** | BB-2 instead of BB-1 (15 Jan) |

Only one person per group has to be present. Mail to Saliger sent 19 Oct,
confirmed by her on 19 Oct.

### Course dates set the rhythm (BB-1 lectures, BB-2 presentation dates)

We put our packages behind the lectures, not in front of them:

| Date | Topic | What it means for us |
|---|---|---|
| Thu 17 Sep | Introduction to Onshape | CAD starts **after** this, not before |
| Sat 19 Sep | Modelling and drawing | Modelling skills in place |
| Tue 22 Sep | BOM, sheet metal, MDB | BOM logic for E3 |
| Sat 24 Oct | Assemblies, data exchange | Lands right inside the CAD phase |
| Wed 4 Nov | Digital validation, simulation | Directly before the simulation phase |
| Wed 2 Dec | CAE-CAD-CAP-CAM process chain | - |
| **Thu 3 Dec** | **Interim report** (BB-2) | 🔴 |
| **Fri 22 Jan** | **Final review, Q&A** (BB-2) | 🔴 |

### Milestones

| Milestone | Deadline | Result | Who |
|---|---|---|---|
| **M0 - Kick-off** | Sun 20 Sep | Plan agreed, roles fixed, Onshape + repo up, open questions sent | both |
| **M1 - Concept fixed** | Sun 4 Oct | Robot chosen (E1), gripping concept + sketch (V1), force calculated (V2) → **design freeze** | both |
| **M2 - CAD closes** | Sun 1 Nov | Jaws (V3) and body (E2) designed separately, assembly closes without collisions, interface frozen | both |
| **M3 - Proofs** | Sun 22 Nov | Analytical done (V4), FEM computed incl. mesh convergence (E5), both paths compared, deviation ≤ 5 % | both |
| 🔴 **Interim report** | **Thu 3 Dec** | Presentation: concept, sizing, CAD, first proofs | both |
| **M4 - Drawing + simulation + print** | Sun 20 Dec | Cell assembly (E7), assembly drawing + BOM (E3), MuJoCo sequence + grip analysis + collision log + video (E4), jaw printed and measured (V5) | parallel |
| *Christmas break* | 21 Dec - 3 Jan | - | - |
| **M5 - Merge** | Sun 11 Jan | Documentation merged, cross-review, ZIP packed | both |
| 🔴 **Final review** | **Fri 22 Jan** | ZIP uploaded, on-site date | both |

**Buffer:** 11 days before the interim report, 11 days before the final review
(milestones kept, the BB-2 week went into buffer). The FEM chain still sits **before** the interim report and
not after - if something goes wrong there we still have December.

**The drawing sits in December on purpose, not January.** Warning 3. Two weeks
for drawing and BOM is realistic, not generous.

**Documentation runs from M1.** Every package is written when it is done, not at
the end. M5 is merging and proofreading - no writing. That is warning 4, and it
is worth 10 points.

**Standing appointment:** a short sync every week, 20 minutes, status +
blockers.

---

## 8. Open points

These need clearing early, because they get expensive later:

0. ~~**Split course groups.**~~ **Cleared 19 Oct 2026.** Viktoriia is in BB-1,
   Elias in BB-2 (CIS). Both run on BB-2 (interim report 3 Dec, final review
   22 Jan); Alexandra Saliger confirmed by mail on 19 Oct. Per the introduction
   slides **at least one person per group** has to attend. Still open: the
   **75 % attendance duty** is counted in your own group.
1. **Group number.** `task.md` says group 10, Tab. 1 of the assignment only
   knows groups 1-8 (part B = group 2). Ask the course leader before any
   document is printed with a number in its header.
2. **Placement orientation on the workpiece carrier.** Per the assignment this
   is defined at the assignment handout. Currently assumed as a 90° rotation. A
   different orientation changes the path and possibly the grip direction.
   **This is the only open point that can still break the design freeze.**
3. **FDM printer.** Clear availability and material (lab or private).
4. **Holding on power failure.** A rack and pinion is back-drivable; the
   pinion has to resist **0.29 N·m** (times safety) unpowered. In the series
   design a self-locking stage or a holding brake in the drive solves it -
   choose which in V2.
5. **Tilting moment on the jaw guide.** The grip force acts below the rack,
   the dovetail slot takes the moment. Size guide length and clearance so the
   jaws stay parallel under 14.3 N and under the servo's maximum force.
6. **Onshape FEM.** Check whether the education plan unlocks Simulation. If not,
   the existing CalculiX chain steps in - equivalent result, more manual work.
7. **Onshape licence.** The free plan makes documents public. For student work
   that is usually fine, but it should be a conscious choice - the education
   plan is free and private.
8. **Language of the submitted report.** The course, the assignment and the
   slides are German. Our working language is English. Confirm with Saliger
   which language the submitted documentation should be in - this does not
   affect the repo either way.

---

## 9. How we work in the repo

```
PDE/
├─ Elias/            ← Elias commits only here
├─ Viktoriia/        ← Viktoriia commits only here
├─ shared/           ← parameters, templates, submission ZIP
│  ├─ submission/    ← final structure per Fig. 2 - German names, prescribed
│  └─ references/
├─ cad/ fem/ sim/    ← existing script chain (preliminary work, kept as reference)
└─ PLAN.md
```

**Rule:** each of us commits only inside our own folder. Then there are no merge
conflicts. Merging happens exclusively in `shared/submission/`, at the end,
deliberately and together.

Commit messages in English, one line, what changed and why.

**`shared/submission/` keeps its German folder names on purpose.** The
assignment says the structure is to be used *exclusively* as given, so those
four names are copied verbatim and must not be translated:

```
1. Dokumentation und Berechnung
2. Konstruktionsdaten (3D-CAD-Daten)
3. Baugruppenzeichnung (Greifer)
4. Kollisionskontrolle und Bewegungsanalyse
```

---

## 10. What Viktoriia should say about this

This is a **proposal**, not a decision. Feedback wanted specifically on:

- Does the split work? If you would rather do CAD than the analytical part (or
  the other way round), we swap - the packages are cut so V3↔E2 and V4↔E5 are
  exchangeable.
- Onshape fine, or would you rather work in something else? (SolidWorks,
  Fusion, Inventor - then we only need to settle the interface via STEP.)
- Is the timeline too tight or too loose?
- The FEM moved to Elias on purpose so your side does not overflow. You compute
  analytically, he computes numerically. If you would rather run the FEM
  yourself, we swap back.
