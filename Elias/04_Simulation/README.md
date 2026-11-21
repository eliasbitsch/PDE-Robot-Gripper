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
| `power_fail.py` | power loss with the part in the jaws: detent only until the brake engages |
| `viz.py` | slow-motion video with four cameras, contact points and forces, clearance check (convex hulls) |
| `cell_video.py` | running cell at the 5 s takt: parts queue on the buffer line, the robot loads chain 2 and chain 1 in turn (chain 1 = mirrored E4 path), the carriers index into the machine; kinematic replay, [`figures/cell_takt.mp4`](figures/cell_takt.mp4) |
| `validation_notebook.py` | everything live: `marimo edit validation_notebook.py` |

**Model.** Robot joints follow the path through stiff position control with gravity compensation (the robot's own
dynamics are not the question here). The two jaws are slide joints driven together through a tendon (pinion and two
racks) by one position actuator whose force limit is 2 × the force per jaw: the current-limited motor. Rotor,
gearbox, brake and bevel pair sit on the slides as armature (3.2 kg reflected on the jaw coordinate, `drive.py`). NBR pad on
PA 6 with μ = 0.2 (oily part), elliptic friction cone, noslip solver. Part B as a free body, 0.602 kg, contact shape
from two boxes (body with the flat grip faces, lug) because its convex hull would touch the left pad.

**Path.** Pick on the belt, lift 60 mm in 0.4 s, transfer to the carrier on the ABB MoveJ path through the via
(see "Enclosure and transfer path"), lower 60 mm. Axis 1 turns about 160°, so the TCP sees up to 5 m/s², not the 2.05 m/s²
of a straight 1.8 m line that the hand calculation assumes. E-stop (category 0) at the worst instant found by
`e_stop.py`: each axis brakes on its own with its deceleration from the ABB product specification (3HAC070393
rev. M, p. 91) and the robot leaves the path; 14.2 m/s² at the TCP. The brake locks the drive with the preload kept.

**Results, 21 Nov 2026** (`out/grip_results.json`, `figures/grip_slip.png`), wide T-jaw pad, on the transfer
through the via (the 1200 mm enclosure, see below) at the time law with TCP acceleration <= 4.96 m/s2 (the
simulated robot following it shows peaks of 5.35 m/s2: servo response at the corner path; the friction use includes it). Friction use = |vector sum
of the pad friction forces| / (μ ΣF_n), median over the moving phases and 50 ms peak.

| Case | Force per jaw | friction use median / peak | part movement (raw / beyond creep base) | |
|---|---|---|---|---|
| required (hand calc., 2.05 m/s²) | 35.7 N | 0.41 / 0.80 | 2.18 / +0.02 mm | held |
| no safety (F / S) | 17.9 N | - / 1.00 | dropped | expected: sized for 2.05 m/s², the path needs 4.96 |
| **setpoint** (path 4.96 m/s², WorldAccLim) | 44.5 N | 0.33 / 0.72 (S_eff 1.39) | 2.17 / +0.02 mm | held |
| e-stop at required, category 0, axes brake on their own | 35.7 N | 0.41 / 0.80 | 0.58 / +0.00 mm | held |
| e-stop at setpoint | 44.5 N | 0.33 / 0.72 | 0.58 / +0.00 mm | held |

## Emergency stop anywhere in the cycle (`e_stop.py`, `out/e_stop.json`, `figures/e_stop.png`)

Review 21 Nov: the e-stop above used to brake along the path (7.0 m/s²), which is milder than a real category 0
stop, and the ABB stop data (p. 91) hold for max speed, arm stretched and max load, i.e. the *weakest*
deceleration; with our half-nominal load and the arm pulled in the robot brakes harder. `e_stop.py` starts a
category 0 stop every 10 ms of the loaded cycle on both chains: each axis decelerates at a constant rate until it
stands, the robot leaves the path; part B's acceleration (gravity included) and the gripper's angular acceleration
go through the pad's friction limit surface (shear + twist, `02_CAD_Body/grip_check.py`, shift of normal force
between the jaws included). Normal force per jaw needed at S = 1:

| Deceleration of the axes | worst instant | part acceleration incl. g | needed per jaw | setpoint 44.5 N |
|---|---|---|---|---|
| ABB stop data (p. 91) | loaded transfer, t = 0.95 s | 22.3 m/s² | 34.3 N | held, S 1.30 |
| twice the ABB rates (sensitivity) | start of the transfer, t = 0.58 s | 42.7 m/s² | 57.5 N | slips |
| ABB design value 82 m/s² (p. 54, nominal load) | - | 82 m/s² | ~110 N | slips |
| ABB stop data, grooved pad µ 0.3 (expected, V5) | t = 0.55 s | 23.7 m/s² | 24.2 N | held, S 1.8 |
| twice the ABB rates, grooved pad µ 0.3 | t = 0.58 s | 42.7 m/s² | 41.5 N | held, S 1.07 |

Decision 21 Nov: more friction, not more force. More force would mean a bigger motor (60.5 N per jaw at full current); a higher friction coefficient costs nothing in loads and weight. The pads get vertical oil-drain grooves;
the design proof keeps µ = 0.2, the grooved pad's expected µ ≥ 0.3 (oil-resistant NBR gripper pads 0.3 ... 0.4,
HKI) is the reserve for hard stops and is measured in V5 (`Viktoriia/05_Print_Check/friction_test.md`). The 82 m/s²
case is not held by any reasonable friction grip (it would need 123 N at µ 0.2); there the closed enclosure keeps a
slipping part inside the cell, and the brake keeps the jaws closed.

The grip force does not rise in an e-stop (the brake holds the preload; the part's inertia goes into the pad
friction), so the drive sees no extra load. Bevel pair, key and overloads: `02_CAD_Body/drive_margin.py`
(review 1.7: m 1.5 pair since 21 Nov). The 82 m/s² are ABB's value for tool strength: FEM load case LC4 uses them (S ≥ 12.5).

**Creep base.** On the via path the part creeps 2.3-2.4 mm along the pads whatever the grip force: 25, 35.7, 44.5,
48.5 and 100 N per jaw all give the same movement, a finer time step gives more (1 ms 1.72, 0.5 ms 2.38, 0.25 ms
2.70 mm), a 100 x stiffer contact impedance only 1.72 mm, the friction cone (elliptic / pyramidal, condim 3/4/6)
and noslip iterations change nothing essential. Real slip would shrink with the grip force, so this is the soft
contact model, not the grip. Slip is therefore measured beyond a reference run at 100 N per jaw on the same path
(`creep_base`), limit 0.5 mm. Two pad springs (NBR shear, `build_model.py`) now share the tangential load between
the pads as the rubber does: before, one pad sat at its friction limit while the other pulled the other way.

Hand calculation at each instant (`validate_grip.quasi_static`, `friction_check`, `out/friction_check.json`): the
pads must carry m (a - g) in their plane, with a from the simulated part motion. Median friction use over the
transfer, quasi-static vs simulated agree within 0.1 %; note that the quasi-static value uses the simulated
normal force and part acceleration, so this checks the friction bookkeeping, not V2's inputs. Short peaks on the via path: simulation 0.72 ... 0.81 against 0.47 ... 0.58
quasi-static; the fast axis 1 turn on the small radius adds dynamics the hand formula does not contain. The setpoint
covers it (peak 0.72).

## Enclosure and transfer path (`clearance.py`, `sim.transfer_path`, `sim.TimeLaw`)

Abb. 1: the enclosure is 1200 mm wide (y = ±600) and 2100 mm high. A plain joint swing pick -> place takes the
forearm 318 mm through the fence. One via in the middle of the swing, where the arm points at the fence (axis 1 at
90°, wrist pulled in to 200 mm from axis 1 at 1450 mm height), passed in a 200 mm corner path without a stop.

**The path is the one the ABB controller runs** (review 1.5, `abb_path.py`; RobotStudio is not used): MoveJ moves
straight lines in joint space between the targets and blends them in the zone (corner path from where the TCP comes
within the zone radius of the via to where it leaves it, quadratic Bezier in joint space with the via as control
point, tangent to both lines, no axis overshoot). The earlier model interpolated a PCHIP curve through the via
(R 350 mm, Z 1700 mm); on the real MoveJ line to that via the forearm cut **4.9 mm into the fence**, the curve had
hidden it. New via search on the MoveJ path (A1 60 ... 120°, R 200 ... 500 mm, Z 1250 ... 1800 mm, zone 100 /
200 mm, both chains): R 200, Z 1450, z200 gives 101.7 mm to the fence on both chains.

**Acceleration as programmed, not assumed.** The 4.96 m/s² the grip force is sized for are a cap in the robot
program, not a property of MoveJ: the loaded moves run with `WorldAccLim \On := 4.96;` (acceleration of tool and
payload in the world frame, tangential and centripetal; `PathAccLim` would only cap the tangential part), the empty
way back with `WorldAccLim \Off`. The time law (`sim.TimeLaw`) stands for this: fastest motion with the TCP
acceleration below 4.96 m/s² loaded / 15 m/s² empty and every axis below 80 % of its speed. Checked on the replayed
joint path (part B's centre of mass, Savitzky-Golay, 2 ms): 4.96 m/s² in lift and lower, 5.19 m/s² peak in the
transfer where the corner path starts (the curvature jumps there), 15.0 m/s² empty. A bug in the replay was found on
the way: u(t) was interpolated linearly between the time law's steps (up to 40 ms long at the start), the speed
jumped and the replay showed 8.9 m/s² loaded (the "6.4 m/s² servo response" in MuJoCo was mostly this). Fixed with
constant acceleration inside each step; MuJoCo now shows 5.35 m/s².

```
WorldAccLim \On := 4.96;
MoveJ pPickUp,   v1000, fine, tGripper;
MoveJ pVia2,     v1000, z200, tGripper;
MoveJ pPlaceUp2, v1000, fine, tGripper;
WorldAccLim \Off;
```

## E4: the full cycle measured (`e4_cycle.py`, `out/e4_cycle.json`, `out/e4_collision_log[_chain1].csv`)

The robot loads both chains in turn: chain 2 (y = +300) through the via at axis 1 = +90°, chain 1 (y = -300) through
its mirror image at -90°. At the place, axis 6 turns by -20.6° (chain 2) / +20.6° (chain 1) so part B lands square
on the carrier (yaw 180°); with axis 6 at 0 it would land 20.6° skewed (`sim.q_place`). Both cycles are timed and
checked; they are identical in time.

| Phase | Duration |
|---|---|
| close on the belt (two-stage, grip force stands after 0.263 s) | 0.30 s |
| lift 60 mm (loaded) | 0.22 s |
| transfer through the via (loaded) | 1.86 s |
| lower onto the carrier | 0.22 s |
| open | 0.20 s |
| lift (empty) | 0.13 s |
| transfer back (empty) | 1.33 s |
| lower around the next part | 0.13 s |
| **cycle** | **4.38 s of 5.0 s** |

Collision log every 10 ms, robot links 2-6 and the gripper against every obstacle (convex hulls, on the safe side):
no contact anywhere, on either chain. Closest: fence 101.7 mm (both chains, loaded way and empty way back near the
via), production line 153 mm, chain / carrier on the other chain 423 mm, pedestal 434 mm, belt 93 mm while carrying
the part off; the 33 mm to belt and carrier are the jaws at the part while picking and placing, 63 mm the jaw to the
near chain. Axes: smallest margin to a limit 16.1° (axis 3, arm pulled in at the via) and 20.6° (axis 1 at the
place position), speeds at most 80 % of the data sheet (axes 1 and 2).

![clearance](figures/e4_clearance.png)

![clearance chain 1](figures/e4_clearance_chain1.png)

![axes](figures/e4_axes.png)

![axes chain 1](figures/e4_axes_chain1.png)

## Closing time (`closing_time.py`, `out/closing_results.json`, `figures/closing.png`)

Requirement from the hand calculation V2: close 8 mm per jaw and have the grip force within 0.3 s. The drive runs
position controlled with the current limit as grip force (setpoint 44.5 N per jaw). Motor curve from the datasheet's electrical
values at 24 V (phasor sum of back-EMF and winding impedance), 70 % usable in closed loop; the Nanotec curve still
has to be laid over it.

| Profile | contact | grip force within 5 % | peak pad force | motor peak |
|---|---|---|---|---|
| direct, trapezoid 0.25 s | 177 ms at 52 mm/s | 195 ms | 66.5 N (1.5 ×) | 324 rpm, 0.018 of 0.136 N·m |
| direct, trapezoid 0.12 s | 86 ms at 111 mm/s | 101 ms | 159.8 N (3.6 ×) | 675 rpm, 0.030 of 0.136 N·m |
| two-stage: 0.12 s to 1 mm before the part, then 10 mm/s | 173 ms at 10 mm/s | 266 ms | 45.9 N (1.0 ×) | 591 rpm, 0.028 of 0.136 N·m |

- Torque is not the limit: the motor could cover the 8 mm in 45 ms. The motor uses at most a quarter of its torque.
- The limit is the impact. Rotor, gearbox and brake act like 3.2 kg on the jaws, seven times the jaws themselves
  (0.47 kg), and the current limit cannot stop this inertia before the pads hit: driving straight in at 50 mm/s
  gives 67 N, more than the FEM's full-current case (63.6 N); the undamped estimate with a 3 mm NBR pad is even
  higher (`impact_estimate`, no damping, upper bound).
- **Recommendation:** two-stage profile, fast to 1 mm before the part and the last millimetre at 10 mm/s
  (NanoJ program or two position records in the CL4-E). Meets 0.3 s with 34 ms to spare, no force overshoot.
- The MuJoCo contact stiffness is not calibrated to the pad; the peaks are a comparison between the profiles,
  the order of magnitude is backed by the estimate.

## Videos

`viz.py` renders any run in slow motion with four cameras (cell; both jaws with the housing see-through; pad close-up;
back view on the lug side), contact points and forces split into normal and friction, and a clearance check between
the part and the jaws, pad plates, holders, cover and housing, and between gripper / robot and belt, carrier and
pedestal (convex hulls, so conservative). Colliding parts turn red; the HUD shows pad forces, friction use and
part movement. [`figures/closing_direct_120ms.mp4`](figures/closing_direct_120ms.mp4), [`figures/closing_two-stage_120ms.mp4`](figures/closing_two-stage_120ms.mp4) at 0.05 ×, [`figures/cycle_estop.mp4`](figures/cycle_estop.mp4) at 0.25 ×; in the notebook with your own values.

## Power failure (`power_fail.py`, `out/power_fail.json`, `figures/power_fail.png`)

The robot holds the part above the belt at the setpoint (44.5 N per jaw), then the drive loses power: only the
stepper's detent torque is left (6 mN·m → 1.9 N per jaw), until the spring-applied brake locks the drive.

| brake engages after | pad force left | part |
|---|---|---|
| 0 ms (drive holds until the brake is closed) | 44.5 N | held |
| 2, 5, 10, 20, 50 ms | 0 N | **dropped** (58 mm, onto the belt) |

The part needs 14.8 N per jaw against its weight; the elastic preload of pads, jaws and slides opens the jaws within
about 2 ms. No spring-applied brake is that fast, so the brake alone does not hold the part: the controller must keep
the motor powered until the brake has closed. Remedy in D11 (`02_CAD_Body/CAD_PLAN.md`): 24 V buffer module (PULS
UF20.241) and a brake-first shutdown in the NanoJ program. Gearbox friction is left out (conservative).

![power failure](figures/power_fail.png)

## Looks of the cell (NVIDIA SimReady, CC-BY-4.0)

Belt, fence and floor are NVIDIA SimReady assets (`06_Cell/simready`: `fetch.py` downloads them into the
git-ignored `06_Cell/vendor/simready`, `usd2obj.py` converts them in Blender, `derived.py` adapts them, `layout.py`
scales and places them onto the exact cell geometry). They are visual only: the collision check and the physics use
the plain boxes from `cell_params.py` (Abb. 1), which stay underneath. Without the downloaded assets the model falls
back to the boxes. The buffer belt is the flat belt `warehouse_conveyorBelt_a06`; the catalogue only has chain
modules with legs at one end, so `derived.py` mirrors the leg pair and cuts the posts at the belt top. Fence
`wire_machineGuard_a04`, floor `environment_floor_a01`, © NVIDIA, https://huggingface.co/datasets/nvidia/simready-assets.
Own visual parts (`cell_parts.py`): a CNC machining centre (finishes slot and bore of part B) closes the place
side and takes both carrier chains through inlets; the chains run as in Abb. 1 (carriers fixed to the chain, top
run into the machine, return underneath, end sprocket). The buffer line housing closes the pick side, the belt leaves
it through an 850 mm hood (ISO 13857: no reach through the 110 mm slot). Mesh panels fill the gaps, so the
enclosure is closed all round; the door with guard locking sits on the operator side next to the switch cabinet.
E4 and the grip validation use the same poses (`sim.q_place`). The switch cabinet with the OmniCore C30 on top (449 x 443 x 170 mm per the ABB brochure) is modelled in
`derived.py` with drawn front textures (C30 front, door with warning sign, labels, pilot lights, E-stop, main switch).
