# Explainer film - shot list (draft 22 Nov 2026)

Job: show how the gripper works and why it holds. Calm, technical: MuJoCo / SimReady renders, white diagram cards,
the report's colours. No music or a quiet bed, optional voice-over. 16:9, 1920 x 1080, 30 fps, target 80 - 90 s.
Every number on screen comes from a file in the repo (column "source"); a number that changes in the repo is
regenerated, never typed into the film.

Overlay kit (same in every scene): force arrows on the pads (length = N, label in N), one friction-use bar per pad
(0 - 1, green -> amber at 0.8 -> red at 1), a step strip at the bottom (approach - close - lift - transfer - lower -
open), captions top left in one line, a numbered feature card on the right when a design feature is named.

| # | Time | Picture | Overlay / on-screen text | Source | Footage |
|---|---|---|---|---|---|
| 1 | 0 - 8 s | **Task and cell.** Top view of the cell running at takt, then a slow push to the robot. | "Part B, 0.602 kg, from the buffer belt (1000 mm) to the workpiece carrier (1300 mm)" · "5 s takt, enclosure 1200 x 2100 mm" · cell outline and the two heights dimensioned | `06_Cell/cell_params.py`, `figures/cell_takt.mp4` | exists, re-render with dimension overlay |
| 2 | 8 - 16 s | **Where it is held.** Close on the jaws around part B, the part turns once; the grip faces light up, then the centre of mass drops as a dot under the pads. | "Grip on the two flat side faces" · feature card 1: wide T-jaw, NBR pads with oil-drain grooves · "centre of mass under the pads: no tilting moment" | `02_CAD_Body/grip_check.py` | new MuJoCo close-up (static) |
| 3 | 16 - 30 s | **Grip force.** White card, the formula builds term by term, then the number drops onto the pad arrows in the sim. | F = S · m · (g + a) / (2 µ) → µ 0.2 (oily, worst case) · S 2 · a 4.96 m/s² (real path, not the 2.05 of a straight line) → **44.5 N per jaw** · then the twist of the part: friction limit surface (shear + twist) | V2 §§ 1, 2a, `grip_results.json` | card: new; sim arrows: from `viz.py` |
| 4 | 30 - 40 s | **The mechanism.** Section through the gripper (hero film shot 5 material, but in the sim look): motor -> brake -> gearbox -> bevel pair -> pinion -> two racks; a line follows the force flow. Jaws close by 10 mm. | "Motor current = grip force" · "one pinion, two racks: both jaws move together, the part is always centred" · "57° pinion for 10 mm, i 7.07" · feature cards 2 (current-limited stepper), 3 (zero-backlash brake) | V1, V2 § 10, `gripper_params.py` | Blender section render, flat light (new) |
| 5 | 40 - 52 s | **Closing.** Split screen, slow motion: left a direct fast close, right the built tolerant two-stage close; force curve below each. Then the right side again with the part 5 mm off centre: it is pushed slowly to the middle. | left peak **160 N**, right **45 N** · "fast to just before contact, then slow" · "5 mm off centre: pushed to the middle, peak 45.5 N" | `closing_time.py`, `closing_results.json`, `figures/closing_*.mp4` | exists, overlays + split new |
| 6 | 52 - 62 s | **Transfer and e-stop.** The real ABB MoveJ path with the via, friction bars live; at the worst instant a category 0 e-stop, the axes brake on their own, the robot leaves the path. | "peak friction use 0.72 at the via corner, held" · e-stop: "22.3 m/s² on the part, needs 34.3 N, set 44.5 N: held, S 1.30" | `validate_grip.py`, `e_stop.py`, `grip_results.json`, `e_stop.json` | `figures/cycle_estop.mp4` exists, bars new |
| 7 | 62 - 70 s | **Power failure.** Power off with the part in the jaws, slowed 100x: without a brake the preload is gone in 2 ms and the part drops; with the permanent-magnet brake the 44.5 N stay. | "preload gone in 2.1 ms" · "brake engages before the motor lets go: brake-first shutdown" · "brake free of play: allowed play < 1.1°" | `power_fail.py`, `brake_play.py`, `*.json` | new render from `power_fail.py` |
| 8 | 70 - 78 s | **Strength.** FEM of the jaw and the gear train, stress colour map turning slowly; tilt of the jaw under load. | load cases and safety factors as in the report (LC4: 82 m/s² ABB tool load, S >= 12.5) · V4 vs FEM within 5 % | `05_FEM` (`fem_jaw.py`, `v4_fem_results.json`, `tilt_check.py`) | FEM figures exist, turn render new |
| 9 | 78 - 90 s | **Full cycle with collision check.** One complete cycle in the cell at real speed, the step strip runs along; the closest approach to each obstacle flashes with its distance. Ends on the cycle time against the takt. | min. clearance: belt 33 mm · carrier 33 mm · chain 63 mm · fence 99 mm · "no contact" · **4.48 s of 5 s** | `e4_cycle.py`, `clearance.json`, `e4_cycle.json` | sim render exists, clearance flashes new |

Interim presentation (3 Dec): scenes 1, 3, 5 and 9 (about 45 s), as planned in `RENDER_PLAN.md`. The rest by the
final review (22 Jan).

## Decisions (Elias, 22 Nov)

- On-screen text in English, captions only, no voice-over.
- Overlays and edit in HyperFrames (HTML over the sim videos).
- Proposed: the diagram cards (formula build-up in scene 3, force-flow schematic in 4, force-time curves in 5 and 7,
  friction limit surface) in Manim CE (3Blue1Brown style), rendered as clips and placed in HyperFrames; the sim
  footage stays MuJoCo / Blender.
- Scene 2 shows the wide T-jaw, which still waits for Viktoriia's answer; with her original jaw only the pad
  picture changes.
