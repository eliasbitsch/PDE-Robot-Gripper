# Render plan - two films

Group 10, part B - Bitsch / Ovdiienko. Status 21 Nov 2026.

Two separate films, because they have different jobs:

| | Hero film | Explainer film |
|---|---|---|
| Job | make people want to look: the gripper as a product | show how it works and why it holds |
| Look | product commercial: black void, glossy reflections, glow, slow camera moves, reveals; a white lab for the self-assembly | neutral, technical, calm; simulation footage and diagrams |
| Numbers | only on the end card | everywhere: forces, friction, safety factors |
| Length | 60 - 75 s (16:9), plus a 9:16 cut of 20 - 30 s | 60 - 90 s |
| Tool | Blender, Cycles (EEVEE for drafts) | MuJoCo renders (exist) + overlays (HyperFrames or Blender compositor) |
| Use | presentation opener, LinkedIn | report, "visual sequence incl. collision check" (assignment), final review |

Mixing both would weaken both: the explainer needs stillness and arrows, the hero film needs none of that.

---

## 0. Decision 26 Nov (Elias)

- **Hero film (entertainment):** flight reveal -> self-assembly in the white lab -> exploded view -> robot (wake-up,
  wave, pick-and-place getting faster). The shot list below is the original plan; these four scenes replace it.
- **Keynote structure (Elias, 26 Nov):** the hero film is split in two. Opening (flight + assembly, ~51 s, ends when
  the lab lights go out) starts the presentation; then design and calculations are presented; the closing film
  (exploded view + robot, ~44 s, comes up out of the black) ends it. Each part gets its own sound mix.
- **Extra (teaching):** the explainer film.
- **Cell field:** 100 cells at random phase of the cycle, each with one sample of the robustness study (part B up to
  +-5 mm off on the belt, centred by the jaws), coloured by offset, legend in mm (`04_Simulation/cell_grid.py`).
- Made but not in the film: grid (knolling), transformer build, X-ray wipe (`hero_v1.py -- grid|transform|xray`);
  material for the 9:16 cut.
- Sound is a separate track (`sound_shots.py`, cinematic only, effects 6 dB under the music).
- Opening sound (Elias, 26 Nov): the flight reveal gets our private mix (drums intro, melody, choir in the last
  third; `out/private/pixel_music.py` - ripped ad music, only for the class presentation, never publish); the
  assembly has no music, effects only (`SFX_DB=0 sound_shots.py assemble`). For anything public: licensed music.

## 1. Hero film - shot list

Assets come straight from the CAD (`02_CAD_Body/out/gripper.step`, one solid per BOM position, names kept), the robot
from the ABB STEP links (`06_Cell/vendor`), motion from our own data (assembly order, E4 joint trajectory), so every
frame shows the real design.

| # | Time | Shot | Camera | Light / look | Motion and feel |
|---|---|---|---|---|---|
| 1 | 0 - 4 s | **Cold open.** Black. A thin line of light runs along the edge of the hood. | locked, macro | one moving strip light, everything else black | the light sweep is the reveal |
| 2 | 4 - 12 s | **Reveal.** The gripper floats above a black glossy floor, jaws slowly opening. | slow 90° orbit, slight push-in | key + two rim lights, soft glow on the anodised edges, mirror image in the floor | floats with a slow 2 mm bob, never fully still |
| 3 | 12 - 20 s | **Macro details.** Rack teeth, the KHK spiral bevel pair, the oil grooves in the NBR pad, the ISO flange. | close-ups, shallow depth of field, rack focus from tooth to tooth | glints running over the steel | each detail turns a few degrees |
| 4 | 20 - 38 s | **Self-assembly in a white lab.** Cut to white (exposure flash). The housing hangs in the air in a clean, softly lit lab; the parts fly in one after another in the real assembly order (`parts/demo_seq.py`, `STEP_TEXT`): rail, drive unit, pinion shaft with bearings and bevel wheel, hood, carriages and slides, racks, holders, cover, jaws, pads. | slow orbit around the growing assembly, cuts on the snaps | bright, soft, ambient occlusion; the parts stay metallic | **spring motion, not linear:** every part arrives with a small overshoot (5 - 8 %) and settles in 0.2 - 0.3 s, turns a few degrees on the way, a short scale pulse and a click at the moment it seats; screws spin in. The whole assembly bounces a little when a big part lands. |
| 5 | 38 - 48 s | **Mechanism focus.** The hood turns transparent / a section plane slides through: motor, brake, gearbox, bevel pair, pinion, two racks. The pinion turns 57°, the racks run in opposite directions, the jaws close on part B. | slow push into the section, then top view on pinion and racks | back to the dark look; a glowing line follows the force flow motor → gearbox → bevel → pinion → racks → pads | real kinematics (`gripper_params`: i 7.07, 57° pinion for 10 mm) |
| 5b | (with 5) | **Exploded view, partial.** From the closed gripper the parts lift off along their mounting axes, outer ones first (cover, hood, window, screws, housing halves), and stop as soon as the drive train is free: motor, brake, gearbox, bevel pair, pinion, racks. Then the motor turns forward and back a few times; pinion, racks and jaws follow by the same drivers; then everything closes up again. | slow orbit, slight push-in when the mechanism is free | dark look as shot 2 | spring easing per part, staggered 0.1 - 0.15 s; motor 2 - 3 strokes, eased reversals; sound: low swell while it opens, servo whir and rack ticks per stroke, metal clack on each jaw stop, screw ratchets when the screws back out |
| 6 | 48 - 62 s | **The robot wakes up.** IRB 1300 in the dark cell, gripper on the flange. Axes come alive one by one from the base up, a small "breath", a short wave with axis 6, then one real pick-and-place cycle. | low, wide; follows the gripper during the transfer | dark cell with rim light, the fence as silhouettes | wake-up as keyframed motion; the cycle from the E4 trajectory (`e4_cycle.timeline`, 4.48 s) |
| 7 | 62 - 70 s | **End card.** Gripper on the black floor, slow turn. | locked | as shot 2 | text: gripper mass from gripper_params (2.95 kg) · 44.5 N · 4.48 s cycle (all read from the repo) · group 10 · names |

The 9:16 cut uses shots 2, 4 (shortened), 5 and 7.

### How the motion is built (Blender, Python)

- **Spring easing for the assembly:** one keyframe set per part, computed from a damped spring
  x(t) = 1 - e^(-ζωt)(cos ω_d t + ζω/ω_d sin ω_d t), ζ ≈ 0.45, settle 0.25 s, approach from the part's explode
  direction (outward along its mounting axis), start offsets staggered 0.35 - 0.6 s, small random tilt that damps out.
- **Snap:** at seat time a 3 % scale pulse over 4 frames and a light flash on the joint edge; sound cue on the same frame.
- **Mechanism:** driven by one property (jaw travel d); pinion angle, rack and jaw positions follow it by drivers, so
  the shot cannot show wrong kinematics.
- **Robot:** links parented along the IRB 1300 joint axes (`06_Cell/cell_params.py`), joint angles keyframed from the
  E4 trajectory exported as JSON (100 Hz); the wake-up is hand-keyed in front of it.
- **Materials:** housing and hood bead-blasted aluminium (light grey) with anodised dark accents, steel parts
  polished, NBR matte black, part B white PA 6 (slight subsurface). Black floor: glossy, roughness 0.05 - 0.1.
- **Render:** 1920 × 1080, 30 fps (60 fps for the assembly snaps if slow-mo is wanted), Cycles with denoising, motion
  blur on. About 2100 frames for 70 s; at 10 - 30 s per frame on the RTX 3080 Ti that is 6 - 17 h, so shots are
  rendered separately and drafted in EEVEE first.
- **Sound (direction from Elias, 22 Nov):** Apple-style: deep, bassy, epic (sub drone in D, low whooshes on camera
  moves, deep hits on reveals, a short stop-down before the big moment), and on metal the mechanical detail: clicks,
  rack ticks, servo whir, screw ratchets. Draft: `sound_flight.py` (earshot bank, synthesized; runs in earshot's venv,
  muxes `out/flight_preview_sound.mp4`). For the final: a real music track and real recordings for screws / ratchets
  (the bank has none), checked with earshot.
- **Sound (original note):** soft electronic bed; clicks and snaps for the assembly, a low whoosh for the camera moves, servo hum for
  the robot (licensed recordings, checked by listening with earshot before mixing).

---

## 2. Explainer film - outline

Neutral cell look (the MuJoCo / SimReady renders we already have), white background for diagrams, the same colours
as the report. Every claim on screen comes from a script in the repo.

| # | Content | Source in the repo |
|---|---|---|
| 1 | Task and cell: belt 1000 mm, carriers 1300 mm, 1800 mm apart, 5 s cycle, enclosure (Abb. 1) | `06_Cell`, `figures/cell_takt.mp4` |
| 2 | Where the part is held: the two side faces, the wide grooved NBR pads, part B's centre of mass under the pads | `02_CAD_Body/grip_check.py` |
| 3 | Grip force: F = S m (g + a) / (2 µ) with the numbers (µ 0.2 oily, S 2, 4.96 m/s²) → 44.5 N; then the twist of the part and the friction limit surface | V2 §§ 1, 2a |
| 4 | The mechanism: motor current = grip force; pinion, racks, centring | V1, V2 § 10 |
| 5 | Closing: why two stages; impact 123 N vs 45 N; the part 5 mm off centre is pushed slowly to the middle | `closing_time.py`, videos `closing_*.mp4` |
| 6 | E-stop anywhere in the cycle: part acceleration, friction use, still held | `e_stop.py`, `validate_grip.py` |
| 7 | Power failure: preload gone in 2 ms, brake-first shutdown, why the brake must be free of play | `power_fail.py`, `brake_play.py` |
| 8 | Strength and stiffness: FEM, jaw tilt, pressure wedge | `05_FEM`, `tilt_check.py` |
| 9 | Full cycle with collision check: clearances, 4.48 s | `e4_cycle.py`, clearance plots |

Overlays: force arrows on the pads, a friction-use bar per pad (green → red), the formula building up term by term,
short captions. No music or only a quiet bed; optional voice-over.

---

## 3. What exists

`hero_v1.py` (Blender 5.1, Cycles): the first shots from 12 Nov, recovered 21 Nov and updated to the current CAD
(200 mm body, BKE brake, V3 jaws). Shot A: the gripper turns slowly on the black glossy floor while the jaws open and
close (frames 1-150); shot B: close-up of the drive train with the shell hidden (151-270). It is the base for shots
2 and 5 above. Render: `blender -b -P hero_v1.py -- anim` (≈ 50 min at 128 samples, 1080p), then ffmpeg to
`hero_v1.mp4`. Frames and the .blend go to `out/` (git-ignored); the script and the mp4 are kept in the repo.

**Review 21 Nov (Elias):** light and materials approved. Of the eight test views (`hero_v1.py -- views`) all are kept
except 3 (low front): 1/2 hero, 4 three-quarter high, 5 side, 6 top, 7 jaw macro, 8 drive close-up. They are not
separate fixed shots but waypoints of **one flowing camera move** (eased spline path with speed changes, no cuts
inside a sequence). Before any render: previews at 480p in EEVEE, short clips only; the long Cycles render only
after approval.

## 4. Order of work

1. Export the assembly with names and BOM positions (FreeCAD → glTF / OBJ) and the E4 trajectory as JSON.
2. Blender base scene: materials, black studio, white lab, camera rig; one EEVEE draft of shot 2.
3. Assembly animation script (spring easing, snap, sound cue list) → draft of shot 4.
4. Mechanism drivers and section view → shot 5.
5. Robot rig and E4 motion, wake-up keys → shot 6.
6. Final renders in Cycles shot by shot, edit, sound, 16:9 and 9:16.
7. Explainer: overlays on the existing simulation renders, then the diagram scenes.

For the interim presentation (3 Dec): shots 2 and 4 of the hero film as a teaser, and the explainer scenes 1, 3, 5
and 9. Everything else by the final review (22 Jan).
