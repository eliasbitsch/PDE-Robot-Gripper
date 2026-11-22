# Handover

**22 Nov 2026, branch `master`, status: flight preview v2 with sound, waiting for Elias' approval.**

## Done (this session)

- Opus review points 1.5 - 1.14, section 2 (contradictions) and section 3 (missing items) all worked through:
  ABB MoveJ path, drive margins (bevel pair KHK SMSG1.5-20), V2 brought to the built state, hanging rail with the
  cover as catch, zero-backlash brake BKE-0,4-5,0 (body 200 mm), tolerant two-stage close (cycle 4.48 s), tilt
  criterion with carriage and bonded pad, wrist torque by inverse dynamics, legacy `cad/params.py` synced.
- V1 created (`Viktoriia/01_Concept`), V2 § 2a, V4 § 5a; V4 vs FEM within 5 % on all like-for-like comparisons
  (stress +1.6 %, beam +4.1 %, beam + Weber +5.0 %).
- E3 drawing and the cell video regenerated. Pneumatic version: not pursued, not shown.
- Render: `Elias/07_Render/RENDER_PLAN.md`, `hero_v1.py` (recovered from the session log) with modes
  `views` (stills), `preview` (480p clips), `flight` (one camera move), `anim` (final Cycles).

- 22 Nov evening, after the review (Codex): grip proof with the E4 timing (built close 0.40 s, lift 0.22 s at the
  loaded 4.96 m/s²), CAD mass properties of part B, "held" = both pads in contact and below the friction limit at
  every step. Governing instant is the lift (design case): setpoint peak 0.52, S_eff 1.93 (servo overshoot to
  5.9 m/s²; hand 2.0). Monte Carlo 10 000 in spec all held (max 0.557, S 1.80), 16 tolerance corners all held (worst
  S 1.78), stress: lowest held mu F 4.38 N, highest failed 4.56 N (overlap; hand 4.45 N), heavier (saturated) part
  held (S 1.75). Width: within ISO
  2768-m fine; > 0.75 mm too thin is lifted before the force is complete (fixed-time lift), control is out of scope.
  The old 0.74 came with the snap start at the lift start; e-stop now replayed on chain 1 (governing). Flicker fixes (coplanar faces), side camera keys,
  cell_grid.py (100 cells). S = 2 kept (Elias): reasons in 04_Simulation/README.md.
- Part mass scatter: +0 ... +3 % = PA 6 moisture in standard climate (hall agreed as standard climate, Elias 22 Nov);
  heavier part as if saturated (+9 ... +10 %) checked separately: 200/200 held, peak 0.571 (S 1.75).

## Open

- [ ] Elias' approval of `out/flight_preview_C_new_bass.mp4` for the final Cycles render (then also the music mux
      on the Cycles frames: `python music_flight.py 720 45.76 C_new_bass`).
- [x] 22 Nov, decisions (Elias): music C = Mixkit 720 "New Bass 01" from 45.76 s; cut version with 0.8 s dips (the
      continuous move, also with dips between the views, felt like "someone switching the light on and off";
      VARIANT=_cont is kept in hero_v1.py but rejected). Reveal: black, edge glint sweep from behind, lights come
      up, close low start. Cuts at 103/231/317 and the big contact at 387 sit on beats of the track (beats at
      0.987 s + n * 0.476 s, measured with earshot).
- [x] 22 Nov, review of v4 (Elias): cut swoosh too fast / cheap, music still distorted (limiter drive 2.6 -> clipped),
      the effects only disturb -> no effects, library music instead. Apple ad music is not openly licensed; Mixkit
      (free in videos, no attribution, not on its own) used instead. earshot ranked 20 Mixkit tracks; Digital Clouds
      fits the iPhone 17 films best (cos 0.77, feature distance 0.45, only one "fits the set"). Dips now 12 frames
      per side. `sound_flight.py` (synth bed + effects) is kept but no longer used.
- [x] 22 Nov, review of v3 (Elias): cuts felt random, the camera should drive towards a part and away; the last
      part (smooth zoom into the moving mechanism) is good; the music sounded slowed down / rumbly. Reference: two
      Apple iPhone 17 films (yt-dlp, scratchpad): one move per shot, dips to dark between angles, cuts ~1.5 s,
      music-led at ~130 BPM, full spectrum. v4: every shot pulls out from close, is slow at its view (speed factor
      per key), pushes in on a detail; the cut falls inside that push with a 5-frame exposure dip; end part longer
      (9 jaw strokes), fade to black 586-600. Sound: everything had been pitched to D1/D2 and saturated -> now a
      clean D-minor pad (Dm9-Bbmaj7-Gm9-Asus4, 120 BPM pulse), bank sounds at native pitch, suck + soft hit per
      cut. earshot critique: launch-grade, every band in range, -15.3 LUFS, -1.6 dBTP, cues on the frame.
- [x] 22 Nov, review of v2 (Elias): "slower at the points, ease in/out, cuts from different angles are fine". v3: four
      shots with 3 cuts, each drifting slowly through its view (Hermite time warp, no stop), flowing moves only
      three-quarter -> top and jaws -> drive (pull-back key at 401 so the camera does not fly through the parts),
      own jaw schedule per shot (7 strokes), shell fade 372-412 now in view. `out/flight_timeline.json` drives the
      sound script. earshot: all 10 cues on the frame, -16.9 LUFS, -1.6 dBTP.
- [x] 22 Nov: top view fixed (pool spot dips to 10 W around frame 100, top light no longer reflects in the floor);
      shell fades out over frames 178-210 (own material copies, alpha eased) instead of vanishing at 205.
      Note: at the jaw macro the shell is mostly out of frame, so the fade is barely visible there.
- [x] 22 Nov: draft sound `sound_flight.py`: sub drone in D, low whooshes per camera segment, motor whir + rack ticks
      per jaw stroke, metal clacks on contact, stop-down before the second grip. earshot: contacts on the frame
      (0 ms), -15.9 LUFS, -1.6 dBTP after AAC; open per critique: no music track (critical vs launch films), mids
      a bit low (-24 dB share, refs from -19), all sounds synthesized (bank has no screw / ratchet recordings).
- [ ] Only after approval: final Cycles render (≈ 50 min for 270 frames at 1080p / 128 samples).
- [ ] Elias' feedback on the 480p previews `out/explode_preview_C.mp4` (shot 5b, `hero_v1.py -- explode`, 14 s:
      shell lifts off with spring overshoot, drive train free, three strokes, closes without overshoot) and
      `out/assemble_preview_C.mp4` (shot 4, `-- assemble`, 16 s: white lab, order and directions from
      parts/demo_seq.py, critically damped landing + 3 % scale pulse + bounce on big parts, screws turn in).
      Known: in the white lab the aluminium reads almost white/plastic and the gripper is small in frame (camera
      0.86 m) -> needs an HDRI or darker floor for reflections and a closer camera.
- [x] 22 Nov, music decided (Elias): the ORIGINAL track C (Mixkit 720 from 45.76 s). Stripped versions (low-pass,
      Demucs bass + drums, epic sub boosts) sounded unfinished. Tools stay: MUSIC_LP and derived mixes in
      music_flight.py, Demucs in ~/tools/demucs. Private comparison only (never on the board or in the film):
      out/private/ holds the Google Pixel "Fur Elise" spot without the speaker and a 20 s loop of its choir part.
- [x] 22 Nov: Viktoriia agreed to the wider T-shaped jaw (it was already the current model, nothing to rebuild).
- [ ] Board: put the three film previews back (out/*_C.mp4), see the next point for the background.
- [ ] (was) Waiting for Viktoriia's answer on the wider T-shaped jaw (asked 22 Nov as a proposal; the variant is the current
      model in Viktoriia/03_CAD_Jaws). Until she agrees the board shows only her original V3 jaw
      (img/jaw_v3_viktoriia.png from commit da4f7f5) and no film previews. If yes: put the three previews back on the
      board (out/*_C.mp4). If no: rebuild with her contour (jaws.py at da4f7f5) and rerun FEM, V4, grip_check, E4.
- [x] 22 Nov: board https://claude.ai/artifact/3gQTn4NuiM27SdwwwSC6XU updated (Current state: three film previews,
      key numbers, changes since the last update; tasks crosscheck/e2/e3/e4/e7 notes, v1 -> review, new hero-film).
- [ ] Next hero shots per RENDER_PLAN: self-assembly in the white lab (spring easing, snap), robot wake-up,
      mechanism section; explainer film scenes 1, 3, 5, 9 for the interim presentation (3 Dec).
- [ ] Send Viktoriia the change message (English, text in the chat of 21 Nov): V1 new, V2 §§ 2a/3/5/6a/7/8/9/10,
      V4 §§ 2/5a/6, grip force calibration at commissioning.

## Traps and assumptions

- Virtual design only: no supplier queries; datasheet gaps are stated as assumptions (BKE mounting, rear shaft).
- Render rule: stills from several cameras or ≤ 5 s previews first, long renders only after Elias approves.
  Render scripts and finished videos live in `Elias/07_Render` (out/ is git-ignored), never only in the scratchpad.
- Weber row of V4 sits at 4.995 %; the 45° load spreading into the plate is an assumption (sensitivity ±12 %).
- Group 10 confirmed (assignment Tab. 1 only lists 1-8, ignore).

## Continue

```sh
cd Elias/07_Render
ENGINE=EEVEE SAMPLES=16 STEP=2 RES=854x480 blender -b -P hero_v1.py -- flight   # preview, ~1.5 min
blender -b -P hero_v1.py -- views                                               # stills of the 8 cameras
# frames are every 2nd (STEP=2): encode with a concat list at 15 fps, then the sound:
<earshot>/.venv/Scripts/python.exe sound_flight.py out/flight_preview.mp4       # -> out/flight_preview_sound.mp4
```
CAD after a change: `fc.sh Elias/02_CAD_Body/build.py`, then the scripts in `04_Simulation` (README lists them).
