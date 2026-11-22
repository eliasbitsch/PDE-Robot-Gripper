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

## Open

- [ ] Elias picks the music: `out/flight_preview_{A_digital_clouds,B_scifi_score,C_new_bass}.mp4` (v4 picture with
      0.8 s dips). Build: `python music_flight.py <mixkit id> <start s> <tag>` (A 175 @0, B 464 @23.03 so its
      swell at 35.66 s lands on the big contact, C 720 @45.55).
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
- [ ] New shot 5b in RENDER_PLAN: partial exploded view up to the mechanism, motor forward/back a few times.
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
