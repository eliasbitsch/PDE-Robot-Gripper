# Handover

**27 Nov 2026, branch `master`, status: interim presentation held (26 Nov), hand-in deck final; a whole-project
review lists the fixes needed before the final submission.**

## Done (24 - 27 Nov)

- Interim deck `OneDrive - FH Technikum Wien/PDE/2026-11-26_Interim_Presentation_Group10.pptx`: 25 slides,
  14.8 MB (limit 20 MB). Videos re-encoded to 720p / 25 fps (~270 kbit/s, audio 64k), poster frames 960 px,
  all videos autoplay once (no loop), speaker notes removed for the hand-in. The 1080p originals stay in
  `Elias/07_Render/out/praesentation/`.
- Final Cycles films: `1_opening_1080p_CYCLES_FULL.mp4` (51.1 s, flight + assemble) and
  `2_abschluss_1080p_CYCLES.mp4` (43.9 s, explode + robot), both in the deck.
- `Elias/01_Robot/placement_overlay.py` (placement map over the cell side view), `Elias/05_FEM/fem_train_anim.py`
  (LC2 animation mp4 + gif), notebook launcher in six chapters (`shared/presentation/notebooks.py`).
- Whole-project review done (27 Nov); verified findings are below.

## Open - before the final submission (ordered)

Critical:
- [ ] Report PDF missing, `shared/submission/` is empty (documentation = 10 points, assignment p. 4-5).
      Start a report skeleton and a pack script with a dry run.
- [ ] `Elias/02_CAD_Body/drive_margin.py:25-26`: `F_NEED` lacks the lift case (44.5 N), so "lowest 42.3 N ok"
      passes although 42.3 < 44.5.
- [ ] One acceptance table for the safety factor (e.g. nominal >= 2, tolerances >= 1.5, e-stop >= 1.0 with
      reasoning) and the correct source for S = 2; check whether VDI 2860 really prescribes a grip-force factor.
      Today slide 19 says S = 2 but shows S_eff 1.93, slide 21 has 1.78, slide 23 1.30.
- [ ] "covers e-stop" (`gripper_params.py:52`, `class_presentation.py:288`): vertical e-stop gives only S 1.10;
      S 1.5 would need 60.8 N > 60.5 N motor. Remove the claim or justify why the case cannot occur.
- [ ] Deck slide 11: wrist torque 9.1 N·m is the horizontal-tool bound; real cycle 6.76 N·m (52 %),
      `Elias/01_Robot/wrist_torque.json`.

Important:
- [ ] Stale numbers: `Elias/01_Robot/README.md:12-19` (3.64 kg, 72 %, 7.0 Nm, 12.5° -> 3.552 kg, 70 %, 6.76 Nm,
      13.7°), `README.md:64` / `PLAN.md:74,86` (35.7 N "required"), `cad/params.py:72` (3.04 kg).
- [ ] FEM: holder is EN AW-6082 T6 (250 MPa), not 460 MPa; 15.4 / 8.4 MPa are edge peaks (README says 8.2);
      three different "max forces" (60.5 / 63.6 / 77.8 N). The 5 % hand-vs-FEM agreement is partly calibrated
      (45° spreading, `05_FEM/README.md:127-131`) - say so instead of "nothing tuned".
- [ ] `class_presentation.py:294,448-450,513-520` calls `jaw_v3` superseded although it is the final T-jaw;
      `:185-186` pairs S 1.30 with the wrong case (widget shows 66 N in red, text says held).
- [ ] `validation_notebook.py:148,191` says 0.3 s, code checks 0.4 s (`closing_time.py:30`); defaults do not
      reproduce 373 ms.
- [ ] Reproducibility: `pyproject.toml` has no dependencies, `fc.sh` is referenced but missing, results/STEP/BOM
      live in git-ignored `out/` (class_presentation crashes on a clean checkout), hard path in
      `Elias/06_Cell/cell_view.FCMacro:11`.
- [ ] Overload cases: brake slip and direct close (160 N) push bearing s0 to 1.5-1.8; one-sided push on LC5.
- [ ] Missing: cell safety (ISO 12100 hazard list, ISO 13849 PL, ISO 13857 distances, axis-1 limit), service
      life (bevel gears 2.9e7 cycles, rail L10, motor heating while holding), GP42 datasheet.
- [ ] Brake claims: only valid with 24 V buffer and brake-first shutdown; "zero backlash" not in the datasheet;
      `power_fail.py:1` still says spring-applied.
- [ ] Red thread: README for `shared/presentation/` with a how-to-read table of the six chapters; drawing and
      collision check have no chapter; `PLAN.md:7` still "proposal"; `Elias/README.md:8` mentions dovetail and
      servo; `fem_train_notebook.py` "mock jaw until V3".
- [ ] German leftovers: `cad/params.py` (live, documented as reference), root `scripts/ fem/ sim/ onshape/`,
      `out/Baugruppenzeichnung_Greifer.pdf`, burnt-in German text in `cell_video.py` / `cell_grid.py` videos,
      German BOM on the drawing (`de.py`). Move legacy to `legacy/`.
- [ ] Robustness: `out/robustness.json` missing until the GPU study is rerun (see Continue); the CPU stress run
      still used the old pad criterion.
- [ ] Cost estimate of the parts (PLAN.md point 9).

Minor: no script makes `sheet_jaw.png`; phase table sums to 4.49 s; `robot_notebook.py:116` "short on reach"
means speed; `class_presentation.py:494` claims convergence for every load case (only LC2); "+-"/"mu" on slide 21;
emojis in README/PLAN; "peak 118 N" only exists as the vacuum normal force in `PLAN.md:102`.

Viktoriia's part (do not edit, raise with her): vertical e-stop S 1.10 vs "covered S 1.5" (slide 6,
`v2_sizing.py:32,815`); slide 6 diagram marks 35.7 N under 44.5 N, motor line 63.6 vs 60.5 N; slide 7 brake
position and governing part (rack S_F ~ 11.9); slide 8 shows the old jaw; her calculations exist only as
notebooks, the one PDF is German and outdated.

## Traps and assumptions

- Viktoriia's notebooks, slides and folder are never edited by us; only link to them.
- Deck: no PowerPoint changes unless Elias asks. Edit a copy via COM and write back only if the original is
  unchanged; never `Quit()` PowerPoint while presentations are open.
- Virtual design only: no supplier queries; datasheet gaps are stated as assumptions (BKE mounting, rear shaft).
- Render rule: previews first, long renders only after Elias approves; renders run on the Razer, not the laptop.
  Music in the films is for the class presentation only, never publish.
- Weber row of V4 sits at 4.995 %; the 45° load spreading is an assumption (sensitivity ±12 %).

## Continue

```sh
cd Elias/02_CAD_Body && python drive_margin.py        # first fix: add the lift case to F_NEED
cd Elias/04_Simulation && GPU_BATCH=3072 python robustness_gpu.py && python robustness_gpu.py --corners
python shared/presentation/notebooks.py               # starts the six chapter notebooks
```
