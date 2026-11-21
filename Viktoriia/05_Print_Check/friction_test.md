# V5 add-on: friction of the pad on oily part B (proposal, Elias 21 Nov 2026)

**Why.** The grip force rests on µ = 0.2 for the NBR pad on a dirty or oily PA 6 part, the most sensitive value
in the whole sizing and so far only bracketed by literature (steel on steel 0.1, oil-resistant NBR gripper pads
0.3 ... 0.4). The pads now have vertical oil-drain grooves (1.5 wide, 1 deep, pitch 4 mm). The design proof keeps
µ = 0.2; if the grooved pad reaches µ ≥ 0.3 on oily PA 6, the setpoint 44.5 N also holds part B in an emergency
stop twice as hard as the ABB stop data (`Elias/04_Simulation/e_stop.py`). The measured value replaces the
assumption in V2.

## Set-up (inclined plane, no extra equipment)

- **Pad:** a piece of the real NBR 70 Shore A (oil resistant), 3 mm, with the grooves cut in (or a bought grooved
  NBR strip), glued flat onto a small printed block. The block plus a weight gives a known normal force; weigh the
  whole slider (kitchen scale, ±1 g).
- **Counter-face:** PA 6 with a surface like part B (Ra ≈ 1.6): a PA 6 sheet or a machined PA 6 block. Not the
  printed part, PLA / PETG behave differently.
- **Contamination**, one series each: clean and degreased; thin film of cutting oil or machine oil (wipe on, then
  wipe off once, as a part from machining would come); flooded (a drop of oil left on).
- Slider on the PA 6 plate, raise the plate slowly (phone inclinometer app on the plate, or measure height h over
  length L). Angle θ at which the slider starts to move: **µ_static = tan θ**.
- 10 runs per condition, wipe the pad between oil changes. Two normal forces (slider alone, slider + 500 g) to see
  whether µ depends on the pressure (rubber often does; our pad pressure is only about 0.04 MPa).
- Repeat the oily series with a pad **without grooves** to show what the grooves bring.

## Evaluation

- Per condition: mean, standard deviation, minimum of µ. Design value = the **minimum** of the oily series (or
  mean - 2 s), not the mean.
- Result table into V2 § 1 with the photo of the set-up. If µ_min(oily, grooved) ≥ 0.3: note in V2 that the
  e-stop reserve (`e_stop.py`, "x2" scenario) is covered. If it is below 0.2: the design value has to go down and
  the setpoint / gear stage has to be reopened - tell Elias.
- Optional, closer to the gripper: two printed jaws with pads squeezed onto the oily PA 6 block by a known force
  (screw clamp with a spring scale or a luggage scale), pull the block out with a second scale: µ = F_pull / (2 F_N).
