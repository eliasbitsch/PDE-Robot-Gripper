# V3 - Jaws and pad plates

Status 19 Nov 2026. Option **a)**: designed to Elias' frozen jaw interface (handover § 4), nothing on his side moves.

![Jaw L](jaw_L_render.png)

| File | Part | Material | Mass |
|---|---|---|---|
| `jaw_L.step`, `jaw_R.step` | jaw (R = mirror of L) | EN AW-7075 T6 | 30.9 g |
| `pad_plate_L.step`, `pad_plate_R.step` | pad plate, 2 x M3 thread | S235, 3 mm | 18.4 g |
| `pad_NBR_L.step`, `pad_NBR_R.step` | pad, vulcanised on the plate | NBR 70 Shore A, 3 mm, oil resistant | 2.9 g |
| `jaws_V3_assembly.step` | all six, coloured | | |
| `jaws.py` | CadQuery script, reads `gripper_params.py` | | |

All parts sit in the gripper frame at the grip position (d = 2 mm), so they replace the placeholders in
`gripper_assembly.step` one to one. Regenerate: `pip install cadquery`, then `python jaws.py`.

## Interface (unchanged)

- 69.5 x 25 x 8 mm, outer face on the holder from the top edge down to 18.5 mm
- 2 x O4 H7, 6.5 deep, y = +-5.5, 3.5 below the top (dowels ISO 8734 4 m6 x 12)
- O5.2, 6 deep, y = 0, 10.5 below the top (indexing plunger GN 617-5-A)
- pocket 23 x 35 x 3 mm, 31.5 ... 66.5 below the top; 2 x O3.4 through at 39 and 59 below the top

## What I changed inside the jaw

| Change | Why |
|---|---|
| Pocket corners **R1** (vertical edges) | a milled pocket has a radius anyway; removes the sharp notch that gave the non-converging FEM peak |
| **Counterbore O6.5 x 1** on the outer face at both M3 holes | the BOM has ISO 4762 **M3x6** (pos. 25): through 5 mm of jaw that leaves only 1 mm of thread in the 3 mm plate; with the counterbore it is 2 mm, and the screw still ends 1 mm before the rubber |
| Pad plate 22.8 x 34.8 mm, corners R1.5 | 0.1 mm clearance per side in the pocket, radius larger than the pocket's so it sits flat |
| NBR 0.5 mm smaller than the plate all round | vulcanising edge; contact area 21.8 x 33.8 = 737 mm2 (V2 used 805 mm2 - pressure stays < 0.1 MPa) |
| all outer edges 0.5 x 45 | deburring |

The screws carry almost nothing: the grip force presses the plate into the pocket, and the pocket walls take the
friction force by form fit.

## Checks

- Collision check against all 105 solids of `gripper_assembly.step` (11 Nov): the new parts only overlap the
  placeholder jaw, plate and pad they replace - no contact with holder, dowels, plunger, slides or housing.
- V4 stays valid: same section (25 x 8, pocket web 25 x 5 + 1 mm rims); the R1 corners and the 1 mm counterbores
  only make it marginally softer locally.

## Open

- **Part B contour not checked against the real part** - `Bauteil_B.STEP` is FH data on Moodle and not in the repo.
  The jaw keeps Elias' z extent (ends 4 mm above the lug on -X, pad on the flat faces z = -3 ... 32 of the part).
  To do: load the part, check the open position d = 10 against the lug, and add a lead-in chamfer if the part can
  sit +-5 mm off centre.
- Drawing of the pad plate ("bought to drawing") - comes with E3 / the drawing set.
