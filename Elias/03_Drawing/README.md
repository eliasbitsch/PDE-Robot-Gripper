# E3 - Assembly drawing, parts list and part drawing

Texts on the sheets are German (submission); designations are translated in `de.py`, the model keeps English.

`assembly_drawing.pdf`: sheet 1 assembly drawing (A3, 1:2, first-angle projection): full section A-A through the
drive axis, top view with the section line, view from the left, isometric view, balloons, main dimensions, fits;
sheet 2 parts list (ISO 7573) straight from `02_CAD_Body/out/bom.csv`.

Regenerate after a CAD change:

```sh
fc.sh ../02_CAD_Body/build.py        # model, bom.csv, bom_map.json (part -> position)
fc.sh drawing_export.py              # views as 2D line data (HLR projection, cut faces) -> out/views.json
python drawing_sheet.py              # sheets -> assembly_drawing.pdf, previews in out/
```

Section per ISO 128-44: own parts and bearings are cut and hatched; shafts, screws, pins, keys, circlips and the
bought drive units (motor, gearbox, brake, encoder) stay uncut. Hidden edges are left out.

`part_drawing_jaw.pdf`: part drawing of the gripper jaw (Viktoriia's V3, A3, 2:1): inner face, section A-A, outer face,
full dimensions, fits (dowel holes 4 H7), datum A on the holder face with flatness and perpendicularity, surfaces.
`fc.sh part_export.py` then `python part_drawing.py`.
