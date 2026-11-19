# Viktoriia - design & analytical proof

Working area of Ovdiienko Viktoriia. See [PLAN.md](../PLAN.md), § 5.

| Folder | Package | Result |
|---|---|---|
| `01_Concept` | V1 | Variant comparison (VDI 2225) incl. the Robotiq 2F-85 as commercial reference and vacuum as a reasoned rejection, dimensioned concept sketch |
| `02_Sizing` | V2 | Gripping force, friction coefficient with a source, surface pressure on PA 6, four-bar via virtual work incl. worst transmission angle, screw and motor selection |
| `03_CAD_Jaws` | V3 | Jaws + soft pads, parametric in Onshape |
| `04_Analytical` | V4 | Bending stress and deflection by hand - the reference the FEM has to meet |
| `05_Print_Check` | V5 | Print the jaw, measure deflection under a defined load, hold it against Elias' FEM |

**The thread:** you design the jaws, work them out by hand, and then check the
FEM against a printed part. V3 -> V4 -> V5 is one chain.

**Interface to Elias:** the mounting plane carrier <-> jaw.
Frozen from week 41, changes only together.

**If something does not suit you:** § 10 in PLAN.md. V3<->E2 and V4<->E5 are cut
to be exchangeable on purpose.
