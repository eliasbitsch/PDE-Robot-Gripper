"""Where does the last 5 % between V4 and the FEM come from? Same FEM and beam model on three jaws:

  V3            Viktoriia's jaw as built (pocket 23 x 3, two O3.4 holes with O6.5 x 1 counterbores)
  no pocket     pocket filled, holes and counterbores kept
  solid         load zone filled completely (no pocket, no holes): a plain 25 x 8 bar

FEM: cross-section fully clamped at z = -47 (the beam's own boundary condition), load F_JAW_MAX spread over the pad
zone. Beam: Euler-Bernoulli + Timoshenko shear (kappa 5/6), same load distribution and sections as V4.
Suggested by Viktoriia (review response, 20 Nov 2026, point 2).

python jaw_variants.py  ->  jaw_variants.json (committed, read by the review notebook)
"""
import json
import math
import os

import numpy as np

import fem_jaw as J

HERE = os.path.dirname(os.path.abspath(__file__))
P = J.P
SRC = os.path.join(HERE, "jaw_v3.step")
X_IN = 36.0                                          # inner face of the jaw at d = 0
Y_POCKET, Y_JAW = 11.5, 12.5
Z0, Z1 = P.PAD_Z                                     # -95 ... -60
B, H, PD, RIM = 25.0, 8.0, 3.0, 1.0
HOLES, HOLE_D, CB_D, CB_T = (20.5, 40.5), 3.4, 6.5, 1.0
S_A, S_B, S_END = 13.0, 48.0, 51.0
G = J.E / (2 * (1 + J.NU))


def make_step(name, x1):
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    v = gmsh.model.occ.importShapes(SRC)
    y = Y_POCKET if x1 < 40 else Y_JAW
    box = gmsh.model.occ.addBox(X_IN, -y, Z0, x1 - X_IN, 2 * y, Z1 - Z0)
    gmsh.model.occ.fuse(v, [(3, box)])
    gmsh.model.occ.synchronize()
    path = os.path.join(J.OUT, f"jaw_{name}.step")
    gmsh.write(path)
    gmsh.finalize()
    return path


def _chord(s, sc, d):
    u = s - sc
    return 2 * math.sqrt((d / 2) ** 2 - u ** 2) if abs(u) < d / 2 else 0.0


def strips(s, variant):
    if not (S_A <= s <= S_B) or variant == "solid":
        return [(B, 0.0, H)]
    hb = sum(_chord(s, c, HOLE_D) for c in HOLES)
    cb = sum(_chord(s, c, CB_D) for c in HOLES)
    x0 = PD if variant == "V3" else 0.0
    out = [(2 * RIM, 0.0, PD)] if variant == "V3" else []
    return out + [(B - hb, x0, H - CB_T), (B - max(hb, cb), H - CB_T, H)]


def beam(variant, F=P.F_JAW_MAX, rims=True):
    s = np.linspace(0, S_END, 4081)
    A, I = [], []
    for si in s:
        st = strips(si, variant)
        if not rims and variant == "V3" and S_A <= si <= S_B:
            st = st[1:]                              # rims carry nothing: lower bound of the stiffness
        a = sum(b * (x1 - x0) for b, x0, x1 in st)
        xc = sum(b * (x1 - x0) * (x0 + x1) / 2 for b, x0, x1 in st) / a
        A.append(a)
        I.append(sum(b * (x1 - x0) ** 3 / 12 + b * (x1 - x0) * ((x0 + x1) / 2 - xc) ** 2 for b, x0, x1 in st))
    A, I = np.array(A), np.array(I)
    sf = 0.5 * (S_A + S_B)
    V = np.where(s < S_A, F, np.where(s <= S_B, F * (S_B - s) / (S_B - S_A), 0.0))
    M = np.where(s < S_A, F * (sf - s), np.where(s <= S_B, F * (S_B - s) ** 2 / (2 * (S_B - S_A)), 0.0))
    cum = lambda y: np.concatenate([[0.0], np.cumsum((y[1:] + y[:-1]) / 2 * np.diff(s))])
    wb, ws = cum(cum(M / (J.E * I))), cum(V / (G * 5 / 6 * A))
    at = lambda w: float(np.interp(sf, s, w)) * 1000
    return at(wb), at(ws)


def main(lc=1.6):
    out = {}
    for name, x1 in (("V3", None), ("no pocket", X_IN + PD), ("solid", X_IN + H)):
        J.STEP = SRC if x1 is None else make_step(name.replace(" ", "_"), x1)
        J.X_LOAD = J.X_FLOOR if x1 is None else X_IN
        r = J.run(lc, base="var_" + name.replace(" ", "_"), clamp="section")
        wb, ws = beam(name)
        fem = r["ux_load"] * 1000
        out[name] = dict(fem=fem, beam_bending=wb, beam_shear=ws, beam=wb + ws, dev=(wb + ws - fem) / fem * 100,
                         elements=r["elements"])
        if name == "V3":
            wb0, ws0 = beam(name, rims=False)
            out[name].update(beam_no_rims=wb0 + ws0, dev_no_rims=(wb0 + ws0 - fem) / fem * 100)
        print(f"{name:10s} FEM {fem:6.2f} um   beam {wb:6.2f} + {ws:4.2f} = {wb + ws:6.2f} um   "
              f"{(wb + ws - fem) / fem * 100:+5.1f} %   ({r['elements']} el.)")
    J.STEP, J.X_LOAD = os.path.join(HERE, "jaw.step"), J.X_FLOOR
    json.dump(out, open(os.path.join(HERE, "jaw_variants.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
