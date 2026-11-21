"""Pad tilt with the carriage's own compliance, the real NBR pad stiffness, and what the tilt does to the grip
(review 1.12). Replaces the fixed 0.1 deg limit by a functional criterion.

python tilt_check.py  ->  out/tilt_check.json

1. Tilt = FEM (carriage rigid, out/train_results.json) + carriage pitch from the Rexroth stiffness chart.
2. The NBR pad is a bonded layer: 2 mm base (bulges only at the pad edge) under 1 mm ribs between the oil grooves.
   Its compression modulus is far above E (shape factor), so it compresses only a few µm.
3. A tilt larger than twice that compression over the pad height lifts the lower pad edge: the pressure becomes a
   wedge from the top edge (the jaw bends open at the bottom). Friction does not care about the contact area, but
   the pressure centre moves up, away from the part's centre of mass, and the twist lever grows. Criterion: with
   the wedge, the setpoint must still give the required safety against slip (grip_check limit surface): S 2 on the
   path, S >= 1 in the e-stop (rare; 1.5 recommended).
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402
import grip_check as G  # noqa: E402

# Rexroth R999001206 p. 13, long carriage R0444 size 12, lifting load (the softer direction): about 6.5 µm at 500 N,
# 26 µm at 2500 N. The ball-row forces from the pitch moment stay below 500 N, where the curve is softest.
K_CARRIAGE = 500.0 / 6.5                     # N/µm
L_BALLS = 6 * P.CARRIAGE_ML0 * 1000 / P.CARRIAGE_C0     # mm, effective ball-row length from M_L0 = C0 L / 6 (31 mm)

NBR_E = (4.5, 5.5, 8.0)                      # MPa, NBR 70 Shore A (Gent from the hardness: 5.5)
K_BOND = 0.57                                # Gent / Lindley, E_c = E (1 + 2 k S^2), 70 Shore A
K_BULK = 2000.0                              # MPa, rubber bulk modulus: caps E_c for the thin base
GW, GD, GP = P.PAD_GROOVE
PAD_W, PAD_H = P.PAD_W - 1.2, P.PAD_Z[1] - P.PAD_Z[0] - 1.2       # NBR face 80.8 x 19.8
T_RIB, T_BASE = GD, P.PAD_T - GD
PHI = 1 - GW / GP                            # rib share of the face

Z_MESH = P.PINION_Z0 + P.PINION_B / 2
LEVER = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1])
LEVER = abs(Z_MESH - LEVER)                  # pad centre to the rack mesh: pitch moment on the carriage per N


def bonded(e, t, a, b):
    s = a * b / (2 * (a + b) * t)
    ec = e * (1 + 2 * K_BOND * s * s)
    return 1 / (1 / ec + 1 / K_BULK), s


def pad_k(e):
    """Winkler stiffness of the pad [N/mm3]: pressure on the face per mm of compression."""
    ec_rib, s_rib = bonded(e, T_RIB, GP - GW, PAD_H)
    ec_base, s_base = bonded(e, T_BASE, PAD_W, PAD_H)
    return 1 / (T_RIB / (PHI * ec_rib) + T_BASE / ec_base), dict(ec_rib=ec_rib, s_rib=s_rib, ec_base=ec_base, s_base=s_base)


def pad_compression_um(force, e):
    return force / (PAD_W * PAD_H) / pad_k(e)[0] * 1000


def carriage_tilt_rad(force):
    return 12 * force * LEVER / (K_CARRIAGE * 1000 * L_BALLS ** 2)


def main():
    fem = json.load(open(os.path.join(HERE, "out", "train_results.json"), encoding="utf-8"))
    forces = {"LC1 setpoint grip": P.F_GRIP_SET, "LC2 full current": P.F_JAW_MAX, "LC3 transport": P.F_GRIP_SET,
              "LC4 emergency stop": P.F_JAW_MAX}
    lc1 = next(c for c in fem["cases"] if c["case"].startswith("LC1"))
    kappa = math.radians(lc1["tilt_deg"]) / P.F_GRIP_SET + carriage_tilt_rad(1.0)    # total tilt per N of grip force
    res = dict(k_carriage=K_CARRIAGE, l_balls=L_BALLS, lever=LEVER, cases={}, pad={})
    print(f"carriage pitch: {K_CARRIAGE:.0f} N/µm per ball row end, L {L_BALLS:.1f} mm, lever {LEVER:.1f} mm "
          f"-> {math.degrees(carriage_tilt_rad(1.0)) * 1000:.2f} mdeg per N")
    for c in fem["cases"]:
        if c["case"] not in forces:
            continue
        f = forces[c["case"]]
        tc = math.degrees(carriage_tilt_rad(f))
        res["cases"][c["case"]] = dict(force=f, fem=c["tilt_deg"], carriage=tc, total=c["tilt_deg"] + tc)
        print(f"  {c['case']:20s} {f:5.1f} N: FEM {c['tilt_deg']:.3f} + carriage {tc:.3f} = {c['tilt_deg'] + tc:.3f} deg")
    hull_u = G.limit_hull(PAD_W / 1000, PAD_H / 1000)
    w, h, dz = G.pad(P.PAD_Z, P.PAD_W, P.PART_OFFSET[2])
    m, g = P.B_MASSE, P.G
    for e in NBR_E:
        kw, info = pad_k(e)
        delta = pad_compression_um(P.F_GRIP_SET, e)
        theta_full = 2 * delta / 1000 / PAD_H                                   # rad: lower edge just lifts
        c = min(PAD_H, math.sqrt(2 / (kw * PAD_W * kappa)))                     # mm, same at every force (tilt ~ F)
        p_peak = kw * kappa * P.F_GRIP_SET * c
        hull = G.limit_hull(w, h, c=c / 1000) if c < PAD_H else hull_u
        need = {}
        for name, ay, az, S in G.CASES:
            load = m / 2 * np.array([ay, g + az, ay * dz])                       # S 1: safety = setpoint / need
            need[name] = (G.normal_force(hull_u, load, P.MU_OILY), G.normal_force(hull, load, P.MU_OILY),
                          2.0 if name.startswith("path") else 1.0)
        res["pad"][f"E {e}"] = dict(k_w=kw, **info, compression_um=delta, tilt_full_contact_deg=math.degrees(theta_full),
                                    contact_mm=c, p_peak=p_peak, need={k: v[:2] for k, v in need.items()})
        print(f"\nNBR E {e} MPa: ribs S {info['s_rib']:.2f} -> E_c {info['ec_rib']:.0f} MPa, base S {info['s_base']:.2f} -> "
              f"{info['ec_base']:.0f} MPa; compression at {P.F_GRIP_SET:.1f} N {delta:.1f} µm (E alone: "
              f"{P.F_GRIP_SET / (PAD_W * PAD_H) / e * P.PAD_T * 1000:.0f} µm)")
        print(f"  full contact only up to {math.degrees(theta_full):.3f} deg; with the tilt the pad bears on the top "
              f"{c:.1f} of {PAD_H:.1f} mm, peak pressure {p_peak:.3f} MPa at the setpoint")
        for name, (nu, nt, s_req) in need.items():
            su, st = P.F_GRIP_SET / nu, P.F_GRIP_SET / nt
            print(f"    {name:38s}: S against slip at the setpoint {su:4.2f} uniform -> {st:4.2f} with the wedge "
                  f"(required {s_req:.0f}) {'ok' if st >= s_req else 'TOO LOW'}")
    json.dump(res, open(os.path.join(HERE, "out", "tilt_check.json"), "w", encoding="utf-8"), indent=1)
    return res


if __name__ == "__main__":
    main()
