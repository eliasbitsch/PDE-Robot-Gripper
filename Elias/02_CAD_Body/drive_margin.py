"""Drive chain margins with tolerances: setpoint < current limit < bevel pair, key and overloads (review 1.7, 21 Nov 2026).

python drive_margin.py  ->  table per bevel option, out/drive_margin.json

Normal operation: the controller sets a motor current; the grip force that results scatters with the motor's torque
constant, the driver's current accuracy and the efficiencies. Low end: does the part still hold? High end: torque on
the bevel teeth (driving side, before the bevel loss) against the catalogue rating with the application factor.
Two ways to set the current: from the data sheet values, or calibrated once at commissioning with a grip force gauge
(then only the gauge and the drift with temperature remain).
Rare overloads (a few hundred in the life at most): judged against the static capacity, ISO 6336 life factors for
induction-hardened steel at N <= 1e5 (flank Z_NT 1.6, torque ~ sigma_H^2; root Y_NT 2.5).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_params as P  # noqa: E402

R = P.PINION_D / 2 / 1000
K_A = 1.25                       # application factor, V2 (electric motor, light shocks at the part contact)
LIFE_GRIPS = 16 * 3600 / P.TAKTZEIT * 250 * 10      # 2 shifts, 250 days, 10 years: 2.9e7 load cycles, same teeth each time
Z_NT_STATIC, Y_NT_STATIC = 1.6, 2.5
F_NEED = {"path, twist incl., S 2 (grip_check, 5.35 m/s2)": 35.9,
          "e-stop ABB stop data, S 1 (e_stop.py)": 34.3}

TOL = {   # (low, high) relative to nominal
    "datasheet": dict(k_t=(0.90, 1.10), current=(0.97, 1.03), eta_g=(0.85 / 0.90, 0.95 / 0.90),
                      eta_b=(0.97 / 0.98, 0.99 / 0.98), eta_rp=(0.95 / 0.97, 0.98 / 0.97)),
    "calibrated": dict(force=(0.95, 1.05)),      # force gauge 1 % + k_T drift over 20 ... 60 deg C (NdFeB ~ -0.1 %/K)
}

BEVELS = {
    "KHK SMSG1-20 (before 21 Nov)": dict(t_bend=1.17, t_surf=0.97, key_l=8.0, mass_pair=0.038),
    "KHK SMSG1.5-20 (built)": dict(t_bend=4.1, t_surf=3.47, key_l=12.0, mass_pair=0.055),
}
KEY_D, KEY_HUB_H = 8.0, P.KEY_H - P.KEY_T1          # shaft d, key height in the hub (0.8 mm)
P_KEY_STATIC = 0.9 * 345.0                         # MPa, hub S45C normalised (Re >= 345), DIN 6892 static


def torque_on_bevel(f_jaw, eta_b=P.ETA_BEVEL, eta_rp=P.ETA_RP):
    """Driving-side bevel torque [N*m] for a jaw force [N] (one pinion drives both racks)."""
    return 2 * R * f_jaw / (eta_b * eta_rp)


def operation(f_set=P.F_GRIP_SET):
    out = {}
    t_nom = torque_on_bevel(f_set)
    d = TOL["datasheet"]
    lo = d["k_t"][0] * d["current"][0] * d["eta_g"][0] * d["eta_b"][0] * d["eta_rp"][0]
    out["datasheet"] = dict(f_low=f_set * lo, t_high=t_nom * d["k_t"][1] * d["current"][1] * d["eta_g"][1])
    c = TOL["calibrated"]["force"]
    out["calibrated"] = dict(f_low=f_set * c[0], t_high=torque_on_bevel(f_set * c[1], 0.97, 0.95))
    return t_nom, out


def key_pressure(t, key_l):
    return 2 * t * 1000 / (KEY_D * KEY_HUB_H * (key_l - P.KEY_B))


def impact_peak():
    """Largest pad force when closing straight in without the two-stage profile (closing_time.py)."""
    f = os.path.join(HERE, "..", "04_Simulation", "out", "closing_results.json")
    cases = json.load(open(f, encoding="utf-8"))["cases"]
    return max(c["f_peak"] for c in cases if c["profile"] == "direct")


def overloads():
    eta_g_hi = 0.95
    f_imp = impact_peak()
    return {
        "full current (current limit lost), k_T +10 %": P.MOTOR_T_HOLD * 1.10 * P.GEAR_I * eta_g_hi,
        f"direct close without the two-stage profile ({f_imp:.0f} N impact)": torque_on_bevel(f_imp),
        "LC5: jaw pushed with power off until the brake slips": P.BRAKE_T * P.GEAR_I,
    }


def main():
    t_nom, op = operation()
    res = dict(setpoint=P.F_GRIP_SET, t_nominal=t_nom, K_A=K_A, life_grips=LIFE_GRIPS, f_need=F_NEED,
               operation=op, overloads=overloads(), bevels={})
    print(f"setpoint {P.F_GRIP_SET:.1f} N per jaw -> bevel torque {t_nom:.3f} N*m nominal; "
          f"{LIFE_GRIPS:.2e} grips in 10 years, K_A {K_A}")
    for mode, v in op.items():
        print(f"  current set by {mode:10s}: jaw force >= {v['f_low']:.1f} N (needed {max(F_NEED.values()):.1f}),"
              f" bevel torque <= {v['t_high']:.3f} N*m")
    for name, b in BEVELS.items():
        rows = {}
        print(f"\n{name}: bending {b['t_bend']} N*m, surface {b['t_surf']} N*m (catalogue, 1e7 cycles)")
        for mode, v in op.items():
            ub, us = v["t_high"] * K_A / b["t_bend"], v["t_high"] * K_A / b["t_surf"]
            rows[mode] = dict(use_bending=ub, use_surface=us, key_MPa=key_pressure(v["t_high"], b["key_l"]))
            print(f"  operation, {mode:10s}: K_A T / T_bend {ub:5.2f}   K_A T / T_surf {us:5.2f}   "
                  f"key {rows[mode]['key_MPa']:5.0f} MPa")
        for oname, t in res["overloads"].items():
            ub = t / (b["t_bend"] * Y_NT_STATIC)
            us = t / (b["t_surf"] * Z_NT_STATIC ** 2)
            pk = key_pressure(t, b["key_l"])
            rows[oname] = dict(torque=t, use_bending=ub, use_surface=us, key_MPa=pk)
            print(f"  {oname[:58]:58s} {t:5.2f} N*m: static use bending {ub:4.2f}, flank {us:4.2f}, "
                  f"key {pk:4.0f} of {P_KEY_STATIC:.0f} MPa")
        res["bevels"][name] = dict(b, rows=rows)
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, "out", "drive_margin.json"), "w", encoding="utf-8"), indent=1)


if __name__ == "__main__":
    main()
