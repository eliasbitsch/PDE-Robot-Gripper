"""Brake-first shutdown on power loss: how much grip force survives the brake's rotational play? (review 1.10)

Result: with the BWA's datasheet play the part drops even with the brake-first shutdown, hence the zero-backlash
permanent-magnet BKE since 21 Nov.

python brake_play.py  ->  out/brake_play.json, figures/brake_play.png

The brake sits on the motor's rear shaft. Its friction disc is driven by a hex hub with play (BWA: < 2 deg). While
the jaws grip, the brake is released and the hub rests on the closing-side flank (last motion was closing). When the
brake has engaged and the motor current goes off, the elastic preload turns the shaft back through the play before
the brake can hold: the jaws open by the play and the grip force drops along the chain stiffness.
Stiffness counts only the compliances that are known (FEM jaw train, NBR pad, pinion shaft torsion); carriage tilt,
gear meshes and gearbox torsion are left out, which makes the chain stiffer and the result conservative.
Also: how fast the preload is gone without current, from the same stiffness and the reflected drive inertia - the
check on the MuJoCo result (power_fail.py, about 2 ms), which hangs on an uncalibrated contact stiffness.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import drive as DRV  # noqa: E402
import sim as S  # noqa: E402

GP = S.GP
F_SET = GP.F_GRIP_SET
F_HOLD = GP.B_MASSE * GP.G / (GP.N_BACKEN * GP.MU_OILY)       # per jaw against the weight alone (S 1)
R_P = GP.PINION_D / 2                                         # mm

U_TRAIN_LC1 = 29.2                 # µm at the pad centre, FEM LC1 (44.5 N), carriage rigid (05_FEM/README.md)
NBR_E = (4.5, 5.5, 8.0)            # MPa, NBR 70 Shore A: Gent from the hardness gives 5.5; 8 = stiff end (worst here)
NBR_T = GP.PAD_T
_gw, _gd, _gp = GP.PAD_GROOVE
PAD_W, PAD_H = GP.PAD_W - 1.2, GP.PAD_Z[1] - GP.PAD_Z[0] - 1.2
PAD_A = PAD_W * PAD_H * (1 - _gw / _gp)                       # mm2 without the grooves
_strip = _gp - _gw
SHAPE = _strip * PAD_H / (2 * (_strip + PAD_H) * NBR_T)       # shape factor of one strip between two grooves
SHAFT_D, SHAFT_L, G_STEEL = 8.0, GP.DRIVE_Z - 12.2 + 6.0, 81000.0   # pinion shaft from the bevel contact to the teeth

PLAY_MOTOR_DEG = {"BWA-0,35-5 (spring-applied, hex hub, until 21 Nov)": 2.0, "BKE-0,4-5,0 (permanent magnet, built)": 0.0}


def chain_um_per_n(e_nbr):
    """Compliance per jaw at the pad [µm/N]: jaw train + NBR pad + pinion shaft torsion (in series)."""
    ec = e_nbr * (1 + 2 * 0.57 * SHAPE ** 2)                  # bonded layer, Gent / Lindley, k = 0.57 for 70 Shore A
    c_pad = NBR_T / (PAD_A * ec) * 1000
    c_train = U_TRAIN_LC1 / 44.5
    j = math.pi * SHAFT_D ** 4 / 32
    c_shaft = (2 * R_P) * R_P * SHAFT_L / (G_STEEL * j) * 1000   # one pinion carries both jaws: T = 2 F r
    return c_train + c_pad + c_shaft, dict(train=c_train, pad=c_pad, shaft=c_shaft)


def play_at_jaw(play_motor_deg):
    return math.radians(play_motor_deg) / GP.GEAR_I * R_P * 1000      # µm per jaw


def residual(play_motor_deg, e_nbr, f0=F_SET):
    c, _ = chain_um_per_n(e_nbr)
    return max(0.0, f0 - play_at_jaw(play_motor_deg) / c)


def opening_time(e_nbr, f0=F_SET):
    """Quarter period of the preload spring against the reflected drive inertia and the jaws (no current, no brake):
    the time until the pad force is gone."""
    c, _ = chain_um_per_n(e_nbr)
    k_total = 2 / (c * 1e-6)                                  # N/m on the jaw coordinate, both jaws in parallel
    m = DRV.M_REFLECTED + 2 * DRV.M_JAW
    return 0.5 * math.pi / math.sqrt(k_total / m)


def main():
    res = dict(f_set=F_SET, f_hold=F_HOLD, pad_area=PAD_A, shape_factor=SHAPE, cases={})
    print(f"setpoint {F_SET:.1f} N per jaw; the part needs {F_HOLD:.1f} N per jaw against its weight (mu {GP.MU_OILY}, S 1)")
    for e in NBR_E:
        c, parts = chain_um_per_n(e)
        t = opening_time(e)
        row = dict(compliance_um_per_N=c, parts=parts, stored_um=c * F_SET, t_open_ms=t * 1000,
                   play_allowed_deg_S1=math.degrees((c * (F_SET - F_HOLD)) / 1000 / R_P * GP.GEAR_I),
                   play_allowed_deg_S15=math.degrees((c * (F_SET - 1.5 * F_HOLD)) / 1000 / R_P * GP.GEAR_I),
                   residual={k: residual(p, e) for k, p in PLAY_MOTOR_DEG.items()})
        res["cases"][f"NBR E {e} MPa"] = row
        print(f"NBR E {e} MPa: train {parts['train'] * F_SET:.1f} + pad {parts['pad'] * F_SET:.1f} + shaft "
              f"{parts['shaft'] * F_SET:.1f} = {row['stored_um']:.1f} µm stored per jaw; preload gone after "
              f"{row['t_open_ms']:.1f} ms without current; play allowed at the motor {row['play_allowed_deg_S1']:.2f} deg "
              f"(S 1) / {row['play_allowed_deg_S15']:.2f} deg (S 1.5)")
        for k, v in row["residual"].items():
            print(f"    {k:42s}: {play_at_jaw(PLAY_MOTOR_DEG[k]):4.1f} µm play per jaw -> {v:5.1f} N left "
                  f"{'held' if v >= F_HOLD else 'DROPPED'}")
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    json.dump(res, open(os.path.join(HERE, "out", "brake_play.json"), "w", encoding="utf-8"), indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 3.4))
    play = np.linspace(0, 2.0, 100)
    for e, ls in zip(NBR_E, (":", "-", "--")):
        ax.plot(play, [residual(p, e) for p in play], ls, color="#0f766e", label=f"NBR E {e} MPa")
    ax.axhline(F_HOLD, color="#C2410C", lw=1, label="needed against the weight (S 1)")
    ax.axhline(1.5 * F_HOLD, color="#C2410C", lw=1, ls="--", label="S 1.5")
    ax.axvline(2.0, color="0.5", lw=1)
    ax.text(1.97, 40, "BWA datasheet\n< 2°", ha="right", fontsize=8, color="0.3")
    ax.set_xlabel("brake play at the motor shaft [°]")
    ax.set_ylabel("grip force left per jaw [N]")
    ax.set_ylim(0, 48)
    ax.grid(alpha=0.3)
    ax.legend(frameon=False, fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "brake_play.png"), dpi=130)
    return res


if __name__ == "__main__":
    main()
