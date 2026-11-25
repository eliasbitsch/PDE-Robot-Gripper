"""E4: the full work cycle measured - collision log, axis check and cycle time (no rendering).

python e4_cycle.py  ->  out/e4_cycle.json, out/e4_collision_log.csv, figures/e4_clearance.png, figures/e4_axes.png

Cycle: close jaws on the belt, lift 60 mm, transfer through the vias, lower onto the carrier, open, lift (empty),
transfer back, lower around the next part. Every motion runs on S.TimeLaw: loaded with the TCP acceleration the
grip force is sized for (4.96 m/s2), empty with 15 m/s2, every axis at most 80 % of its speed. The robot follows the joint path kinematically; distances
to the cell (convex hulls, so on the safe side) every 10 ms. The grip itself is validate_grip.py's subject.
"""
import csv
import json
import os

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline

import clearance as C
import sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
GP, CP = S.GP, S.CP
A_LOADED, A_EMPTY = GP.A_PATH_REAL, 15.0
V_LOADED, V_EMPTY = 2.0, 3.0           # m/s, programmed TCP speed: RAPID v2000 loaded, v3000 empty (abb_path.py)


def v_lim(a_lim):
    return V_LOADED if a_lim == A_LOADED else V_EMPTY
SPEED_USE = 0.8                     # axes at most 80 % of the data sheet speed: the cycle has time to spare
T_CLOSE = 0.40                      # s, tolerant two-stage close (slow phase over the +-5 mm position tolerance) until
                                    # the grip force stands (closing_time.py: 0.373 s; 0.30 s before review 1.11)
T_OPEN = 0.20                       # s, open by 8 mm (cad/params.py T_LOESEN)
LIFT = 60.0
DT = 0.01
VMAX = np.array([249.0, 180.0, 247.0, 500.0, 415.0, 720.0])   # deg/s, spec p. 66
CYCLE = GP.TAKTZEIT


def line(qa, qb):
    return CubicSpline([0.0, 1.0], np.array([qa, qb]), bc_type="clamped")


def segments(side=+1):
    """(name, path q(u) or a fixed q, a_lim or a duration, jaw opening [mm], loaded); side +1 chain 2, -1 chain 1."""
    q0, _ = S.q_pick()
    q1, _ = S.q_pick(LIFT)
    q2, _ = S.q_place(LIFT, side)
    q3, _ = S.q_place(0.0, side)
    fwd = S.transfer_path(LIFT, side)
    def back(u, nu=0):                                  # the same path backwards
        return fwd(1.0 - np.asarray(u), nu) * (-1) ** nu
    return [("close on the belt", q0, T_CLOSE, GP.D_GRIP, True),
            ("lift", line(q0, q1), A_LOADED, GP.D_GRIP, True),
            ("transfer (loaded)", fwd, A_LOADED, GP.D_GRIP, True),
            ("lower onto the carrier", line(q2, q3), A_LOADED, GP.D_GRIP, True),
            ("open", q3, T_OPEN, GP.D_OPEN, False),
            ("lift (empty)", line(q3, q2), A_EMPTY, GP.D_OPEN, False),
            ("transfer back (empty)", back, A_EMPTY, GP.D_OPEN, False),
            ("lower around the next part", line(q1, q0), A_EMPTY, GP.D_OPEN, False)]


def timeline(m, d, side=+1):
    """Sampled cycle: t, q (rad), jaw [mm], segment index."""
    ts, qs, jaws, seg, table = [], [], [], [], []
    t0 = 0.0
    for k, (name, path, lim, jaw, loaded) in enumerate(segments(side)):
        if isinstance(path, np.ndarray):
            T = lim
            for t in np.arange(0.0, T, DT):
                ts.append(t0 + t); qs.append(path); jaws.append(jaw); seg.append(k)
        else:
            law = S.TimeLaw(path, lim, m=m, d=d, vmax_deg=tuple(SPEED_USE * VMAX), v_lim=v_lim(lim))
            T = law.T
            for t in np.arange(0.0, T, DT):
                u, _ = law(t)
                ts.append(t0 + t); qs.append(path(u)); jaws.append(jaw); seg.append(k)
        table.append(dict(segment=name, t_start=t0, duration=T, loaded=loaded,
                          a_lim=None if isinstance(path, np.ndarray) else lim))
        t0 += T
    return np.array(ts), np.array(qs), np.array(jaws), np.array(seg), table, t0


CHAINS = ((+1, "chain_2", "carrier", ""), (-1, "chain_1", "carrier_1", "_chain1"))


def run_chain(m, d, side, chain, carrier, suffix):
    t, q, jaw, seg, table, t_cycle = timeline(m, d, side)
    names = [s["segment"] for s in table]
    rg = C.robot_geoms(m)
    obs = {o: m.geom(o).id for o in C.OBSTACLES if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, o) >= 0}
    dist = {o: np.zeros(len(t)) for o in obs}
    ft = np.zeros(6)
    for i in range(len(t)):
        S.set_robot(m, d, q[i], jaw=jaw[i] * S.MM)
        for o, og in obs.items():
            dist[o][i] = min(mujoco.mj_geomDistance(m, d, g, og, C.D_MAX, ft) for g in rg) * 1000
    qd = np.degrees(np.gradient(q, t, axis=0))
    lim = np.array(CP.ROBOT["lim"], float)
    qdeg = np.degrees(q)
    margin = np.minimum(qdeg - lim[:, 0], lim[:, 1] - qdeg)
    speed_use = np.abs(qd) / VMAX

    # collision log: per segment and obstacle the smallest distance and when
    log = []
    for k, n in enumerate(names):
        w = seg == k
        for o in obs:
            i = int(np.argmin(np.where(w, dist[o], np.inf)))
            log.append(dict(segment=n, obstacle=o, min_mm=float(dist[o][i]), t=float(t[i])))
    # where the jaws are meant to be close: at the part on the belt, at the target carrier and its chain
    expected = {("close on the belt", "belt"), ("lift", "belt"), ("lower around the next part", "belt")}
    expected |= {(sg, o) for sg in ("lower onto the carrier", "open", "lift (empty)") for o in (carrier, chain)}
    worst = min((r for r in log if (r["segment"], r["obstacle"]) not in expected), key=lambda r: r["min_mm"])
    with open(os.path.join(S.B.OUT, f"e4_collision_log{suffix}.csv"), "w", newline="") as fh:
        w = csv.writer(fh, delimiter=";")
        w.writerow(["t [s]", "segment", "jaw [mm]"] + [f"{o} [mm]" for o in obs] + [f"A{j + 1} [deg]" for j in range(6)])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.2f}", names[seg[i]], f"{jaw[i]:.1f}"] + [f"{dist[o][i]:.1f}" for o in obs]
                       + [f"{v:.2f}" for v in qdeg[i]])
    out = dict(chain=chain, a6_place_deg=float(qdeg[seg == 4][0, 5]), cycle_time=t_cycle, budget=CYCLE, segments=table,
               collision_log=log, worst_unexpected=worst, axis_margin_min=[float(v) for v in margin.min(axis=0)],
               axis_speed_use_max=[float(v) for v in speed_use.max(axis=0)],
               any_contact=bool(min(r["min_mm"] for r in log) <= 0.0))
    print(f"--- {chain}: cycle {t_cycle:.2f} s of {CYCLE:.1f} s, A6 at the place {out['a6_place_deg']:+.1f} deg")
    for s_ in table:
        print(f"  {s_['segment']:28s} {s_['t_start']:5.2f} + {s_['duration']:4.2f} s")
    print("closest approach per obstacle (whole cycle):")
    for o in obs:
        i = int(np.argmin(dist[o]))
        print(f"  {o:12s} {dist[o][i]:7.1f} mm  at t {t[i]:.2f} s ({names[seg[i]]})")
    print(f"closest unexpected: {worst['obstacle']} {worst['min_mm']:.1f} mm in '{worst['segment']}'")
    print("axis margin min [deg]:", np.round(margin.min(axis=0), 1), " speed use max:", np.round(speed_use.max(axis=0), 2))
    plots(t, dist, seg, names, margin, speed_use, t_cycle, suffix, chain)
    return out


def main():
    m, d = S.load()
    res = {chain: run_chain(m, d, side, chain, carrier, suffix) for side, chain, carrier, suffix in CHAINS}
    out = dict(cycle_time=max(r["cycle_time"] for r in res.values()), budget=CYCLE,
               any_contact=any(r["any_contact"] for r in res.values()), **res)
    json.dump(out, open(os.path.join(S.B.OUT, "e4_cycle.json"), "w"), indent=1)
    return out


def plots(t, dist, seg, names, margin, speed_use, t_cycle, suffix="", chain="chain_2"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = plt.cm.tab10(np.linspace(0, 1, 10))

    def shade(ax):
        for k, n in enumerate(names):
            w = np.where(seg == k)[0]
            if len(w):
                ax.axvspan(t[w[0]], t[w[-1]] + DT, color="0.92" if k % 2 else "white", zorder=0)
                ax.text(0.5 * (t[w[0]] + t[w[-1]]), 1.0, n, rotation=90, fontsize=6.5, color="0.35", va="top",
                        ha="center", transform=ax.get_xaxis_transform())
        ax.axvline(CYCLE, color="#C2410C", lw=1.2, ls="--")
        ax.set_xlim(0, max(CYCLE, t_cycle) + 0.05)

    fig, ax = plt.subplots(figsize=(11, 4.2))
    shade(ax)
    for j, (o, v) in enumerate(dist.items()):
        ax.plot(t, v, color=cols[j % 10], lw=1.4, label=o)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_ylim(-20, 520)
    ax.set_xlabel("t in the cycle [s]   (dashed: 5 s cycle)")
    ax.set_ylabel("smallest distance robot + gripper [mm]")
    ax.legend(ncol=5, fontsize=7.5, frameon=False, loc="upper left", bbox_to_anchor=(0, -0.16))
    fig.tight_layout()
    ax.set_title(f"place on {chain.replace('_', ' ')}", fontsize=9, loc="right")
    fig.savefig(os.path.join(HERE, "figures", f"e4_clearance{suffix}.png"), dpi=130)

    fig, ax = plt.subplots(2, 1, figsize=(11, 5.2), sharex=True)
    for a in ax:
        shade(a)
    for j in range(6):
        ax[0].plot(t, margin[:, j], color=cols[j], label=f"A{j + 1}")
        ax[1].plot(t, speed_use[:, j] * 100, color=cols[j], label=f"A{j + 1}")
    ax[0].axhline(10, color="k", lw=0.8, ls=":")
    ax[0].set_ylabel("margin to the axis limit [deg]")
    ax[0].set_ylim(0, 200)
    ax[1].axhline(100, color="k", lw=0.8)
    ax[1].set_ylabel("axis speed / data sheet [%]")
    ax[1].set_xlabel("t in the cycle [s]")
    ax[0].legend(ncol=6, fontsize=8, frameon=False, loc="lower right")
    fig.tight_layout()
    ax[0].set_title(f"place on {chain.replace('_', ' ')}", fontsize=9, loc="right")
    fig.savefig(os.path.join(HERE, "figures", f"e4_axes{suffix}.png"), dpi=130)


if __name__ == "__main__":
    main()
