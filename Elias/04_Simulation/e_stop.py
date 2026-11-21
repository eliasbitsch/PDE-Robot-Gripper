"""Emergency stop (category 0) anywhere in the cycle: grip force needed to keep part B (review 21 Nov 2026).

python e_stop.py  ->  out/e_stop.json, figures/e_stop.png

Category 0 = power off, every axis brakes on its own and the robot leaves the path. Each axis decelerates at a
constant alpha_i (opposite to its speed) until it stands still. alpha from the ABB stop data (product specification
p. 91: max speed, arm stretched, max load, one axis moving): a lower bound, since our load is half the nominal one
and the arm is pulled in; the "x2" scenario doubles every alpha as a sensitivity. Wrist axes (not in the table):
from their top speed to rest in the axis-3 stop time. A stop is started every 10 ms of the E4 cycle (both chains);
along each stop the acceleration of part B's centre of mass and the angular acceleration of the gripper are taken
in the gripper frame (gravity included), and the normal force per jaw that holds this load is found with the
friction limit surface of the pad (grip_check.py): shear along y/z + twist about x, plus the shift of normal force
between the jaws from the acceleration along x.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

import e4_cycle as E
import sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import grip_check as GC  # noqa: E402

GP, CP, MM = S.GP, S.CP, S.MM
STOP_CAT0 = {1: (55.25, 0.40), 2: (33.39, 0.37), 3: (35.66, 0.27)}     # axis: (deg, s), spec p. 91
T_WRIST = STOP_CAT0[3][1]
ALPHA = np.array([2 * math.radians(STOP_CAT0[k][0]) / STOP_CAT0[k][1] ** 2 for k in (1, 2, 3)]
                 + [math.radians(v) / T_WRIST for v in E.VMAX[3:]])          # rad/s^2
DT, DT_STOP = 0.001, 0.002
STEP = 0.01                                                                  # s between stop instants


def part_inertia():
    import trimesh
    mesh = trimesh.load(os.path.join(HERE, "meshes", "part_b.stl"))
    rho = GP.B_MASSE / (mesh.volume * 1e-9)
    return mesh.moment_inertia * 1e-15 * rho, np.array(mesh.center_mass) * MM     # kg m^2 (about the CoG), m


def make_pose_fn(m, d):
    gid = m.body("gripper_frame").id
    adr = [m.joint(f"A{i + 1}").qposadr[0] for i in range(6)]

    def pose(q):
        d.qpos[adr] = q
        mujoco.mj_kinematics(m, d)
        return d.xpos[gid].copy(), d.xmat[gid].reshape(3, 3).copy()
    return pose


def stop_samples(q0, qd0, alpha):
    """Joint positions along the stop, every DT_STOP, until all axes stand."""
    sgn = np.sign(qd0)
    t_end = np.abs(qd0) / alpha
    ts = np.arange(0.0, t_end.max() + 2 * DT_STOP, DT_STOP)
    out = []
    for t in ts:
        tt = np.minimum(t, t_end)
        out.append(q0 + qd0 * tt - 0.5 * alpha * sgn * tt ** 2)
    return ts, np.array(out)


def load_at(pose, q_of_t, t, r_cog, I_part, g=np.array([0, 0, -GP.G])):
    """Acceleration of part B's CoG minus gravity, and the angular acceleration, in the gripper frame at time t."""
    P, R = zip(*(pose(q_of_t(t + k * DT)) for k in (-1, 0, 1)))
    c = [p + Rm @ r_cog for p, Rm in zip(P, R)]
    a = (c[2] - 2 * c[1] + c[0]) / DT ** 2
    W0 = R[1].T @ (R[2] - R[0]) / (2 * DT)                                    # skew(omega) in the body frame
    Wp, Wm = R[2].T @ (R[2] - R[1]) / DT, R[0].T @ (R[1] - R[0]) / DT
    w = lambda Wx: np.array([Wx[2, 1], Wx[0, 2], Wx[1, 0]])
    alpha_b = (w(Wp) - w(Wm)) / DT
    a_b = R[1].T @ (a - g)
    return a_b, alpha_b, w(W0)


def needed_force(a_b, alpha_b, omega_b, r_pad, I_part, hull, mu):
    """Normal force per jaw for the inertial load (mass m, CoG at r_pad from the pad centre, gripper frame)."""
    m = GP.B_MASSE
    f = m * a_b                                   # force the pads have to supply (d'Alembert), gripper frame
    M = np.cross(r_pad, f) + I_part @ alpha_b + np.cross(omega_b, I_part @ omega_b)
    load = np.array([f[1], f[2], M[0]]) / 2       # per pad: shear y, shear z, twist about the grip axis x
    return GC.normal_force(hull, load, mu) + abs(f[0]) / 2


def run(scale=1.0, mu=GP.MU_OILY):
    m, d = S.load()
    pose = make_pose_fn(m, d)
    I_part, cog_step = part_inertia()
    r_cog = np.array(GP.PART_OFFSET) * MM + cog_step                          # gripper frame
    w, h, _ = GC.pad(GP.PAD_Z, GP.PAD_W, GP.PART_OFFSET[2])
    r_pad = r_cog - np.array([0.0, 0.0, 0.5 * (GP.PAD_Z[0] + GP.PAD_Z[1]) * MM])
    hull = GC.limit_hull(w, h)
    alpha = ALPHA * scale
    res = {}
    for side, chain, _, _ in E.CHAINS:
        t, q, jaw, seg, table, _ = E.timeline(m, d, side)
        qd = np.gradient(q, t, axis=0)
        loaded = np.array([table[k]["loaded"] for k in seg])
        rows = []
        for i in range(0, len(t), int(round(STEP / E.DT))):
            if not loaded[i]:
                continue
            ts, _ = stop_samples(q[i], qd[i], alpha)
            t_end, sgn = np.abs(qd[i]) / alpha, np.sign(qd[i])

            def q_of_t(tt, q0=q[i], v0=qd[i], t_end=t_end, sgn=sgn):     # analytic: constant deceleration per axis
                tc = np.minimum(max(tt, 0.0), t_end)
                return q0 + v0 * tc - 0.5 * alpha * sgn * tc ** 2
            worst = (0.0, None)
            for tb in np.arange(DT, ts[-1] - DT, DT_STOP * 5) if ts[-1] > 3 * DT else [DT]:
                a_b, al_b, om_b = load_at(pose, q_of_t, tb, r_cog, I_part)
                n = needed_force(a_b, al_b, om_b, r_pad, I_part, hull, mu)
                if n > worst[0]:
                    worst = (n, (tb, a_b, al_b))
            n, (tb, a_b, al_b) = worst
            rows.append(dict(t=float(t[i]), segment=table[seg[i]]["segment"], N=n, t_in_stop=float(tb),
                             a_grip=[float(v) for v in a_b], a_abs=float(np.linalg.norm(a_b)),
                             alpha_grip=[float(v) for v in al_b]))
        k = int(np.argmax([r["N"] for r in rows]))
        res[chain] = dict(worst=rows[k], rows=rows)
    return res


def main():
    out = {"alpha_rad_s2": [float(a) for a in ALPHA], "mu": GP.MU_OILY, "setpoint": GP.F_GRIP_SET,
           "current_limit": GP.F_JAW_LIMIT, "scenarios": {}}
    for tag, sc, mu in (("data sheet", 1.0, GP.MU_OILY), ("x2", 2.0, GP.MU_OILY),
                        ("data sheet, grooved pad mu 0.3", 1.0, GP.MU_PAD_EXPECTED), ("x2, grooved pad mu 0.3", 2.0, GP.MU_PAD_EXPECTED)):
        r = run(sc, mu)
        out["scenarios"][tag] = {c: v["worst"] for c, v in r.items()}
        out["scenarios"][tag]["_rows"] = {c: [(x["t"], x["N"], x["a_abs"]) for x in v["rows"]] for c, v in r.items()}
        for c, v in r.items():
            w = v["worst"]
            print(f"{tag:10s} {c}: worst stop at t {w['t']:.2f} s ({w['segment']}), {w['t_in_stop'] * 1000:.0f} ms into the stop: "
                  f"|a - g| {w['a_abs']:.1f} m/s2, a_grip {np.round(w['a_grip'], 1)}, N needed {w['N']:.1f} N per jaw (S 1)")
    json.dump(out, open(os.path.join(S.B.OUT, "e_stop.json"), "w"), indent=1)
    plot(out)


def plot(out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 4))
    for (tag, sc), ls in zip(out["scenarios"].items(), ("-", "--", "-.", ":")):
        for c, col in (("chain_2", "#0f766e"), ("chain_1", "#2563eb")):
            rows = np.array(sc["_rows"][c])
            ax.plot(rows[:, 0], rows[:, 1], ls, color=col, lw=1.5, label=f"{c.replace('_', ' ')}, alpha {tag}")
    ax.axhline(out["setpoint"], color="#C2410C", lw=1.2, label=f"setpoint {out['setpoint']:.1f} N")
    ax.axhline(out["current_limit"], color="k", lw=1.0, ls=":", label=f"current limit (motor) {out['current_limit']:.1f} N")
    ax.set_xlabel("instant of the category 0 stop in the cycle [s] (loaded part of the cycle)")
    ax.set_ylabel("normal force per jaw to hold, S 1 [N]")
    ax.set_title("emergency stop anywhere in the cycle (design mu 0.2; grooved pad mu 0.3 expected, V5)", fontsize=10)
    ax.legend(fontsize=8, frameon=False, ncol=2)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "e_stop.png"), dpi=130)


if __name__ == "__main__":
    main()
