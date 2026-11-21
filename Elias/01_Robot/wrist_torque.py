"""Payload torques about axes 4/5 and 6 along the real cycle (review 1.14) - the open-source stand-in for RobotLoad.

python wrist_torque.py  ->  wrist_torque.json, figures/wrist_torque.png

robot_selection.py checks the wrist torque with the tool horizontal (worst lever, 70 %), which this cell never
does: the tool points down. Here the payload (gripper + part on the loaded way, gripper alone on the way back)
follows the E4 cycle (ABB MoveJ path, time law, both chains). Per sample: payload force F = m (a - g) at its centre
of mass and moment M = J alpha + omega x J omega, both from finite differences of the MuJoCo forward kinematics;
torque about a joint = axis . ((r_cog - anchor) x F + M). ABB rates 13 N m (axis 5) and 6.9 N m (axis 6) for the
payload (spec p. 53, there as a reference: the binding check is ABB RobotLoad, which we do not use).
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "04_Simulation"))
import e4_cycle as E  # noqa: E402
import robot_selection as RS  # noqa: E402
import sim as S  # noqa: E402

DT = 0.002
SMOOTH = 5                                    # samples in the moving average before differencing (10 ms)


def smooth(x, n=SMOOTH):
    k = np.ones(n) / n
    return np.apply_along_axis(lambda v: np.convolve(np.pad(v, (n // 2, n - 1 - n // 2), mode="edge"), k, "valid"), 0, x)


def run(side):
    m, d = S.load()
    E.DT = DT
    t, q, _, seg, table, _ = E.timeline(m, d, side)
    loaded = np.array([table[k]["loaded"] for k in seg])
    pl = RS.payload()
    import build_model as B
    fz = B.flange_face_z() * 1e-3
    mg, mp = pl["m_gripper"], pl["m_part"]
    cg = np.array(pl["cog_gripper_mm"]) * 1e-3 + [0, 0, fz]          # gripper frame, origin of the gripper model
    cp = np.array(pl["cog_part_mm"]) * 1e-3 + [0, 0, fz]
    jg = np.array(json.load(open(os.path.join(ROOT, "02_CAD_Body", "out", "checks.json")))["inertia_cog_kgm2"])
    bf = m.body("gripper_frame").id
    adr = [m.joint(f"A{i + 1}").qposadr[0] for i in range(6)]
    j5, j6 = m.joint("A5").id, m.joint("A6").id
    pos, rot, ax5, an5, ax6, an6 = [], [], [], [], [], []
    for qi in q:
        d.qpos[adr] = qi
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        pos.append(d.xpos[bf].copy()); rot.append(d.xmat[bf].reshape(3, 3).copy())
        ax5.append(d.xaxis[j5].copy()); an5.append(d.xanchor[j5].copy())
        ax6.append(d.xaxis[j6].copy()); an6.append(d.xanchor[j6].copy())
    pos, rot = np.array(pos), np.array(rot)
    g = np.array([0, 0, -S.GP.G])
    out = {"t": t, "T5": [], "T6": [], "loaded": loaded}
    for case, mask in (("loaded", loaded), ("empty", ~loaded)):
        mass = mg + mp if case == "loaded" else mg
        c = (mg * cg + mp * cp) / (mg + mp) if case == "loaded" else cg
        r = pos + np.einsum("nij,j->ni", rot, c)
        a = np.gradient(np.gradient(smooth(r), DT, axis=0), DT, axis=0)
        # angular velocity from R_dot R^T (skew part)
        rd = np.gradient(smooth(rot.reshape(len(t), 9)).reshape(-1, 3, 3), DT, axis=0)
        w_sk = np.einsum("nij,nkj->nik", rd, rot)
        w = np.c_[w_sk[:, 2, 1] - w_sk[:, 1, 2], w_sk[:, 0, 2] - w_sk[:, 2, 0], w_sk[:, 1, 0] - w_sk[:, 0, 1]] / 2
        al = np.gradient(smooth(w), DT, axis=0)
        J = jg if case == "empty" else jg + 0.0                       # part inertia is small next to its lever (in F)
        jw = np.einsum("nij,jk,nlk->nil", rot, J, rot)
        mom = np.einsum("nij,nj->ni", jw, al) + np.cross(w, np.einsum("nij,nj->ni", jw, w))
        f = mass * (a - g)
        for key, ax, an in (("T5", ax5, an5), ("T6", ax6, an6)):
            tau = np.einsum("ni,ni->n", np.array(ax), np.cross(r - np.array(an), f) + mom)
            out.setdefault(key + "_" + case, np.where(mask, tau, np.nan))
    out["T5"] = np.fmax(np.nan_to_num(np.abs(out["T5_loaded"])), np.nan_to_num(np.abs(out["T5_empty"])))
    out["T6"] = np.fmax(np.nan_to_num(np.abs(out["T6_loaded"])), np.nan_to_num(np.abs(out["T6_empty"])))
    return out


def main():
    res = {}
    runs = {}
    for side, chain in ((+1, "chain_2"), (-1, "chain_1")):
        o = run(side)
        runs[chain] = o
        i5, i6 = int(np.argmax(o["T5"])), int(np.argmax(o["T6"]))
        static5 = float(np.nanmax(np.abs(o["T5_loaded"][:5])))
        res[chain] = dict(t5_max=float(o["T5"][i5]), t5_at=float(o["t"][i5]), t6_max=float(o["T6"][i6]),
                          t6_at=float(o["t"][i6]), t5_static_pick=static5)
        print(f"{chain}: axis 5 max {o['T5'][i5]:.2f} N m at t {o['t'][i5]:.2f} s ({o['T5'][i5] / RS.T45_MAX * 100:.0f} % "
              f"of {RS.T45_MAX}), static at the pick {static5:.2f} N m; axis 6 max {o['T6'][i6]:.2f} N m "
              f"({o['T6'][i6] / RS.T6_MAX * 100:.0f} % of {RS.T6_MAX})")
    pl = RS.payload()
    t45_horiz = [c for c in RS.load_checks(pl) if c["check"].startswith("wrist torque axis 4/5")][0]["value"]
    res["tool_horizontal_bound"] = t45_horiz
    print(f"bound with the tool horizontal (robot_selection.py): {t45_horiz:.2f} N m")
    json.dump(res, open(os.path.join(HERE, "wrist_torque.json"), "w"), indent=1)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 3.2))
    o = runs["chain_2"]
    ax.plot(o["t"], o["T5"], color="#0f766e", label="axis 5 (payload torque)")
    ax.plot(o["t"], o["T6"], color="#64748b", label="axis 6")
    ax.axhline(RS.T45_MAX, color="#C2410C", ls="--", lw=1, label=f"ABB {RS.T45_MAX} N m (axis 4/5)")
    ax.axhline(t45_horiz, color="#C2410C", ls=":", lw=1, label=f"tool horizontal bound {t45_horiz:.1f} N m")
    ax.set_xlabel("t in the cycle [s] (chain 2)")
    ax.set_ylabel("torque [N m]")
    ax.grid(alpha=0.3)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "wrist_torque.png"), dpi=130)
    return res


if __name__ == "__main__":
    main()
