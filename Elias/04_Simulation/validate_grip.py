"""Grip validation in MuJoCo: does part B stay in the jaws over the real robot path?

python validate_grip.py  ->  out/grip_results.json, figures/grip_slip.png

Cases (force = jaw force limit of the current-limited drive, per jaw):
  nominal      F = F_GRIP_OILY (S = 2 in the hand calculation): the part must not slip
  limit        F = F_GRIP_OILY / S: no safety left, slip expected to start
  e-stop       F nominal, category 0 stop in the middle of the transfer with the ABB axis decelerations; the brake locks the jaws
Path: pick on the buffer belt, lift, transfer to the workpiece carrier in T_EINWEG with a trapezoidal profile
(A_BAHN, V_BAHN from cad/params.py), lower. The robot follows the joint path through stiff position control.
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "05_FEM"))
import sim as S  # noqa: E402
import load_cases as L  # noqa: E402

GP, MM = S.GP, S.MM
LIFT = 60.0                                   # mm above pick and place
T_CLOSE, T_SETTLE, T_LIFT = 0.25, 0.25, 0.4   # s
T_TRANSFER = GP.T_EINWEG                      # 2.0 s
TA = GP.V_BAHN / GP.A_BAHN                    # accel phase of the trapezoid
SLIP_OK = 2.0                                 # mm: soft contacts creep a little near the friction limit; above that it slips


def trapezoid(t, T, ta):
    """Normalised path s(t) in [0, 1] with constant accel / cruise / decel (ta each end)."""
    vmax = 1.0 / (T - ta)
    a = vmax / ta
    if t <= 0:
        return 0.0, 0.0
    if t < ta:
        return 0.5 * a * t * t, a * t
    if t < T - ta:
        return 0.5 * a * ta * ta + vmax * (t - ta), vmax
    if t < T:
        r = T - t
        return 1.0 - 0.5 * a * r * r, a * r
    return 1.0, 0.0


def part_in_gripper(m, d):
    b = m.body("gripper_frame").id
    R = d.xmat[b].reshape(3, 3)
    return R.T @ (d.body("part").xpos - d.xpos[b])


def set_force(m, f_jaw):
    """One actuator drives both coupled jaws (like the pinion drives both racks): it needs 2 x the force per jaw."""
    a = m.actuator("jaw").id
    m.actuator_forcerange[a] = (-2 * f_jaw, 2 * f_jaw)


def run(case, force, estop=False, mu=None, t_transfer=None, dt_log=0.005, rec=None):
    """One cycle. mu: pad friction (default MU_PAD), t_transfer: transfer time (default T_EINWEG, ramps T/3),
    rec: viz.Recorder for the slow-motion video."""
    m, d = S.load()
    m.opt.timestep = 0.0005
    mu = S.B.MU_PAD if mu is None else mu
    m.pair_friction[:, 0:2] = mu
    T_TR = T_TRANSFER if t_transfer is None else t_transfer
    TA_ = T_TR / 3.0
    q0, _ = S.q_pick()
    q1, _ = S.q_pick(LIFT)
    q2, _ = S.q_place(LIFT)
    q3, _ = S.q_place()
    # jaws start just before the part: the closing stroke and its impact are closing_time.py's subject
    S.set_robot(m, d, q0, jaw=(GP.D_GRIP + 0.3) * MM)
    S.place_part_in_gripper(m, d)
    set_force(m, force)
    ja = m.actuator("jaw").id
    d.ctrl[ja] = -0.001                       # close against the part: the force limit sets the grip force

    t_lift0 = T_CLOSE + T_SETTLE
    t_tr0 = t_lift0 + T_LIFT
    t_low0 = t_tr0 + T_TR
    t_end = t_low0 + T_LIFT + 0.3
    t_stop = t_tr0 + 0.5 * T_TR if estop else None
    L_tcp = np.linalg.norm(np.array([GP.X_PLACE if hasattr(GP, "X_PLACE") else S.CP.X_PLACE, S.CP.DY_CHAIN])
                           - np.array([S.CP.X_PICK, 0.0])) * MM          # rough TCP path length per unit s
    log = {k: [] for k in ("t", "slip", "dz", "tcp", "jaw_force", "part_z", "util", "util_pad")}
    pads = {m.geom("pad_L").id, m.geom("pad_R").id}
    part_g = m.geom("part").id
    f6 = np.zeros(6)
    ref = None
    s_stop, sd_stop, t_halt, sdd_stop = None, None, None, None
    n = int(t_end / m.opt.timestep)
    for k in range(n):
        t = k * m.opt.timestep
        if t < t_lift0:
            q = q0
        elif t < t_tr0:
            s, _ = trapezoid(t - t_lift0, T_LIFT, T_LIFT / 3)
            q = q0 + s * (q1 - q0)
        elif t < t_low0 or (estop and t_halt is not None):
            if estop and t >= t_stop:
                if s_stop is None:                             # e-stop: decelerate the path at A_STOP, brake the jaws
                    s_stop, sd_stop = trapezoid(t_stop - t_tr0, T_TR, TA_)
                    # brake: the jaws stay where they are, up to the brake limit (2 x per jaw, see set_force)
                    # the brake locks the drive under load: the elastic preload (the grip force) stays
                    d.ctrl[ja] = d.ten_length[m.tendon("jaws").id] - 2 * force / m.actuator_gainprm[ja, 0]
                    m.actuator_forcerange[ja] = (-2 * L.F_BRAKE, 2 * L.F_BRAKE)
                    t_halt = t
                    # category 0: every axis brakes at its ABB deceleration; the path stops with the slowest one
                    dq = np.abs(q2 - q1)
                    alpha = np.array([L.ALPHA_STOP.get(i + 1, L.ALPHA_STOP[3]) for i in range(6)])
                    sdd_stop = float(np.min(alpha[dq > 1e-6] / dq[dq > 1e-6]))
                tau = t - t_halt
                sdd = sdd_stop
                tau_end = sd_stop / sdd
                tau = min(tau, tau_end)
                s = s_stop + sd_stop * tau - 0.5 * sdd * tau * tau
            else:
                s, _ = trapezoid(t - t_tr0, T_TR, TA_)
            q = q1 + s * (q2 - q1)
        else:
            s, _ = trapezoid(t - t_low0, T_LIFT, T_LIFT / 3)
            q = q2 + s * (q3 - q2)
        d.ctrl[:6] = q
        mujoco.mj_step(m, d)
        if rec is not None:
            ph = ("grip" if t < t_lift0 else "lift" if t < t_tr0 else
                  ("E-STOP, jaws braked" if t_halt is not None else "transfer") if t < t_low0 or t_halt is not None else "lower")
            rel = part_in_gripper(m, d)
            rec(m, d, phase=ph, mu=mu, jaw_mm=d.ten_length[m.tendon("jaws").id] / MM,
                slip_mm=float(np.linalg.norm(rel - ref) / MM) if ref is not None else 0.0)
        if t >= T_CLOSE + T_SETTLE * 0.9 and ref is None:
            ref = part_in_gripper(m, d)
        if k % int(dt_log / m.opt.timestep) == 0:
            rel = part_in_gripper(m, d)
            log["t"].append(t)
            log["slip"].append(float(np.linalg.norm(rel - ref) / MM) if ref is not None else 0.0)
            log["dz"].append(float((rel - ref)[2] / MM) if ref is not None else 0.0)
            log["tcp"].append(d.site("tcp").xpos.copy())
            log["jaw_force"].append(float(d.actuator_force[ja]))
            log["part_z"].append(float(d.body("part").xpos[2] / MM))
            # friction use: net = |vector sum of the pad friction forces| / (mu sum Fn), the load the friction carries
            # (hand calculation); worst pad = same per pad, includes the internal locked-in shear of the statically
            # indeterminate two-pad grip, which depends on how the jaws closed
            fn, ftv = {"L": 0.0, "R": 0.0}, {"L": np.zeros(3), "R": np.zeros(3)}
            for i in range(d.ncon):
                c = d.contact[i]
                if {c.geom1, c.geom2} & pads and part_g in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, i, f6)
                    side = "L" if m.geom("pad_L").id in (c.geom1, c.geom2) else "R"
                    fn[side] += f6[0]
                    ftv[side] += f6[1] * c.frame[3:6] + f6[2] * c.frame[6:9]
            n = fn["L"] + fn["R"]
            log["util"].append(float(np.linalg.norm(ftv["L"] + ftv["R"]) / (mu * n)) if n > 1e-6 else float("nan"))
            log["util_pad"].append(max(float(np.linalg.norm(ftv[k]) / (mu * fn[k])) if fn[k] > 1e-6 else 0.0 for k in "LR"))
    tcp = np.array(log["tcp"])
    tt = np.array(log["t"])
    acc = np.gradient(np.gradient(tcp, tt, axis=0), tt, axis=0)
    a_mag = np.linalg.norm(acc, axis=1)
    log["a_tcp"] = a_mag.tolist()
    win = (tt > t_lift0) & (tt < t_low0 + T_LIFT)
    slip = np.array(log["slip"])
    util = np.nan_to_num(np.array(log["util"]))
    k = max(1, int(0.05 / dt_log))                          # 50 ms average: single contact impulses do not count
    u_s = np.convolve(util, np.ones(k) / k, mode="same")
    u_max = float(u_s[win].max())
    u_cruise = float(np.median(util[win]))
    u_pad = float(np.convolve(np.nan_to_num(np.array(log["util_pad"])), np.ones(k) / k, mode="same")[win].max())
    dropped = bool(log["part_z"][-1] < S.CP.H_PLACE - 50)
    res = dict(case=case, force=force, estop=estop, slip_max=float(slip[win].max()),
               slip_end=float(slip[-1]), util_max=u_max, util_median=u_cruise, util_pad_max=u_pad,
               safety_eff=float(1.0 / u_max) if u_max > 0 else float("inf"),
               held=bool(not dropped and slip[win].max() < SLIP_OK),
               a_tcp_max=float(a_mag[win].max()), dropped=dropped)
    return res, log


def main():
    os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
    cases = [("nominal", GP.F_GRIP_OILY, False), ("limit (no safety)", GP.F_GRIP_OILY / GP.S_SICHERHEIT, False),
             ("full current", GP.F_JAW_MAX, False), ("e-stop", GP.F_GRIP_OILY, True), ("e-stop full current", GP.F_JAW_MAX, True)]
    out, logs = [], {}
    for name, f, es in cases:
        r, lg = run(name, f, es)
        out.append(r)
        logs[name] = lg
        print(f"{name:20s} F {f:5.1f} N  friction use median {r['util_median']:4.2f} peak {r['util_max']:4.2f} "
              f"(S_eff {r['safety_eff']:4.2f}) worst pad {r['util_pad_max']:4.2f}  "
              f"slip max {r['slip_max']:7.2f} mm  "
              f"a_tcp max {r['a_tcp_max']:5.2f} m/s2  {'held' if r['held'] else 'SLIPS'}{'  DROPPED' if r['dropped'] else ''}")
    json.dump(dict(mu=S.B.MU_PAD, a_bahn=GP.A_BAHN, a_stop=L.A_STOP, cases=out),
              open(os.path.join(S.B.OUT, "grip_results.json"), "w"), indent=1)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 3.6))
    for name, lg in logs.items():
        ax.plot(lg["t"], lg["slip"], label=name)
    ax.axhline(SLIP_OK, color="#C2410C", ls="--", lw=1, label=f"{SLIP_OK} mm")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("part movement in the jaws [mm]")
    ax.set_yscale("symlog", linthresh=0.1)
    ax.grid(alpha=0.3)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "grip_slip.png"), dpi=130)



def video(case, force, path, slowmo=0.25, **kw):
    """Slow-motion multi-view video of one cycle (viz.py): cameras on the gripper, contacts, clearances."""
    import viz
    rec = viz.Recorder(dt=slowmo / 30)
    res, _ = run(case, force, rec=rec, **kw)
    return viz.render_video(rec, path, slowmo=slowmo, label=f"{case}, {force:.1f} N per jaw"), res


if __name__ == "__main__":
    main()
