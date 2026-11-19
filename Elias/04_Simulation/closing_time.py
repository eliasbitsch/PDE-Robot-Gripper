"""Closing time of the jaws with the motor curve, and what the jaws do to the part when they arrive.

python closing_time.py  ->  out/closing_results.json, figures/closing.png, out/closing_<profile>.mp4

Requirement (handover V2): close 8 mm per jaw (open d = 10 mm to the part at d = 2 mm) and build up the grip
force within 0.3 s. The drive is position controlled (closed loop) with a current limit, which is the grip
force. Two motion profiles:
  direct     one trapezoid into the part, the current limit stops the jaws
  two-stage  fast to 1 mm before the part, then 10 mm/s into contact (typical for stepper grippers)
The reflected inertia of rotor, gearbox and brake (drive.py) sits on the jaw slides as armature.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import drive as DRV  # noqa: E402
import sim as S  # noqa: E402
import viz  # noqa: E402

GP, MM = S.GP, S.MM
T_REQ = 0.3
D_OPEN, D_GRIP = GP.D_OPEN * MM, GP.D_GRIP * MM
V_TOUCH = 0.010                       # m/s, approach speed for the last millimetre
T_SETTLE0 = 0.1                       # s, part settles on the belt before the close command
# NBR 70 Shore A pad, 3 mm thick on its plate: effective compression modulus with the shape factor of a wide
# thin pad, Gent: E_c = E (1 + 2 k S^2), E = 6 MPa, k = 0.6 (rough, for the impact estimate only)
PAD_E, PAD_T = 6.0e6, 0.003


def trapezoid(t, T, s0, s1):
    ta = T / 3
    u, _ = _trap(t, T, ta)
    return s0 + u * (s1 - s0)


def _trap(t, T, ta):
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


def profile(name, t_fast):
    """Reference jaw position d_ref(t) [m] from the close command (t = 0)."""
    end = D_GRIP - 0.0015                       # beyond the part: the current limit sets the force
    if name == "direct":
        return lambda t: trapezoid(t, t_fast, D_OPEN, end)
    pre = D_GRIP + 0.001
    def f(t):
        if t < t_fast:
            return trapezoid(t, t_fast, D_OPEN, pre)
        return max(end, pre - V_TOUCH * (t - t_fast))
    return f


def profile_torque_check(t_fast, travel, usable=0.7):
    """Peak motor speed and torque of the fast trapezoid against the motor curve."""
    vmax = 1.5 * travel / t_fast
    a = vmax / (t_fast / 3)
    w = vmax / DRV.R_P * DRV.I_G
    m_eq = DRV.M_REFLECTED + 2 * DRV.M_JAW
    t_need = m_eq * a * DRV.R_P / (DRV.I_G * DRV.ETA) + DRV.load_torque()
    return dict(v_max=vmax, rpm=w * 60 / (2 * math.pi), torque_need=t_need, torque_avail=usable * DRV.torque_avail(w))


def run(name, t_fast, force=GP.F_GRIP_SET, rec=None):
    m, d = S.load()
    q0, _ = S.q_pick()
    S.set_robot(m, d, q0, jaw=D_OPEN)
    S.place_part_in_gripper(m, d)
    ja = m.actuator("jaw").id
    m.actuator_forcerange[ja] = (-2 * force, 2 * force)
    ref = profile(name, t_fast)
    p0 = None
    t_end = T_SETTLE0 + 0.45
    log = {k: [] for k in ("t", "d", "d_ref", "fL", "fR", "push")}
    while d.time < t_end:
        tc = d.time - T_SETTLE0
        d.ctrl[ja] = D_OPEN if tc < 0 else ref(tc)
        d.ctrl[:6] = q0
        mujoco.mj_step(m, d)
        if tc >= 0 and p0 is None:
            p0 = d.body("part").xpos.copy()
        pf, _ = viz.pad_forces(m, d)
        dj = d.ten_length[m.tendon("jaws").id]
        push = float(np.linalg.norm(d.body("part").xpos - p0) / MM) if p0 is not None else 0.0
        if tc >= -0.02:
            log["t"].append(tc)
            log["d"].append(dj / MM)
            log["d_ref"].append(d.ctrl[ja] / MM)
            log["fL"].append(pf["L"][0])
            log["fR"].append(pf["R"][0])
            log["push"].append(push)
        if rec is not None and tc >= -0.02:
            rec(m, d, phase=f"closing ({name})", jaw_mm=dj / MM, slip_mm=push)
    t = np.array(log["t"])
    f = 0.5 * (np.array(log["fL"]) + np.array(log["fR"]))
    touch = np.argmax(f > 1.0)
    ok = np.abs(f - force) < 0.05 * force
    settled = next((i for i in range(len(f)) if ok[i:].all()), len(f) - 1)   # within 5 % for good
    return dict(profile=name, t_fast=t_fast, force=force, t_contact=float(t[touch]), t_grip=float(t[settled]),
                f_peak=float(max(np.max(log["fL"]), np.max(log["fR"]))), overshoot=float(np.max(f) / force),
                v_contact=float(-np.gradient(np.array(log["d"]) * MM, t)[touch]), part_push_mm=float(max(log["push"])),
                meets=bool(t[settled] <= T_REQ)), log


def impact_estimate(v):
    """Peak pad force per jaw from the jaw's kinetic energy into the pad spring (no damping, conservative)."""
    _, h = S.B.box_of("Pad_L")
    a = 4 * h[1] * h[2] * MM ** 2 if h[0] < h[1] else 4 * h[0] * h[2] * MM ** 2
    side = math.sqrt(a)
    shape = a / (4 * side * PAD_T)
    k = PAD_E * (1 + 2 * 0.6 * shape ** 2) * a / PAD_T
    m_eff = (DRV.M_REFLECTED + 2 * DRV.M_JAW) / 2
    return v * math.sqrt(k * m_eff), k


def main(video=True):
    travel = D_OPEN - D_GRIP
    mt = DRV.min_time(travel)
    print(f"motor curve: fastest 8 mm to standstill {mt['t'] * 1000:.0f} ms (v_max {mt['v_max'] * 1000:.0f} mm/s)")
    cases = [("direct", 0.25), ("direct", 0.12), ("two-stage", 0.12)]
    out, logs = [], {}
    for name, tf in cases:
        tq = profile_torque_check(tf, travel if name == "direct" else travel - 0.001)
        rec = viz.Recorder(dt=0.0005) if video and (name, tf) != ("direct", 0.25) else None
        r, lg = run(name, tf, rec=rec)
        r.update(torque=tq, f_impact_est=impact_estimate(r["v_contact"])[0])
        out.append(r)
        logs[f"{name} {tf:.2f} s"] = lg
        print(f"{name:9s} fast {tf:.2f} s: contact {r['t_contact'] * 1000:4.0f} ms at {r['v_contact'] * 1000:4.0f} mm/s, "
              f"grip force reached {r['t_grip'] * 1000:4.0f} ms ({'ok' if r['meets'] else 'TOO SLOW'}), "
              f"peak {r['f_peak']:5.1f} N (x{r['overshoot']:.2f}), estimate NBR {r['f_impact_est']:5.1f} N, "
              f"part pushed {r['part_push_mm']:.2f} mm | motor {tq['rpm']:.0f} rpm, "
              f"{tq['torque_need']:.3f} of {tq['torque_avail']:.3f} N*m")
        if rec is not None:
            tag = f"{name}_{int(tf * 1000)}ms"
            v = viz.render_video(rec, os.path.join(S.B.OUT, f"closing_{tag}.mp4"), slowmo=0.05,
                                 label=f"Closing, {name}, fast phase {tf:.2f} s")
            r["video"] = os.path.relpath(v["path"], HERE)
            r["min_clearance_mm"] = v["min_clearance_mm"]
    json.dump(dict(t_req=T_REQ, min_time=mt["t"], m_reflected=DRV.M_REFLECTED, cases=out),
              open(os.path.join(S.B.OUT, "closing_results.json"), "w"), indent=1)
    plot(logs, os.path.join(HERE, "figures", "closing.png"))
    return out, logs


def plot(logs, path, fig=None):
    import matplotlib
    if fig is None:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    for name, lg in logs.items():
        t = np.array(lg["t"]) * 1000
        ax[0].plot(t, lg["d"], label=name)
        ax[1].plot(t, 0.5 * (np.array(lg["fL"]) + np.array(lg["fR"])), label=name)
    ax[0].axhline(GP.D_GRIP, color="0.5", lw=0.8, ls=":")
    ax[0].set_ylabel("jaw opening d [mm]")
    ax[1].axhline(GP.F_GRIP_SET, color="0.5", lw=0.8, ls=":")
    ax[1].set_ylabel("pad normal force per jaw [N]")
    for a in ax:
        a.axvline(T_REQ * 1000, color="#C2410C", ls="--", lw=1)
        a.set_xlabel("t after the close command [ms]")
        a.grid(alpha=0.3)
    ax[0].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=130)
    return fig


if __name__ == "__main__":
    main()
