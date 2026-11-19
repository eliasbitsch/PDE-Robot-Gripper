"""Power failure while the part hangs in the jaws: does it stay until the spring-applied brake engages?

python power_fail.py  ->  out/power_fail.json, figures/power_fail.png

The robot holds still above the belt, the part is gripped at the setpoint. At t0 the drive loses power: only the
stepper's detent torque (6 mN*m, ~2 N per jaw through the gearbox) is left, until the brake engages t_brake later and
locks the jaws where they are. Gearbox friction is left out (conservative: it would slow the jaws down).
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import drive as DRV  # noqa: E402
import sim as S  # noqa: E402
import validate_grip as V  # noqa: E402
import viz  # noqa: E402
sys.path.insert(0, os.path.join(HERE, "..", "05_FEM"))
import load_cases as L  # noqa: E402

GP, MM = S.GP, S.MM
F_DETENT = DRV.T_DETENT * DRV.I_G * GP.GEAR_ETA / (2 * DRV.R_P)       # N per jaw the unpowered motor still holds
T_SETTLE, T_AFTER = 0.6, 0.6                                          # s
DROP = 5.0                                                            # mm of part movement = lost


def run(t_brake, force=GP.F_GRIP_SET, rec=None):
    m, d = S.load()
    m.opt.timestep = 0.0005
    q, _ = S.q_pick(V.LIFT)
    S.set_robot(m, d, q, jaw=(GP.D_GRIP + 0.3) * MM)
    S.place_part_in_gripper(m, d)
    ja, tj = m.actuator("jaw").id, m.tendon("jaws").id
    V.set_force(m, force)
    d.ctrl[ja] = -0.001
    kp = m.actuator_gainprm[ja, 0]
    t0, ref, phase = T_SETTLE, None, "gripped"
    log = {k: [] for k in ("t", "fn", "slip", "jaw")}
    while d.time < T_SETTLE + T_AFTER:
        t = d.time
        if t >= t0 and phase == "gripped":                  # power gone: detent only, no preload command
            phase = "unpowered"
            m.actuator_forcerange[ja] = (-2 * F_DETENT, 2 * F_DETENT)
        if phase == "unpowered" and t >= t0 + t_brake:      # brake locks the drive with the clamping force left now
            phase = "braked"
            m.actuator_forcerange[ja] = (-2 * L.F_BRAKE, 2 * L.F_BRAKE)
            pf_now, _ = viz.pad_forces(m, d)
            d.ctrl[ja] = d.ten_length[tj] - (pf_now["L"][0] + pf_now["R"][0]) / kp
        if phase == "unpowered":
            d.ctrl[ja] = d.ten_length[tj] - 2 * F_DETENT / kp   # the detent resists opening (closing force, sign -)
        d.ctrl[:6] = q
        mujoco.mj_step(m, d)
        if ref is None and t > 0.4:
            ref = V.part_in_gripper(m, d)
        pf, _ = viz.pad_forces(m, d)
        slip = float(np.linalg.norm(V.part_in_gripper(m, d) - ref) / MM) if ref is not None else 0.0
        if rec is not None:
            rec(m, d, phase=f"{phase} (brake after {t_brake * 1000:.0f} ms)", jaw_mm=d.ten_length[tj] / MM, slip_mm=slip)
        if t >= t0 - 0.05:
            log["t"].append(t - t0)
            log["fn"].append(0.5 * (pf["L"][0] + pf["R"][0]))
            log["slip"].append(slip)
            log["jaw"].append(d.ten_length[tj] / MM)
    fn = np.array(log["fn"])
    after = np.array(log["t"]) >= 0
    return dict(t_brake=t_brake, force=force, fn_min=float(fn[after].min()), fn_end=float(fn[-1]),
                slip_end=float(log["slip"][-1]), held=bool(log["slip"][-1] < DROP)), log


def main():
    f_hold = GP.B_MASSE * GP.G / (GP.N_BACKEN * GP.MU_OILY)
    print(f"detent holds {F_DETENT:.1f} N per jaw; the part needs {f_hold:.1f} N per jaw against its weight (mu {GP.MU_OILY})")
    out, logs = [], {}
    for tb in (0.0, 0.002, 0.005, 0.010, 0.020, 0.050):
        r, lg = run(tb)
        out.append(r)
        logs[f"brake after {tb * 1000:.0f} ms"] = lg
        print(f"brake after {tb * 1000:4.0f} ms: pad force min {r['fn_min']:5.1f} N, end {r['fn_end']:5.1f} N, "
              f"part moved {r['slip_end']:6.2f} mm  {'held' if r['held'] else 'DROPPED'}")
    json.dump(dict(f_detent=F_DETENT, f_hold=f_hold, cases=out), open(os.path.join(S.B.OUT, "power_fail.json"), "w"), indent=1)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    for name, lg in logs.items():
        t = np.array(lg["t"]) * 1000
        ax[0].plot(t, lg["fn"], label=name)
        ax[1].plot(t, lg["slip"], label=name)
    ax[0].axhline(f_hold, color="#C2410C", ls="--", lw=1, label="needed against the weight")
    ax[0].set_ylabel("pad normal force per jaw [N]")
    ax[1].set_ylabel("part movement [mm]")
    ax[1].set_yscale("symlog", linthresh=0.1)
    for a in ax:
        a.set_xlabel("t after the power loss [ms]")
        a.set_xlim(-20, 300)
        a.grid(alpha=0.3)
    ax[0].legend(frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "power_fail.png"), dpi=130)
    return out


if __name__ == "__main__":
    main()
