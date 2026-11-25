"""E1: robot selection and placement for the ABB IRB 1300-7/1.4.

python robot_selection.py  ->  robot_selection.json, figures/placement_map.png, figures/load_diagram.png

1. Payload: gripper (CAD build: mass, CoG, inertia) + part B against the load diagram, the axis 5/6 inertia limits
   and the wrist torques of the product specification 3HAC070393-001 rev. M (marked: mark_datasheets.py).
2. Cycle time: MoveJ pick -> place from the axis travel and the axis speeds, against the 2.0 s per way of the 5 s cycle.
3. Placement: grid over base offset x and pedestal height; at every waypoint (pick, place, each 60 mm lifted) the
   closed-form IK must exist and keep every axis >= 10 deg from its limit; the arm must not be stretched (<= 95 %).
4. Candidates: reach, payload and axis 1 speed of the alternatives (data sheet values, [verify] where not local).
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "04_Simulation"))
sys.path.insert(0, os.path.join(ROOT, "06_Cell"))
import cell_params as CP  # noqa: E402

G = 9.81
MARGIN_DEG, STRETCH_MAX, LIFT = 10.0, 0.95, 60.0
AXIS_SPEED = [249.0, 180.0, 247.0, 500.0, 415.0, 720.0]     # deg/s, IRB 1300-7/1.4, OmniCore C30 (spec p. 66)
T_WAY = 2.0                                               # s per way (cad/params.py: 5 s cycle)
JA5_MAX, JA6_MAX, D_A5_FLANGE = 0.5, 0.2, 0.09            # kg m2, kg m2, m (spec p. 51)
T45_MAX, T6_MAX = 13.0, 6.9                               # N m (spec p. 53)
A_TCP_MAX = 66.0                                          # m/s2 controlled motion (spec p. 54)
# load diagram IRB 1300-7/1.4 (spec p. 47), read off at L = 0: max Z [m] per mass, and the vertical branch (max L)
LD_MASS = [1, 2, 3, 4, 5, 6, 7]
LD_ZMAX = [0.465, 0.292, 0.217, 0.172, 0.141, 0.119, 0.101]
LD_LMAX = [0.312, 0.212, 0.168, 0.141, 0.124, 0.111, 0.100]

CANDIDATES = [  # name, reach mm, payload kg, axis 1 deg/s, controller generation, source
    ("ABB IRB 1300-7/1.4", 1400, 7.0, 249, "OmniCore (current)", "3HAC070393-001 rev. M, local"),
    ("ABB IRB 1600-6/1.45", 1450, 6.0, 150, "IRC5 (previous)", "ABB product specification PR10282 rev. E (21 Nov)"),
    ("KUKA KR 10 R1420", 1420, 10.0, 220, "KR C4 compact", "RoboDK listing; axis 1 from a distributor sheet (21 Nov)"),
    ("FANUC M-10iD/12", 1441, 12.0, 260, "R-30iB Plus", "FANUC America product page (21 Nov)"),
    ("Universal Robots UR10e", 1300, 12.5, 120, "cobot, TCP speed limited", "UR10e data sheet (21 Nov)"),
]


def payload():
    """Gripper + part: mass, CoG (Z below the flange, L radial) and inertia about the CoG, from build and MuJoCo."""
    chk = json.load(open(os.path.join(ROOT, "02_CAD_Body", "out", "checks.json")))
    import build_model as B
    import sim as S
    m_g = chk["mass_kg"]
    c_g = np.array(chk["cog_from_flange_mm"]) * 1e-3                 # gripper frame axes, origin on the flange face
    J_g = np.array(chk["inertia_cog_kgm2"])
    m_model, _ = S.load()
    bp = m_model.body("part")
    m_p = float(bp.mass[0])
    c_p = (np.array(S.GP.PART_OFFSET) + np.array(bp.ipos) * 1e3) * 1e-3
    c_p[2] -= B.flange_face_z() * 1e-3
    Rq = np.zeros(9)
    mujoco.mju_quat2Mat(Rq, bp.iquat)                                # MuJoCo keeps the CAD tensor in principal axes
    Rq = Rq.reshape(3, 3)
    J_p = Rq @ np.diag(bp.inertia) @ Rq.T
    m = m_g + m_p
    c = (m_g * c_g + m_p * c_p) / m

    def shift(J, mi, d):
        return J + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    J = shift(J_g, m_g, c_g - c) + shift(J_p, m_p, c_p - c)
    Z, L = -c[2], math.hypot(c[0], c[1])
    return dict(m_gripper=m_g, m_part=m_p, mass=m, Z=Z, L=L, J_cog=J.tolist(), cog_gripper_mm=(c_g * 1e3).tolist(),
                cog_part_mm=(c_p * 1e3).tolist())


def load_checks(pl, a_path=4.96):
    m, Z, L, J = pl["mass"], pl["Z"], pl["L"], np.array(pl["J_cog"])
    zmax = float(np.interp(m, LD_MASS, LD_ZMAX))
    lmax = float(np.interp(m, LD_MASS, LD_LMAX))
    ja5 = m * ((Z + D_A5_FLANGE) ** 2 + L ** 2) + max(J[0, 0], J[1, 1])
    ja6 = m * L ** 2 + J[2, 2]
    t45 = m * (G + a_path) * (Z + D_A5_FLANGE)                       # tool horizontal, worst lever about axis 5
    t6 = m * (G + a_path) * L
    rows = [("load diagram, Z of the CoG [m]", Z, zmax), ("load diagram, L of the CoG [m]", L, lmax),
            ("inertia about axis 5 Ja5 [kg m2]", ja5, JA5_MAX), ("inertia about axis 6 Ja6 [kg m2]", ja6, JA6_MAX),
            ("wrist torque axis 4/5 [N m] (incl. path acc.)", t45, T45_MAX), ("wrist torque axis 6 [N m]", t6, T6_MAX),
            ("payload mass [kg]", m, 7.0)]
    return [dict(check=q, value=v, limit=lim, use=v / lim) for q, v, lim in rows]


def waypoints(base):
    import sim as S
    CP.BASE = base
    up = S.wrist_above_part_bottom()
    pts = {"pick": (CP.X_PICK, 0.0, CP.H_PICK + up), "pick lifted": (CP.X_PICK, 0.0, CP.H_PICK + up + LIFT),
           "place": (CP.X_PLACE, CP.DY_CHAIN, CP.H_PLACE + up), "place lifted": (CP.X_PLACE, CP.DY_CHAIN, CP.H_PLACE + up + LIFT)}
    out = {}
    for k, w in pts.items():
        try:
            q, bad = CP.ik_tool_down(w)
        except ValueError:
            return None
        wr = [w[i] - base[i] for i in range(3)]
        reach = math.hypot(math.hypot(wr[0], wr[1]) - CP.A2_X, wr[2] - CP.A2_Z) / (CP.L2 + math.hypot(CP.L3X, CP.C4))
        marg = min(min(v - lo, hi - v) for v, (lo, hi) in zip(q, CP.ROBOT["lim"]))
        out[k] = dict(q=q, bad=bad, margin=marg, stretch=reach)
    return out


def move_time(wp):
    dq = np.abs(np.array(wp["place lifted"]["q"]) - np.array(wp["pick lifted"]["q"]))
    t_const = float(np.max(dq / np.array(AXIS_SPEED)))
    return 1.5 * t_const, int(np.argmax(dq / np.array(AXIS_SPEED))) + 1   # trapezoid with 1/3 acceleration phases


def placement(xs=np.arange(-300, 101, 25), hs=np.arange(0, 801, 50)):
    grid = np.full((len(hs), len(xs)), np.nan)
    rows = []
    for i, h in enumerate(hs):
        for j, x in enumerate(xs):
            wp = waypoints((float(x), 0.0, float(h)))
            if wp is None:
                continue
            marg = min(p["margin"] for p in wp.values())
            stretch = max(p["stretch"] for p in wp.values())
            ok = marg >= MARGIN_DEG and stretch <= STRETCH_MAX
            t, ax = move_time(wp)
            grid[i, j] = marg if stretch <= STRETCH_MAX else -1
            rows.append(dict(x=float(x), h=float(h), margin=marg, stretch=stretch, ok=ok, t_move=t, axis=ax,
                             a5=[round(p["q"][4], 1) for p in wp.values()]))
    CP.BASE = CP_BASE0                                       # waypoints() moved the base around
    return xs, hs, grid, rows


def main():
    pl = payload()
    checks = load_checks(pl)
    print(f"payload {pl['mass']:.3f} kg (gripper {pl['m_gripper']:.3f} + part {pl['m_part']:.3f}), "
          f"CoG Z {pl['Z'] * 1e3:.1f} mm, L {pl['L'] * 1e3:.1f} mm")
    for c in checks:
        print(f"  {c['check']:46s} {c['value']:8.4f}  limit {c['limit']:7.3f}  use {c['use'] * 100:5.1f} %")
    xs, hs, grid, rows = placement()
    ok = [r for r in rows if r["ok"]]
    best = min(ok, key=lambda r: (r["h"], -r["margin"])) if ok else None
    cur = next(r for r in rows if r["x"] == CP_BASE0[0] and r["h"] == CP_BASE0[2])
    print(f"current placement x {cur['x']:.0f}, pedestal {cur['h']:.0f}: margin {cur['margin']:.1f} deg, "
          f"stretch {cur['stretch'] * 100:.0f} %, MoveJ {cur['t_move']:.2f} s (axis {cur['axis']}), A5 {cur['a5']}")
    if best:
        print(f"lowest feasible pedestal: x {best['x']:.0f}, h {best['h']:.0f}: margin {best['margin']:.1f} deg, "
              f"stretch {best['stretch'] * 100:.0f} %, MoveJ {best['t_move']:.2f} s")
    import sim as S
    path = S.transfer_path(LIFT)
    v80 = tuple(0.8 * np.array(AXIS_SPEED))                 # axes at most 80 % of the data sheet speed
    t_loaded = S.TimeLaw(path, S.GP.A_PATH_REAL, vmax_deg=v80, v_lim=2.0).T   # v2000, TCP acceleration the grip is sized for
    t_empty = S.TimeLaw(path, 15.0, vmax_deg=v80, v_lim=3.0).T                # empty way back (15 m/s2, v3000)
    t_budget = 2 * T_WAY
    print(f"transfer through the vias (fence): loaded {t_loaded:.2f} s at {S.GP.A_PATH_REAL} m/s2, empty {t_empty:.2f} s, "
          f"together {t_loaded + t_empty:.2f} of {t_budget:.1f} s")
    req_reach = max(math.hypot(CP.X_PLACE - cur["x"], CP.DY_CHAIN), abs(CP.X_PICK - cur["x"]))
    cands = [dict(name=n, reach=r, payload=p, a1=a1, ctrl=c, src=s, reach_ok=r >= req_reach, payload_ok=p >= pl["mass"],
                  v_tcp=math.radians(a1) * req_reach / 1000) for n, r, p, a1, c, s in CANDIDATES]
    out = dict(payload=pl, checks=checks, placement=dict(current=cur, best=best, n_ok=len(ok), n=len(rows)),
               req_reach_mm=req_reach, candidates=cands, t_way=T_WAY, a_tcp_max=A_TCP_MAX,
               transfer=dict(t_loaded=t_loaded, t_empty=t_empty, t_budget=t_budget, via_r=S.VIA_R, via_z=S.VIA_Z))
    json.dump(out, open(os.path.join(HERE, "robot_selection.json"), "w"), indent=1)
    plots(xs, hs, grid, cur, best, pl)
    return out


def plots(xs, hs, grid, cur, best, pl):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4.6))
    g = np.ma.masked_invalid(grid)
    im = ax.pcolormesh(xs, hs, g, cmap="RdYlGn", vmin=-5, vmax=40, shading="nearest")
    cs = ax.contour(xs, hs, np.nan_to_num(grid, nan=-10), levels=[MARGIN_DEG], colors="k", linewidths=1.2)
    ax.clabel(cs, fmt=f"{MARGIN_DEG:.0f}° margin", fontsize=8)
    ax.plot(cur["x"], cur["h"], "o", ms=11, mfc="none", mec="#C2410C", mew=2.5, label="chosen: x -100, pedestal 500")
    if best:
        ax.plot(best["x"], best["h"], "s", ms=8, mfc="none", mec="k", mew=1.5, label="lowest feasible pedestal")
    ax.set_xlabel("base offset x towards the belt [mm]")
    ax.set_ylabel("pedestal height [mm]")
    ax.set_title("IRB 1300-7/1.4: smallest axis margin over all waypoints [deg]\n(white = no IK solution, red = arm > 95 % stretched)")
    fig.colorbar(im, ax=ax, label="min. axis margin [deg]")
    ax.legend(loc="lower left", fontsize=8, frameon=True)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "placement_map.png"), dpi=130)

    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    for mk, zm, lm in zip(LD_MASS, LD_ZMAX, LD_LMAX):
        ax.plot([0, lm, lm], [zm, zm, 0], color="0.4", lw=1)
        ax.text(lm * 0.55, zm + 0.004, f"{mk} kg", fontsize=8, color="0.3")
    ax.plot(pl["L"], pl["Z"], "o", color="#C2410C", ms=9, label=f"gripper + part B, {pl['mass']:.2f} kg")
    ax.set_xlim(0, 0.35)
    ax.set_ylim(0, 0.5)
    ax.set_xlabel("L-distance [m]")
    ax.set_ylabel("Z-distance below the flange [m]")
    ax.set_title("IRB 1300-7/1.4 load diagram (simplified, spec p. 47)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "figures", "load_diagram.png"), dpi=130)


CP_BASE0 = CP.BASE

if __name__ == "__main__":
    main()
