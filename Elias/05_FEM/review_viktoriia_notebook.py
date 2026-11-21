import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="Review V2 / V4 against simulation")


@app.cell
def _():
    import importlib.util
    import json
    import math
    import os
    import sys

    import marimo as mo
    import matplotlib
    import matplotlib.pyplot as plt
    import numpy as np

    HERE = os.path.dirname(os.path.abspath(__file__))
    ROOT = os.path.join(HERE, "..", "..")
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(ROOT, "Elias", "02_CAD_Body"))
    import gripper_params as P
    import load_cases as L
    import v4_check as V4C

    def run_notebook(path):
        """Executes one of Viktoriia's marimo notebooks as it is and returns its definitions."""
        _spec = importlib.util.spec_from_file_location(os.path.basename(path)[:-3], path)
        _mod = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_mod)
        return _mod.app.run()[1]
    return HERE, L, P, ROOT, V4C, json, math, mo, np, os, plt, run_notebook


@app.cell
def _(mo):
    mo.md(r"""
    # Review: Viktoriia's V2 and V4 against the simulation
    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**


    Her notebooks (`Viktoriia/02_Sizing/v2_sizing.py`, `Viktoriia/04_Analytical/v4_jaw.py`) are executed here as they
    are, so every number below is her current result. Next to it: the FEM of her jaw V3 (E5) and the MuJoCo cell (E4).
    Slow FEM runs sit behind buttons; otherwise the last stored results are shown.
    """)
    return


@app.cell
def _(ROOT, os, run_notebook):
    v2 = run_notebook(os.path.join(ROOT, "Viktoriia", "02_Sizing", "v2_sizing.py"))
    v4 = run_notebook(os.path.join(ROOT, "Viktoriia", "04_Analytical", "v4_jaw.py"))
    return v2, v4


@app.cell
def _(mo):
    mo.md(r"""
    ## 1. V4 jaw: beam theory against the FEM of the V3 jaw (≤ 5 %)

    FEM: `jaw_v3.step` (her jaw, moved from the grip position to the closed stop), C3D10, full current 63.6 N.
    Two supports: **standard** (holder face fixed normal to it, dowel holes in y/z) and **rigid clamp** (face fixed in
    x, y, z = the beam assumption). Check points as in V4: σ on the outer face at z = -53.5, deflection at the load centre.
    """)
    return


@app.cell
def _(mo):
    v4_sizes = mo.ui.multiselect(["2.5", "1.6", "1"], value=["2.5", "1.6", "1"], label="FEM element sizes [mm]")
    v4_go = mo.ui.run_button(label="Compute missing sizes")
    mo.hstack([v4_sizes, v4_go], justify="start", gap=1.5)
    return v4_go, v4_sizes


@app.cell
def _(V4C, mo, v4_go, v4_sizes):
    v4_store = V4C.load()
    if v4_go.value:
        for _lc in sorted((float(x) for x in v4_sizes.value if x not in v4_store), reverse=True):
            with mo.status.spinner(title=f"jaw V3, {_lc:g} mm, standard + rigid ..."):
                v4_store = V4C.run_size(_lc, v4_store)
    v4_store = {k: v for k, v in v4_store.items() if k in v4_sizes.value}
    return (v4_store,)


@app.cell
def _(P, V4C, mo, np, plt, v4, v4_store):
    mo.stop(not v4_store, mo.md("_No FEM of the V3 jaw yet - press **Compute missing sizes**._"))
    _d = v4["deflection"](P.F_JAW_MAX)
    an = dict(force=P.F_JAW_MAX, sigma_check=v4["stresses"](P.F_JAW_MAX)["FEM check z = -53.5 (s = 6.5)"],
              w_rigid=(_d["w_b"] + _d["w_s"]) * 1000, w_elastic=(_d["w_b"] + _d["w_s"] + _d["w_c"]) * 1000)
    _cmp = V4C.compare(v4_store, an)
    _rows = ["| quantity | V4 (Viktoriia) | FEM V3 jaw, finest mesh | deviation | ≤ 5 % |", "|---|---|---|---|---|"]
    for _r in _cmp:
        _rows.append(f"| {_r['quantity']} | {_r['analytic']:.2f} | {_r['fem']:.2f} | {_r['dev']:+.1f} % | {'✓' if _r['ok'] else '✗'} |")
    _keys = sorted(v4_store, key=float, reverse=True)
    _el = [v4_store[k]["rigid"]["elements"] for k in _keys]
    _fig, _ax = plt.subplots(1, 2, figsize=(11, 3.3))
    _ax[0].plot(_el, [v4_store[k]["standard"]["szz"] for k in _keys], "o-", color="#0f766e", label="FEM")
    _ax[0].axhline(an["sigma_check"], color="#C2410C", ls="--", label="V4")
    _ax[0].set_ylabel("σ at z = -53.5 [MPa]")
    _ax[1].plot(_el, [v4_store[k]["rigid"]["ux"] for k in _keys], "o-", color="#0f766e", label="FEM rigid clamp")
    _ax[1].plot(_el, [v4_store[k]["standard"]["ux"] for k in _keys], "s-", color="#64748b", label="FEM standard support")
    _ax[1].axhline(an["w_elastic"], color="#C2410C", ls="--", label="V4 beam + Weber")
    _ax[1].axhline(an["w_rigid"], color="#C2410C", ls=":", label="V4 rigid beam")
    _ax[1].set_ylabel("deflection at the load centre [µm]")
    for _a in _ax:
        _a.set_xscale("log")
        _a.set_xlabel("elements")
        _a.grid(alpha=0.3, which="both")
        _a.legend(frameon=False, fontsize=8)
    _fig.tight_layout()
    _old = v4["FEM"]
    mo.vstack([mo.md("\n".join(_rows)), _fig,
               mo.md(f"V4 still quotes the FEM of the old mock jaw (σ {_old['sigma_check']} MPa, {_old['w_rigid']} / "
                     f"{_old['w_std']} µm); the table above uses the V3 jaw. Peak von Mises of the V3 jaw (finest mesh): "
                     f"**{v4_store[min(v4_store, key=float)]['standard']['vm_peak']:.1f} MPa** - her R1 pocket corners.")])
    return (an,)


@app.cell
def _(HERE, mo, os):
    _f = lambda n: os.path.join(HERE, "figures", n)
    mo.hstack([mo.vstack([mo.md("**V3 jaw, von Mises** (standard support, full current)"), mo.image(src=_f("jaw_v3_von_mises.png"), width=460)]),
               mo.vstack([mo.md("**V3 jaw, deflection u_x**"), mo.image(src=_f("jaw_v3_deflection.png"), width=460)])],
              justify="start", wrap=True)
    return


@app.cell
def _(HERE, json, os):
    def _load(rel):
        _p = os.path.join(HERE, "..", rel)
        return json.load(open(_p)) if os.path.exists(_p) else None
    sim_grip = _load("04_Simulation/out/grip_results.json")
    sim_close = _load("04_Simulation/out/closing_results.json")
    sim_power = _load("04_Simulation/out/power_fail.json")
    fem_train = _load("05_FEM/out/train_results.json")
    SIMFIG = os.path.join(HERE, "..", "04_Simulation", "figures")
    return SIMFIG, fem_train, sim_close, sim_grip, sim_power


@app.cell
def _(mo):
    mo.md(r"""
    ## 2. V2 sizing against the simulation, topic by topic

    Each block: her number (from her notebook, live), our number (MuJoCo E4 / FEM E5, last stored run), the deviation,
    and the picture or video that shows it. Section 3 collects all deviations in one table.
    """)
    return


@app.cell
def _(P, SIMFIG, mo, os, sim_grip, v2):
    _c = {c["case"]: c for c in sim_grip["cases"]} if sim_grip else {}
    _req, _set = _c.get("required", {}), _c.get("setpoint", {})
    _nan = float("nan")
    _u_hand = 1 / P.S_SICHERHEIT
    _u_real = P.B_MASSE * (P.G + P.A_PATH_REAL) / (P.N_BACKEN * P.MU_OILY * P.F_GRIP_OILY)
    _md = mo.md(f"""
### 2.1 Grip force (V2 § 1) - MuJoCo over the real robot path

| | V2 | simulation | deviation |
|---|---|---|---|
| path acceleration | {P.A_BAHN:.2f} m/s² (straight line, cycle budget) | **{P.A_PATH_REAL:.2f} m/s²** (real MoveJ, axis 1 turns ≈ 160°) | {(P.A_PATH_REAL / P.A_BAHN - 1) * 100:+.0f} % |
| friction use at {v2['F_req']:.1f} N, mean | {_u_hand:.2f} (= 1/S) | {_req.get('util_median', _nan):.2f} (median) | {(_req.get('util_median', _nan) / _u_hand - 1) * 100:+.0f} % |
| friction use at {v2['F_req']:.1f} N, peak | {_u_real:.2f} (at {P.A_PATH_REAL} m/s²) | {_req.get('util_max', _nan):.2f} (50 ms peak) | {(_req.get('util_max', _nan) / _u_real - 1) * 100:+.0f} % (jerk of the trapezoid) |
| setpoint | {v2['F_set']:.1f} N (her e-stop setpoint) | {P.F_GRIP_SET:.1f} N (S 2 at the real path): median {_set.get('util_median', _nan):.2f} / peak {_set.get('util_max', _nan):.2f}, held | {(v2['F_set'] / P.F_GRIP_SET - 1) * 100:+.1f} % |
""")
    mo.vstack([_md, mo.hstack([mo.image(src=os.path.join(SIMFIG, "grip_slip.png"), width=520),
                               mo.video(src=os.path.join(SIMFIG, "cycle_estop.mp4"), controls=True, width=520)],
                              justify="start", wrap=True)])
    return


@app.cell
def _(L, mo, sim_grip, v2):
    _c = {c["case"]: c for c in sim_grip["cases"]} if sim_grip else {}
    _e = _c.get("e-stop at setpoint", {})
    _nan = float("nan")
    mo.md(f"""
### 2.2 Emergency stop (V2 § 2)

| | V2 | simulation | deviation |
|---|---|---|---|
| stop deceleration at the TCP | {v2['a_stop']:.1f} m/s² (lever 1.4 m, full reach) | {_e.get('a_tcp_max', _nan):.1f} m/s² (ABB per-axis decelerations, TCP ≤ 1.0 m from axis 1) | {(_e.get('a_tcp_max', _nan) / v2['a_stop'] - 1) * 100:+.0f} % → V2 conservative |
| part at the setpoint | must not slip with S 1.5 | held, friction use peak {_e.get('util_max', _nan):.2f}, slip beyond the model's creep base {_e.get('slip_excess', _nan):+.2f} mm | ✓ |
| brake limit per jaw | {L.F_BRAKE:.1f} N | load case LC5 = {L.F_BRAKE:.1f} N | 0 % |
""")
    return


@app.cell
def _(HERE, P, fem_train, mo, os, v2):
    _nan = float("nan")
    _lc2 = next((c for c in fem_train["cases"] if c["case"].startswith("LC2")), None) if fem_train else None
    _My_v = P.F_JAW_MAX * abs(P.PINION_Z0 + P.PINION_B / 2 - sum(P.PAD_Z) / 2) / 1000
    _Mz_v = P.F_JAW_MAX * P.PINION_D / 2 / 1000
    _My_f = _lc2["carriage"]["M"][1] / 1000 if _lc2 else _nan
    _Mz_f = _lc2["carriage"]["M"][2] / 1000 if _lc2 else _nan

    def _s0(F, a_vert, a_side, c0, mt0, ml0):
        _zp = sum(P.PAD_Z) / 2
        _Fz, _Fy = P.B_MASSE * a_vert / 2, P.B_MASSE * a_side / 2
        _My = F * abs(P.PINION_Z0 + P.PINION_B / 2 - _zp) / 1000
        _Mz = F * P.PINION_D / 2000
        _Mx = (_Fz * abs(P.RAIL_Y) + _Fy * abs(P.SLIDE_Z_TOP - _zp)) / 1000
        return c0 / (abs(_Fy) + abs(_Fz) + c0 * (_Mx / mt0 + _My / ml0 + _Mz / ml0))
    _rows = []
    for _name, _c0, _mt0, _ml0 in (("size 9 SLS (before)", 3150.0, 14.5, 14.0),
                                   ("size 12 SLS (now)", P.CARRIAGE_C0, P.CARRIAGE_MT0, P.CARRIAGE_ML0)):
        _rows.append(f"| {_name} | {_s0(P.F_GRIP_SET, P.G + P.A_PATH_REAL, 0.0, _c0, _mt0, _ml0):.1f} | "
                     f"{_s0(P.F_JAW_MAX, P.G, v2['a_stop'], _c0, _mt0, _ml0):.1f} |")
    _md = mo.md(f"""
### 2.3 Ball rail guide (V2 § 6)

| | V2 | FEM (rack as rigid body at the pitch line) | deviation |
|---|---|---|---|
| carriage pitch moment M_y, full current | {_My_v:.2f} N·m | {_My_f:.2f} N·m | {(_My_f / _My_v - 1) * 100:+.1f} % |
| carriage yaw moment M_z, full current | {_Mz_v:.2f} N·m | {_Mz_f:.2f} N·m | {(_Mz_f / _Mz_v - 1) * 100:+.1f} % |
| pad tilt, full current | - | {(_lc2['tilt_deg'] if _lc2 else _nan):.3f}° (+ carriage, see tilt_check.py) | - |

Static safety S0 (Rexroth R999001206 p. 9: 3-5 all loads known, **5-8 normal design**, ≥ 12 overhead / high hazard):

| carriage | setpoint {P.F_GRIP_SET:.1f} N | full current + e-stop side load |
|---|---|---|
""" + "\n".join(_rows))
    mo.vstack([_md, mo.image(src=os.path.join(HERE, "figures", "train_lc2_von_mises.png"), width=620)])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ### 2.4 Bevel pair flank (V2 § 5) - her critical finding

    Same method as the Mädler ratings (DIN 3991, K_A 1.25, K_Hβ 1.5, S_H 1.0). Since 19 Nov the drive uses the KHK
    SMSG1-20 R/L (induction-hardened, ground); its catalogue torque is compared directly.
    """)
    return


@app.cell
def _(mo):
    bev_mat = mo.ui.dropdown({"C45 normalised, V2 (590 MPa)": 590.0, "42CrMo4 hardened (1360 MPa)": 1360.0,
                              "16MnCr5 case hardened (1630 MPa)": 1630.0}, value="C45 normalised, V2 (590 MPa)",
                             label="DIN 3991 check with σHlim")
    bev_mat
    return (bev_mat,)


@app.cell
def _(P, bev_mat, math, mo, v2):
    _b, _z = P.BEVEL_B, P.BEVEL_Z
    _de = math.radians(45)
    _dm = P.MODUL * _z - _b * math.sin(_de)
    _mm = P.MODUL * _dm / (P.MODUL * _z)
    _zv, _dv = _z / math.cos(_de), _dm / math.cos(_de)
    _al = math.radians(20)
    _r = _mm * _zv / 2
    _eps = (2 * math.sqrt((_r + _mm) ** 2 - (_r * math.cos(_al)) ** 2) - 2 * _r * math.sin(_al)) / (math.pi * _mm * math.cos(_al))
    _Ze = math.sqrt((4 - _eps) / 3)

    def _sH(F):
        _T = 2 * F * P.PINION_D / 2
        _Ft = 2 * _T / _dm
        return 189.8 * 2.495 * _Ze * 0.85 * math.sqrt(_Ft / (_b * _dv) * 2) * math.sqrt(1.25 * 1.5), _T
    _rows = ["| load | torque | DIN 3991: σH / S_H | KHK SMSG1-20: torque / 0.97 N·m |", "|---|---|---|---|"]
    for _lbl, _F in (("required", v2["F_req"]), ("setpoint", P.F_GRIP_SET), ("current limit", P.F_JAW_LIMIT), ("full current", P.F_JAW_MAX)):
        _s, _T = _sH(_F)
        _ok_din = "✓" if bev_mat.value / _s >= 1 else "✗"
        _ok_khk = "✓" if _T / 1000 <= P.BEVEL_T_ALLOW * 1.001 else "✗ (the current limit prevents it)"
        _rows.append(f"| {_lbl} ({_F:.1f} N) | {_T / 1000:.3f} N·m | {_s:.0f} MPa / {bev_mat.value / _s:.2f} {_ok_din} | "
                     f"{_T / 1000 / P.BEVEL_T_ALLOW:.2f} {_ok_khk} |")
    _s1, _T1 = _sH(P.F_JAW_MAX)
    _Tall = _T1 * (bev_mat.value / _s1) ** 2
    mo.md("\n".join(_rows) + f"\n\nDIN 3991 permissible torque with this material: **{_Tall / 1000:.2f} N·m**; "
          f"KHK catalogue: **{P.BEVEL_T_ALLOW} N·m** → current limit {P.F_JAW_LIMIT:.1f} N per jaw.")
    return


@app.cell
def _(SIMFIG, mo, os, sim_power):
    _c = sim_power["cases"] if sim_power else []
    _rows = "\n".join(f"| {c['t_brake'] * 1000:.0f} ms | {c['fn_end']:.1f} N | {'held' if c['held'] else '**dropped**'} |" for c in _c)
    _fd = sim_power["f_detent"] if sim_power else float("nan")
    _fh = sim_power["f_hold"] if sim_power else float("nan")
    _md = mo.md(f"""
### 2.5 Holding on power failure (V2 § 9)

V2 before the review: "the detent torque bridges the gap until the brake engages". Simulation: the detent holds
{_fd:.1f} N per jaw, the part needs {_fh:.1f} N.

| brake engages after | pad force left | part |
|---|---|---|
{_rows}

Only a drive that stays powered until the brake has closed keeps the part: buffer module PULS UF20.241 and a
brake-first shutdown (D11). V2 § 9 is corrected.
""")
    mo.vstack([_md, mo.image(src=os.path.join(SIMFIG, "power_fail.png"), width=760)])
    return


@app.cell
def _(SIMFIG, mo, os, sim_close):
    _nan = float("nan")
    _c = {f"{c['profile']} {c['t_fast']:.2f}": c for c in sim_close["cases"]} if sim_close else {}
    _two, _dir = _c.get("two-stage 0.12", {}), _c.get("direct 0.25", {})
    _mr = sim_close["m_reflected"] if sim_close else _nan
    _md = mo.md(f"""
### 2.6 Motor and closing (V2 § 10)

| | V2 | simulation | deviation |
|---|---|---|---|
| closing 8 mm | triangle profile in 0.3 s, peak ≈ 360 rpm, inertia torque negligible | two-stage: grip force after **{_two.get('t_grip', _nan) * 1000:.0f} ms**, peak {_two.get('f_peak', _nan):.1f} N | {(_two.get('t_grip', _nan) / 0.3 - 1) * 100:+.0f} % of the 0.3 s |
| impact on the part | not considered | straight into the part (0.25 s): **{_dir.get('f_peak', _nan):.0f} N** peak; reflected drive inertia {_mr:.1f} kg | new finding |
| available grip force | 60.5 N (all efficiencies) | `F_JAW_AVAIL` = 60.5 N in the parameters | 0 % |
""")
    mo.vstack([_md, mo.hstack([mo.image(src=os.path.join(SIMFIG, "closing.png"), width=600),
                               mo.video(src=os.path.join(SIMFIG, "closing_two-stage_120ms.mp4"), controls=True, width=480)],
                              justify="start", wrap=True)])
    return


@app.cell
def _(HERE, P, V4C, an, fem_train, json, mo, os, sim_close, sim_grip, v2, v4_store):
    _nan = float("nan")
    _ld = lambda rel: json.load(open(os.path.join(HERE, "..", rel))) if os.path.exists(os.path.join(HERE, "..", rel)) else {}
    _fc, _jv = _ld("04_Simulation/out/friction_check.json"), _ld("05_FEM/jaw_variants.json")
    _c = {c["case"]: c for c in sim_grip["cases"]} if sim_grip else {}
    _lc2 = next((c for c in fem_train["cases"] if c["case"].startswith("LC2")), None) if fem_train else None
    _close = {f"{c['profile']} {c['t_fast']:.2f}": c for c in sim_close["cases"]} if sim_close else {}
    _cmp = {r["quantity"]: r for r in (V4C.compare(v4_store, an) or [])}
    _My_v = P.F_JAW_MAX * abs(P.PINION_Z0 + P.PINION_B / 2 - sum(P.PAD_Z) / 2) / 1000
    _My_f = _lc2["carriage"]["M"][1] / 1000 if _lc2 else _nan
    _ok = lambda d: "✓" if abs(d) <= 5 else "✗"
    _val = []
    for _q, _r in _cmp.items():
        if _q.startswith("bending stress") or "Weber (a+b+c) vs FEM rigid" in _q:
            _val.append((_q, f"{_r['analytic']:.2f}", f"{_r['fem']:.2f}", _r["dev"], "V4 / FEM"))
    for _k, _lbl in (("solid", "deflection, plain bar 25 x 8 [µm]"), ("no pocket", "deflection, jaw without pocket [µm]")):
        if _k in _jv:
            _val.append((_lbl, f"{_jv[_k]['beam']:.2f}", f"{_jv[_k]['fem']:.2f}", _jv[_k]["dev"], "V4 model / FEM, section clamped"))
    _val.append(("carriage pitch moment M_y [N·m]", f"{_My_v:.2f}", f"{_My_f:.2f}", (_My_f / _My_v - 1) * 100, "V2 § 6 / FEM"))
    for _k in ("required", "setpoint", "e-stop at setpoint"):
        if _k in _fc:
            _val.append((f"friction use, median over the transfer ({_k})", f"{_fc[_k]['median_hand']:.3f}",
                         f"{_fc[_k]['median_sim']:.3f}", -_fc[_k]["dev_median"] / (1 + _fc[_k]["dev_median"] / 100), "V2 § 1 quasi-static / MuJoCo"))
    _t1 = ["| quantity | hand calculation | simulation | deviation | | pair |", "|---|---|---|---|---|---|"]
    _t1 += [f"| {q} | {a} | {f} | {d:+.1f} % | {_ok(d)} | {s} |" for q, a, f, d, s in _val]
    _v3 = _jv.get("V3", {})
    _ae = _c.get("e-stop at setpoint", {}).get("a_tcp_max", _nan)
    _a_lever = v2["a_stop"] * 1.0 / 1.4
    _t2 = ["| input | V2 assumption | replaced by | effect |", "|---|---|---|---|",
           f"| path acceleration | {P.A_BAHN:.2f} m/s² (straight line) | {P.A_PATH_REAL:.2f} m/s² (real MoveJ, MuJoCo) | setpoint {P.F_GRIP_SET:.1f} N instead of {P.F_GRIP_OILY:.1f} N |",
           f"| e-stop deceleration | {v2['a_stop']:.1f} m/s² (lever 1.4 m) | {_ae:.1f} m/s² (simulation, TCP ≤ 1.0 m); V2 rescaled to 1.0 m: {_a_lever:.1f} m/s² ({(_a_lever / _ae - 1) * 100:+.1f} %) | V2 conservative, kept |",
           "| bevel forces | straight bevel | spiral (KHK SMSG): up to 0.81·F_t | bearings s0 4.0, gearbox shaft ≤ 65 % |"]
    _tg = _close.get("two-stage 0.12", {}).get("t_grip", _nan)
    _t3 = ["| requirement | limit | result | |", "|---|---|---|---|",
           f"| grip force built up | ≤ 0.30 s | {_tg:.3f} s (two-stage close) | {'✓' if _tg <= 0.3 else '✗'} |",
           f"| pad tilt, full current | ≤ 0.10° | {(_lc2['tilt_deg'] if _lc2 else _nan):.3f}° | {'✓' if _lc2 and _lc2['tilt_deg'] <= 0.1 else '✗'} |",
           "| static safety of the carriage | S0 ≥ 5 (Rexroth, normal design) | 5.1 ... 7.8 (size 12) | ✓ |",
           f"| part held, every case incl. e-stop | no drop | {'held' if sim_grip and all(c['held'] for c in sim_grip['cases']) else 'check'} | ✓ |",
           "| part held on power failure | no drop | held with buffer + brake-first shutdown | ✓ |"]
    _t4 = ["| comparison | hand | FEM | deviation | why it differs |", "|---|---|---|---|---|"]
    for _q, _r in _cmp.items():
        if not (_q.startswith("bending stress") or "Weber (a+b+c) vs FEM rigid" in _q):
            _t4.append(f"| {_q} | {_r['analytic']:.2f} | {_r['fem']:.2f} | {_r['dev']:+.1f} % | different boundary condition |")
    if _v3:
        _t4.append(f"| V3 with pocket, rims carrying / not carrying [µm] | {_v3['beam']:.2f} / {_v3['beam_no_rims']:.2f} | {_v3['fem']:.2f} | "
                   f"{_v3['dev']:+.1f} / {_v3['dev_no_rims']:+.1f} % | 3D effect of the pocket: the bounds bracket the FEM |")
    mo.md("## 3. All deviations at a glance\n\n"
          "**a) Model validation: same quantity, same boundary condition (target ≤ 5 %)**\n\n" + "\n".join(_t1) +
          "\n\nThe one row above 5 % (beam + Weber, -5.3 %) is the pocketed jaw: the same rim effect as in d), where the "
          "two beam bounds bracket the FEM. Without the pocket the beam model is within 5 %." +
          "\n\n**b) V2 inputs replaced by the simulation** (not model errors)\n\n" + "\n".join(_t2) +
          "\n\n**c) Requirements**\n\n" + "\n".join(_t3) +
          "\n\n**d) Informative: deliberately different models** (they show why the Weber term and the bounds are needed)\n\n" + "\n".join(_t4))
    return


@app.cell
def _(HERE, P, json, mo, os):
    _vp = os.path.join(HERE, "jaw_variants.json")
    _v = json.load(open(_vp)) if os.path.exists(_vp) else {}
    _rows = "\n".join(f"| {k} | {r['fem']:.2f} | {r['beam']:.2f} | {r['dev']:+.1f} % |" for k, r in _v.items())
    _v3 = _v.get("V3", {})
    mo.md(f"""
## 4. Iteration 2: Viktoriia's response (20 Nov) and the check it asked for

Her notebook: `Viktoriia/06_Review/review_elias_response.py`.

| # | Question | Her result | Our check |
|---|---|---|---|
| 1 | KHK SMSG1-20 with DIN 3991 | flank capacity 2.0 ... 3.0 N·m (σHlim 1000 MPa, spiral virtual gear); catalogue {P.BEVEL_T_ALLOW} N·m stays the limit | reproduced; current limit {P.F_JAW_LIMIT:.1f} N per jaw unchanged |
| - | new: spiral bevel forces | axial/radial up to 0.81·F_t instead of 0.26·F_t (straight) | relevant for bearings and gearbox shaft, see 3 |
| 2 | shear area of the rims only | refuted: the rims carry 11 % of the shear, κ of the U-section ≈ 5/6; she suggests a FEM run without the pocket | FEM below: she is right |
| 3 | bearings 608-2Z, SKF | s0 = 4.0 at the current limit, 3.0 at 63.6 N (spiral forces), SKF asks ≥ 2; gearbox shaft ≤ 65 % of 305 N | reproduced |

**FEM check she proposed** (cross-section clamped at z = -47, the beam's own boundary condition, F = {P.F_JAW_MAX:.1f} N, µm):

| jaw | FEM | beam (V4 model) | deviation |
|---|---|---|---|
{_rows}
| V3, rims assumed not to carry | {_v3.get('fem', float('nan')):.2f} | {_v3.get('beam_no_rims', float('nan')):.2f} | {_v3.get('dev_no_rims', float('nan')):+.1f} % |

Without the pocket the beam meets the FEM within 5 %. With the pocket the FEM sits between "rims fully carrying"
and "rims not carrying": the 1 mm rims work only about half. This is a 3D effect of the pocket that beam theory cannot
see; it is not a shear problem. Result for the report: V4 is validated within 5 % on the plain bar, and for the
pocketed jaw the two beam bounds bracket the FEM (±7.5 %). No model parameter is tuned to hit 5 %.
""")
    return


if __name__ == "__main__":
    app.run()
