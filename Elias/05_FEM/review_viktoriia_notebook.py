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
def _(mo):
    mo.md(r"""
    ## 2. V2 sizing against the simulation
    """)
    return


@app.cell
def _(HERE, L, P, json, mo, os, v2):
    _g = os.path.join(HERE, "..", "04_Simulation", "out", "grip_results.json")
    _grip = {c["case"]: c for c in json.load(open(_g))["cases"]} if os.path.exists(_g) else {}
    _t = os.path.join(HERE, "out", "train_results.json")
    _train = json.load(open(_t)) if os.path.exists(_t) else None
    _lc2 = next((c for c in _train["cases"] if c["case"].startswith("LC2")), None) if _train else None
    _My_v = P.F_JAW_MAX * abs(P.PINION_Z0 + P.PINION_B / 2 - sum(P.PAD_Z) / 2)
    _Mz_v = P.F_JAW_MAX * P.PINION_D / 2
    _carr = _lc2.get("carriage") if _lc2 else None
    _nom = _grip.get("nominal", {})
    _es = _grip.get("e-stop", {})
    mo.md(f"""
| # | V2 (Viktoriia) | Simulation | verdict |
|---|---|---|---|
| 1 | grip force {v2['F_req']:.1f} N (µ 0.2, S 2, a = {P.A_BAHN:.2f} m/s²) | MuJoCo real MoveJ path: TCP up to **5.0 m/s²**, friction use median {_nom.get('util_median', float('nan')):.2f} / peak {_nom.get('util_max', float('nan')):.2f} | formula confirmed; path acceleration 2.05 → 5 m/s² would give ≈ 44 N |
| 2 | e-stop {v2['a_stop']:.0f} m/s² (CRB 1300 data, lever 1.4 m), setpoint {v2['F_set']:.1f} N | IRB 1300 spec 3HAC070393 rev. M p. 91: **same stop data** (55.25° / 0.40 s); MuJoCo with the per-axis decelerations: {_es.get('a_tcp_max', float('nan')):.1f} m/s² at the TCP, part held at 35.7 N (peak {_es.get('util_max', float('nan')):.2f}) | her 17 m/s² is conservative (lever 1.4 m = full reach, our TCP is at ≈ 1.0 m) |
| 3 | surface pressure with A = 805 mm² | her V3 pad has 737 mm² | update the area, still < 0.1 MPa |
| 6 | carriage moments M_y = {_My_v / 1000:.2f} N·m, M_z = {_Mz_v / 1000:.2f} N·m (full current) | FEM train, rack as rigid body at the pitch line: M_y = **{(_carr['M'][1] / 1000) if _carr else float('nan'):.2f}** N·m, M_z = **{(_carr['M'][2] / 1000) if _carr else float('nan'):.2f}** N·m | identical (statics); FEM adds the pad tilt {(_lc2['tilt_deg'] if _lc2 else float('nan')):.3f}° ≤ {L.TILT_MAX}° |
| 9 | brake holds {P.T_PINION_BRAKE / (2 * P.PINION_D / 2000):.0f} N per jaw | load case LC5 uses F_BRAKE = {L.F_BRAKE:.1f} N | same |
| 10 | 0.23 N·m at 340 rpm (Nanotec curve) | pull-out cannot exceed the 0.20 N·m holding torque; model at 24 V: 0.19 N·m flat to ≈ 800 rpm; closing study: torque uses ≤ 25 %, **impact** limits the speed (3.2 kg reflected inertia) | check the curve value; add the closing impact / two-stage profile |
""")
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Bevel pair flank (V2 § 5) - her critical finding

    Same method as the Mädler catalogue ratings (DIN 3991, K_A 1.25, K_Hβ 1.5, S_H 1.0, endurance), so the result
    is directly comparable to Mädler's torque tables. Pick the material:
    """)
    return


@app.cell
def _(mo):
    bev_mat = mo.ui.dropdown({"C45 normalised (590 MPa)": 590.0, "42CrMo4 hardened (1360 MPa)": 1360.0,
                              "16MnCr5 case hardened (1630 MPa)": 1630.0}, value="C45 normalised (590 MPa)",
                             label="bevel gear material, σHlim")
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

    _rows = ["| load | torque | σH | S_H | |", "|---|---|---|---|---|"]
    for _lbl, _F in (("regular", v2["F_req"]), ("setpoint (V2)", v2["F_set"]), ("full current", P.F_JAW_MAX)):
        _s, _T = _sH(_F)
        _S = bev_mat.value / _s
        _rows.append(f"| {_lbl} ({_F:.1f} N) | {_T / 1000:.3f} N·m | {_s:.0f} MPa | {_S:.2f} | {'✓' if _S >= 1 else '✗'} |")
    _s1, _T1 = _sH(P.F_JAW_MAX)
    _Tall = _T1 * (bev_mat.value / _s1) ** 2
    mo.md("\n".join(_rows) + f"\n\nPermissible torque: **{_Tall / 1000:.2f} N·m** ({_Tall / P.PINION_D:.0f} N per jaw). "
          "Mädler lists hardened spiral bevel sets 1:1 from m 0.6 with 0.97 N·m upwards (catalogue p. 326): "
          "a drop-in candidate for the C45 pair; the mounting distance has to be checked in the CAD.")
    return


if __name__ == "__main__":
    app.run()
