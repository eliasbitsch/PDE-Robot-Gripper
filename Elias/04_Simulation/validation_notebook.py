import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="Gripper validation")


@app.cell
def _():
    import json
    import os
    import sys

    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np

    HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(HERE, "..", "05_FEM"))
    import validate_grip as V
    import load_cases as L
    GP = V.GP
    return GP, HERE, L, V, json, mo, np, os, plt


@app.cell
def _(mo):
    mo.md(r"""
    # Gripper validation - hand calculation against simulation

    ABB IRB 1300 with the gripper on the flange, part B from the buffer belt to the workpiece carrier.
    Change a value and the hand calculation **and** the MuJoCo simulation recompute (a few seconds).
    The FEM further down takes minutes and has its own button.
    """)
    return


@app.cell
def _(GP, mo):
    mu = mo.ui.slider(0.1, 0.6, step=0.05, value=GP.MU_OILY, label="friction µ (pad on part)", show_value=True)
    force = mo.ui.slider(10.0, round(GP.F_JAW_MAX, 1), step=0.5, value=round(GP.F_GRIP_OILY, 1),
                         label="grip force per jaw [N]", show_value=True)
    t_tr = mo.ui.slider(1.2, 4.0, step=0.1, value=GP.T_EINWEG, label="transfer time [s]", show_value=True)
    safety = mo.ui.slider(1.0, 3.0, step=0.25, value=GP.S_SICHERHEIT, label="safety factor S (hand calc.)", show_value=True)
    estop = mo.ui.checkbox(value=True, label="also simulate an emergency stop (ABB category 0)")
    mo.vstack([mo.hstack([mu, force], justify="start", gap=2), mo.hstack([t_tr, safety], justify="start", gap=2), estop])
    return estop, force, mu, safety, t_tr


@app.cell
def _(GP, L, mo, mu, safety, t_tr):
    _m, _g = GP.B_MASSE, GP.G
    a_design = 4.5 * GP.WEG_RAEUMLICH / t_tr.value ** 2        # trapezoid with T/3 ramps over the straight line
    f_req = safety.value * _m * (_g + a_design) / (GP.N_BACKEN * mu.value)
    mo.md(rf"""
    ## 1. Hand calculation (VDI 2860)

    $$a = \frac{{4.5\, s}}{{T^2}} = \frac{{4.5 \cdot {GP.WEG_RAEUMLICH:.2f}\,\mathrm{{m}}}}{{({t_tr.value:.1f}\,\mathrm{{s}})^2}}
    = {a_design:.2f}\ \mathrm{{m/s^2}} \qquad
    F_\mathrm{{G}} = \frac{{S\, m\,(g + a)}}{{n\, \mu}} = \frac{{{safety.value:.2f} \cdot {_m:.3f} \cdot ({_g:.2f} + {a_design:.2f})}}{{2 \cdot {mu.value:.2f}}}
    = \mathbf{{{f_req:.1f}\ N}}$$

    Emergency stop from the ABB stop data: axis 1 at {L.ALPHA_STOP[1]:.1f} rad/s², TCP at {L.R_TCP_A1} m
    → {L.A_STOP:.1f} m/s².
    """)
    return a_design, f_req


@app.cell
def _(V, estop, force, mo, mu, t_tr):
    with mo.status.spinner(title="MuJoCo: running the cycle ..."):
        runs = {"set force": V.run("set force", force.value, mu=mu.value, t_transfer=t_tr.value, record=True)}
        if estop.value:
            runs["set force + e-stop"] = V.run("e-stop", force.value, estop=True, mu=mu.value, t_transfer=t_tr.value)
    return (runs,)


@app.cell
def _(GP, a_design, force, mo, mu, runs):
    exp_util = GP.B_MASSE * (GP.G + a_design) / (GP.N_BACKEN * mu.value * force.value)
    _rows = ["| Case | friction use: hand calc. | sim median | sim peak (50 ms) | TCP accel. max | part movement | result |",
            "|---|---|---|---|---|---|---|"]
    for _name, (_r, _) in runs.items():
        _res = "held" if _r["held"] else ("**dropped**" if _r["dropped"] else "**slips**")
        _rows.append(f"| {_name} | {exp_util:.2f} | {_r['util_median']:.2f} | {_r['util_max']:.2f} | "
                     f"{_r['a_tcp_max']:.1f} m/s² | {_r['slip_max']:.2f} mm | {_res} |")
    mo.md(f"""
    ## 2. Simulation against the hand calculation

    Friction use = tangential force / (µ · normal force) on the pads; 1.0 is the friction limit, the hand
    calculation with S gives 1/S at its design acceleration.

    {chr(10).join(_rows)}
    """)
    return


@app.cell
def _(np, plt, runs):
    _fig, _ax = plt.subplots(1, 3, figsize=(12, 3.2))
    for _name, (_, _lg) in runs.items():
        _t = np.array(_lg["t"])
        _ax[0].plot(_t, _lg["slip"], label=_name)
        _u = np.nan_to_num(np.array(_lg["util"]))
        _ax[1].plot(_t, np.convolve(_u, np.ones(10) / 10, mode="same"), label=_name)
        _ax[2].plot(_t, _lg["a_tcp"], label=_name)
    _ax[0].set_ylabel("part movement in the jaws [mm]")
    _ax[1].set_ylabel("friction use [-]")
    _ax[1].axhline(1.0, color="#C2410C", ls="--", lw=1)
    _ax[2].set_ylabel("TCP acceleration [m/s²]")
    for _a in _ax:
        _a.set_xlabel("t [s]")
        _a.grid(alpha=0.3)
    _ax[0].legend(frameon=False, fontsize=8)
    _fig.tight_layout()
    _fig
    return


@app.cell
def _(mo):
    anim = mo.ui.run_button(label="Render animation of the cycle")
    anim
    return (anim,)


@app.cell
def _(HERE, V, anim, mo, os, runs):
    mo.stop(not anim.value, mo.md("_Press the button for a GIF of the cycle (about 20 s)._"))
    with mo.status.spinner(title="rendering ..."):
        _gif = V.animate(runs["set force"][1], os.path.join(HERE, "out", "cycle.gif"))
    mo.image(src=_gif, width=720)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. FEM of the jaw train (slide + holder + mock jaw)

    Last results below. **Run FEM** recomputes all five load cases (about 5 min at 3 mm).
    """)
    return


@app.cell
def _(mo):
    lc = mo.ui.slider(2.0, 4.0, step=0.5, value=3.0, label="element size [mm]", show_value=True)
    fem_go = mo.ui.run_button(label="Run FEM")
    mo.hstack([lc, fem_go], justify="start", gap=2)
    return fem_go, lc


@app.cell
def _(HERE, fem_go, json, lc, mo, os):
    import fem_train as _T
    _path = os.path.join(HERE, "..", "05_FEM", "out", "train_results.json")
    if fem_go.value:
        with mo.status.spinner(title="FEM: meshing and solving LC1-LC5 ..."):
            _fem = _T.run_all(lc.value)
    else:
        mo.stop(not os.path.exists(_path), mo.md("_No FEM results yet - press **Run FEM**._"))
        _fem = json.load(open(_path))
    _rows = ["| Load case | u pad [µm] | tilt [°] | slide MPa / S | holder MPa / S | jaw MPa / S |", "|---|---|---|---|---|---|"]
    for _c in _fem["cases"]:
        _cells = [f"{_c['parts'][_k]['vm_peak']:.1f} / {_c['parts'][_k]['safety']:.1f}" for _k in ("slide", "holder", "jaw (mock)")]
        _rows.append(f"| {_c['case']} | {_c['u_pad'] * 1000:.1f} | {_c.get('tilt_deg', float('nan')):.3f} | " + " | ".join(_cells) + " |")
    _png = os.path.join(HERE, "..", "05_FEM", "figures", "train_lc2_von_mises.png")
    mo.vstack([mo.md(f"{_fem['elements']} elements, element size {_fem['lc']} mm\n\n" + "\n".join(_rows)),
               mo.image(src=_png, width=700) if os.path.exists(_png) else mo.md("")])
    return


if __name__ == "__main__":
    app.run()
