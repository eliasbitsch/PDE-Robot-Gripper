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
    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**


    ABB IRB 1300 with the gripper on the flange, part B from the buffer belt to the workpiece carrier.
    Change a value and the hand calculation **and** the MuJoCo simulation recompute (a few seconds).
    The FEM further down takes minutes and has its own button.
    """)
    return


@app.cell
def _(GP, mo):
    mu = mo.ui.slider(0.1, 0.6, step=0.05, value=GP.MU_OILY, label="friction µ (pad on part)", show_value=True)
    force = mo.ui.slider(10.0, round(GP.F_JAW_LIMIT, 1), step=0.5, value=round(GP.F_GRIP_SET, 1),
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
        runs = {"set force": V.run("set force", force.value, mu=mu.value, t_transfer=t_tr.value)}
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
                     f"{_r['a_tcp_max']:.1f} m/s² | {_r['slip_max']:.2f} mm (beyond creep base {_r['slip_excess']:+.2f}) | {_res} |")
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
    slow = mo.ui.slider(0.05, 1.0, step=0.05, value=0.25, label="playback speed", show_value=True)
    vid_case = mo.ui.dropdown(["set force", "set force + e-stop"], value="set force + e-stop", label="case")
    anim = mo.ui.run_button(label="Render slow-motion video")
    mo.vstack([mo.md("""Four cameras: cell, both jaws (housing see-through), pad close-up with the contact forces
    (yellow points, arrows split into normal and friction), back view on the lug side. Parts the part must not touch
    turn red when their clearance goes negative."""), mo.hstack([vid_case, slow, anim], justify="start", gap=2)])
    return anim, slow, vid_case


@app.cell
def _(HERE, V, anim, estop, force, mo, mu, os, slow, t_tr, vid_case):
    mo.stop(not anim.value, mo.md("_Press the button: about 1 min per 10 s of video._"))
    with mo.status.spinner(title="simulating and rendering four views ..."):
        _v, _r = V.video(vid_case.value, force.value, os.path.join(HERE, "out", "cycle_notebook.mp4"), slowmo=slow.value,
                         estop="e-stop" in vid_case.value, mu=mu.value, t_transfer=t_tr.value)
    mo.vstack([mo.video(src=_v["path"], controls=True, width=1000),
               mo.md(f"min clearance over the cycle: **{_v['min_clearance_mm']:.1f} mm** ({_v['min_clearance_pair'][0]} / {_v['min_clearance_pair'][1]})")])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Closing time with the motor curve

    Requirement: close 8 mm per jaw and build up the grip force within 0.3 s. Stepper ST4118S on GP42 1:7.07, curve from
    the datasheet's electrical values at 24 V, 70 % of it usable in closed loop. Rotor, gearbox and brake act at the jaws
    like a mass of several kg: this decides the impact on the part, not the torque.
    """)
    return


@app.cell
def _(GP, mo):
    cl_prof = mo.ui.dropdown(["two-stage", "direct"], value="two-stage", label="profile")
    cl_fast = mo.ui.slider(0.06, 0.3, step=0.01, value=0.12, label="fast phase [s]", show_value=True)
    cl_vt = mo.ui.slider(2.0, 50.0, step=1.0, value=10.0, label="approach speed last mm [mm/s]", show_value=True)
    cl_force = mo.ui.slider(10.0, round(GP.F_JAW_LIMIT, 1), step=0.5, value=round(GP.F_GRIP_SET, 1),
                            label="current limit = grip force per jaw [N]", show_value=True)
    mo.vstack([mo.hstack([cl_prof, cl_fast], justify="start", gap=2), mo.hstack([cl_vt, cl_force], justify="start", gap=2)])
    return cl_fast, cl_force, cl_prof, cl_vt


@app.cell
def _(cl_fast, cl_force, cl_prof, cl_vt, mo, np, plt):
    import closing_time as C
    import drive as DRV
    C.V_TOUCH = cl_vt.value / 1000
    with mo.status.spinner(title="MuJoCo: closing ..."):
        _r, _lg = C.run(cl_prof.value, cl_fast.value, force=cl_force.value)
    _tq = C.profile_torque_check(cl_fast.value, (C.D_OPEN - C.D_GRIP) if cl_prof.value == "direct" else (C.D_OPEN - C.D_GRIP - 0.001))
    _mt = DRV.min_time(C.D_OPEN - C.D_GRIP)
    _fig = C.plot({cl_prof.value: _lg}, None, fig=True)
    _f2, _a2 = plt.subplots(figsize=(5, 3.2))
    _rpm = np.linspace(0, 1600, 200)
    _a2.plot(_rpm, [DRV.torque_avail(_w * 2 * np.pi / 60) for _w in _rpm], label="pull-out torque (model, 24 V)")
    _a2.plot(_rpm, [0.7 * DRV.torque_avail(_w * 2 * np.pi / 60) for _w in _rpm], ls="--", label="usable (70 %)")
    _a2.plot([_tq["rpm"]], [_tq["torque_need"]], "o", color="#C2410C", label="this profile, peak")
    _a2.set_xlabel("motor speed [rpm]")
    _a2.set_ylabel("torque [N·m]")
    _a2.grid(alpha=0.3)
    _a2.legend(frameon=False, fontsize=8)
    _f2.tight_layout()
    mo.vstack([
        mo.md(f"""
| | value |
|---|---|
| contact after | {_r['t_contact'] * 1000:.0f} ms at {_r['v_contact'] * 1000:.0f} mm/s |
| grip force within 5 % after | **{_r['t_grip'] * 1000:.0f} ms** ({'meets' if _r['meets'] else '**misses**'} 0.3 s) |
| peak pad force (impact) | {_r['f_peak']:.1f} N = {_r['overshoot']:.2f} x set force (FEM LC2 covers {C.GP.F_JAW_MAX:.1f} N) |
| impact estimate, undamped NBR pad | {C.impact_estimate(_r['v_contact'])[0]:.0f} N |
| part pushed on the belt | {_r['part_push_mm']:.2f} mm |
| motor at peak | {_tq['rpm']:.0f} rpm, needs {_tq['torque_need']:.3f} of {_tq['torque_avail']:.3f} N·m |
| fastest possible (motor curve, 8 mm to standstill) | {_mt['t'] * 1000:.0f} ms |
| reflected drive inertia at the jaws | {DRV.M_REFLECTED:.2f} kg (jaws {2 * DRV.M_JAW:.2f} kg) |
"""),
        mo.hstack([_fig, _f2], justify="start")])
    return
@app.cell
def _(mo):
    mo.md(r"""
    ## 4. FEM of the jaw train (slide + holder + mock jaw)

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
        _cells = [f"{_c['parts'][_k]['vm_peak']:.1f} / {_c['parts'][_k]['safety']:.1f}" for _k in ("slide", "holder", "jaw")]
        _rows.append(f"| {_c['case']} | {_c['u_pad'] * 1000:.1f} | {_c.get('tilt_deg', float('nan')):.3f} | " + " | ".join(_cells) + " |")
    _png = os.path.join(HERE, "..", "05_FEM", "figures", "train_lc2_von_mises.png")
    mo.vstack([mo.md(f"{_fem['elements']} elements, element size {_fem['lc']} mm\n\n" + "\n".join(_rows)),
               mo.image(src=_png, width=700) if os.path.exists(_png) else mo.md("")])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## Robustness - Monte Carlo over the scattered inputs
    Full cycles (built two-stage closing from fully open, lift, transfer, lower) with the part position (+-5 mm),
    pad friction (0.2 ... 0.3), grip force (44.5 N +-5 %) and part mass (+0 ... +3 %) scattered by Latin hypercube;
    a stress set pushes friction and force far below spec. The stored study comes from `python robustness.py`
    (12 000 cycles, about an hour on 9 processes); the button below runs a small serial sample in the notebook.
    """)
    return


@app.cell
def _(mo):
    rb_n = mo.ui.slider(4, 40, step=4, value=8, label="sample: grips inside spec", show_value=True)
    rb_go = mo.ui.run_button(label="Run sample")
    mo.hstack([rb_n, rb_go], justify="start", gap=2)
    return rb_go, rb_n


@app.cell
def _(HERE, json, mo, np, os, rb_go, rb_n):
    import robustness as _R
    _path = os.path.join(HERE, "out", "robustness.json")
    mo.stop(not os.path.exists(_path), mo.md("_No study yet - run `python robustness.py`._"))
    _res = json.load(open(_path))["results"]
    _spec = [r for r in _res if r["study"] == "spec"]
    _stress = [r for r in _res if r["study"] == "stress"]
    _u = np.array([r["util_max"] for r in _spec])
    _mf_fail = max(r["mu"] * r["force"] for r in _stress if not r["held"])
    _mf_spec = min(r["mu"] * r["force"] for r in _spec)
    _png = os.path.join(HERE, "figures", "robustness.png")
    _txt = (f"**Stored study:** {sum(r['held'] for r in _spec)} of {len(_spec)} grips inside spec held; peak friction use "
            f"median {np.median(_u):.2f}, highest {_u.max():.2f} (safety {1 / _u.max():.2f} in the worst grip). "
            f"Stress: {sum(not r['held'] for r in _stress)} of {len(_stress)} fail; the weakest grip in spec holds "
            f"{_mf_spec / _mf_fail:.2f} x the mu F of the strongest failure.")
    _out = [mo.md(_txt), mo.image(src=_png, width=900)]
    if rb_go.value:
        _base = json.load(open(_path))["creep_grid"]["base"]
        with mo.status.spinner(title=f"{rb_n.value} cycles, about 3 s each ..."):
            _smp = [_R.one((s, _base)) for s in _R.sample("spec", rb_n.value, int(np.random.default_rng().integers(1 << 30)))]
        _rows = ["| offset [mm] | mu | F [N] | mass x | peak friction use | slip beyond creep [mm] | |", "|---|---|---|---|---|---|---|"]
        for r in _smp:
            _rows.append(f"| {r['offset'] * 1000:+.1f} | {r['mu']:.3f} | {r['force']:.1f} | {r['mass']:.3f} | {r['util_max']:.2f} | "
                         f"{r['slip_excess']:+.3f} | {'held' if r['held'] else 'SLIPS'} |")
        _out.append(mo.md("**Fresh sample:**" + chr(10) * 2 + chr(10).join(_rows)))
    mo.vstack(_out)
    return


if __name__ == "__main__":
    app.run()
