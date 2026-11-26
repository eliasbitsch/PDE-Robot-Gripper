import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", css_file="../marimo_paper.css", app_title="Robot gripper - design journey")


@app.cell
def _():
    import os
    import sys

    import marimo as mo

    HERE = os.path.dirname(os.path.abspath(__file__))
    ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
    sys.path.insert(0, os.path.join(ROOT, "Elias", "02_CAD_Body"))
    import gripper_params as P

    def f(*parts):
        return os.path.join(ROOT, *parts)

    def img(*parts, caption=None):
        _path = f(*parts)
        if not os.path.exists(_path):
            return mo.md(f"_missing: `{'/'.join(parts)}`_")
        return mo.image(src=_path, width="100%", caption=caption)

    def vid(*parts):
        _path = f(*parts)
        if not os.path.exists(_path):
            return mo.md(f"_missing: `{'/'.join(parts)}`_")
        return mo.video(src=_path, controls=True, loop=True, muted=True, width="100%")

    SIM = ("Elias", "04_Simulation", "figures")
    FEM = ("Elias", "05_FEM", "figures")
    return FEM, P, SIM, img, mo, vid


@app.cell
def _(mo):
    mo.md(r"""
    # Robot gripper for part B - our design journey
    **Produktdesign und Produktentwicklung, MRE WS 2026/27 - group 10 - Viktoriia Ovdiienko / Elias Bitsch**

    An ABB robot picks part B (PA 6) off a buffer belt, carries it 1800 mm and places it on a workpiece carrier -
    **one part every 5 s**. We deliver the whole cell: gripper, robot, fence, belt, carrier and the motion and grip
    analysis.
    """)
    return


@app.cell
def _(SIM, vid):
    vid(*SIM, "cell_takt.mp4")
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 1. The journey at a glance

    | when | step | what changed and why |
    |---|---|---|
    | 18 Sep | first concept | 2-finger parallel gripper, linkage in the style of the Robotiq 2F-85 |
    | 17 Oct | scope | the whole robot cell, analytical vs. FEM within 5 % |
    | 30 Oct | **mechanism switch** | linkage → **rack and pinion** (one half of the Cartesian Hand): jaws stay parallel, constant force over the stroke, catalogue gears |
    | 10 Nov | **CAD and robot** | FreeCAD built by Python scripts from one parameter file; ABB IRB 1600 → **IRB 1300** (axis 5 ±130°), 500 mm pedestal |
    | 11-16 Nov | analysis chain | FEM with gmsh + CalculiX, MuJoCo cell with the real robot path |
    | 19 Nov | **iteration 1** | Viktoriia's sizing (V2), jaws V3 and beam proof (V4) checked against simulation and FEM |
    | 20 Nov | **iteration 2** | bevel gear to DIN 3991 (KHK catalogue), bearing check, extra FEM load case |
    | 21 Nov | design review | oily part: µ = 0.2 → setpoint 44.5 N; Viktoriia's jaw widened into the **final T-jaw** against twist; zero-backlash brake; V4 vs. FEM within 5 % |
    | 23 Nov | closing strategy | two-stage closing: fast approach, slow touch - no impact on the part |
    """)
    return


@app.cell
def _(img, mo):
    mo.vstack([
        mo.md("## 2. Concept\n\nPart B has two flat, parallel side faces 70 mm apart - friction is enough, no form fit needed."),
        mo.ui.tabs({
            "Concept sketch": img("Viktoriia", "01_Concept", "konzeptskizze_v1.png"),
            "Inspiration: Cartesian Hand": img("shared", "references", "cartesian-hand.png",
                                               caption="Xia et al. (2026), CC BY 4.0 - we take one of its two parallel grippers"),
        }),
        mo.md(r"""
    **Variant comparison (VDI 2225)** over four electric transmissions: **rack and pinion 0.89**, lead screw 0.72.
    **Vacuum** was considered and rejected: the assignment asks for jaws, parts can only be gripped from the side
    (cup in shear), and a cup lets go on power failure.
    """),
    ])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Sizing: how much grip force?

    Required force per jaw from friction: $F = \dfrac{S \cdot m \cdot (g + a)}{n \cdot \mu}$.
    Move the sliders to see what drives it - the oily part (low µ) and the acceleration dominate.
    """)
    return


@app.cell
def _(P, mo):
    mu_sl = mo.ui.slider(0.1, 0.6, step=0.05, value=P.MU_OILY, label="friction µ (NBR on oily PA 6)", show_value=True)
    a_sl = mo.ui.slider(0.0, 8.0, step=0.1, value=P.A_PATH_REAL, label="acceleration a [m/s²]", show_value=True)
    s_sl = mo.ui.slider(1.0, 3.0, step=0.1, value=P.S_SICHERHEIT, label="safety factor S", show_value=True)
    mo.vstack([mu_sl, a_sl, s_sl])
    return a_sl, mu_sl, s_sl


@app.cell
def _(P, a_sl, mo, mu_sl, s_sl):
    _F = s_sl.value * P.B_MASSE * (P.G + a_sl.value) / (P.N_BACKEN * mu_sl.value)
    _ok = _F <= P.F_JAW_AVAIL
    mo.hstack([
        mo.stat(f"{_F:.1f} N", label="required per jaw", caption=f"m = {P.B_MASSE:.3f} kg", bordered=True),
        mo.stat(f"{P.F_GRIP_SET:.1f} N", label="our setpoint", caption="µ 0.2, a 4.96 m/s², S 2", bordered=True),
        mo.stat(f"{P.F_JAW_AVAIL:.1f} N", label="drive delivers", caption="full current, all losses",
                direction="increase" if _ok else "decrease", bordered=True),
    ], widths="equal", gap=1)
    return


@app.cell
def _(P, mo):
    mo.md(f"""
    ### Key values

    | quantity | value |
    |---|---|
    | part B | 95 × 100 × 80 mm, PA 6, **{P.B_MASSE:.3f} kg** |
    | required grip force (hand calc., a = {P.A_BAHN:.2f} m/s²) | **{P.F_GRIP_OILY:.1f} N** per jaw |
    | setpoint (real path cap {P.A_PATH_REAL} m/s², covers e-stop) | **{P.F_GRIP_SET:.1f} N** per jaw |
    | available at full current | **{P.F_JAW_AVAIL:.1f} N** per jaw |
    | drive | NEMA 17 closed-loop stepper, gearbox i = {P.GEAR_I}, bevel pair, pinion d = {P.PINION_D:.0f} mm |
    | stroke | {P.D_OPEN:.0f} mm per jaw |
    | gripper mass | **{P.GRIPPER_MASS_TOTAL:.2f} kg** |
    | robot | ABB IRB 1300-7/1.4, wrist torque along the cycle 7.0 N·m = 54 % |
    | jaw: beam theory (V4) vs. FEM | stress +1.6 %, deflection +4.1 % / +5.0 % - **within 5 %** |
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ### How everything is connected

    Every box is a value of the design. Change an input and the chain shows what moves with it:
    **orange** = higher than our design point, **teal** = lower. Jaw stress and deflection are scaled linearly from the
    FEM of jaw V3 (6.9 MPa peak, 5.82 µm at 63.6 N).
    """)
    return


@app.cell
def _(P, mo):
    MATERIALS = {  # E [GPa], Rp0.2 [MPa], density [g/cm³]
        "EN AW-7075 T6 (ours)": (71.7, 460.0, 2.81),
        "EN AW-6082 T6": (70.0, 250.0, 2.70),
        "S355 steel": (210.0, 355.0, 7.85),
        "PA 6 printed": (2.5, 50.0, 1.14),
    }
    c_mu = mo.ui.slider(0.1, 0.6, step=0.05, value=P.MU_OILY, label="friction µ", show_value=True)
    c_a = mo.ui.slider(0.0, 8.0, step=0.1, value=P.A_PATH_REAL, label="acceleration a [m/s²]", show_value=True)
    c_m = mo.ui.slider(0.3, 1.5, step=0.05, value=round(P.B_MASSE, 2), label="part mass m [kg]", show_value=True)
    c_d = mo.ui.slider(12, 32, step=2, value=int(P.PINION_D), label="pinion diameter d [mm]", show_value=True)
    c_i = mo.ui.slider(3.0, 15.0, step=0.5, value=P.GEAR_I, label="gearbox ratio i", show_value=True)
    c_mat = mo.ui.dropdown(list(MATERIALS), value="EN AW-7075 T6 (ours)", label="jaw material")
    mo.hstack([mo.vstack([c_mu, c_a, c_m]), mo.vstack([c_d, c_i, c_mat])], widths="equal", gap=2)
    return MATERIALS, c_a, c_d, c_i, c_m, c_mat, c_mu


@app.cell
def _(MATERIALS, P, c_a, c_d, c_i, c_m, c_mat, c_mu, mo):
    def chain(mu, a, m, d, i, mat):
        _E, _rp, _ = MATERIALS[mat]
        _r = d / 2000
        F = P.S_SICHERHEIT * m * (P.G + a) / (P.N_BACKEN * mu)
        T_pin = 2 * F * _r / (P.ETA_BEVEL * P.ETA_RP)
        T_mot = T_pin / (i * P.GEAR_ETA)
        F_av = P.MOTOR_T_HOLD * i * P.GEAR_ETA * P.ETA_BEVEL * P.ETA_RP / (2 * _r)
        sig = 6.9 * F / 63.6
        return dict(F=F, T_pin=T_pin, T_mot=T_mot, util=T_mot / P.MOTOR_T_HOLD * 100, F_av=F_av, margin=F_av / F,
                    sig=sig, S_rp=_rp / sig, w=5.82 * F / 63.6 * 71.7 / _E)

    _base = chain(P.MU_OILY, P.A_PATH_REAL, P.B_MASSE, P.PINION_D, P.GEAR_I, "EN AW-7075 T6 (ours)")
    _now = chain(c_mu.value, c_a.value, c_m.value, c_d.value, c_i.value, c_mat.value)

    def node(key, title, fmt, unit=""):
        _v, _b = _now[key], _base[key]
        _dv = (_v / _b - 1) * 100 if _b else 0
        _cls = "up" if _dv > 0.5 else "down" if _dv < -0.5 else "same"
        _delta = f"<br/><small>{_dv:+.0f} %</small>" if _cls != "same" else ""
        return f'{key}["<b>{title}</b><br/>{_v:{fmt}} {unit}{_delta}"]:::{_cls}'

    _in = lambda k, t, v: f'{k}(["{t}<br/><b>{v}</b>"]):::inp'
    _diagram = "\n".join([
        "flowchart LR",
        "classDef inp fill:#f5f5f4,stroke:#a8a29e,color:#1c1917",
        "classDef same fill:#ffffff,stroke:#78716c,color:#1c1917",
        "classDef up fill:#fed7aa,stroke:#C2410C,color:#7c2d12",
        "classDef down fill:#ccfbf1,stroke:#0f766e,color:#134e4a",
        _in("mu", "friction µ", f"{c_mu.value:.2f}"),
        _in("a", "acceleration", f"{c_a.value:.1f} m/s²"),
        _in("m", "part mass", f"{c_m.value:.2f} kg"),
        _in("d", "pinion d", f"{c_d.value} mm"),
        _in("i", "gearbox i", f"{c_i.value:g}"),
        _in("mat", "jaw material", c_mat.value.split(" (")[0]),
        node("F", "grip force / jaw", ".1f", "N"),
        node("T_pin", "pinion torque", ".2f", "N·m"),
        node("T_mot", "motor torque", ".3f", "N·m"),
        node("util", "motor use", ".0f", "%"),
        node("F_av", "available force", ".1f", "N"),
        node("margin", "force margin", ".2f", "×"),
        node("sig", "jaw stress", ".1f", "MPa"),
        node("S_rp", "safety vs Rp0.2", ".0f", ""),
        node("w", "jaw deflection", ".1f", "µm"),
        "mu --> F", "a --> F", "m --> F",
        "F --> T_pin", "d --> T_pin", "T_pin --> T_mot", "i --> T_mot", "T_mot --> util",
        "d --> F_av", "i --> F_av", "F_av --> margin", "F --> margin",
        "F --> sig", "sig --> S_rp", "mat --> S_rp", "F --> w", "mat --> w",
    ])
    _warn = []
    if _now["margin"] < 1:
        _warn.append("**The drive cannot hold the part** - force margin below 1.")
    if _now["S_rp"] < 2:
        _warn.append("**Jaw safety below 2.**")
    mo.vstack([mo.mermaid(_diagram)] + [mo.callout(mo.md(w), kind="danger") for w in _warn])
    return


@app.cell
def _(img, mo):
    mo.vstack([
        mo.md("## 4. CAD and robot cell"),
        mo.ui.tabs({
            "Assembly (animation)": img("shared", "handover_viktoriia", "images", "assembly.gif"),
            "Drive train (animation)": img("shared", "handover_viktoriia", "images", "drive_train.gif"),
            "Gripping cycle (animation)": img("shared", "handover_viktoriia", "images", "gripper_cycle.gif"),
            "Assembly drawing": img("Elias", "03_Drawing", "out", "sheet1.png"),
            "Final jaw: T-shape": img("shared", "presentation", "media", "t_jaw.png",
                                      caption="EN AW-7075 T6, wide low pad against the twist of part B (21 Nov)"),
            "Jaw drawing": img("Elias", "03_Drawing", "out", "sheet_jaw.png"),
            "Journey: Viktoriia's jaw V3": img("Viktoriia", "03_CAD_Jaws", "jaw_L_render.png",
                                               caption="V3 (19 Nov) - the design review widened it into the T-jaw"),
            "Robot placement": img("Elias", "01_Robot", "figures", "placement_map.png"),
            "Wrist torque": img("Elias", "01_Robot", "figures", "wrist_torque.png"),
        }),
    ])
    return


@app.cell
def _(FEM, img, mo):
    mo.vstack([
        mo.md("## 5. Strength: FEM of the jaw\n\ngmsh + CalculiX, quadratic tetrahedra, mesh convergence for every load case."),
        mo.ui.tabs({
            "T-jaw train, full current (LC2)": img(*FEM, "train_lc2_von_mises.png",
                                                   caption="slide + holder + T-jaw; peak 15 MPa at the bolted joint, safety ≥ 16"),
            "Mesh convergence": img(*FEM, "convergence_train.png"),
            "Journey: von Mises, jaw V3": img(*FEM, "jaw_v3_von_mises.png", caption="Viktoriia's jaw V3 before the review"),
            "Journey: deflection, jaw V3": img(*FEM, "jaw_v3_deflection.png"),
        }),
    ])
    return


@app.cell
def _(SIM, img, mo, vid):
    mo.vstack([
        mo.md("## 6. Simulation in MuJoCo\n\nThe real robot path, contact forces at the pads, and what happens when things go wrong."),
        mo.ui.tabs({
            "Grip over the cycle": mo.vstack([img(*SIM, "grip_slip.png"), vid(*SIM, "cycle_estop.mp4")]),
            "Emergency stop": img(*SIM, "e_stop.png"),
            "Power failure": mo.vstack([img(*SIM, "power_fail.png"), img(*SIM, "brake_play.png")]),
            "Closing": mo.vstack([img(*SIM, "closing.png"), vid(*SIM, "closing_two-stage_120ms.mp4")]),
            "Robustness": img(*SIM, "robustness.png"),
        }),
    ])
    return


@app.cell
def _(SIM, mo, vid):
    mo.vstack([mo.md("## 7. The whole cell"), vid(*SIM, "cell_clean.mp4")])
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 8. What we learned

    - **Change the concept early.** The switch to rack and pinion on 30 Oct cost a week; later it would have cost the whole chain.
    - **One parameter file** drives CAD, sizing, FEM and simulation - a design change is one edit and a rebuild.
    - **Two independent checks** (hand calculation and FEM/simulation) found real errors in both directions, and the iterations closed them.
    - **The force had to be limited, not raised** - the drive can do far more than part B needs.
    """)
    return


if __name__ == "__main__":
    app.run()
