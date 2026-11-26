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

    BOX_H = 520                                        # px: every media row has this height, so tabs do not jump

    def f(*parts):
        return os.path.join(ROOT, *parts)

    def _fit(inner, h):
        return mo.Html(f'<div style="height:{h}px;display:flex;gap:12px;align-items:center;justify-content:center;'
                       f'overflow:hidden">{inner}</div>')

    def _media(parts, caption=None, h=BOX_H):
        _path = f(*parts)
        if not os.path.exists(_path):
            return f"<em>missing: <code>{'/'.join(parts)}</code></em>"
        _style = f"max-height:{h - (28 if caption else 0)}px;max-width:100%;object-fit:contain"
        if _path.endswith(".mp4"):
            _el = mo.video(src=_path, controls=True, loop=True, muted=True).text
            _el = _el.replace("<video ", f'<video style="{_style}" ', 1)
        else:
            _el = mo.image(src=_path, style={"max-height": f"{h - (28 if caption else 0)}px", "max-width": "100%",
                                             "object-fit": "contain"}).text
        _cap = f'<div style="font-size:0.85em;color:#78716c;text-align:center">{caption}</div>' if caption else ""
        return f'<figure style="margin:0;flex:1;min-width:0;text-align:center">{_el}{_cap}</figure>'

    def img(*parts, caption=None, h=BOX_H):
        """one image or video in a fixed-height frame"""
        return _fit(_media(parts, caption, h), h)

    vid = img

    def row(*items, h=BOX_H):
        """several media side by side in one fixed-height frame; items are path tuples or (path tuple, caption)"""
        _html = []
        for it in items:
            _parts, _cap = (it[0], it[1]) if isinstance(it[0], tuple) else (it, None)
            _html.append(_media(_parts, _cap, h))
        return _fit("".join(_html), h)

    SIM = ("Elias", "04_Simulation", "figures")
    FEM = ("Elias", "05_FEM", "figures")
    SHEETS = ("Elias", "01_Robot", "out", "datasheets")
    return FEM, P, SHEETS, SIM, img, mo, row, vid


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
    Every input starts at **our design value**; the scenario list shows where each value comes from, the slider
    lets you try anything in between. Open *why this value* for the source.
    """)
    return


@app.cell
def _(P):
    # every input: scenarios (label -> value, why, datasheet crop or None); the first one is the design value
    INPUTS = {
        "mu": dict(label="friction µ", unit="", rng=(0.05, 0.6, 0.05), scen={
            "oily part - design (0.20)": (P.MU_OILY, "The part may come dirty or oily (feedback 10 Nov). NBR pad on "
                                          "oily PA 6, the low end of the oil-resistant NBR range. Brackets: oiled steel "
                                          "0.05 - 0.1, NBR gripper pads 0.3 - 0.4 (HKI), dry knurled NBR > 1 (PFA).", None),
            "grooved pads - expected (0.30)": (P.MU_PAD_EXPECTED, "Oil drains off through the grooves in the pad. A "
                                               "reserve for hard e-stops, to be measured (V5) - not used in the proof.", None),
            "steel on steel - Schunk rating (0.10)": (0.10, "What Schunk assumes for its payload ratings: shows how "
                                                      "much our pads gain over plain metal jaws.", None),
            "dry NBR - first assumption (0.50)": (0.50, "Our earlier design value, superseded once the part was declared "
                                                  "possibly oily (cad/params.py).", None),
        }),
        "a": dict(label="acceleration a", unit="m/s²", rng=(0.0, 14.0, 0.1), scen={
            "robot program cap - design (4.96)": (P.A_PATH_REAL, "We cap the loaded moves at 4.96 m/s² in the robot "
                                                  "program (WorldAccLim); the IRB 1300 could do 66 m/s² controlled "
                                                  "(product spec p. 54). The 5 s takt still holds (4.48 s).", "irb_tcp_acc.png"),
            "hand calculation, straight line (2.05)": (P.A_BAHN, "1800 mm in the time the takt leaves, trapezoid "
                                                       "profile on a straight line - the real joint path needs more.", None),
            "e-stop at the TCP (12.1)": (12.1, "Category 0 stop, axis 1 braking at 1.0 m (product spec p. 91). The "
                                         "setpoint still holds it: S 1.30 with shear and twist (MuJoCo, e_stop.py).",
                                         "irb_stop_data.png"),
            "standing still (0)": (0.0, "Only the weight: the lower bound.", None),
        }),
        "s": dict(label="safety factor S", unit="", rng=(1.0, 3.0, 0.1), scen={
            "VDI 2860 - design (2.0)": (P.S_SICHERHEIT, "Safety against slip for friction grippers after VDI 2860.", None),
            "no safety (1.0)": (1.0, "The bare friction limit: where the part would start to slip.", None),
        }),
        "m": dict(label="part mass m", unit="kg", rng=(0.3, 1.5, 0.01), scen={
            "part B from CAD - design (0.602)": (round(P.B_MASSE, 3), "STEP volume 528.5 cm³ × PA 6 density "
                                                 "1140 kg/m³ (datasheet 1.12 - 1.15).", None),
            "water-saturated PA 6 (0.66)": (round(P.B_MASSE * 1.095, 3), "PA 6 takes up to 9.5 % water when saturated; "
                                            "the robustness study checked it: 200 of 200 held.", None),
        }),
        "d": dict(label="pinion diameter d", unit="mm", rng=(12, 32, 1), scen={
            "m 1, z 20 - design (20)": (P.PINION_D, "Module 1 rack and pinion (Mädler, DIN 867); z = 20 lies above the "
                                        "limit of 17 teeth, so no profile shift.", None),
            "z 17 at the limit (17)": (17.0, "The smallest pinion without undercut: more force per motor torque.", None),
        }),
        "i": dict(label="gearbox ratio i", unit="", rng=(3.0, 15.0, 0.01), scen={
            "Nanotec GP42-S1-7 - design (7.07)": (P.GEAR_I, "Planetary gearbox on the NEMA 17 stepper, η 0.90 "
                                                  "(Nanotec catalogue, vendor/SOURCES.md).", None),
        }),
    }
    MATERIALS = {  # E [GPa], Rp0.2 [MPa], density [g/cm³], why
        "EN AW-7075 T6 (ours)": (71.7, 460.0, 2.81, "High-strength aluminium (EN 485-2, plate 6 - 12.5 mm): the jaw "
                                 "stays light and stiff, safety ≥ 16 at full current (FEM)."),
        "EN AW-6082 T6": (70.0, 250.0, 2.70, "The standard machining alloy: same stiffness, half the yield strength."),
        "S355 steel": (210.0, 355.0, 7.85, "Three times stiffer, but almost three times the moving mass on the jaws."),
        "PA 6 printed": (2.5, 50.0, 1.14, "A printed prototype jaw: cheap, but 30 × softer."),
    }
    return INPUTS, MATERIALS


@app.cell
def _(INPUTS, MATERIALS, mo):
    pick = {k: mo.ui.dropdown(list(v["scen"]), value=list(v["scen"])[0], label=v["label"]) for k, v in INPUTS.items()}
    pick_mat = mo.ui.dropdown(list(MATERIALS), value=list(MATERIALS)[0], label="jaw material")
    return pick, pick_mat


@app.cell
def _(INPUTS, mo, pick):
    # a new scenario re-creates its slider at the scenario value; the slider is the free play around it
    def _sl(k):
        _v = INPUTS[k]
        _lo, _hi, _st = _v["rng"]
        _val = _v["scen"][pick[k].value][0]
        return mo.ui.slider(_lo, _hi, step=_st, value=min(max(_val, _lo), _hi), show_value=True,
                            label=f'{_v["label"]}' + (f' [{_v["unit"]}]' if _v["unit"] else ""), full_width=True)
    sl = {k: _sl(k) for k in INPUTS}
    return (sl,)


@app.cell
def _(INPUTS, MATERIALS, SHEETS, img, mo, pick, pick_mat, sl):
    def _why(k):
        _val, _txt, _crop = INPUTS[k]["scen"][pick[k].value]
        _items = [mo.md(_txt)]
        if _crop:
            _items.append(img(*SHEETS, _crop, h=260))
        return mo.accordion({"why this value": mo.vstack(_items)})

    def _card(k):
        return mo.vstack([pick[k], sl[k], _why(k)], gap=0.3)

    _mat = mo.vstack([pick_mat, mo.accordion({"why this value": mo.md(MATERIALS[pick_mat.value][3])})], gap=0.3)
    controls = mo.vstack([
        mo.hstack([_card("mu"), _card("a")], widths="equal", gap=2),
        mo.hstack([_card("s"), _card("m")], widths="equal", gap=2),
    ], gap=1)
    controls_drive = mo.hstack([_card("d"), _card("i"), _mat], widths="equal", gap=2)
    controls
    return (controls_drive,)


@app.cell
def _(P, mo, sl):
    _F = sl["s"].value * sl["m"].value * (P.G + sl["a"].value) / (P.N_BACKEN * sl["mu"].value)
    _ok = _F <= P.F_JAW_AVAIL
    mo.hstack([
        mo.stat(f"{_F:.1f} N", label="required per jaw", caption=f"m = {sl['m'].value:.3f} kg", bordered=True),
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
def _(controls_drive, mo):
    mo.vstack([mo.md(r"""
    ### How everything is connected

    Every box is a value of the design. The sizing inputs above drive it too; here come the drive and the jaw
    material. **orange** = higher than our design point, **teal** = lower. Jaw stress and deflection are scaled
    linearly from the FEM of jaw V3 (6.9 MPa peak, 5.82 µm at 63.6 N).
    """), controls_drive])
    return


@app.cell
def _(MATERIALS, P, mo, pick_mat, sl):
    def chain(mu, a, m, s, d, i, mat):
        _E, _rp, _, _ = MATERIALS[mat]
        _r = d / 2000
        F = s * m * (P.G + a) / (P.N_BACKEN * mu)
        T_pin = 2 * F * _r / (P.ETA_BEVEL * P.ETA_RP)
        T_mot = T_pin / (i * P.GEAR_ETA)
        F_av = P.MOTOR_T_HOLD * i * P.GEAR_ETA * P.ETA_BEVEL * P.ETA_RP / (2 * _r)
        sig = 6.9 * F / 63.6
        return dict(F=F, T_pin=T_pin, T_mot=T_mot, util=T_mot / P.MOTOR_T_HOLD * 100, F_av=F_av, margin=F_av / F,
                    sig=sig, S_rp=_rp / sig, w=5.82 * F / 63.6 * 71.7 / _E)

    _base = chain(P.MU_OILY, P.A_PATH_REAL, P.B_MASSE, P.S_SICHERHEIT, P.PINION_D, P.GEAR_I, "EN AW-7075 T6 (ours)")
    _now = chain(sl["mu"].value, sl["a"].value, sl["m"].value, sl["s"].value, sl["d"].value, sl["i"].value,
                 pick_mat.value)

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
        _in("mu", "friction µ", f"{sl['mu'].value:.2f}"),
        _in("a", "acceleration", f"{sl['a'].value:.1f} m/s²"),
        _in("m", "part mass", f"{sl['m'].value:.2f} kg"),
        _in("s", "safety S", f"{sl['s'].value:.1f}"),
        _in("d", "pinion d", f"{sl['d'].value:g} mm"),
        _in("i", "gearbox i", f"{sl['i'].value:g}"),
        _in("mat", "jaw material", pick_mat.value.split(" (")[0]),
        node("F", "grip force / jaw", ".1f", "N"),
        node("T_pin", "pinion torque", ".2f", "N·m"),
        node("T_mot", "motor torque", ".3f", "N·m"),
        node("util", "motor use", ".0f", "%"),
        node("F_av", "available force", ".1f", "N"),
        node("margin", "force margin", ".2f", "×"),
        node("sig", "jaw stress", ".1f", "MPa"),
        node("S_rp", "safety vs Rp0.2", ".0f", ""),
        node("w", "jaw deflection", ".1f", "µm"),
        "mu --> F", "a --> F", "m --> F", "s --> F",
        "F --> T_pin", "d --> T_pin", "T_pin --> T_mot", "i --> T_mot", "T_mot --> util",
        "d --> F_av", "i --> F_av", "F_av --> margin", "F --> margin",
        "F --> sig", "sig --> S_rp", "mat --> S_rp", "F --> w", "mat --> w",
    ])
    _warn = []
    if _now["margin"] < 1:
        _warn.append("**The drive cannot hold the part** - force margin below 1.")
    if _now["S_rp"] < 2:
        _warn.append("**Jaw safety below 2.**")
    _msg = mo.callout(mo.md(" ".join(_warn)), kind="danger") if _warn else mo.md("&nbsp;")
    # fixed heights: the diagram and the warning line keep their place whatever the values
    mo.vstack([mo.Html(f'<div style="height:430px;overflow:hidden">{mo.mermaid(_diagram).text}</div>'),
               mo.Html(f'<div style="height:64px">{_msg.text}</div>')])
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
def _(SIM, img, mo, row):
    mo.vstack([
        mo.md("## 6. Simulation in MuJoCo\n\nThe real robot path, contact forces at the pads, and what happens when things go wrong."),
        mo.ui.tabs({
            "Grip over the cycle": row(SIM + ("grip_slip.png",), SIM + ("cycle_estop.mp4",)),
            "Emergency stop": img(*SIM, "e_stop.png"),
            "Power failure": row(SIM + ("power_fail.png",), SIM + ("brake_play.png",)),
            "Closing": row(SIM + ("closing.png",), SIM + ("closing_two-stage_120ms.mp4",)),
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
