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

    def explain(axes=None, see=None, so=None, h=150):
        """fixed-height note under a figure: what the axes are, what you see, what follows from it"""
        _rows = [(t, v) for t, v in (("Axes", axes), ("What you see", see), ("So", so)) if v]
        _html = "".join(f'<p style="margin:0.15em 0"><b>{t}.</b> {mo.md(v).text}</p>' for t, v in _rows)
        return mo.Html(f'<div style="height:{h}px;overflow:auto;font-size:0.92em;line-height:1.35">{_html}</div>')

    def panel(media, **kw):
        return mo.vstack([media, explain(**kw)], gap=0.4)

    return FEM, P, SHEETS, SIM, explain, f, img, mo, panel, row, vid


@app.cell
def _(f):
    import json as _json

    def _j(*parts):
        return _json.load(open(f(*parts), encoding="utf-8"))

    D = dict(
        estop=_j("Elias", "04_Simulation", "out", "e_stop.json"),
        power=_j("Elias", "04_Simulation", "out", "power_fail.json"),
        brake=_j("Elias", "04_Simulation", "out", "brake_play.json"),
        closing=_j("Elias", "04_Simulation", "out", "closing_results.json"),
        rob=_j("Elias", "04_Simulation", "out", "robustness_cpu.json"),
        wrist=_j("Elias", "01_Robot", "wrist_torque.json"),
        robot=_j("Elias", "01_Robot", "robot_selection.json"),
        train=_j("Elias", "05_FEM", "out", "train_results.json"),
        conv=_j("Elias", "05_FEM", "out", "convergence_train.json"),
        v4=_j("Elias", "05_FEM", "out", "v4_check.json"),
    )
    LC2 = next(c for c in D["train"]["cases"] if c["case"].startswith("LC2"))
    return D, LC2


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
def _(D, P, mo):
    _t5 = max(v["t5_max"] for v in D["wrist"].values() if isinstance(v, dict))
    _bound = next(c for c in D["robot"]["checks"] if c["check"].startswith("wrist torque axis 4/5"))
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
    | robot | ABB IRB 1300-7/1.4, wrist torque along the cycle {_t5:.1f} N·m = {_t5 / 13 * 100:.0f} % of 13 N·m (worst-case bound, tool horizontal: {_bound['value']:.1f} N·m = {_bound['use'] * 100:.0f} %) |
    | jaw V3: beam theory (V4) vs. FEM | stress +1.6 %, deflection +4.1 % / +5.0 % - **within 5 %** (the check of the analytical method, done on jaw V3) |
    """)
    return


@app.cell
def _(controls_drive, mo):
    mo.vstack([mo.md(r"""
    ### How everything is connected

    Every box is a value of the design. The sizing inputs above drive it too; here come the drive and the jaw
    material. **orange** = higher than our design point, **teal** = lower. Jaw stress and pad deflection scale
    linearly with the force from the FEM of the final T-jaw (slide + holder + jaw, full-current case); the jaw
    material changes the jaw's safety, the deflection comes from all three parts together.
    """), controls_drive])
    return


@app.cell
def _(LC2, MATERIALS, P, mo, pick_mat, sl):
    # jaw stress and pad deflection scale linearly from the T-jaw FEM (train: slide + holder + T-jaw, LC2)
    _F2, _sig2, _w2 = LC2["F"][0], LC2["parts"]["jaw"]["vm_peak"], LC2["u_pad"] * 1000

    def chain(mu, a, m, s, d, i, mat):
        _E, _rp, _, _ = MATERIALS[mat]
        _r = d / 2000
        F = s * m * (P.G + a) / (P.N_BACKEN * mu)
        T_pin = 2 * F * _r / (P.ETA_BEVEL * P.ETA_RP)
        T_mot = T_pin / (i * P.GEAR_ETA)
        F_av = P.MOTOR_T_HOLD * i * P.GEAR_ETA * P.ETA_BEVEL * P.ETA_RP / (2 * _r)
        sig = _sig2 * F / _F2
        return dict(F=F, T_pin=T_pin, T_mot=T_mot, util=T_mot / P.MOTOR_T_HOLD * 100, F_av=F_av, margin=F_av / F,
                    sig=sig, S_rp=_rp / sig, w=_w2 * F / _F2)

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
        node("w", "pad deflection", ".0f", "µm"),
        "mu --> F", "a --> F", "m --> F", "s --> F",
        "F --> T_pin", "d --> T_pin", "T_pin --> T_mot", "i --> T_mot", "T_mot --> util",
        "d --> F_av", "i --> F_av", "F_av --> margin", "F --> margin",
        "F --> sig", "sig --> S_rp", "mat --> S_rp", "F --> w",
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
def _(D, P, img, mo, panel):
    _pl = D["robot"]["placement"]["current"]
    _w = {c: v for c, v in D["wrist"].items() if isinstance(v, dict)}
    _t5 = max(v["t5_max"] for v in _w.values())
    _t6 = max(v["t6_max"] for v in _w.values())
    _bound = next(c for c in D["robot"]["checks"] if c["check"].startswith("wrist torque axis 4/5"))
    _H = 470
    mo.vstack([
        mo.md("## 4. CAD and robot cell"),
        mo.ui.tabs({
            "Assembly (animation)": panel(img("shared", "handover_viktoriia", "images", "assembly.gif", h=_H),
                see="The assembly order the CAD scripts build, each part along its own joining direction.",
                so="The order is fixed in the parts script, so drawing, animation and the self-assembly in the film all follow the same sequence."),
            "Drive train (animation)": panel(img("shared", "handover_viktoriia", "images", "drive_train.gif", h=_H),
                see=f"Stepper motor → planetary gearbox i = {P.GEAR_I} → bevel pair (90°) → pinion d = {P.PINION_D:.0f} mm "
                    "→ two racks that move in opposite directions.",
                so="One pinion drives both jaws: they always move together, so the part is centred and both pads press with the same force."),
            "Gripping cycle (animation)": panel(img("shared", "handover_viktoriia", "images", "gripper_cycle.gif", h=_H),
                see=f"Open ({P.D_OPEN:.0f} mm per jaw) → close on part B → hold → open.",
                so="Open, the pads stand 86 mm apart around the 70 mm wide part: room for the position scatter on the belt."),
            "Assembly drawing": panel(img("Elias", "03_Drawing", "out", "sheet1.png", h=_H),
                see="Assembly drawing with the parts list (positions, standard parts, materials).",
                so="Every part is either a standard part or one of our machined parts with a drawing."),
            "Final jaw: T-shape": panel(img("shared", "presentation", "media", "t_jaw.png", h=_H),
                see="EN AW-7075 T6 jaw with a wide, low NBR pad on the flat zone of part B.",
                so="Part B's centre of mass sits below the pads; a wide pad gives the friction a long lever against the "
                   "twist of the part when the robot accelerates (design review 21 Nov)."),
            "Jaw drawing": panel(img("Elias", "03_Drawing", "out", "sheet_jaw.png", h=_H),
                see="Manufacturing drawing of the T-jaw.", so="Dimensions, tolerances and material for making the jaw."),
            "Journey: Viktoriia's jaw V3": panel(img("Viktoriia", "03_CAD_Jaws", "jaw_L_render.png", h=_H),
                see="Jaw V3 (19 Nov), narrow pad.",
                so="Strong enough (FEM 6.9 MPa), but the narrow pad could not hold the twist of the part - the review widened it into the T-jaw."),
            "Robot placement": panel(img("Elias", "01_Robot", "figures", "placement_map.png", h=_H),
                axes="x = how far the robot base sits towards the belt [mm]; y = pedestal height [mm]; colour = the "
                     "smallest distance of any joint to its limit over all waypoints of the cycle [°]. White = no IK "
                     "solution, red = arm more than 95 % stretched.",
                see="Low pedestals run into the joint limits (axis 5 at the pick); the margin grows with height.",
                so=f"We chose x = {_pl['x']:.0f} mm, pedestal {_pl['h']:.0f} mm: every joint keeps at least "
                   f"{_pl['margin']:.1f}° to its limit and the arm is only {_pl['stretch'] * 100:.0f} % stretched. "
                   "The lowest possible pedestal (200 mm) would leave only about 10°."),
            "Wrist torque": panel(img("Elias", "01_Robot", "figures", "wrist_torque.png", h=_H),
                axes="x = time in one cycle [s]; y = torque at the robot wrist [N·m].",
                see=f"Axis 5 carries the offset centre of mass of gripper + part: peaks {_t5:.1f} N·m in the transfers; "
                    f"axis 6 stays below {_t6:.1f} N·m. Dashed: ABB limit 13 N·m; dotted: our worst-case bound with the "
                    f"tool horizontal, {_bound['value']:.1f} N·m.",
                so=f"{_t5 / 13 * 100:.0f} % of the limit along the real cycle, {_bound['use'] * 100:.0f} % in the worst case: "
                   "the IRB 1300 carries the gripper with reserve."),
        }),
    ])
    return


@app.cell
def _(D, FEM, LC2, img, mo, panel):
    _parts = LC2["parts"]
    _conv = D["conv"]
    _keys = sorted(_conv, key=lambda k: -float(k))
    _last = lambda q: abs(_conv[_keys[-1]][q] / _conv[_keys[-2]][q] - 1) * 100
    _v3 = D["v4"]["1"]["standard"]
    _rp_h = _parts["holder"]["safety"] * _parts["holder"]["vm_peak"]      # yield of the holder material, from the FEM run
    _fine_h = _conv[_keys[-1]]["vm"]["holder"]
    _H = 470
    mo.vstack([
        mo.md("## 5. Strength: FEM of the jaw\n\ngmsh + CalculiX, quadratic tetrahedra, mesh convergence for every load case."),
        mo.ui.tabs({
            "T-jaw train, full current (LC2)": panel(img(*FEM, "train_lc2_von_mises.png", h=_H),
                axes=f"Colour = von Mises stress [MPa], scale cut at 7 MPa so the jaw stays readable; deformation "
                     f"drawn 300 × larger. Load: full motor current, {LC2['F'][0]:.1f} N per jaw on the pad.",
                see=f"Slide, holder and T-jaw as one model. Highest stresses: holder {_parts['holder']['vm_peak']:.1f} MPa "
                    f"and slide {_parts['slide']['vm_peak']:.1f} MPa at the bolted joint, jaw {_parts['jaw']['vm_peak']:.1f} MPa. "
                    f"Pad tilt {LC2['tilt_deg']:.3f}°, pad deflection {LC2['u_pad'] * 1000:.0f} µm.",
                so=f"Safety against yield at least {min(v['safety'] for v in _parts.values()):.0f} (slide and holder EN AW-6082 T6, "
                   f"Rp0.2 {_rp_h:.0f} MPa; jaw EN AW-7075 T6, 460 MPa), at the highest force the drive can produce: strength "
                   "is not what limits the jaw train."),
            "Mesh convergence": panel(img(*FEM, "convergence_train.png", h=_H),
                axes="x = number of elements (log); y = strain energy, pad tilt, pad deflection and peak stress of the same load case.",
                see=f"The global values settle: last refinement changes the energy by {_last('energy'):.2f} %, the tilt by "
                    f"{_last('tilt_deg'):.2f} %, the deflection by {_last('u_pad'):.2f} %. The peak stresses keep rising "
                    "with every refinement: they sit on sharp edges (singularities).",
                so=f"Tilt and deflection are mesh-independent results. The peak stresses are not: on the finest mesh the "
                   f"holder edge reaches {_fine_h:.0f} MPa, still a safety of {_rp_h / _fine_h:.0f} - the real stress away from "
                   "the edge is lower."),
            "Journey: von Mises, jaw V3": panel(img(*FEM, "jaw_v3_von_mises.png", h=_H),
                axes="Colour = von Mises stress [MPa] in Viktoriia's jaw V3, full-current load.",
                see=f"Peak {_v3['vm_peak']:.1f} MPa in the clean section between holder and pocket.",
                so="V3 was strong enough; the review changed it because of the grip (twist of the part), not because of strength."),
            "Journey: deflection, jaw V3": panel(img(*FEM, "jaw_v3_deflection.png", h=_H),
                axes="Colour = displacement of jaw V3 [µm] under the same load.",
                see=f"Largest deflection {_v3['ux']:.2f} µm at the pad.",
                so="Beam theory (V4) and FEM agree within 5 % on this jaw - the check that our hand calculation can be trusted."),
        }),
    ])
    return


@app.cell
def _(D, P, SIM, img, mo, panel, row):
    _es = D["estop"]["scenarios"]
    _mx = lambda k: max(v["N"] for v in _es[k].values() if isinstance(v, dict) and "N" in v)
    _set, _lim = D["estop"]["setpoint"], D["estop"]["current_limit"]
    _pw = D["power"]
    _br = D["brake"]["cases"]["NBR E 5.5 MPa"]
    _cl = D["closing"]
    _case = lambda **q: next(c for c in _cl["cases"] if all(abs(c[k] - v) < 1e-6 if isinstance(v, float) else c[k] == v
                                                              for k, v in q.items()))
    _dir = _case(profile="direct", t_fast=0.12)
    _built = _case(profile="tolerant", v_touch=0.02, offset_mm=5.0, blocked=False)
    _rb = D["rob"]["results"]
    _spec = [r for r in _rb if r["study"] == "spec"]
    _stress = [r for r in _rb if r["study"] == "stress"]
    _rs = D["rob"]["ranges"]
    _H = 420
    mo.vstack([
        mo.md("## 6. Simulation in MuJoCo\n\nThe real robot path, contact forces at the pads, and what happens when things go wrong."),
        mo.ui.tabs({
            "Grip over the cycle": panel(row(SIM + ("grip_slip.png",), SIM + ("cycle_estop.mp4",), h=_H), h=190,
                axes="x = time in the cycle [s]; y = how far the part moves in the jaws [mm], logarithmic. Black dashed: "
                     "the same run at 100 N - the creep of MuJoCo's soft contact, not real slip. Dotted: slip limit = creep + 0.5 mm.",
                see=f"Without safety (S = 1) the part slips at the lift (about 0.45 s) and falls out. The required force "
                    f"({P.F_GRIP_OILY:.1f} N), our setpoint ({P.F_GRIP_SET:.1f} N) and both e-stop runs lie on the creep "
                    "line. Video: the e-stop cycle, slowed down 4 ×.",
                so="The safety factor 2 is needed and it is enough: the part holds through the whole cycle, e-stop included."),
            "Emergency stop": panel(img(*SIM, "e_stop.png", h=_H), h=190,
                axes="x = the moment in the loaded part of the cycle where the category 0 stop hits [s]; y = normal force "
                     "per jaw that is just enough to hold the part (S = 1) [N]. Curves: braking from the data sheet or twice "
                     "as hard, pad friction 0.2 or 0.3, both chains.",
                see=f"With the data-sheet braking the worst moment needs {_mx('data sheet'):.1f} N - below our setpoint "
                    f"{_set:.1f} N (S {_set / _mx('data sheet'):.1f}). Twice as hard braking needs {_mx('x2'):.1f} N: above the "
                    f"setpoint, below the current limit {_lim:.1f} N; with the grooved pads (µ 0.3) {_mx('x2, grooved pad mu 0.3'):.1f} N.",
                so="With the robot's own stop data the part holds. If the real robot brakes twice as hard, it holds only "
                   "with the grooved pads' friction or the full current - that is why µ 0.3 is to be measured (V5)."),
            "Power failure": panel(row(SIM + ("power_fail.png",), SIM + ("brake_play.png",), h=_H), h=190,
                axes="Left two plots: time after the power loss [ms] against pad force [N] and part movement [mm], for "
                     "brake delays of 0 - 50 ms; dashed = force needed against the weight. Right: brake play at the motor "
                     "shaft [°] against the grip force left [N].",
                see=f"Brake at once: the {_pw['cases'][0]['force']:.1f} N stay. Any delay from 2 ms on: the preload is gone "
                    f"within about 2 ms and the part slides {_pw['cases'][1]['slip_end']:.0f} mm, out of the jaws. Brake play "
                    f"eats the preload linearly: at {_br['play_allowed_deg_S1']:.2f}° nothing is left against the weight; the "
                    "spring brake we had first allowed up to 2°.",
                so="The brake must close before the motor lets go (brake-first shutdown), and it must be free of play: we "
                   "replaced the spring brake (BWA) by the permanent-magnet brake BKE."),
            "Closing": panel(row(SIM + ("closing.png",), SIM + ("closing_tolerant_50ms_off5mm_slow20.mp4",), h=_H), h=190,
                axes="x = time after the close command [ms]; left y = jaw opening per side [mm], right y = pad force [N]. "
                     f"Red dashed: the {_cl['t_req'] * 1000:.0f} ms the takt allows for closing.",
                see=f"Closing at full speed hits the part with {_dir['f_peak']:.0f} N ({_dir['f_peak'] / P.F_GRIP_SET:.1f} × the "
                    "setpoint). Fast until just before contact, then slow, the force rises gently: our built profile grips "
                    f"in {_built['t_grip'] * 1000:.0f} ms with a peak of {_built['f_peak']:.1f} N, even with the part 5 mm off "
                    "centre (video: this profile, slowed down 20 ×). 'Blocked' shows a jammed part.",
                so="Two-stage closing: inside the time budget and no impact on the part."),
            "Robustness": panel(img(*SIM, "robustness.png", h=_H), h=190,
                axes="Left: how many of the simulated grips reach which peak friction use (1 = slip). Right: each dot is "
                     "one grip, x = pad friction, y = grip force per jaw; dashed = the hand calculation at S = 1; box = our "
                     "specification.",
                see=f"{len(_spec):,} grips with all inputs scattered inside the specification (offset ±5 mm, µ "
                    f"{_rs['spec']['mu'][0]}-{_rs['spec']['mu'][1]}, force ±5 %, mass +0-3 %): all held, highest friction use "
                    f"{max(r['util_max'] for r in _spec):.2f}. Stress test with µ down to {_rs['stress']['mu'][0]} and half the "
                    f"force: {len(_stress) - sum(r['held'] for r in _stress)} of {len(_stress):,} fail - all of them left of the "
                    "hand-calculation line.",
                so="The simulation confirms the formula, and our specification lies far from the limit."),
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
