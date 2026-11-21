import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="V1 - Concept and variant comparison")


@app.cell
def _():
    import sys
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

    sys.path.insert(0, str(mo.notebook_dir() / ".." / ".." / "Elias" / "02_CAD_Body"))
    import gripper_params as P
    return Circle, FancyArrowPatch, P, Rectangle, mo, np, plt


@app.cell
def _(P, mo):
    mo.md(rf"""
    # V1 - Concept and variant comparison

    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch** · status 21 Nov 2026 · milestone M1

    ## 1. What the gripper has to do (assignment WS26, Tab. 1, Abb. 1)

    | Requirement | Value | Consequence |
    |---|---|---|
    | Part B | PA 6, {P.B_MASSE:.3f} kg, 95 × 100 × 80 mm, Ra 1.6 | soft pads, no marks |
    | Grip at the buffer belt | **only from the side, no gripping from below** | jaws on the two flat side faces (± X, 70 mm apart) |
    | Jaws | "adapted to the part, with a quick-change system" | jaw pair on a quick-change holder |
    | Cycle | 5 s per part, belt 1000 mm → carrier 1300 mm, 1800 mm apart | grip and release within a few tenths of a second |
    | Part may be oily | feedback 10 Nov | friction grip with µ = 0.2 as design value |
    | Holding | the part must not drop on power failure | self-locking stage or holding brake |

    ## 2. Morphological box

    | Function | Solutions | Chosen |
    |---|---|---|
    | Hold the part | friction (force fit) · form fit · vacuum | **friction**: form fit needs an undercut, the part only allows side faces; vacuum is ruled out by the assignment ("Greiferzangen ... Schnellwechselsystem") |
    | Jaw motion | parallel · angular | **parallel**: the side faces are flat and parallel, an angular jaw would touch with an edge |
    | Transmission to two jaws | rack and pinion · left/right lead screw · parallelogram linkage · wedge hook | **compared in § 3** |
    | Drive | electric (motor current = grip force) | **electric**: grip force set in software, part detection from current and position, no compressed air in the cell |
    | Holding on power failure | holding brake · self-locking stage | depends on the transmission (§ 3) |
    | Jaw guide | ball rail · dovetail slide | **ball rail**: catalogue part with rated moments, no play |
    """)
    return


@app.cell
def _(P, mo, np):
    _stroke = P.D_OPEN - P.D_GRIP + 2.0      # mm per jaw from open to the part plus the closed reserve (10 mm)
    _r = P.PINION_D / 2
    rp_deg = np.degrees(P.D_OPEN / _r)
    rp_motor_rev = rp_deg / 360 * P.GEAR_I
    _p, _d2, _mu = 2.0, 9.0, 0.10            # Tr10x2, flank diameter, steel/bronze greased
    _phi = np.arctan(_p / (np.pi * _d2))
    _rho = np.arctan(_mu / np.cos(np.radians(15)))
    eta_screw = np.tan(_phi) / np.tan(_phi + _rho)
    eta_screw_hi = np.tan(_phi) / np.tan(_phi + np.arctan(0.15 / np.cos(np.radians(15))))
    eta_screw_lo = np.tan(_phi) / np.tan(_phi + np.arctan(0.06 / np.cos(np.radians(15))))
    screw_rev = P.D_OPEN / _p
    _L = 60.0                                 # mm parallelogram link length
    _dz_link = _L * (1 - np.cos(np.arcsin(P.D_OPEN / _L)))
    mo.md(rf"""
    ## 3. Four transmissions, all electric

    Numbers for the stroke of {P.D_OPEN:.0f} mm per jaw (open 86 mm, part 70 mm) and the 0.4 s closing budget.

    | | **A rack and pinion** | **B left/right lead screw** | **C parallelogram linkage** | **D wedge hook** |
    |---|---|---|---|---|
    | Principle | one pinion, two opposed racks (half of the Cartesian Hand) | one Tr10x2 screw, left and right thread, one nut per jaw | two parallel links per jaw, driven by a crank | a pushed wedge moves both jaws across |
    | Motion for the stroke | pinion {rp_deg:.0f}°, motor {rp_motor_rev:.1f} rev (i {P.GEAR_I}) | {screw_rev:.0f} screw rev | crank ≈ 10° | wedge stroke ≈ 2 × jaw stroke |
    | Efficiency | ≈ 0.97 per mesh | {eta_screw:.2f} (µ 0.1), {eta_screw_hi:.2f} ... {eta_screw_lo:.2f} for µ 0.15 ... 0.06 | ≈ 0.95, but varies with the angle | ≈ 0.6 (sliding wedge) |
    | Grip force from current | accurate (high, steady efficiency) | poor: friction scatter changes the force by ± 30 % | changes over the stroke | fair |
    | Self-locking | no → holding brake needed | **yes** (lead angle {np.degrees(_phi):.1f}° < friction angle {np.degrees(_rho):.1f}°) | no | yes if the wedge angle is small |
    | Jaw path | straight, force constant | straight, force constant | arc: jaw moves {_dz_link:.1f} mm up/down over the stroke (60 mm links) | straight, short stroke |
    | Centring | synchronous by the pinion | synchronous by the screw | synchronous by the crank | synchronous |
    | Parts | catalogue racks + pinion | catalogue screw + 2 nuts | many pins and links | wedge + linear actuator (again a screw) |

    The lead screw is the only variant that holds without a brake. Its price: low and scattering efficiency, so the
    grip force can no longer be set by the motor current, and {screw_rev:.0f} turns in the fast phase need high motor speed.
    """)
    return eta_screw, rp_deg


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. Weighted evaluation (VDI 2225)

    Scores 0 (unsatisfactory) ... 4 (ideal); weights 1 ... 3 after the requirements in § 1. Technical value
    $x = \sum g_i\,p_i \,/\, (p_{max} \sum g_i)$. The weights can be changed with the sliders to test the decision.
    """)
    return


@app.cell
def _(mo):
    CRITERIA = [   # name, default weight, scores A B C D, reason
        ("grip force set by the motor current", 3, (4, 1, 3, 2), "efficiency and its scatter (§ 3)"),
        ("holding on power failure", 2, (2, 4, 1, 3), "A, C need a brake; B self-locking; D only with a flat wedge"),
        ("closing speed (0.4 s budget)", 2, (4, 2, 3, 2), "A 1.1 motor rev; B 5 screw rev; D short wedge stroke"),
        ("parallel jaws, constant force over the stroke", 3, (4, 4, 2, 3), "C moves on an arc, force varies"),
        ("centring, synchronous jaws", 2, (4, 4, 3, 4), "all synchronous; C has play in many joints"),
        ("build space and mass", 2, (3, 3, 2, 4), "D most compact; C needs room for the links"),
        ("robustness, oily environment, wear", 2, (3, 2, 3, 2), "screw and wedge slide in the dirt"),
        ("parts, catalogue availability, cost", 1, (4, 3, 2, 2), "A, B catalogue parts"),
        ("stroke reserve for the part tolerance", 1, (4, 4, 2, 1), "A up to 12.5 mm without redesign"),
    ]
    weights = mo.ui.array([mo.ui.slider(0, 3, step=1, value=w, label=n) for n, w, _, _ in CRITERIA])
    weights
    return CRITERIA, weights


@app.cell
def _(CRITERIA, mo, np, plt, weights):
    _names = ["A rack and pinion", "B lead screw", "C linkage", "D wedge hook"]
    _g = np.array(weights.value, float)
    _p = np.array([c[2] for c in CRITERIA], float)
    x_tech = (_g @ _p) / (4 * max(_g.sum(), 1e-9))
    _rows = "".join(f"| {c[0]} | {int(g)} | " + " | ".join(str(v) for v in c[2]) + f" | {c[3]} |\n"
                    for c, g in zip(CRITERIA, _g))
    fig_v, _ax = plt.subplots(figsize=(7, 2.6))
    _cols = ["#0f766e", "#94a3b8", "#94a3b8", "#94a3b8"]
    _ax.barh(_names[::-1], x_tech[::-1], color=_cols[::-1])
    for _i, _v in enumerate(x_tech[::-1]):
        _ax.text(_v + 0.01, _i, f"{_v:.2f}", va="center")
    _ax.axvline(0.8, color="#C2410C", ls="--", lw=1)
    _ax.set_xlim(0, 1)
    _ax.set_xlabel("technical value x (VDI 2225); 0.8 = good")
    fig_v.tight_layout()
    mo.vstack([mo.md(f"""
    | Criterion | g | A | B | C | D | Reason |
    |---|---|---|---|---|---|---|
    {_rows}| **technical value x** | | **{x_tech[0]:.2f}** | {x_tech[1]:.2f} | {x_tech[2]:.2f} | {x_tech[3]:.2f} | |
    """), fig_v])
    return (x_tech,)


@app.cell
def _(mo, x_tech):
    mo.md(rf"""
    **Result: A, rack and pinion (x = {x_tech[0]:.2f})**, ahead of the lead screw ({x_tech[1]:.2f}). Doubling the weight of
    "holding on power failure" does not change the order: the lead screw's self-locking does not make up for a grip
    force that cannot be set by current. The price of A is fixed in the concept: rack and pinion is back-drivable,
    so a **holding brake on the motor** is mandatory (sized in V2 § 9; it has to be free of play, see V2 § 9).
    """)
    return


@app.cell
def _(Circle, FancyArrowPatch, P, Rectangle, mo, plt):
    x_pad = P.PAD_X0 + P.D_GRIP                         # 35: pad face at the grip
    x_open = P.PAD_X0 + P.D_OPEN                         # 43
    zf = P.HOOD_TOP + P.HOOD_SEAT + P.ADAPTER_T          # 78: flange face
    z_tcp = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1])
    part_z = (P.PART_OFFSET[2] - 40, P.PART_OFFSET[2] + 40)
    fig_s, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 7.2), gridspec_kw=dict(width_ratios=[1.45, 1]))
    def box(a, x0, x1, z0, z1, fc, ec="#334155", lw=1.0, **k):
        a.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=fc, ec=ec, lw=lw, **k))
    hx = P.HOUSING_X / 2
    box(ax, -hx, hx, P.POCKET_Z - P.COVER_T, P.POCKET_Z, "#cbd5e1")             # cover
    box(ax, -hx, hx, P.POCKET_Z, P.PLATE_T, "#e2e8f0")                          # housing
    box(ax, -hx, hx, P.PLATE_T, P.HOOD_TOP, "#f1f5f9")                          # hood
    box(ax, -38, 38, P.HOOD_TOP, zf, "#94a3b8")                                 # adapter / flange
    box(ax, -15, 82, P.DRIVE_Z - 21, P.DRIVE_Z + 21, "#fde68a", ls="--")        # lying drive
    ax.text(33, P.DRIVE_Z, "Motor + Bremse + Getriebe\n(liegend)", ha="center", va="center", fontsize=8)
    box(ax, P.PINION_X - 10, P.PINION_X + 10, P.PINION_Z0, P.PINION_Z0 + P.PINION_B, "#0f766e")
    ax.text(P.PINION_X, P.PINION_Z0 - 4, "Ritzel z 20", ha="center", va="top", fontsize=8)
    for s in (-1, 1):
        xj = s * (x_pad + P.PAD_T + P.PAD_PLATE_T)
        box(ax, min(xj, xj + s * P.JAW_T), max(xj, xj + s * P.JAW_T), P.JAW_Z_BOTTOM, P.JAW_Z_TOP, "#64748b")
        box(ax, min(s * x_pad, s * (x_pad + P.PAD_T)), max(s * x_pad, s * (x_pad + P.PAD_T)),
            P.PAD_Z[0], P.PAD_Z[1], "#1e293b")
        box(ax, min(xj, xj + s * 14), max(xj, xj + s * 14), P.POCKET_Z - 0.5 - P.COVER_T, P.SLIDE_Z_TOP - 8, "#94a3b8")
        ax.add_patch(FancyArrowPatch((s * (x_pad + 22), z_tcp), (s * (x_pad + 8), z_tcp), arrowstyle="-|>",
                                     mutation_scale=14, color="#C2410C", lw=2))
    box(ax, -x_pad, x_pad, part_z[0], part_z[1], "#dbeafe", ec="#1d4ed8")
    ax.text(0, part_z[0] + 12, f"Bauteil B\nPA 6, {P.B_MASSE:.2f} kg", ha="center", fontsize=8, color="#1d4ed8")
    ax.text(x_pad + 24, z_tcp + 3, f"F$_G$ = {P.F_GRIP_SET:.1f} N", color="#C2410C", fontsize=9)
    def dim(a, p0, p1, txt, off=(0, 0), vertical=False):
        a.annotate("", p0, p1, arrowprops=dict(arrowstyle="<->", color="#0f172a", lw=0.8))
        a.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], txt, ha="center", va="center", fontsize=8,
               rotation=90 if vertical else 0, bbox=dict(fc="white", ec="none", pad=1))
    dim(ax, (-x_pad, part_z[0] - 8), (x_pad, part_z[0] - 8), f"{2 * x_pad:.0f} (Greifabstand)")
    dim(ax, (-x_open, part_z[0] - 20), (x_open, part_z[0] - 20), f"{2 * x_open:.0f} offen (Hub {P.D_OPEN:.0f} je Backe)")
    dim(ax, (-hx, zf + 10), (hx, zf + 10), f"{P.HOUSING_X:.0f}")
    dim(ax, (hx + 12, z_tcp), (hx + 12, zf), f"{zf - z_tcp:.0f} Flansch - TCP", vertical=True, off=(5, 0))
    ax.text(0, zf + 2, "ISO 9409-1-40-4-M6", ha="center", va="bottom", fontsize=8)
    ax.text(10, P.POCKET_Z + 8, "Kugelschiene Gr. 12, Schlitten (im Gehäuse)", fontsize=8)
    ax.text(-hx + 4, P.PAD_Z[0] - 14, "Backe + NBR-Belag\n(Schnellwechsel)", fontsize=8)
    ax.set_aspect("equal"); ax.set_xlim(-hx - 10, hx + 30); ax.set_ylim(part_z[0] - 30, zf + 22); ax.axis("off")
    ax.set_title("Vorderansicht (Schnitt durch die Greifachse)", fontsize=10)
    # principle, top view
    r = P.PINION_D / 2
    ax2.add_patch(Circle((0, 0), r, fc="#0f766e", ec="#334155"))
    for s in (-1, 1):
        box(ax2, -45, 45, s * r, s * (r + 6), "#94a3b8")
        ax2.add_patch(FancyArrowPatch((0, s * (r + 13)), (s * 22, s * (r + 13)), arrowstyle="-|>", mutation_scale=12,
                                      color="#0f172a"))
        ax2.text(s * 50, s * (r + 3), "Backe " + ("R" if s > 0 else "L"), va="center", ha="left" if s > 0 else "right",
                 fontsize=8)
    ax2.annotate("", (r * 0.7, -r * 0.7), (r * 0.7, r * 0.7), arrowprops=dict(arrowstyle="<-", connectionstyle="arc3,rad=0.6",
                                                                          color="white", lw=1.5))
    ax2.text(0, -r - 22, "Ein Ritzel treibt zwei gegenläufige Zahnstangen:\nbeide Backen laufen synchron und zentrisch,\n"
             f"Greifkraft = Motorstrom (Stromgrenze), Haltebremse am Motor.\nRitzel {P.D_OPEN / r * 57.2958:.0f}° für {P.D_OPEN:.0f} mm Hub",
             ha="center", va="top", fontsize=8)
    ax2.set_aspect("equal"); ax2.set_xlim(-75, 75); ax2.set_ylim(-75, 45); ax2.axis("off")
    ax2.set_title("Prinzip (Draufsicht)", fontsize=10)
    fig_s.suptitle("Konzeptskizze Greifer - Gruppe 10, Bauteil B (PA 6) - Ovdiienko / Bitsch", fontsize=11)
    fig_s.tight_layout(rect=(0, 0, 1, 0.94))
    fig_s.savefig(str(mo.notebook_dir() / "konzeptskizze_v1.png"), dpi=160)
    mo.vstack([mo.md("## 5. Dimensioned concept sketch (from the CAD parameters, saved as `konzeptskizze_v1.png`)"), fig_s])
    return


if __name__ == "__main__":
    app.run()
