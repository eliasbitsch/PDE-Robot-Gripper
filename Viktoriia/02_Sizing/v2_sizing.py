import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="V2 - Gripping force and drive sizing")


@app.cell
def _():
    import sys
    import math
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt

    # Single source of truth: Elias' CAD parameters (which import cad/params.py)
    sys.path.insert(0, str(mo.notebook_dir() / ".." / ".." / "Elias" / "02_CAD_Body"))
    import gripper_params as P
    return P, math, mo, np, plt


@app.cell
def _(mo):
    mo.md(r"""
    # V2 - Gripping force and drive sizing

    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**
    Status 18 Nov 2026, against CAD state E2 of 11 Nov 2026 (commit `707c09b`).

    All geometry and catalogue values are imported from `Elias/02_CAD_Body/gripper_params.py`,
    so a change in the CAD parameters flows through this notebook. Values that are *mine*
    (friction, stop deceleration, gear factors, material limits) are set in the cells below,
    each with its source. Values marked **[verify]** still need a check against the original table.

    Units: N, mm, N·mm and MPa unless stated otherwise.
    """)
    return


@app.cell
def _(mo):
    mu_s = mo.ui.slider(0.1, 0.5, step=0.05, value=0.2, label="friction coefficient µ (NBR pad / oily PA 6)", show_value=True)
    S_s = mo.ui.slider(1.5, 3.0, step=0.25, value=2.0, label="safety factor S (VDI 2860)", show_value=True)
    a_stop_s = mo.ui.slider(5.0, 25.0, step=1.0, value=17.0, label="emergency-stop deceleration [m/s²]", show_value=True)
    mo.vstack([mu_s, S_s, a_stop_s])
    return S_s, a_stop_s, mu_s


@app.cell
def _(mo):
    mo.md(r"""
    ## 1. Gripping force (force fit, 2 jaws)

    The part is held by friction only. Worst regular case: the robot accelerates the part upwards,
    so weight and inertia add up. Two friction faces, safety $S$ against slip:

    $$F_G = \frac{S \, m \, (g + a)}{n \, \mu}$$

    **Friction coefficient.** The part may be oily (feedback 10 Nov). Gripper manufacturers rate their
    catalogue payloads with a static friction coefficient of **µ = 0.1** and S = 2 (SCHUNK, e.g. PGN-plus,
    JGP-P catalogue pages) - that is the value for *steel jaws on a steel part*. SCHUNK's own application
    note gives µ = 0.1 ... 0.4 as the typical range and recommends rubber pads to raise it.
    For NBR pads on an oily PA 6 face we use **µ = 0.2** as design value (lower edge of the rubber range,
    twice the steel/steel catalogue value) and show below what happens at µ = 0.1.
    """)
    return


@app.cell
def _(P, S_s, mo, mu_s):
    m = P.B_MASSE                      # kg, 528.5 cm³ PA 6
    g = P.G
    a_path = P.A_BAHN                  # m/s², from the cycle-time budget
    n_faces = P.N_BACKEN
    mu = mu_s.value
    S = S_s.value
    F_req = S * m * (g + a_path) / (n_faces * mu)          # N per jaw
    F_max = P.F_JAW_MAX                                     # N per jaw at full motor current (Elias)
    F_req_mu01 = S * m * (g + a_path) / (n_faces * 0.1)
    mo.md(f"""
    | Quantity | Value |
    |---|---|
    | part mass $m$ | {m:.3f} kg |
    | weight $m g$ | {m*g:.2f} N |
    | inertia $m a$ ($a$ = {a_path:.2f} m/s²) | {m*a_path:.2f} N |
    | **required grip force per jaw** (µ = {mu:.2f}, S = {S:.1f}) | **{F_req:.1f} N** |
    | available at full motor current (Elias) | {F_max:.1f} N |
    | utilisation | {F_req/F_max*100:.0f} % |
    | same with µ = 0.1 (steel/steel catalogue value) | {F_req_mu01:.1f} N {'⚠ above motor limit' if F_req_mu01 > F_max else ''} |
    """)
    return F_max, F_req, S, a_path, g, m, mu, n_faces


@app.cell
def _(F_max, F_req, S, a_path, g, m, mu, n_faces, np, plt):
    _mus = np.linspace(0.1, 0.5, 100)
    _F = S * m * (g + a_path) / (n_faces * _mus)
    fig_mu, _ax = plt.subplots(figsize=(7, 3.2))
    _ax.plot(_mus, _F, color="#0f766e", lw=2, label="required, regular motion")
    _ax.axhline(F_max, color="#C2410C", ls="--", label="motor limit (full current)")
    _ax.fill_between(_mus, _F, F_max, where=_F > F_max, color="#C2410C", alpha=0.15)
    _ax.plot(mu, F_req, "o", color="black")
    _ax.annotate(f"{F_req:.1f} N", (mu, F_req), textcoords="offset points", xytext=(8, 8))
    _ax.set_xlabel("µ"); _ax.set_ylabel("grip force per jaw [N]"); _ax.set_ylim(0, 120)
    _ax.grid(alpha=0.3); _ax.legend(frameon=False); fig_mu.tight_layout()
    fig_mu
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 2. Dynamic load case: emergency stop

    In an emergency stop the robot brakes much harder than in the planned path. The deceleration is mainly
    horizontal (the path is 1.8 m horizontal, 0.3 m vertical). Worst direction for the friction grip:
    in the plane of the pads, perpendicular to the grip axis, combined with gravity:

    $$a_\mathrm{res} = \sqrt{g^2 + a_\mathrm{stop}^2}, \qquad F_{G,\mathrm{stop}} = \frac{S_\mathrm{stop}\, m \, a_\mathrm{res}}{n\,\mu}$$

    **Stop deceleration [verify].** ABB lists stopping distance and time for category 0 stops (max speed,
    arm stretched, max load) in the product specification. The IRB 1300 sheet (3HAC070393) was not accessible;
    the **CRB 1300-7/1.4** (same 1.4 m / 7 kg arm family, OmniCore) gives axis 1: 55.25° in 0.40 s,
    axis 2: 33.39° in 0.37 s, axis 3: 35.66° in 0.27 s. Assuming constant deceleration,
    $\alpha = 2\varphi / t^2$, times the lever arm to the TCP (1.4 m for axes 1 and 2, about 0.8 m for axis 3)
    gives 12 ... 17 m/s². **Design value 17 m/s² (≈ 1.7 g).** Replace by the IRB 1300 table when available.
    Note: ABB defines a *category 1* (controlled, on-path) stop as default for the IRB 1300 - that is gentler.

    The emergency stop is a rare, accidental load case, so it is checked with $S_\mathrm{stop}$ = 1 (must not slip)
    and 1.5 (recommended).
    """)
    return


@app.cell
def _(F_max, F_req, a_stop_s, g, m, math, mo, mu, n_faces):
    _stops = {"axis 1": (55.25, 0.40, 1.4), "axis 2": (33.39, 0.37, 1.4), "axis 3": (35.66, 0.27, 0.8)}
    _rows = ""
    for _k, (_phi, _t, _r) in _stops.items():
        _alpha = 2 * math.radians(_phi) / _t**2
        _rows += f"| {_k} | {_phi:.2f}° | {_t:.2f} s | {_alpha:.1f} rad/s² | {_r:.1f} m | {_alpha*_r:.1f} m/s² |\n"
    a_stop = a_stop_s.value
    a_res = math.hypot(g, a_stop)
    F_stop_S1 = 1.0 * m * a_res / (n_faces * mu)
    F_stop_S15 = 1.5 * m * a_res / (n_faces * mu)
    F_set = max(F_req, F_stop_S15)                 # recommended grip-force setpoint
    mo.md(f"""
    | CRB 1300-7/1.4, cat. 0 | angle | time | α = 2φ/t² | lever | a at TCP |
    |---|---|---|---|---|---|
    {_rows}

    | Quantity | Value |
    |---|---|
    | design stop deceleration | {a_stop:.1f} m/s² |
    | resulting acceleration in the pad plane $a_{{res}}$ | {a_res:.1f} m/s² |
    | required grip force, $S_{{stop}}$ = 1.0 | {F_stop_S1:.1f} N |
    | required grip force, $S_{{stop}}$ = 1.5 | {F_stop_S15:.1f} N |
    | safety of the regular setting {F_req:.1f} N against slip in an E-stop | **{F_req/F_stop_S1:.2f}** |
    | **recommended grip-force setpoint** max(regular, E-stop with 1.5) | **{F_set:.1f} N** ({F_set/F_max*100:.0f} % of the motor limit) |

    If the deceleration acts along the grip axis instead, one jaw takes it in compression (form fit):
    extra {m*a_stop:.1f} N on that jaw - inside the {F_max:.1f} N envelope used below.
    """)
    return F_set, a_res, a_stop


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Surface pressure pad / PA 6

    The pad plate (steel 3 mm + NBR 70 Shore A 3 mm) sits in the 23 mm wide pocket over the 35 mm pad zone.
    The contact area is the **pad**, not the 6200 mm² side face of the part.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, mo):
    A_pad = 23.0 * 35.0                       # mm², pocket width x pad zone length (handover § 4)
    E_nbr = 5.0                               # MPa, NBR 70 Shore A (same value as cad/params.py)
    t_nbr = 3.0                               # mm
    sigma_pa6 = 70.0                          # MPa, PA 6 yield 60-80 MPa (Röchling / Ensinger data sheets)
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _p = _F / A_pad
        _rows += f"| {_lbl} | {_F:.1f} N | {_p:.3f} MPa | {sigma_pa6/_p:.0f} | {_p/E_nbr*t_nbr*1000:.0f} µm |\n"
    mo.md(f"""
    Pad area $A$ = 23 × 35 = {A_pad:.0f} mm²

    | Load | F | p = F/A | safety vs. PA 6 yield ({sigma_pa6:.0f} MPa) | NBR compression |
    |---|---|---|---|---|
    {_rows}

    The pressure is three orders of magnitude below the PA 6 limit, and the soft NBR layer leaves no marks on
    the Ra 1.6 surface. Surface pressure is not a design criterion here.
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. Rack and pinion: tooth root and flank (DIN 3990, method B simplified, Roloff/Matek ch. 21)

    One pinion (m = 1, z = 20, 16MnCr5 case hardened) drives two racks (m = 1, C45) on opposite sides.
    Each mesh carries the force of one jaw, $F_t = F_G$ (rail friction neglected).

    $$\sigma_F = \frac{F_t}{b\,m}\,Y_{Fa}Y_{Sa}Y_\varepsilon\;K_A K_V K_{F\beta}K_{F\alpha}
    \le \sigma_{FP} = \frac{\sigma_{F\lim} Y_{ST}}{S_{F\min}}$$

    $$\sigma_H = Z_E Z_H Z_\varepsilon \sqrt{\frac{F_t}{b\,d_1}\,\frac{u+1}{u}}\;\sqrt{K_A K_V K_{H\beta}K_{H\alpha}}
    \le \sigma_{HP} = \frac{\sigma_{H\lim} Z_{NT}}{S_{H\min}}$$

    Factors as in the Mädler catalogue calculation (DIN 3990/3991): $K_A$ = 1.25, $K_{F\beta}=K_{H\beta}$ = 1.5,
    $K_V$ = 1 (pitch-line speed < 0.1 m/s), $S_{F\min}$ = 1.5, $S_{H\min}$ = 1.0. For the rack $u \to \infty$.
    Load cycles: every grip loads the **same teeth** (the pinion only swings ≈ 46° per grip), about
    720 grips/h → > 10⁷ cycles in a few years, so fatigue limits apply ($Z_{NT}$ = 1).

    Material values: C45 normalised σFlim = 200 MPa, σHlim = 590 MPa (Mädler catalogue);
    16MnCr5 case hardened σFlim = 430 MPa, σHlim = 1470 MPa **[verify: Roloff/Matek TB 20-1]**.
    Form factors from the DIN 3990-3 chart for x = 0 **[read from chart, ± 3 %]**.
    """)
    return


@app.cell
def _(math):
    def contact_ratio_rack(m_, z_, alpha_deg=20.0):
        """Transverse contact ratio of a spur pinion (x = 0, ha = m) meshing with a rack."""
        al = math.radians(alpha_deg)
        r = m_ * z_ / 2
        ra, rb = r + m_, r * math.cos(al)
        return (math.sqrt(ra**2 - rb**2) - r * math.sin(al) + m_ / math.sin(al)) / (math.pi * m_ * math.cos(al))

    def contact_ratio_pair(m_, z1, z2, alpha_deg=20.0):
        """Transverse contact ratio of two external spur gears (x = 0, ha = m)."""
        al = math.radians(alpha_deg)
        r1, r2 = m_ * z1 / 2, m_ * z2 / 2
        g1 = math.sqrt((r1 + m_)**2 - (r1 * math.cos(al))**2)
        g2 = math.sqrt((r2 + m_)**2 - (r2 * math.cos(al))**2)
        return (g1 + g2 - (r1 + r2) * math.sin(al)) / (math.pi * m_ * math.cos(al))

    KA, KV, KFb, KHb = 1.25, 1.0, 1.5, 1.5
    SF_min, SH_min, Y_ST = 1.5, 1.0, 2.0
    ZE, ZH = 189.8, 2.495                       # steel/steel, alpha = 20°, x = 0
    MAT = {"C45 N": (200.0, 590.0), "16MnCr5 E": (430.0, 1470.0)}   # sigma_Flim, sigma_Hlim
    return (KA, KFb, KHb, KV, MAT, SF_min, SH_min, Y_ST, ZE, ZH,
            contact_ratio_pair, contact_ratio_rack)


@app.cell
def _(F_max, F_req, KA, KFb, KHb, KV, MAT, P, SF_min, SH_min, Y_ST, ZE, ZH, contact_ratio_rack, math, mo):
    m_n, z_p = P.MODUL, P.Z_RITZEL
    b_rp = min(P.PINION_B, P.RACK_W)            # 10 mm common face width
    d_p = P.PINION_D
    eps_rp = contact_ratio_rack(m_n, z_p)
    Y_eps = 0.25 + 0.75 / eps_rp
    Z_eps = math.sqrt((4 - eps_rp) / 3)
    YFS = {"pinion z = 20": 2.80 * 1.55, "rack": 2.06 * 2.03}

    def rp_check(Ft):
        out = {}
        sF0 = Ft / (b_rp * m_n) * Y_eps * KA * KV * KFb
        for _part, _mat in (("pinion z = 20", "16MnCr5 E"), ("rack", "C45 N")):
            _sF = sF0 * YFS[_part]
            _SF = MAT[_mat][0] * Y_ST / _sF
            out[_part] = (_sF, _SF)
        _sH = ZE * ZH * Z_eps * math.sqrt(Ft / (b_rp * d_p)) * math.sqrt(KA * KV * KHb)
        out["flank (rack C45 governs)"] = (_sH, MAT["C45 N"][1] / _sH)
        return out

    _rows = ""
    for _lbl, _F in (("regular", F_req), ("full current", F_max)):
        for _k, (_s, _S) in rp_check(_F).items():
            _lim = SH_min if "flank" in _k else SF_min
            _rows += f"| {_lbl} ({_F:.1f} N) | {_k} | {_s:.1f} MPa | {_S:.2f} | {'✓' if _S >= _lim else '✗'} |\n"
    mo.md(f"""
    Contact ratio pinion/rack $\\varepsilon_\\alpha$ = {eps_rp:.3f} → $Y_\\varepsilon$ = {Y_eps:.3f}, $Z_\\varepsilon$ = {Z_eps:.3f};
    face width b = {b_rp:.0f} mm.

    | Load case | Check | Stress | Safety | |
    |---|---|---|---|---|
    {_rows}

    Rack and pinion are far from their limits - the m = 1 catalogue parts are more than strong enough.
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 5. Bevel pair (Mädler m = 1, z = 20, i = 1, b = 5 mm, C45) - DIN 3991 via virtual spur gears

    The bevel pair sits **after** the gearbox, so it carries the full pinion torque
    $T = 2\,F_G\,r$ (i = 1). Mean diameter $d_m = d - b\sin\delta$, virtual spur gear
    $z_v = z/\cos\delta$, $d_v = d_m/\cos\delta$, $u_v = u^2 = 1$, bevel factor $Z_K$ = 0.85.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, KA, KFb, KHb, KV, MAT, P, SF_min, Y_ST, ZE, ZH, contact_ratio_pair, math, mo):
    z_b, b_b, d_b = P.BEVEL_Z, P.BEVEL_B, P.MODUL * P.BEVEL_Z
    delta = math.radians(45.0)
    d_m = d_b - b_b * math.sin(delta)
    m_m = P.MODUL * d_m / d_b
    z_v = z_b / math.cos(delta)
    d_v = d_m / math.cos(delta)
    eps_v = contact_ratio_pair(m_m, z_v, z_v)
    Ye_b = 0.25 + 0.75 / eps_v
    Ze_b = math.sqrt((4 - eps_v) / 3)
    YFS_b = 2.57 * 1.62                       # z_v ≈ 28, x = 0 [chart]
    Z_K = 0.85
    r_p = P.PINION_D / 2

    def bevel(F):
        T = 2 * F * r_p                       # N·mm at the pinion = at the bevel wheel
        Ft = 2 * T / d_m
        sF = Ft / (b_b * m_m) * YFS_b * Ye_b * KA * KV * KFb
        sH = ZE * ZH * Ze_b * Z_K * math.sqrt(Ft / (b_b * d_v) * 2) * math.sqrt(KA * KV * KHb)
        return T, Ft, sF, sH

    def bevel_forces(F):
        T, Ft, _, _ = bevel(F)
        al = math.radians(20.0)
        Fr = Ft * math.tan(al) * math.cos(delta)    # radial on the bevel wheel
        Fa = Ft * math.tan(al) * math.sin(delta)    # axial on the bevel wheel (= radial on the bevel pinion)
        return Ft, Fr, Fa

    # torque at which the flank reaches S_H = 1 (sigma_H ~ sqrt(T))
    _T1, _, _, _sH1 = bevel(F_max)
    T_allow_H = _T1 * (MAT["C45 N"][1] / _sH1) ** 2
    F_allow_H = T_allow_H / (2 * r_p)

    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _T, _Ft, _sF, _sH = bevel(_F)
        _SF = MAT["C45 N"][0] * Y_ST / _sF
        _SH = MAT["C45 N"][1] / _sH
        _rows += (f"| {_lbl} ({_F:.1f} N) | {_T/1000:.3f} Nm | {_Ft:.0f} N | {_sF:.0f} MPa / {_SF:.2f} {'✓' if _SF >= SF_min else '✗'} "
                  f"| {_sH:.0f} MPa / {_SH:.2f} {'✓' if _SH >= 1 else '✗'} |\n")
    mo.md(f"""
    $d_m$ = {d_m:.2f} mm, $m_m$ = {m_m:.3f} mm, $z_v$ = {z_v:.1f}, $\\varepsilon_{{\\alpha v}}$ = {eps_v:.2f}

    | Load case | Torque | $F_{{tm}}$ | root σF / S_F | flank σH / S_H |
    |---|---|---|---|---|
    {_rows}

    **⚠ The bevel flank is the weak point of the drive train.** With C45 (σHlim = 590 MPa) the flank reaches
    $S_H$ = 1 at about **{T_allow_H/1000:.2f} Nm**, i.e. **{F_allow_H:.0f} N per jaw**. The regular grip is right at that limit,
    full motor current is clearly above it. Since every grip loads the same two or three teeth, this is a
    fatigue problem, not a static one.

    Options (to decide together with Elias, E2):
    1. **Hardened bevel gears** (induction-hardened teeth or case-hardened steel; σHlim ≥ 1000 MPa) - simplest, no layout change.
    2. **Bevel stage before the gearbox** (on the motor shaft): torque 7× smaller, all checks pass by far - layout change.
    3. **Limit the motor current** so the grip force stays ≤ {F_allow_H:.0f} N - conflicts with the recommended setpoint {F_set:.1f} N.

    Cross-check: compare with the permissible torque printed in the Mädler catalogue for this article.
    """)
    return bevel_forces, r_p


@app.cell
def _(mo):
    mo.md(r"""
    ## 6. Tilting moment on the ball rail guide (Bosch Rexroth size 9, carriage SLS R0444 913 01)

    Each jaw sits on one slide with one carriage on the common rail (y = -32.5). The grip force acts at the
    pad centre (z = -77.5, y = 0) along the rail (X). It is reacted by the rack at the pinion mesh
    (z ≈ -6, rack pitch line at y = ±10). The carriage itself carries no X force - it carries the **couple**:

    - pitch moment $M_y = F_G \cdot (z_\mathrm{mesh} - z_\mathrm{pad})$,
    - yaw moment $M_z = F_G \cdot \Delta y_\mathrm{rack}$,
    - plus the share of the part load $F_z = m(g+a)/2$ at y = 0 → roll $M_x = F_z \cdot 32.5$ mm.

    Combined equivalent static load (Rexroth catalogue R999001206):
    $F_\mathrm{comb} = |F_y| + |F_z| + C_0\frac{|M_x|}{M_{t0}} + C_0\frac{|M_y|}{M_{L0}} + C_0\frac{|M_z|}{M_{L0}}$,
    static safety $S_0 = C_0 / F_\mathrm{comb}$.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, a_path, a_res, g, m, mo):
    z_pad = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1])         # -77.5
    z_mesh = P.PINION_Z0 + P.PINION_B / 2            # -6, middle of the pinion teeth
    dz_tilt = abs(z_mesh - z_pad)                    # 71.5 mm
    dy_rack = P.PINION_D / 2                          # 10 mm
    dy_rail = abs(P.RAIL_Y)                           # 32.5 mm
    C0, MT0, ML0 = P.CARRIAGE_C0, P.CARRIAGE_MT0 * 1000, P.CARRIAGE_ML0 * 1000   # N, N·mm

    def guide(F, a_vert):
        Fz = m * a_vert / 2
        My, Mz, Mx = F * dz_tilt, F * dy_rack, Fz * dy_rail
        Fc = abs(Fz) + C0 * Mx / MT0 + C0 * My / ML0 + C0 * Mz / ML0
        return My, Mz, Mx, Fc, C0 / Fc

    _rows = ""
    for _lbl, _F, _a in (("regular", F_req, g + a_path), ("setpoint", F_set, g + a_path),
                         ("full current + E-stop", F_max, a_res)):
        _My, _Mz, _Mx, _Fc, _S0 = guide(_F, _a)
        _rows += f"| {_lbl} | {_F:.1f} N | {_My/1000:.2f} Nm | {_Mz/1000:.2f} Nm | {_Mx/1000:.2f} Nm | {_Fc:.0f} N | **{_S0:.2f}** |\n"
    mo.md(f"""
    Lever arms: Δz = {dz_tilt:.1f} mm (pad centre to mesh), Δy = {dy_rack:.0f} mm, rail offset {dy_rail:.1f} mm.
    Elias' note used the carriage centre z ≈ -8 (Δz = 69.5 mm); the mesh is the correct reaction point, 3 % more conservative.

    | Load case | F_G | M_y (pitch) | M_z (yaw) | M_x (roll) | F_comb | S0 |
    |---|---|---|---|---|---|---|
    {_rows}

    The pitch moment from the 71.5 mm long jaw dominates (ML0 = 14 Nm). S0 is > 2.5 even in the worst case.
    **[verify]** the minimum S0 Rexroth recommends for operation with shocks in catalogue R999001206 -
    if it is above 2.6, the full-current case needs a current limit (or the long carriage version is enough).
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 7. Bearings 608-2Z of the pinion shaft (DIN ISO 76, static)

    **The pinion mesh is self-balancing:** the two racks sit on opposite sides, so their tangential and radial
    forces on the pinion cancel. The shaft bearings only see the **bevel wheel** forces at the top of the
    shaft (contact z ≈ 24, overhung above the upper bearing). Bearing centres z = 3.5 (A) and 13 (B).
    The shaft only swings ≈ 46° per grip, so the static rating is decisive.

    $P_0 = \max(F_r,\; 0.6F_r + 0.5F_a)$, $s_0 = C_0/P_0$, $C_0$ = 1370 N (SKF 608-2Z).
    """)
    return


@app.cell
def _(F_max, F_req, F_set, bevel_forces, math, mo):
    z_A, z_B, z_bev = 3.5, 13.0, 24.0
    C0_608, C_608 = 1370.0, 3450.0
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _Ft, _Fr, _Fa = bevel_forces(_F)
        _Frad = math.hypot(_Ft, _Fr)                       # radial force on the shaft at the bevel wheel
        _r_mean = 8.23                                     # mm, mean radius of the bevel wheel (d_m / 2)
        _M_ax = _Fa * _r_mean                              # moment from the axial force at the mean radius
        _FB = (_Frad * (z_bev - z_A) + _M_ax) / (z_B - z_A)
        _FA = _FB - _Frad
        _P0 = max(_FB, 0.6 * _FB + 0.5 * _Fa)
        _rows += f"| {_lbl} ({_F:.1f} N) | {_Frad:.0f} N | {_Fa:.0f} N | {_FB:.0f} N | {abs(_FA):.0f} N | {_P0:.0f} N | **{C0_608/_P0:.1f}** |\n"
    mo.md(f"""
    | Load case | radial at bevel | axial | bearing B | bearing A | P0 (B) | s0 |
    |---|---|---|---|---|---|---|
    {_rows}

    s0 > 3.5 even at full current - well above the s0 ≈ 1 ... 2 that bearing makers ask for under shock loads
    **[verify against the SKF table]**.
    Dynamic life (one sentence): with ≈ 0.26 equivalent revolutions per grip cycle and P ≈ 0.2 kN, L10 = (C/P)³ is
    in the order of 10⁹ revolutions - thousands of years at 720 grips per hour, so it is irrelevant.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, bevel_forces, math, mo):
    Fr_adm, Fa_adm = 305.0, 890.0                    # Nanotec GP42-S1-7-SR
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _Ft, _Fr, _Fa = bevel_forces(_F)
        # on the bevel PINION (gearbox shaft, horizontal) radial = Ft and the wheel's axial force; axial = wheel's radial force
        _rad = math.hypot(_Ft, _Fa)
        _ax = _Fr
        _rows += f"| {_lbl} | {_rad:.0f} N | {_rad/Fr_adm*100:.0f} % | {_ax:.0f} N | {_ax/Fa_adm*100:.1f} % |\n"
    mo.md(f"""
    ## 8. Gearbox output shaft load (GP42-S1-7-SR: 305 N radial / 890 N axial)

    | Load case | radial | of 305 N | axial | of 890 N |
    |---|---|---|---|---|
    {_rows}

    OK. Nanotec rates the radial load at a defined distance from the flange; the bevel pinion sits on the
    22.6 mm shaft stub - keep it close to the flange.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, mo, r_p):
    T_brake_pinion = P.BRAKE_T * P.GEAR_I * 1000           # N·mm, gearbox efficiency neglected (conservative)
    F_hold = T_brake_pinion / (2 * r_p)
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _rows += f"| {_lbl} | {_F:.1f} N | {2*_F*r_p/1000:.3f} Nm | **{T_brake_pinion/(2*_F*r_p):.2f}** |\n"
    mo.md(f"""
    ## 9. Holding on power failure

    Rack and pinion is back-drivable. The spring-applied brake BRAKE-BWA-0,35-5 (0.35 Nm) sits on the motor,
    so it acts through the gearbox: {P.BRAKE_T} Nm × {P.GEAR_I} = **{T_brake_pinion/1000:.2f} Nm at the pinion**
    → it can hold **{F_hold:.0f} N per jaw**. Gearbox efficiency is neglected; when the load back-drives the gearbox,
    friction helps the brake, so this is conservative.

    | Load case | grip force | torque to hold | brake safety |
    |---|---|---|---|
    {_rows}

    Spring-applied = fail-safe: no current, brake closed. Note for the report: between loss of current and
    brake engagement (a few ms) the elastic preload in pads and jaws could relax slightly; the stepper's
    detent torque bridges that gap.
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 10. Motor selection with margin (Nanotec ST4118S1404-B + GP42-S1-7-SR)

    Torque at the motor to produce the grip force, all stages in the force flow:

    $$T_M = \frac{2\,F_G\,r}{i\;\eta_G\,\eta_\mathrm{bevel}\,\eta_\mathrm{rp}}$$

    Closing stroke 8 mm per jaw in 0.3 s (triangular speed profile). Rotor inertia 38 g·cm² (Nanotec data sheet);
    the jaws, slides, racks and carriages (≈ 0.25 kg per side, BOM) are reflected through $i^2$.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, math, mo, np, plt, r_p):
    i_g, eta_g, eta_bev, eta_rp = P.GEAR_I, P.GEAR_ETA, 0.98, 0.97
    T_hold = P.MOTOR_T_HOLD * 1000                       # N·mm
    eta_tot = eta_g * eta_bev * eta_rp

    def T_motor(F):
        return 2 * F * r_p / (i_g * eta_tot)

    F_avail = T_hold * i_g * eta_tot / (2 * r_p)

    # closing move
    s_close, t_close = 8.0, 0.3
    phi_p = s_close / r_p                                # rad at the pinion
    phi_m = phi_p * i_g
    w_m = 2 * phi_m / t_close                            # peak motor speed, rad/s
    al_m = 4 * phi_m / t_close**2                        # motor acceleration, rad/s²
    m_side = (31 + 29 + 73 + 37 + 65 + 19) / 1000        # kg: jaw, holder, slide, carriage, rack, pad plate (BOM)
    J_rotor = 38e-7                                       # kg m²
    J_load = 2 * m_side * (r_p / 1000) ** 2 / i_g**2      # translating masses reflected to the motor
    J_gear = 0.5e-6                                       # kg m², gearbox + bevel gears, estimate
    T_acc = (J_rotor + J_load + J_gear) * al_m * 1000     # N·mm
    rpm = w_m * 60 / (2 * math.pi)

    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set)):
        _rows += f"| {_lbl} | {_F:.1f} N | {T_motor(_F)/1000:.3f} Nm | **{T_hold/T_motor(_F):.2f}** |\n"

    fig_m, _ax = plt.subplots(figsize=(7, 3))
    _t = np.linspace(0, t_close, 200)
    _w = np.where(_t < t_close / 2, al_m * _t, al_m * (t_close - _t)) * 60 / (2 * math.pi)
    _ax.plot(_t, _w, color="#0f766e", lw=2)
    _ax.set_xlabel("t [s]"); _ax.set_ylabel("motor speed [rpm]"); _ax.set_title("closing move, 8 mm in 0.3 s")
    _ax.grid(alpha=0.3); fig_m.tight_layout()

    mo.vstack([mo.md(f"""
    Efficiencies: gearbox {eta_g}, bevel {eta_bev}, rack/pinion {eta_rp} → {eta_tot:.3f}.

    | Load case | grip force | motor torque | margin vs. 0.20 Nm holding torque |
    |---|---|---|---|
    {_rows}

    - Max grip force with **all** efficiencies at full current: **{F_avail:.1f} N** (Elias' 63.6 N counts only the gearbox).
    - Closing move: pinion turns {math.degrees(phi_p):.0f}°, motor {math.degrees(phi_m):.0f}°, peak **{rpm:.0f} rpm**,
      acceleration {al_m:.0f} rad/s² → inertia torque **{T_acc:.2f} N·mm** - negligible. At ≈ 340 rpm the ST4118S
      still delivers ≈ 0.23 Nm (Nanotec torque curve, 24 V), so speed is no limit.
    - The grip itself happens at standstill, so the holding torque is the relevant limit: margin
      **{T_hold/T_motor(F_req):.2f}** at the regular force and **{T_hold/T_motor(F_set):.2f}** at the recommended setpoint.
      Run the stepper in closed-loop mode (encoder) so a short overload cannot cost steps.
    """), fig_m])
    return


@app.cell
def _(F_max, F_req, F_set, mo):
    mo.md(f"""
    ## Summary for the interim report

    | # | Check | Result |
    |---|---|---|
    | 1 | Grip force, µ = 0.2, S = 2 | {F_req:.1f} N per jaw ✓ (µ = 0.1 would need twice that - above the motor) |
    | 2 | Emergency stop (17 m/s², S = 1.5) | recommended setpoint **{F_set:.1f} N** ✓ |
    | 3 | Surface pressure | < 0.1 MPa ✓ not critical |
    | 4 | Rack and pinion, root + flank | ✓ large reserves |
    | 5 | **Bevel pair, flank** | **✗ at the limit (regular) / over (full current) → hardened bevels or bevel before the gearbox** |
    | 6 | Ball rail, tilting | S0 > 2.6 ✓ (verify min. S0) |
    | 7 | Bearings 608-2Z | s0 > 3.5 ✓ |
    | 8 | Gearbox shaft load | ≈ 50 % of 305 N ✓ |
    | 9 | Brake on power failure | holds ≈ 124 N per jaw ✓ |
    | 10 | Motor | margin ≈ 1.7 (regular) / 1.4 (setpoint) ✓, max ≈ 60 N with all efficiencies |

    Envelope for all part checks: {F_max:.1f} N (full motor current).
    """)
    return


if __name__ == "__main__":
    app.run()
