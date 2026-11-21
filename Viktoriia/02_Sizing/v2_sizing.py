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
    Status 21 Nov 2026, against the CAD state after the drive-chain review (bevel pair KHK SMSG1.5-20, commit `c0cc291`).

    **Changes 21 Nov:** § 5 now checks the built bevel pair (KHK SMSG1.5-20, hardened ground *spiral* teeth) instead of
    the Mädler C45 straight pair; §§ 7 and 8 use the spiral bevel forces (axial / radial force up to 3× the straight
    value, see my review response); § 6 text follows the CAD (rail size 12, T-shaped jaw); § 3 uses the wide grooved pad;
    the setpoint is the CAD value 44.5 N (real path 4.96 m/s²), which also covers the e-stop with S = 1.5.

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
    F_path = S * m * (g + P.A_PATH_REAL) / (n_faces * mu)   # real path, acceleration capped in the robot program
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
    | same with the real path acceleration {P.A_PATH_REAL:.2f} m/s² (ABB MoveJ path, capped with WorldAccLim, E4) | {F_path:.1f} N |
    """)
    return F_max, F_path, F_req, S, a_path, g, m, mu, n_faces


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

    **Stop deceleration.** ABB lists stopping distance and time for category 0 stops (max speed, arm stretched,
    max load) in the product specification. IRB 1300-7/1.4 (3HAC070393-001 rev. M, p. 91; same values as the
    CRB 1300-7/1.4 sheet): axis 1: 55.25° in 0.40 s, axis 2: 33.39° in 0.37 s, axis 3: 35.66° in 0.27 s. Assuming constant deceleration,
    $\alpha = 2\varphi / t^2$, times the lever arm to the TCP (1.4 m for axes 1 and 2, about 0.8 m for axis 3)
    gives 12 ... 17 m/s². **Design value 17 m/s² (≈ 1.7 g)**, conservative: 1.4 m is the full reach, in our cell the
    TCP is at most ≈ 1.0 m from axis 1 (12.3 m/s², the value used in the MuJoCo e-stop simulation, E4).
    Note: the emergency stop button of the controller is configured for **category 0** on delivery
    (3HAC070393, § 1.9.2, p. 71), so the uncontrolled stop with these decelerations is the right load case.

    The emergency stop is a rare, accidental load case, so it is checked with $S_\mathrm{stop}$ = 1 (must not slip)
    and 1.5 (recommended).
    """)
    return


@app.cell
def _(F_max, F_path, F_req, a_stop_s, g, m, math, mo, mu, n_faces):
    _stops = {"axis 1": (55.25, 0.40, 1.4), "axis 2": (33.39, 0.37, 1.4), "axis 3": (35.66, 0.27, 0.8)}
    _rows = ""
    for _k, (_phi, _t, _r) in _stops.items():
        _alpha = 2 * math.radians(_phi) / _t**2
        _rows += f"| {_k} | {_phi:.2f}° | {_t:.2f} s | {_alpha:.1f} rad/s² | {_r:.1f} m | {_alpha*_r:.1f} m/s² |\n"
    a_stop = a_stop_s.value
    a_res = math.hypot(g, a_stop)
    F_stop_S1 = 1.0 * m * a_res / (n_faces * mu)
    F_stop_S15 = 1.5 * m * a_res / (n_faces * mu)
    F_set = max(F_req, F_path, F_stop_S15)         # recommended grip-force setpoint (= P.F_GRIP_SET at µ 0.2, S 2)
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
    | **recommended grip-force setpoint** max(regular, real path, E-stop with 1.5) | **{F_set:.1f} N** ({F_set/F_max*100:.0f} % of the motor limit) |

    If the deceleration acts along the grip axis instead, one jaw takes it in compression (form fit):
    extra {m*a_stop:.1f} N on that jaw - inside the {F_max:.1f} N envelope used below.
    """)
    return F_set, a_res, a_stop


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Surface pressure pad / PA 6

    The pad plate (steel 3 mm + NBR 70 Shore A 3 mm) sits in the pocket of the T-shaped jaw; the NBR face is 0.5 mm
    smaller all round than the plate (V3). Vertical oil-drain grooves in the NBR take away part of the face.
    The contact area is the **pad** without the grooves, not the 6200 mm² side face of the part.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, mo):
    _w = P.PAD_W - 0.2 - 1.0                  # 80.8 mm: plate 0.1 mm clearance per side in the pocket, NBR 0.5 mm edge
    _h = P.PAD_Z[1] - P.PAD_Z[0] - 0.2 - 1.0  # 19.8 mm
    _gw, _gd, _gp = P.PAD_GROOVE
    A_pad = _w * _h * (1 - _gw / _gp)         # mm², grooves 1.5 mm wide every 4 mm carry nothing
    E_nbr = 5.0                               # MPa, NBR 70 Shore A (same value as cad/params.py)
    t_nbr = 3.0                               # mm
    sigma_pa6 = 70.0                          # MPa, PA 6 yield 60-80 MPa (Röchling / Ensinger data sheets)
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _p = _F / A_pad
        _rows += f"| {_lbl} | {_F:.1f} N | {_p:.3f} MPa | {sigma_pa6/_p:.0f} | {_p/E_nbr*t_nbr*1000:.0f} µm |\n"
    mo.md(f"""
    Pad area $A$ = {_w:.1f} × {_h:.1f} mm × (1 − {_gw}/{_gp}) = {A_pad:.0f} mm² (V3 pad, grooves taken off)

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
    ## 5. Bevel pair (KHK SMSG1.5-20 R/L, spiral, i = 1) - catalogue rating and DIN 3991 cross-check

    The bevel pair sits **after** the gearbox, so it carries the full pinion torque. On the driving side, before the
    bevel and rack losses: $T = 2\,F_G\,r / (\eta_\mathrm{bevel}\,\eta_\mathrm{rp})$.

    **Built pair** (KHK catalogue, SMSG module 1.5 page): m = 1.5, z = 20, d = 30 mm, b = 8 mm, Gleason spiral teeth,
    α = 20°, β = 35°, S45C teeth induction hardened, ground. Allowable torque **surface durability 3.47 N·m**,
    **bending 4.1 N·m**. Until 21 Nov the CAD had the SMSG1-20 (m = 1, 0.97 / 1.17 N·m); with $K_A$ = 1.25 the setpoint
    alone loaded its flank to 120 % of the catalogue value, so Elias changed to the next size (drive axis 36 mm,
    `Elias/02_CAD_Body/drive_margin.py`). The old Mädler C45 straight pair of the first V2 version is gone.

    **Check 1, catalogue:** $K_A\,T \le T_\mathrm{cat}$. The catalogue values are KHK's own calculation for their assumed
    usage conditions; as in my review response they are the binding limit.

    **Check 2, DIN 3991 via virtual helical gears** (same method as my review response, now with m = 1.5, b = 8):
    $d_m = d - b\sin\delta$, $m_{mn} = m\,(d_m/d)\cos\beta_m$, $d_v = d_m/\cos\delta$,
    $\tan\alpha_{vt} = \tan\alpha_n/\cos\beta_m$, overlap ratio $\varepsilon_{v\beta} > 1$ →
    $Z_\varepsilon = \sqrt{1/\varepsilon_{v\alpha}}$, $Y_\varepsilon$ = 0.625, $Z_\beta = \sqrt{\cos\beta_m}$, $Z_K$ = 0.85.
    Conservative DIN 3991 load factors: $b_e$ = 0.85 b, $K_{H\beta} = K_{F\beta}$ = 1.875 (one gear overhung).
    σHlim = 1000 MPa (lower end for 50 HRC), σFlim = 220 MPa (root taken as not hardened) **[verify: ISO 6336-5]**.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, KA, KV, P, SF_min, Y_ST, ZE, math, mo):
    m_e, z_b, b_b = P.BEVEL_M, P.BEVEL_Z, P.BEVEL_B
    T_cat_H, T_cat_F = P.BEVEL_T_ALLOW * 1000, P.BEVEL_T_BEND * 1000     # N·mm, KHK catalogue
    _eta_b, _eta_rp = P.ETA_BEVEL, P.ETA_RP
    delta, beta_m, alpha_n = math.radians(45.0), math.radians(35.0), math.radians(20.0)
    d_b = m_e * z_b
    d_m = d_b - b_b * math.sin(delta)
    m_mt = m_e * d_m / d_b
    m_mn = m_mt * math.cos(beta_m)
    d_v = d_m / math.cos(delta)
    z_v = z_b / math.cos(delta)
    alpha_vt = math.atan(math.tan(alpha_n) / math.cos(beta_m))
    beta_vb = math.asin(math.sin(beta_m) * math.cos(alpha_n))
    _r = d_v / 2
    _ra, _rb = _r + m_mn, _r * math.cos(alpha_vt)
    eps_va = (2 * math.sqrt(_ra**2 - _rb**2) - 2 * _r * math.sin(alpha_vt)) / (math.pi * m_mt * math.cos(alpha_vt))
    eps_vb = b_b * math.sin(beta_m) / (math.pi * m_mn)
    Z_Hb = math.sqrt(2 * math.cos(beta_vb) / (math.cos(alpha_vt)**2 * math.sin(alpha_vt)))
    Z_eps_b, Z_beta, Z_K = math.sqrt(1 / eps_va), math.sqrt(math.cos(beta_m)), 0.85
    Y_eps_b = 0.625
    Y_beta = max(1 - min(eps_vb, 1.0) * math.degrees(beta_m) / 120, 0.75)
    YFS_b = 2.35 * 1.68                       # z_vn ≈ 49, x = 0 [DIN 3990-3 chart, ± 3 %]
    KHb_b, b_e = 1.875, 0.85 * b_b
    sig_Hlim_b, sig_Flim_b = 1000.0, 220.0
    r_p = P.PINION_D / 2

    def bevel_torque(F):
        return 2 * F * r_p / (_eta_b * _eta_rp)          # N·mm, driving side

    def bevel(F):
        T = bevel_torque(F)
        Ft = 2 * T / d_m
        sH = Z_Hb * ZE * Z_eps_b * Z_beta * Z_K * math.sqrt(Ft / (d_v * b_e) * 2 * KA * KV * KHb_b)
        sF = Ft / (b_b * m_mn) * YFS_b * Y_eps_b * Y_beta * KA * KV * KHb_b
        return T, Ft, sH, sF

    def bevel_forces(F):
        """Spiral bevel forces at the mean diameter (KHK / Gleason), i = 1. The sign depends on hand and direction of
        rotation (closing / opening), so both sets are returned: (Ft, [(Fa, Fr) on the bevel wheel, ...])."""
        Ft = 2 * bevel_torque(F) / d_m
        k = Ft / math.cos(beta_m)
        ta, sb = math.tan(alpha_n), math.sin(beta_m)
        sets = [(k * (ta * math.sin(delta) + sb * math.cos(delta)), k * (ta * math.cos(delta) - sb * math.sin(delta))),
                (k * (ta * math.sin(delta) - sb * math.cos(delta)), k * (ta * math.cos(delta) + sb * math.sin(delta)))]
        return Ft, [(abs(a), abs(r)) for a, r in sets]

    # torque at which the flank reaches S_H = 1 and the root S_F = 1.5 (sigma_H ~ sqrt(T), sigma_F ~ T)
    _T1, _, _sH1, _sF1 = bevel(F_max)
    T_DIN_H = _T1 * (sig_Hlim_b / _sH1) ** 2
    T_DIN_F = _T1 * sig_Flim_b * Y_ST / SF_min / _sF1

    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _T, _Ft, _sH, _sF = bevel(_F)
        _uH, _uF = KA * _T / T_cat_H, KA * _T / T_cat_F
        _SH, _SF = sig_Hlim_b / _sH, sig_Flim_b * Y_ST / _sF
        _rows += (f"| {_lbl} ({_F:.1f} N) | {_T/1000:.3f} Nm | {_uH*100:.0f} % / {_uF*100:.0f} % {'✓' if max(_uH, _uF) <= 1 else '✗'} "
                  f"| {_Ft:.0f} N | {_sH:.0f} MPa / {_SH:.2f} {'✓' if _SH >= 1 else '✗'} "
                  f"| {_sF:.0f} MPa / {_SF:.2f} {'✓' if _SF >= SF_min else '✗'} |\n")
    mo.md(f"""
    $d_m$ = {d_m:.2f} mm, $m_{{mn}}$ = {m_mn:.3f} mm, $d_v$ = {d_v:.2f} mm, $z_v$ = {z_v:.1f},
    $\\varepsilon_{{v\\alpha}}$ = {eps_va:.2f}, $\\varepsilon_{{v\\beta}}$ = {eps_vb:.2f}

    | Load case | Torque T | $K_A T$ / catalogue (flank / root) | $F_{{mt}}$ | DIN flank σH / S_H | DIN root σF / S_F |
    |---|---|---|---|---|---|
    {_rows}

    DIN 3991 capacity: flank **{T_DIN_H/1000:.1f} N·m** at S_H = 1 ({T_DIN_H/T_cat_H:.1f} × catalogue),
    root {T_DIN_F/1000:.1f} N·m at S_F = 1.5. The catalogue value stays the binding limit.

    **The bevel pair is no longer the weak point.** Even full motor current (63.6 N, gearbox loss only) uses
    {KA*bevel_torque(F_max)/T_cat_H*100:.0f} % of the catalogue flank rating with $K_A$. The current limit is now set by the
    motor ({P.F_JAW_LIMIT:.1f} N per jaw with all losses), not by the bevels.

    Tolerances (motor $k_T$ ± 10 %, driver ± 3 %, efficiencies) are in `Elias/02_CAD_Body/drive_margin.py`: highest
    flank use 40 % with the current set from data sheet values; but then the *lowest* grip force can drop to 35.6 N,
    just under the 35.9 N needed (path with the part's twist, `grip_check.py`). **Therefore the grip force is set once
    at commissioning with a force gauge** (± 5 %): lowest 42.3 N ✓. Rare overloads (brake slipping, closing without the
    two-stage profile, 160 N impact) stay ≤ 38 % of the static capacity.
    """)
    return bevel_forces, d_m, r_p


@app.cell
def _(mo):
    mo.md(r"""
    ## 6. Tilting moment on the ball rail guide (Bosch Rexroth size 12, carriage SLS R0444 213 01)

    Each jaw sits on one slide under one carriage on the common rail (the carriages hang under the base plate).
    The grip force acts at the pad centre (y = 0) along the rail (X). It is reacted by the rack at the pinion mesh
    (middle of the pinion teeth, rack pitch line at y = ±10). The carriage itself carries no X force - it carries the
    **couple**:

    - pitch moment $M_y = F_G \cdot (z_\mathrm{mesh} - z_\mathrm{pad})$,
    - yaw moment $M_z = F_G \cdot \Delta y_\mathrm{rack}$,
    - plus the share of the part load $F_z = m(g+a)/2$ at y = 0 → roll $M_x = F_z \cdot |y_\mathrm{rail}|$,
    - in the e-stop the sideways deceleration in the pad plane $F_y = m\,a_\mathrm{stop}/2$ at the pad, $\Delta z_c$ below
      the carriage → adds $F_y$ and roll $F_y \cdot \Delta z_c$.

    Combined equivalent static load (Rexroth catalogue R999001206):
    $F_\mathrm{comb} = |F_y| + |F_z| + C_0\frac{|M_x|}{M_{t0}} + C_0\frac{|M_y|}{M_{L0}} + C_0\frac{|M_z|}{M_{L0}}$,
    static safety $S_0 = C_0 / F_\mathrm{comb}$. All lever arms are read from the CAD parameters.
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, a_path, a_stop, g, m, mo):
    z_pad = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1])          # pad centre
    z_mesh = P.PINION_Z0 + P.PINION_B / 2             # middle of the pinion teeth
    dz_tilt = abs(z_mesh - z_pad)
    dy_rack = P.PINION_D / 2
    dy_rail = abs(P.RAIL_Y)
    C0, MT0, ML0 = P.CARRIAGE_C0, P.CARRIAGE_MT0 * 1000, P.CARRIAGE_ML0 * 1000   # N, N·mm

    dz_carr = abs(P.SLIDE_Z_TOP - z_pad)              # pad centre to the carriage underside
    def guide(F, a_vert, a_side=0.0):
        """a_side: deceleration across the rail in the pad plane (e-stop), carried by the pads as Fy."""
        Fz, Fy = m * a_vert / 2, m * a_side / 2
        My, Mz, Mx = F * dz_tilt, F * dy_rack, Fz * dy_rail + Fy * dz_carr
        Fc = abs(Fy) + abs(Fz) + C0 * Mx / MT0 + C0 * My / ML0 + C0 * Mz / ML0
        return My, Mz, Mx, Fc, C0 / Fc

    _rows = ""
    S0_cases = {}
    for _lbl, _F, _a, _as in (("regular", F_req, g + a_path, 0.0), ("setpoint", F_set, g + a_path, 0.0),
                              ("setpoint +5 % (calibrated) + E-stop", 1.05 * F_set, g, a_stop),
                              ("full current + E-stop", F_max, g, a_stop)):
        _My, _Mz, _Mx, _Fc, _S0 = guide(_F, _a, _as)
        S0_cases[_lbl] = _S0
        _rows += f"| {_lbl} | {_F:.1f} N | {_My/1000:.2f} Nm | {_Mz/1000:.2f} Nm | {_Mx/1000:.2f} Nm | {_Fc:.0f} N | **{_S0:.2f}** |\n"
    mo.md(f"""
    Lever arms: pad centre z = {z_pad:.1f}, mesh z = {z_mesh:.1f} → Δz = {dz_tilt:.1f} mm; Δy = {dy_rack:.0f} mm;
    rail offset {dy_rail:.1f} mm; pad centre to carriage Δz_c = {dz_carr:.1f} mm.
    Carriage size 12: C0 = {C0:.0f} N, Mt0 = {MT0/1000:.1f} N·m, ML0 = {ML0/1000:.1f} N·m.
    The FEM (E5) uses the same support: rack held at the pitch line, carriage patch free along X.

    | Load case | F_G | M_y (pitch) | M_z (yaw) | M_x (roll) | F_comb | S0 |
    |---|---|---|---|---|---|---|
    {_rows}

    The pitch moment of the jaw still dominates, but the size 12 carriage (ML0 {ML0/1000:.1f} instead of 14 N·m) and the
    shorter T-shaped jaw give **S0 = {S0_cases['setpoint']:.1f} at the setpoint and {S0_cases['full current + E-stop']:.1f} in the
    worst case** (full current plus the sideways e-stop load). Which S0 is required: see § 6a.
    """)
    return C0, ML0, MT0, S0_cases, dy_rail, dz_carr, dz_tilt, dy_rack


@app.cell
def _(mo):
    mo.md(r"""
    ### 6a. Required S0 - the carriages hang under the plate

    Rexroth table (R999001206, p. 9, "Statischer Tragsicherheitsfaktor S0"):

    | Conditions of use | S0 |
    |---|---|
    | overhead hanging arrangements or applications with a high hazard potential | ≥ 12 |
    | high dynamic load at standstill, contamination | 8 - 12 |
    | normal design of machines, not all load parameters or connection accuracies fully known | 5 - 8 |
    | all load data fully known, running free of shocks | 3 - 5 |

    Our carriages hang under the base plate, so the weight of slide, jaw and part pulls them off the rail. By the
    wording, the first row applies, and **S0 = 5.7 ... 8.4 does not meet 12.** The row exists because in a hanging
    arrangement a damaged carriage lets the load fall. Two ways out:

    1. **Make sure nothing can fall** (catch), then design with the rows below. In our housing the slide hangs
       **2 mm above the bottom cover** and is wider than the slot the holder reaches through. If a carriage loses
       its balls, the slide drops 2 mm and rests on the cover (≈ 2800 ... 3300 mm² on solid cover, closed and open,
       checked in the CAD: `build.py`, "carriage lost"). The rack drops with it and stays ≈ 7 of its 10 mm in mesh
       with the pinion, so the jaw keeps its position and the gripper can still be opened. Nothing falls out of the
       gripper; the hazard behind the ≥ 12 row is gone.
    2. **A bigger carriage** that reaches 12 (table below) - the housing grows in Y and X.

    It also helps to look at where F_comb comes from: the part that really *hangs* on the carriage (slide, holder,
    jaw, half the part, ≈ 10 N) gives S0 ≈ 500 alone. S0 = 5.7 ... 8.4 comes from the grip-force moment, which is a
    force *inside* the gripper. Exceeding C0 means permanent dents in the raceway (DIN 636: 0.0001 × ball
    diameter), i.e. rougher running - not a carriage that comes off.
    """)
    return


@app.cell
def _(F_max, F_set, P, a_path, a_stop, dy_rack, dy_rail, dz_carr, dz_tilt, g, m, mo):
    _opts = {   # name: C0 [N], Mt0, ML0 [N·m], extra height [mm] (R999001206 p. 19 / 25)
        "SLS 12 (built)": (P.CARRIAGE_C0, P.CARRIAGE_MT0, P.CARRIAGE_ML0, 0.0),
        "BLS 12 (wide, long, 40 mm wide, 59.5 long)": (8250.0, 97.7, 54.6, 1.0),
        "SLS 15 (32 mm wide, 60.8 long, rail 15)": (10170.0, 75.3, 67.1, 3.0),
    }
    _cases = (("setpoint", F_set, g + a_path, 0.0), ("setpoint +5 % + E-stop", 1.05 * F_set, g, a_stop),
              ("full current + E-stop", F_max, g, a_stop))
    _rows = ""
    for _n, (_C0, _Mt0, _ML0, _dh) in _opts.items():
        _s = []
        for _lbl, _F, _a, _as in _cases:
            _Fz, _Fy = m * _a / 2, m * _as / 2
            _Fc = (_Fy + _Fz + _C0 * (_Fz * dy_rail + _Fy * (dz_carr + _dh)) / (_Mt0 * 1000)
                   + _C0 * _F * (dz_tilt + _dh) / (_ML0 * 1000) + _C0 * _F * dy_rack / (_ML0 * 1000))
            _s.append(_C0 / _Fc)
        _rows += f"| {_n} | " + " | ".join(f"{v:.1f} {'✓' if v >= 12 else ''}" for v in _s) + " |\n"
    mo.md(f"""
    | Carriage | S0 setpoint | S0 setpoint +5 % + E-stop | S0 full current + E-stop |
    |---|---|---|---|
    {_rows}

    Only the SLS 15 meets 12 in every case; the wide BLS 12 misses it in the double-fault case (current limit lost
    *and* e-stop at the same moment). Both need 12 ... 13 mm more carriage length per jaw and a wider housing.

    **Chosen (21 Nov): way 1 - SLS 12 with the cover as catch.** With the catch, the "normal design"
    row (5 - 8) applies, not "all load data known" (3 - 5), because of the e-stop shocks. The realistic worst case
    (calibrated setpoint at its upper end plus e-stop) gives S0 ≈ 7.5 ✓, the double fault 5.7 ✓ (≥ 5). The 2 mm gap
    between slide and cover becomes a design feature: it must stay ≤ the rack's mesh reserve and is checked in every
    CAD build.
    """)
    return



@app.cell
def _(mo):
    mo.md(r"""
    ## 7. Bearings 608-2Z of the pinion shaft (DIN ISO 76, static) - with the spiral bevel forces

    **The pinion mesh is self-balancing:** the two racks sit on opposite sides, so their tangential and radial
    forces on the pinion cancel. The shaft bearings only see the **bevel wheel** forces at the top of the
    shaft (mean contact $z = z_\mathrm{drive} - d_m/2$, overhung above the upper bearing). Bearing centres from the CAD.
    The shaft only swings ≈ 46° per grip, so the static rating is decisive.

    **Spiral bevel forces** (KHK / Gleason, mean diameter, i = 1, δ = 45°, β = 35°):

    $$F_a = \frac{F_t}{\cos\beta_m}\left(\tan\alpha_n\sin\delta \pm \sin\beta_m\cos\delta\right),\qquad
      F_r = \frac{F_t}{\cos\beta_m}\left(\tan\alpha_n\cos\delta \mp \sin\beta_m\sin\delta\right)$$

    The sign depends on the hand (R/L) and the direction of rotation (closing / opening), so both sets are checked
    and the worse one is shown. With straight teeth (first V2 version) both were only $F_t\tan\alpha\sin\delta$ ≈ 0.26 $F_t$;
    with the spiral teeth one of them is ≈ 0.81 $F_t$.

    $P_0 = \max(F_r,\; 0.6F_r + 0.5F_a)$, $s_0 = C_0/P_0$, $C_0$ = 1370 N (SKF 608-2Z). SKF asks **s0 ≥ 2** for ball
    bearings with uncertain peak loads and no permanent deformation allowed (my review response, point 3).
    """)
    return


@app.cell
def _(F_max, F_req, F_set, P, bevel_forces, d_m, math, mo):
    z_A = P.BEARING_LOWER_Z + P.BEARING[2] / 2          # bearing centres from the CAD
    z_B = P.BEARING_UPPER_Z + P.BEARING[2] / 2
    z_bev = P.DRIVE_Z - d_m / 2                         # mean contact of the bevel wheel (cone apex on the drive axis)
    C0_608, S0_REQ = 1370.0, 2.0

    def bearing_B(Ft, Fa, Fr):
        F_rad = math.hypot(Ft, Fr)                       # radial force on the shaft at the bevel wheel
        F_B = (F_rad * (z_bev - z_A) + Fa * d_m / 2) / (z_B - z_A)   # axial force at the mean radius adds a moment
        return F_rad, F_B, F_B - F_rad

    _rows = ""
    s0_608 = {}
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _Ft, _sets = bevel_forces(_F)
        _worst = None
        for _Fa, _Fr in _sets:
            _Frad, _FB, _FA = bearing_B(_Ft, _Fa, _Fr)
            _P0 = max(_FB, 0.6 * _FB + 0.5 * _Fa)
            if _worst is None or _P0 > _worst[4]:
                _worst = (_Frad, _Fa, _FB, _FA, _P0)
        _Frad, _Fa, _FB, _FA, _P0 = _worst
        s0_608[_lbl] = C0_608 / _P0
        _rows += (f"| {_lbl} ({_F:.1f} N) | {_Frad:.0f} N | {_Fa:.0f} N | {_FB:.0f} N | {abs(_FA):.0f} N | {_P0:.0f} N "
                  f"| **{C0_608/_P0:.1f}** {'✓' if C0_608/_P0 >= S0_REQ else '✗'} |\n")
    mo.md(f"""
    Bearing centres z = {z_A:.1f} (A) and {z_B:.1f} (B), bevel contact z = {z_bev:.1f}, mean radius {d_m/2:.2f} mm.

    | Load case | radial at bevel | axial | bearing B | bearing A | P0 (B) | s0 (req. ≥ {S0_REQ:.0f}) |
    |---|---|---|---|---|---|---|
    {_rows}

    s0 = {s0_608['setpoint']:.1f} at the setpoint and {s0_608['full current']:.1f} at full current ✓. The spiral forces cost about
    15 % of s0; the larger m = 1.5 bevel (mean diameter {d_m:.1f} instead of 16.5 mm) gives it back, because the same
    torque needs a smaller tooth force; the contact height stays about the same (z {z_bev:.1f} instead of 24.3).
    Dynamic life (one sentence): with ≈ 0.26 equivalent revolutions per grip cycle and P ≈ 0.2 kN, L10 = (C/P)³ is
    in the order of 10⁹ revolutions - thousands of years at 720 grips per hour, so it is irrelevant.
    """)
    return (s0_608,)


@app.cell
def _(F_max, F_req, F_set, bevel_forces, math, mo):
    Fr_adm, Fa_adm = 305.0, 890.0                    # Nanotec GP42-S1-7-SR
    _rows = ""
    for _lbl, _F in (("regular", F_req), ("setpoint", F_set), ("full current", F_max)):
        _Ft, _sets = bevel_forces(_F)
        # on the bevel PINION (gearbox shaft, horizontal): radial = Ft and the wheel's axial force; axial = wheel's radial force
        _rad = max(math.hypot(_Ft, _Fa) for _Fa, _Fr in _sets)
        _ax = max(_Fr for _Fa, _Fr in _sets)
        _rows += f"| {_lbl} | {_rad:.0f} N | {_rad/Fr_adm*100:.0f} % | {_ax:.0f} N | {_ax/Fa_adm*100:.1f} % |\n"
    mo.md(f"""
    ## 8. Gearbox output shaft load (GP42-S1-7-SR: 305 N radial / 890 N axial)

    Spiral bevel forces, worse of the two sign sets for each column.

    | Load case | radial | of 305 N | axial | of 890 N |
    |---|---|---|---|---|
    {_rows}

    OK. Nanotec rates the radial load at a defined distance from the flange; the bevel pinion sits on the
    22.6 mm shaft stub, with the shaft reaching deep into the pinion hub - close to the flange.
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

    Spring-applied = fail-safe: no current, brake closed - **but only if the preload is still there when it closes.**
    The detent torque (6 mN·m) holds just ≈ 1.9 N per jaw, the part needs 14.8 N per jaw against its own weight.
    MuJoCo (E4, `power_fail.py`): without current the elastic preload of pads, jaws and slides pushes the jaws open
    within ≈ 2 ms and the part drops, whatever the brake does afterwards. Remedy (D11): a 24 V buffer module
    (PULS UF20.241, ≥ 200 ms at 20 A) keeps the controller and motor powered; on undervoltage the controller closes
    the brake, waits its engagement time, then switches the motor off. The gripper supply is not cut by the e-stop.
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
    i_g, eta_g, eta_bev, eta_rp = P.GEAR_I, P.GEAR_ETA, P.ETA_BEVEL, P.ETA_RP
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
    m_side = (54 + 29 + 81 + 74 + 65 + 40 + 6) / 1000   # kg: jaw, holder, slide, carriage, rack, pad plate, NBR (BOM 21 Nov)
    J_rotor = 38e-7                                       # kg m²
    J_load = 2 * m_side * (r_p / 1000) ** 2 / i_g**2      # translating masses reflected to the motor
    J_gear = 1.5e-6 + 1.0e-6 + 0.5 * 0.055 * 0.012**2 / i_g**2   # kg m²: gearbox input, brake disc, m 1.5 bevels (drive.py)
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
      acceleration {al_m:.0f} rad/s² → inertia torque **{T_acc:.2f} N·mm** - negligible. Speed is no limit either:
      the pull-out torque cannot exceed the 0.20 Nm holding torque, and at 24 V it stays at ≈ 0.19 Nm up to ≈ 800 rpm
      (model from R, L and holding torque, `Elias/04_Simulation/drive.py`; Nanotec curve to be laid over).
    - Seen from the jaws, rotor + gearbox + brake act like **≈ 3.2 kg** (J·(i/r)², vs {2*m_side:.2f} kg of jaws): the
      current limit cannot stop that before the pads hit. MuJoCo (E4, `closing_time.py`): driving straight into the
      part gives a pad force peak of ≈ 67 N at 50 mm/s and ≈ 160 N at 110 mm/s; a **two-stage close** (fast to 1 mm
      before the part, then 10 mm/s) meets 0.3 s (266 ms) with 46 N, no overshoot - that is the recommended motion
      profile. The 160 N case is checked as a rare overload on the bevel pair in § 5 (38 % of the static capacity).
    - Current limit: {P.F_JAW_LIMIT:.1f} N per jaw with all losses, set by the motor (the bevels allow more, § 5).
      The setpoint is calibrated once at commissioning with a force gauge (§ 5).
    - The grip itself happens at standstill, so the holding torque is the relevant limit: margin
      **{T_hold/T_motor(F_req):.2f}** at the regular force and **{T_hold/T_motor(F_set):.2f}** at the recommended setpoint.
      Run the stepper in closed-loop mode (encoder) so a short overload cannot cost steps.
    """), fig_m])
    return


@app.cell
def _(F_max, F_req, F_set, P, S0_cases, mo, s0_608):
    mo.md(f"""
    ## Summary for the interim report

    | # | Check | Result |
    |---|---|---|
    | 1 | Grip force, µ = 0.2, S = 2 | {F_req:.1f} N per jaw ✓ (µ = 0.1 would need twice that - above the motor) |
    | 2 | Emergency stop (17 m/s², S = 1.5; IRB 1300 stop data) and real path ({P.A_PATH_REAL} m/s²) | recommended setpoint **{F_set:.1f} N** ✓, calibrated at commissioning |
    | 3 | Surface pressure | < 0.1 MPa ✓ not critical |
    | 4 | Rack and pinion, root + flank | ✓ large reserves |
    | 5 | Bevel pair KHK SMSG1.5-20 (spiral, hardened) | $K_A T$ ≤ 34 % (setpoint) / 48 % (full current) of the catalogue flank rating ✓ |
    | 6 | Ball rail size 12, tilting (hanging) | S0 = {S0_cases['setpoint']:.1f} (setpoint) / {S0_cases['full current + E-stop']:.1f} (full current + e-stop) ✓ ≥ 5, because the cover catches a failed slide 2 mm lower (otherwise Rexroth asks ≥ 12, § 6a) |
    | 7 | Bearings 608-2Z, spiral bevel forces | s0 = {s0_608['setpoint']:.1f} (setpoint) / {s0_608['full current']:.1f} (full current) ✓ (SKF ≥ 2) |
    | 8 | Gearbox shaft load, spiral bevel forces | ≤ 46 % of 305 N ✓ |
    | 9 | Brake on power failure | holds ≈ 124 N per jaw ✓ |
    | 10 | Motor | margin ≈ 1.7 (regular) / 1.4 (setpoint) ✓, max ≈ 60 N with all efficiencies = current limit; two-stage close against the impact |

    Envelope for all part checks: {F_max:.1f} N (full motor current).
    """)
    return


if __name__ == "__main__":
    app.run()
