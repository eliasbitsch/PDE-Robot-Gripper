import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")


@app.cell
def _():
    import math
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    return math, mo, np, plt


@app.cell
def _(mo):
    mo.md(r"""
    # Response to Elias' review – points 1 to 3
    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**


    *Viktoriia Ovdiienko, 20 Nov 2026 – answers to "What I still need from you" in Elias' review message.
    Open with `marimo edit review_elias_response.py`.*

    | # | Point | Result |
    |---|---|---|
    | 1 | Bevel gears KHK SMSG1-20, DIN 3991 vs catalogue 0.97 N·m | DIN 3991 gives ≈ 2 … 3 N·m for the flank, so the **catalogue value 0.97 N·m is the binding limit** (conservative). The current limit 48.5 N/jaw = 0.97 N·m stays. |
    | 2 | Deflection, shear area of the rims only | **Does not hold.** The rims carry ≈ 11 % of the shear force; "rims only" overshoots the FEM by +6 … +31 %. The remaining −5 % is a 3D effect at the pocket step, not shear. |
    | 3 | Bearings 608-2Z against the SKF table | **OK.** With the spiral bevel forces (new!) s0 = 4.0 at the current limit, 3.0 at 63.6 N; SKF asks for s0 ≥ 2. |
    | 4 | V5 print test | later – FEM reference values from `Elias/05_FEM/v4_fem_results.json` |

    **New finding while doing point 1:** the KHK gears are *spiral* bevel gears (β = 35°). Their axial and radial
    forces are up to **3× larger** than the straight-bevel forces used in V2, and their direction depends on hand and
    direction of rotation. Bearings and gearbox shaft are rechecked with these forces in section 3.
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## Common input

    Grip force cases from Elias' parameters. The bevel pair sits after the gearbox and carries the pinion torque
    $T = 2\,F_G\,r_p$ with $r_p$ = 10 mm (two racks on the pinion, i = 1).
    """)
    return


@app.cell
def _():
    F_CASES = {
        "required": 35.7,
        "setpoint": 44.5,
        "current limit": 48.5,
        "strength case": 63.6,
    }
    R_PINION = 10.0                      # mm
    def torque(F):                       # N·mm at the bevel pair
        return 2 * F * R_PINION
    return F_CASES, R_PINION, torque


@app.cell
def _(mo):
    mo.md(r"""
    ## 1. Bevel pair KHK SMSG1-20 R/L – DIN 3991 check

    **Catalogue data** (KHK, SMSG module 1 page): m = 1, z = 20, d = 20 mm, b = 5 mm, Gleason spiral teeth,
    α = 20°, β = 35°, i = 1, S45C teeth induction hardened 50 … 60 HRC, ground, JIS B 1704 grade 2.
    Allowable torque **bending 1.17 N·m**, **surface durability 0.97 N·m** – "calculated values according to the
    assumed usage conditions" (the conditions themselves are on the KHK technical page, not on the product page).

    **Method** (DIN 3991 / ISO 10300, same structure as V2 § 5): the bevel pair is replaced by virtual cylindrical
    gears at the mean cone distance. Because the teeth are spiral, the virtual gears are **helical**:

    - $d_m = d - b\sin\delta$, $m_{mt} = m\,d_m/d$, $m_{mn} = m_{mt}\cos\beta_m$
    - $d_v = d_m/\cos\delta$, $z_v = z/\cos\delta$, $u_v = u^2 = 1$
    - $\tan\alpha_{vt} = \tan\alpha_n/\cos\beta_m$, $\sin\beta_{vb} = \sin\beta_m\cos\alpha_n$
    - overlap ratio $\varepsilon_{v\beta} = b\sin\beta_m / (\pi m_{mn})$ – > 1, so $Z_\varepsilon = \sqrt{1/\varepsilon_{v\alpha}}$, $Y_\varepsilon$ = 0.625
    - $Z_\beta = \sqrt{\cos\beta_m}$, $Y_\beta = 1 - \varepsilon_\beta\,\beta/120° \ge 0.75$, $Z_K$ = 0.85 (as V2)

    $$\sigma_H = Z_H Z_E Z_\varepsilon Z_\beta Z_K \sqrt{\frac{F_{mt}}{d_{v}\,b_{e}}\,\frac{u_v+1}{u_v}\,K_A K_V K_{H\beta}}
    \qquad
    \sigma_F = \frac{F_{mt}}{b\,m_{mn}}\,Y_{Fa}Y_{Sa}\,Y_\varepsilon Y_\beta\,K_A K_V K_{F\beta}$$

    Two sets of load factors: **V2 set** (K_A = 1.25, K_Hβ = K_Fβ = 1.5, full b – same as the C45 check, so the
    numbers are directly comparable) and **conservative DIN 3991 set** ($b_e$ = 0.85 b and
    $K_{H\beta} = 1.5\,K_{H\beta-be}$ with $K_{H\beta-be}$ = 1.25 for one overhung gear → 1.875).
    """)
    return


@app.cell
def _(math):
    # geometry
    m_e, z_g, b_g = 1.0, 20, 5.0
    delta = math.radians(45.0)
    beta_m = math.radians(35.0)
    alpha_n = math.radians(20.0)
    d_e = m_e * z_g
    d_m = d_e - b_g * math.sin(delta)
    m_mt = m_e * d_m / d_e
    m_mn = m_mt * math.cos(beta_m)
    d_v = d_m / math.cos(delta)
    z_v = z_g / math.cos(delta)
    alpha_vt = math.atan(math.tan(alpha_n) / math.cos(beta_m))
    beta_vb = math.asin(math.sin(beta_m) * math.cos(alpha_n))
    _r = d_v / 2
    _ra, _rb = _r + m_mn, _r * math.cos(alpha_vt)
    eps_va = (2 * math.sqrt(_ra ** 2 - _rb ** 2) - 2 * _r * math.sin(alpha_vt)) / (math.pi * m_mt * math.cos(alpha_vt))
    eps_vb = b_g * math.sin(beta_m) / (math.pi * m_mn)
    z_vn = z_v / (math.cos(beta_vb) ** 2 * math.cos(beta_m))

    # factors
    Z_H = math.sqrt(2 * math.cos(beta_vb) / (math.cos(alpha_vt) ** 2 * math.sin(alpha_vt)))
    Z_E = 189.8
    Z_eps = math.sqrt(1 / eps_va)
    Z_beta = math.sqrt(math.cos(beta_m))
    Z_K = 0.85
    Y_eps = 0.625
    Y_beta = max(1 - min(eps_vb, 1.0) * math.degrees(beta_m) / 120, 0.75)
    Y_FS = 2.35 * 1.68                    # Y_Fa * Y_Sa for z_vn ≈ 49, x = 0 [DIN 3990-3 chart, ± 3 %]

    # material S45C (≈ C45) induction hardened 50 ... 60 HRC  [verify: ISO 6336-5 / Roloff-Matek TB 20-1]
    SIG_HLIM = 1000.0                     # MPa, lower end for 50 HRC
    SIG_FLIM = 220.0                      # MPa, conservative: root assumed NOT hardened
    Y_ST, SF_MIN = 2.0, 1.5
    T_CAT_H, T_CAT_F = 970.0, 1170.0      # N·mm, KHK catalogue

    FACTOR_SETS = {
        "V2 set (K_A 1.25, K_Hβ 1.5, b)": dict(KA=1.25, KHb=1.5, KFb=1.5, be=b_g),
        "conservative DIN 3991 (K_Hβ 1.875, 0.85 b)": dict(KA=1.25, KHb=1.875, KFb=1.875, be=0.85 * b_g),
    }

    def bevel_stresses(T, fs):
        Ft = 2 * T / d_m
        sH = Z_H * Z_E * Z_eps * Z_beta * Z_K * math.sqrt(Ft / (d_v * fs["be"]) * 2 * fs["KA"] * fs["KHb"])
        sF = Ft / (b_g * m_mn) * Y_FS * Y_eps * Y_beta * fs["KA"] * fs["KFb"]
        return Ft, sH, sF

    def torque_limits(fs):
        _, sH1, sF1 = bevel_stresses(1000.0, fs)
        T_H = 1000.0 * (SIG_HLIM / sH1) ** 2              # S_H = 1
        T_F = 1000.0 * SIG_FLIM * Y_ST / SF_MIN / sF1      # S_F = 1.5
        return T_H, T_F

    return (FACTOR_SETS, SF_MIN, SIG_FLIM, SIG_HLIM, T_CAT_F, T_CAT_H, Y_ST, alpha_n, beta_m,
            bevel_stresses, d_m, d_v, delta, eps_va, eps_vb, m_mn, m_mt, torque_limits, z_v, z_vn)


@app.cell
def _(F_CASES, FACTOR_SETS, SF_MIN, SIG_FLIM, SIG_HLIM, T_CAT_F, T_CAT_H, Y_ST, bevel_stresses,
      d_m, d_v, eps_va, eps_vb, m_mn, m_mt, mo, torque, torque_limits, z_v, z_vn):
    _rows = ""
    for _fsn, _fs in FACTOR_SETS.items():
        for _lbl, _F in F_CASES.items():
            _T = torque(_F)
            _Ft, _sH, _sF = bevel_stresses(_T, _fs)
            _SH, _SF = SIG_HLIM / _sH, SIG_FLIM * Y_ST / _sF
            _cat = "✓" if _T <= T_CAT_H + 1e-6 else "✗ above"
            _rows += (f"| {_fsn} | {_lbl} ({_F} N) | {_T/1000:.3f} | {_cat} | {_Ft:.0f} | {_sH:.0f} / **{_SH:.2f}** "
                      f"| {_sF:.0f} / **{_SF:.2f}** |\n")
    _lim = ""
    for _fsn, _fs in FACTOR_SETS.items():
        _TH, _TF = torque_limits(_fs)
        _lim += (f"| {_fsn} | **{_TH/1000:.2f}** | {_TH/T_CAT_H:.1f} × | {_TF/1000:.2f} | {_TF/T_CAT_F:.1f} × |\n")
    mo.md(f"""
    Virtual gear: $d_m$ = {d_m:.2f} mm, $m_{{mt}}$ = {m_mt:.3f}, $m_{{mn}}$ = {m_mn:.3f}, $d_v$ = {d_v:.2f} mm,
    $z_v$ = {z_v:.1f}, $z_{{vn}}$ = {z_vn:.1f}, $\\varepsilon_{{v\\alpha}}$ = {eps_va:.2f}, $\\varepsilon_{{v\\beta}}$ = {eps_vb:.2f}.
    Limits: σHlim = {SIG_HLIM:.0f} MPa, σFlim = {SIG_FLIM:.0f} MPa (Y_ST = {Y_ST:.0f}, S_F,min = {SF_MIN}).

    | Factor set | Load case | T [N·m] | ≤ catalogue 0.97 | $F_{{mt}}$ [N] | flank σH [MPa] / S_H | root σF [MPa] / S_F |
    |---|---|---|---|---|---|---|
    {_rows}

    **Torque capacity, DIN 3991 vs KHK catalogue**

    | Factor set | flank T at S_H = 1 [N·m] | vs 0.97 | root T at S_F = 1.5 [N·m] | vs 1.17 |
    |---|---|---|---|---|
    {_lim}
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    **Reading – point 1**

    - With the hardened ground spiral gears the flank is no longer the weak point: DIN 3991 gives a flank capacity
      of about **2 … 3 N·m** (depending on the load factors), i.e. 2 to 3 times the catalogue value. The old C45
      straight pair reached S_H = 1 at 0.70 N·m; the gain comes from the material (σHlim 590 → ≥ 1000 MPa,
      factor ≈ 2.9 on torque) and from the spiral teeth (overlap ratio > 1, Z_β; factor ≈ 1.5).
    - **The catalogue value 0.97 N·m is lower than our DIN result.** KHK states it is calculated for its own assumed
      usage conditions (speed, life, lubrication, safety – see the KHK technical data page). Since we cannot check
      those conditions, **we keep 0.97 N·m as the design limit**. Our DIN check shows it is on the safe side:
      at 0.97 N·m S_H ≥ 1.4 even with the conservative factor set.
    - The **current limit 48.5 N per jaw** gives exactly T = 0.97 N·m → it fits the catalogue limit.
      The strength case 63.6 N (1.27 N·m) is above the catalogue value but still S_H > 1 in DIN; it can only occur
      if the current limit fails.
    - The root is not critical: S_F ≥ 2.9 at the current limit and ≥ 2.2 at 63.6 N (required 1.5), even with
      σFlim = 220 MPa (root assumed not hardened).

    **[verify]** σHlim for induction-hardened C45/S45C (I used 1000 MPa as the lower end for 50 HRC) and whether
    KHK hardens the root (contour hardening) – both only make the result safer.
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 2. Jaw deflection – shear area of the rims only

    Same beam model as V4 (section 25 × 8, pocket 23 × 3 over s = 13 … 48 mm, Ø3.4 holes and Ø6.5 × 1
    counterbores included, load uniform over the pad, resultant at s = 30.5 mm, F = 63.6 N = FEM load case).
    Four shear-area variants in the pocket zone:

    - **A** full section, κ = 5/6 (current V4) · **B** exact κ of the U-section from the shear stress distribution
    - **C** rims only, 2 × 1 × 8 mm · **D** rims only, 2 × 1 × 3 mm

    FEM references: **12.7 µm** (section fully clamped at z = −47, the beam's own boundary condition → compare
    with bending + shear) and **15.4 µm** (FEM rigid clamp → compare with beam + Weber).
    """)
    return


@app.cell
def _():
    E_AL, NU_AL = 71700.0, 0.33
    G_AL = E_AL / (2 * (1 + NU_AL))
    B_J, H_J = 25.0, 8.0
    POCKET_W, POCKET_D = 23.0, 3.0
    RIM = (B_J - POCKET_W) / 2
    S_A, S_B, S_END = 13.0, 48.0, 51.0
    S_F = 0.5 * (S_A + S_B)
    HOLES = [(20.5, 3.4), (40.5, 3.4)]
    CB_D, CB_T = 6.5, 1.0
    F_FEM = 63.6
    FEM_FULL_CLAMP, FEM_RIGID = 12.7, 15.4    # µm, Elias 20 Nov 2026 (v4_fem_results.json)
    return (B_J, CB_D, CB_T, E_AL, FEM_FULL_CLAMP, FEM_RIGID, F_FEM, G_AL, HOLES, H_J, NU_AL,
            POCKET_D, RIM, S_A, S_B, S_END, S_F)


@app.cell
def _(B_J, CB_D, CB_T, HOLES, H_J, POCKET_D, RIM, S_A, S_B, math, np):
    def _chord(s, sc, d):
        u = s - sc
        return 2 * math.sqrt((d / 2) ** 2 - u ** 2) if abs(u) < d / 2 else 0.0

    def strips(s):
        """Section at s as rectangles (width, x0, x1); x = 0 inner face (towards the part)."""
        if not (S_A <= s <= S_B):
            return [(B_J, 0.0, H_J)]
        hb = sum(_chord(s, c, d) for c, d in HOLES)
        cb = sum(_chord(s, c, CB_D) for c, _ in HOLES)
        return [(2 * RIM, 0.0, POCKET_D), (B_J - hb, POCKET_D, H_J - CB_T), (B_J - max(hb, cb), H_J - CB_T, H_J)]

    def props(st):
        A = sum(b * (x1 - x0) for b, x0, x1 in st)
        xc = sum(b * (x1 - x0) * (x0 + x1) / 2 for b, x0, x1 in st) / A
        I = sum(b * (x1 - x0) ** 3 / 12 + b * (x1 - x0) * ((x0 + x1) / 2 - xc) ** 2 for b, x0, x1 in st)
        return A, I, xc

    def shear_profile(st, n=800):
        """Zhuravskii q(x) = V Q(x)/I for V = 1 N, and exact kappa = I^2 / (A * int Q^2/b dx)."""
        A, I, xc = props(st)
        x = np.linspace(0, H_J, n + 1)
        xm = 0.5 * (x[1:] + x[:-1])
        dx = H_J / n
        b = np.array([next(bb for bb, x0, x1 in st if x0 <= xi <= x1) for xi in xm])
        Q = -np.cumsum(b * (xm - xc) * dx)
        tau = Q / (I * b)
        kappa = I ** 2 / (A * np.sum(Q ** 2 / b * dx))
        return xm, b, tau, kappa

    return props, shear_profile, strips


@app.cell
def _(POCKET_D, RIM, mo, np, plt, props, shear_profile, strips):
    _st = strips(30.0)
    _xm, _b, _tau, _kappa = shear_profile(_st)
    _A, _I, _ = props(_st)
    _dx = _xm[1] - _xm[0]
    _share_rims = float(np.sum((_tau * _b * _dx)[_xm < POCKET_D]))
    _share_strip = float(np.sum(_tau * 2 * RIM * _dx))
    fig_tau, _ax = plt.subplots(figsize=(6.5, 3))
    _ax.plot(_xm, _tau * _b, color="#0f766e", lw=2)
    _ax.axvspan(0, POCKET_D, color="#C2410C", alpha=0.10, label="pocket: rims only")
    _ax.set_xlabel("x through the thickness [mm] (0 = inner face)")
    _ax.set_ylabel("shear flow q = τ·b [1/mm]")
    _ax.set_title("Where the shear force goes (pocket section, V = 1 N)")
    _ax.grid(alpha=0.3); _ax.legend(frameon=False, fontsize=8)
    fig_tau.tight_layout()
    mo.vstack([
        mo.md(f"""
    **Shear distribution in the pocket section.** The load acts through the thickness. The shear flow
    q(x) = V·Q(x)/I is largest at the neutral axis, which lies in the solid 25 mm wide web.

    - share of the shear force in the rim zone x = 0 … 3: **{_share_rims*100:.1f} %**
    - share in a 1 mm strip at each edge over the full height (2 × 1 × 8): **{_share_strip*100:.1f} %**
    - exact κ of the section = **{_kappa:.3f}** (rectangle 0.833) → κ·A = {_kappa*_A:.0f} mm² (rims only: 6 or 16 mm²)

    A thin-walled section (I-beam, channel) carries shear in its webs because the flanges are thin and
    perpendicular to the load. In the jaw the "flange" is a 5 mm solid web loaded through its thickness, and it
    carries almost all of the shear.
    """),
        fig_tau,
    ])
    return


@app.cell
def _(B_J, E_AL, G_AL, H_J, NU_AL, POCKET_D, RIM, S_A, S_B, S_END, S_F, math, np, props,
      shear_profile, strips):
    s_grid = np.linspace(0, S_END, 2041)

    def _cum(y):
        return np.concatenate([[0.0], np.cumsum((y[1:] + y[:-1]) / 2 * np.diff(s_grid))])

    _secs = [strips(s) for s in s_grid]
    _AI = [props(st) for st in _secs]
    _A = np.array([a for a, _, _ in _AI])
    _I = np.array([i for _, i, _ in _AI])
    _pocket = (s_grid >= S_A) & (s_grid <= S_B)
    _kap = np.array([shear_profile(st, 200)[3] if p else 5 / 6 for st, p in zip(_secs, _pocket)])

    SHEAR_VARIANTS = {
        "A  full section, κ = 5/6 (V4)": 5 / 6 * _A,
        "B  exact κ of the U-section": _kap * _A,
        "C  rims only, 2 × 1 × 8 mm": np.where(_pocket, 2 * RIM * H_J, 5 / 6 * _A),
        "D  rims only, 2 × 1 × 3 mm": np.where(_pocket, 2 * RIM * POCKET_D, 5 / 6 * _A),
    }

    def jaw_deflection(F, As_eff):
        V = np.where(s_grid < S_A, F, np.where(s_grid <= S_B, F * (S_B - s_grid) / (S_B - S_A), 0.0))
        M = np.where(s_grid < S_A, F * (S_F - s_grid),
                     np.where(s_grid <= S_B, F * (S_B - s_grid) ** 2 / (2 * (S_B - S_A)), 0.0))
        w_b = _cum(_cum(M / (E_AL * _I)))
        w_s = _cum(V / (G_AL * As_eff))
        a_h = S_F / H_J
        w_c = F / (B_J * E_AL) * (16.67 / math.pi * a_h ** 2 + 2 * (1 - NU_AL) / math.pi * a_h + 1.534)
        at = lambda w: float(np.interp(S_F, s_grid, w)) * 1000
        return at(w_b), at(w_s), w_c * 1000

    return SHEAR_VARIANTS, jaw_deflection


@app.cell
def _(FEM_FULL_CLAMP, FEM_RIGID, F_FEM, SHEAR_VARIANTS, jaw_deflection, mo):
    def _dev(a, f):
        return (a - f) / f * 100

    def _ok(d):
        return "✓" if abs(d) <= 5 else "✗"

    _rows = ""
    defl_results = {}
    for _name, _As in SHEAR_VARIANTS.items():
        _wb, _ws, _wc = jaw_deflection(F_FEM, _As)
        _r, _t = _wb + _ws, _wb + _ws + _wc
        defl_results[_name] = (_wb, _ws, _wc, _r, _t)
        _rows += (f"| {_name} | {_wb:.2f} | {_ws:.2f} | {_r:.2f} | {_dev(_r, FEM_FULL_CLAMP):+.1f} % {_ok(_dev(_r, FEM_FULL_CLAMP))} "
                  f"| {_t:.2f} | {_dev(_t, FEM_RIGID):+.1f} % {_ok(_dev(_t, FEM_RIGID))} |\n")
    _A0 = defl_results[next(iter(defl_results))]
    mo.md(f"""
    Deflection at the load centre (s = 30.5 mm), F = {F_FEM} N, in µm. Weber term c) = {_A0[2]:.2f} µm for all variants.

    | Shear area | a) bending | b) shear | a+b | vs FEM full clamp {FEM_FULL_CLAMP} | a+b+c | vs FEM rigid clamp {FEM_RIGID} |
    |---|---|---|---|---|---|---|
    {_rows}

    **Reading – point 2**

    - Variant A reproduces the V4 value ({_A0[4]:.1f} µm) – same model as Elias' review notebook.
    - The exact κ of the U-section (B) changes the shear term by only a few hundredths of a µm. The whole shear
      term is ≈ {_A0[1]:.1f} µm; to close the {FEM_RIGID - _A0[4]:.1f} µm gap the effective shear area would have
      to be ≈ 2.5 … 3 times smaller than the shear stress distribution allows.
    - **Rims only overshoots:** 2 × 1 × 8 gives ≈ +6 % against both FEM values (opposite sign to now),
      2 × 1 × 3 gives ≈ +30 %.
    - The beam with rigid clamp is already too stiff against the fully clamped FEM (−7 %), so the missing part sits
      in the beam itself, not in the Weber term.

    **Probable cause of the remaining −5 … −7 %:** 3D effects that beam theory does not contain – the step in
    section at the pocket edge (s = 13 mm, 25 × 8 → U-section), where plane sections do not stay plane, and the local
    flexibility around the Ø3.4 holes and counterbores.
    **Recommendation:** keep the V4 shear term (κ = 5/6, full section) and report −5.3 % as a model limit.
    **Quick check for Elias:** a FEM run without the pocket (solid 25 × 8) should match the beam within 1 … 2 %;
    if it does, the gap comes from the pocket step.
    """)
    return (defl_results,)


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Bearings 608-2Z of the pinion shaft – SKF check, now with spiral bevel forces

    **Required s0 (SKF guideline values, ball bearings):** the table distinguishes the *certainty of the load
    level* and whether *permanent deformation* is acceptable. Our loads are peak loads (current limit, e-stop) →
    low certainty; the shaft oscillates ≈ 46° per grip and should keep running smoothly → no permanent deformation.
    SKF gives **s0 ≥ 2** for ball bearings in that case (high certainty would allow 0.5 … 2).

    **Forces of a spiral bevel gear** (KHK / Gleason formulas, mean diameter, i = 1, δ = 45°):

    $$F_a = \frac{F_t}{\cos\beta_m}\left(\tan\alpha_n\sin\delta \pm \sin\beta_m\cos\delta\right),\qquad
      F_r = \frac{F_t}{\cos\beta_m}\left(\tan\alpha_n\cos\delta \mp \sin\beta_m\sin\delta\right)$$

    The sign depends on the hand (R/L) and the direction of rotation, i.e. closing vs opening. Both combinations are
    checked; the larger one governs. For comparison the straight-bevel forces from V2:
    $F_a = F_r = F_t\tan\alpha\sin\delta$.

    Bearing model unchanged from V2: bevel contact z = 24, bearing centres z = 3.5 (A) and 13 (B), axial force
    acts at the mean radius $d_m/2$, $P_0 = \max(F_r,\,0.6F_r + 0.5F_a)$, $C_0$ = 1370 N (SKF 608-2Z).
    """)
    return


@app.cell
def _(F_CASES, alpha_n, beta_m, d_m, delta, math, mo, torque):
    C0_608 = 1370.0
    S0_REQ = 2.0
    Z_A, Z_B, Z_BEV = 3.5, 13.0, 24.0
    FR_ADM_GB, FA_ADM_GB = 305.0, 890.0           # Nanotec GP42-S1-7-SR output shaft

    def spiral_forces(T):
        Ft = 2 * T / d_m
        k = Ft / math.cos(beta_m)
        f1 = (k * (math.tan(alpha_n) * math.sin(delta) + math.sin(beta_m) * math.cos(delta)),
              k * (math.tan(alpha_n) * math.cos(delta) - math.sin(beta_m) * math.sin(delta)))
        f2 = (k * (math.tan(alpha_n) * math.sin(delta) - math.sin(beta_m) * math.cos(delta)),
              k * (math.tan(alpha_n) * math.cos(delta) + math.sin(beta_m) * math.sin(delta)))
        straight = Ft * math.tan(alpha_n) * math.sin(delta)
        return Ft, {"spiral, direction 1": (abs(f1[0]), abs(f1[1])),
                    "spiral, direction 2": (abs(f2[0]), abs(f2[1])),
                    "straight (V2)": (straight, straight)}

    def bearing_s0(Ft, Fa, Fr):
        F_rad = math.hypot(Ft, Fr)
        F_B = (F_rad * (Z_BEV - Z_A) + Fa * d_m / 2) / (Z_B - Z_A)
        P0 = max(F_B, 0.6 * F_B + 0.5 * Fa)
        return F_B, P0, C0_608 / P0

    _rows, _gb = "", ""
    s0_min = {}
    for _lbl, _F in F_CASES.items():
        _Ft, _sets = spiral_forces(torque(_F))
        for _k, (_Fa, _Fr) in _sets.items():
            _FB, _P0, _s0 = bearing_s0(_Ft, _Fa, _Fr)
            if _k != "straight (V2)":
                s0_min[_lbl] = min(s0_min.get(_lbl, 99), _s0)
            _rows += (f"| {_lbl} ({_F} N) | {_k} | {_Ft:.0f} | {_Fa:.0f} | {_Fr:.0f} | {_FB:.0f} | {_P0:.0f} "
                      f"| **{_s0:.2f}** {'✓' if _s0 >= S0_REQ else '✗'} |\n")
            # gearbox output shaft (bevel pinion): radial = hypot(Ft, wheel axial), axial = wheel radial
            _rad = math.hypot(_Ft, _Fa)
            _gb += (f"| {_lbl} | {_k} | {_rad:.0f} N ({_rad/FR_ADM_GB*100:.0f} %) | {_Fr:.0f} N ({_Fr/FA_ADM_GB*100:.0f} %) |\n")
    mo.md(f"""
    | Load case | Force set | $F_t$ [N] | $F_a$ [N] | $F_r$ [N] | bearing B [N] | $P_0$ [N] | s0 (req. ≥ {S0_REQ:.0f}) |
    |---|---|---|---|---|---|---|---|
    {_rows}

    **Side effect – gearbox output shaft** (GP42-S1-7-SR, 305 N radial / 890 N axial), V2 § 8 with the spiral forces:

    | Load case | Force set | radial (of 305 N) | axial (of 890 N) |
    |---|---|---|---|
    {_gb}

    **Reading – point 3**

    - The straight-bevel rows reproduce V2 (s0 = 6.4 / 3.6), so the model is unchanged.
    - The spiral teeth raise the axial or radial force from ≈ 0.26·F_t to ≈ 0.81·F_t. s0 drops by ≈ 15 %:
      **s0 = {s0_min['current limit']:.1f} at the current limit**, {s0_min['strength case']:.1f} in the 63.6 N strength case.
      Both are above the SKF requirement s0 ≥ 2 → **OK, the [verify] point is closed.**
    - Gearbox shaft: max. ≈ 200 N radial at 63.6 N (65 % of 305 N), ≈ 150 N at the current limit → OK.
      Keep the bevel pinion close to the flange, as noted in V2.
    - Dynamic life stays irrelevant (≈ 0.26 revolutions per grip).
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. V5 print test (later)

    The FEM reference values for the printed jaw will be read from `Elias/05_FEM/v4_fem_results.json`.
    Section 2 suggests one extra measurement point: deflection measured once on a jaw **without** the pocket
    would separate the pocket-step effect from the clamp stiffness.

    ## Sources

    - KHK spiral miter gears SMSG, module 1 catalogue page (MISUMI KHK1_USA_M01, p. 0260)
    - SKF Rolling bearings catalogue – guideline values for the static safety factor s0
    - DIN 3991 / ISO 10300 (bevel gear load capacity), DIN 3990 / ISO 6336 (factors, form factor chart)
    - Rexroth R999001206 (ball rail, S0) – already handled by Elias
    """)
    return


if __name__ == "__main__":
    app.run()
