import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="V4 - Analytical strength proof of the jaw")


@app.cell
def _():
    import sys
    import math
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt

    sys.path.insert(0, str(mo.notebook_dir() / ".." / ".." / "Elias" / "02_CAD_Body"))
    import gripper_params as P
    return P, math, mo, np, plt


@app.cell
def _(P, mo):
    _zc = P.HOLDER_Z_LOW
    mo.md(rf"""
    # V4 - Analytical strength proof of the jaw

    **Robot gripper, part B - group 10 - Ovdiienko / Bitsch** · status 21 Nov 2026 · milestone M3

    Hand calculation (beam theory) of the jaw, done independently of the FEM (E5), and then held against it.
    Target from the lecture (17 Oct): **deviation FEM ↔ analytical ≤ 5 %**.

    ## 1. Model and assumptions

    | Item | Value | Source |
    |---|---|---|
    | Jaw | T-shaped, EN AW-7075 T6: web {P.JAW_W:.0f} × {P.JAW_T:.0f} mm from the top (z = {P.JAW_Z_TOP}) to z = {P.JAW_PAD_TOP}, plate {P.JAW_W_PAD:.0f} × {P.JAW_T:.0f} mm below, down to z = {P.JAW_Z_BOTTOM} | V3 (review 21 Nov: wide, low pad against the twist of the part) |
    | E, ν | 71.7 GPa, 0.33 | EN AW-7075 T6 data sheet value (same as the FEM, so the comparison is about the model, not the material) |
    | Rp0.2 | 460 MPa | EN 485-2, plate 6 - 12.5 mm |
    | Clamp | lower end of the holder contact, z = {_zc} | handover § 4 |
    | Load | grip force as **uniform pressure on the pocket floor**, z = {P.PAD_Z[1]} ... {P.PAD_Z[0]}, resultant at the pad centre, lever **{_zc - sum(P.PAD_Z) / 2:.1f} mm** | handover § 4 (pad plate spreads the load) |
    | Section | web {P.JAW_W:.0f} × 8 above the plate; plate {P.JAW_W_PAD:.0f} × 8; in the pocket: plate {P.JAW_W_PAD:.0f} × 5 + two side rims {(P.JAW_W_PAD - P.PAD_W) / 2:.0f} × 3 | {P.PAD_W:.0f} mm wide, 3 mm deep pocket |
    | Load cases | {P.F_GRIP_OILY:.1f} N (regular, µ = 0.2) and {P.F_JAW_MAX:.1f} N (full motor current) | V2 / handover |

    Coordinate $s$ = distance below the clamp: plate from $s$ = {_zc - P.JAW_PAD_TOP:.1f}, loaded span $s$ = {_zc - P.PAD_Z[1]:.1f} ... {_zc - P.PAD_Z[0]:.1f} mm, jaw end at $s$ = {_zc - P.JAW_Z_BOTTOM:.1f} mm.
    """)
    return


@app.cell
def _(P, mo):
    F_cases = {"regular (µ = 0.2)": P.F_GRIP_OILY, "full current": P.F_JAW_MAX}
    E, NU, RP02 = 71700.0, 0.33, 460.0
    G_mod = E / (2 * (1 + NU))
    B, H = P.JAW_W, P.JAW_T                             # web 25 x 8 (root, Weber term)
    B_PLATE = P.JAW_W_PAD                               # 88: plate under the web
    POCKET_D, POCKET_W = 3.0, P.PAD_W
    RIM = (B_PLATE - POCKET_W) / 2                      # 3 mm
    z_top = P.JAW_Z_TOP                                 # -28.5
    z_clamp = P.HOLDER_Z_LOW                            # -47
    s_p = z_clamp - P.JAW_PAD_TOP                       # 9.5  start of the wide plate
    s_a = z_clamp - P.PAD_Z[1]                          # 13.5 start of the loaded span
    s_b = z_clamp - P.PAD_Z[0]                          # 34.5 end of the loaded span
    s_end = z_clamp - P.JAW_Z_BOTTOM                    # 37
    s_F = 0.5 * (s_a + s_b)                             # 24 resultant
    holes = [(s_F, 3.4)]                                # two pad-plate screws side by side at the pad centre height
    N_HOLES = 2
    CBORE_D, CBORE_T = 6.5, 1.0                         # V3: counterbore on the outer face at both screw holes
    use_holes = mo.ui.checkbox(value=True, label="include the two Ø3.4 holes and Ø6.5 x 1 counterbores (V3) in the stiffness")
    use_holes
    return (B, B_PLATE, CBORE_D, CBORE_T, E, F_cases, G_mod, H, NU, N_HOLES, POCKET_D, RIM, RP02, holes, s_F, s_a,
            s_b, s_end, s_p, use_holes, z_clamp)


@app.cell
def _(mo):
    mo.md(r"""
    ## 2. Section properties

    Bending about the jaw's width axis; fibre coordinate $x$ through the thickness, $x$ = 0 on the inner face
    (towards the part), $x$ = 8 on the outer face (on the holder). In the pocket the material between $x$ = 0 and 3
    is only the two 1 mm rims.

    $$I = \sum \left(\frac{b_i h_i^3}{12} + A_i\,(x_i - x_c)^2\right), \qquad
    \sigma(x) = \frac{M\,(x - x_c)}{I}$$
    """)
    return


@app.cell
def _(B, B_PLATE, CBORE_D, CBORE_T, H, N_HOLES, POCKET_D, RIM, holes, mo, s_a, s_b, s_p, use_holes):
    def _chord(d, dia):
        return 2 * max(0.0, (dia / 2) ** 2 - d ** 2) ** 0.5

    def section(s, with_holes=False):
        """I [mm^4], A [mm^2], x_c [mm] (from the inner face) at distance s below the clamp.
        With holes: the hole chord is taken out of the web, the counterbore chord out of the outer 1 mm layer."""
        if s_a <= s <= s_b:
            d = min(abs(s - sh) for sh, _ in holes) if with_holes else 1e9
            w_hole = _chord(d, holes[0][1])
            w_cb = _chord(d, CBORE_D)
            web_h = H - POCKET_D - CBORE_T
            parts = [(B_PLATE - N_HOLES * w_hole, web_h, POCKET_D + web_h / 2),
                     (B_PLATE - N_HOLES * max(w_hole, w_cb), CBORE_T, H - CBORE_T / 2),
                     (2 * RIM, POCKET_D, POCKET_D / 2)]
        elif s >= s_p:
            parts = [(B_PLATE, H, H / 2)]
        else:
            parts = [(B, H, H / 2)]
        A = sum(w * h for w, h, _ in parts)
        xc = sum(w * h * x for w, h, x in parts) / A
        I = sum(w * h**3 / 12 + w * h * (x - xc) ** 2 for w, h, x in parts)
        return I, A, xc

    I_full, A_full, xc_full = section(0.0)
    I_plate, A_plate, xc_plate = section(0.5 * (s_p + s_a))
    I_pock, A_pock, xc_pock = section(0.5 * (s_a + holes[0][0]))
    I_hole, A_hole, xc_hole = section(holes[0][0], True)
    mo.md(f"""
    | Section | I | A | x_c (from inner face) | outer fibre distances |
    |---|---|---|---|---|
    | web {B:.0f} × 8 | {I_full:.1f} mm⁴ | {A_full:.0f} mm² | {xc_full:.2f} mm | ± {xc_full:.2f} mm |
    | plate {B_PLATE:.0f} × 8 | {I_plate:.1f} mm⁴ | {A_plate:.0f} mm² | {xc_plate:.2f} mm | ± {xc_plate:.2f} mm |
    | pocket (plate + rims) | {I_pock:.1f} mm⁴ | {A_pock:.0f} mm² | {xc_pock:.2f} mm | rim tip {xc_pock:.2f} / pocket floor {xc_pock-3:.2f} / outer {8-xc_pock:.2f} mm |
    | pocket at a screw hole | {I_hole:.1f} mm⁴ | {A_hole:.0f} mm² | {xc_hole:.2f} mm | |

    The plate is {I_plate/I_full:.1f} × as stiff as the web; the pocket takes {100*(1-I_pock/I_plate):.0f} % of the plate's stiffness. Holes in the stiffness: {'on' if use_holes.value else 'off'}.
    """)
    return (section,)


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Internal forces

    Uniform line load $q = F / (s_b - s_a)$ on $s_a \dots s_b$. Above the loaded span the moment is simply
    $M(s) = F\,(s_F - s)$; inside it

    $$M(s) = \frac{q}{2}\,(s_b - s)^2, \qquad V(s) = q\,(s_b - s) \qquad (s_a \le s \le s_b)$$
    """)
    return


@app.cell
def _(np, s_F, s_a, s_b, s_end):
    def internal(F, s):
        q = F / (s_b - s_a)
        s = np.asarray(s, dtype=float)
        V = np.where(s < s_a, F, np.where(s <= s_b, q * (s_b - s), 0.0))
        M = np.where(s < s_a, F * (s_F - s), np.where(s <= s_b, q * (s_b - s) ** 2 / 2, 0.0))
        return V, M

    s_grid = np.linspace(0.0, s_end, 5101)
    return internal, s_grid


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. Bending stress

    Check points:
    - **clamp** $s$ = 0 (largest moment, full section),
    - **FEM check section** z = -53.5 ($s$ = 6.5, full section, away from the clamp singularity - this is where Elias reads the FEM),
    - **web root at the plate** (end of the 25 mm web, the plate below is stiffer),
    - **start of the pocket** (step in the section: pocket floor and rim tips),
    - **screw holes** at the pad centre height (net section, two holes side by side).
    """)
    return


@app.cell
def _(F_cases, H, RP02, holes, internal, mo, s_a, s_p, section, z_clamp):
    def stresses(F):
        pts = {"clamp (s = 0)": (0.0, False, "outer fibre"),
               "FEM check z = -53.5 (s = 6.5)": (z_clamp - (-53.5), False, "outer fibre"),
               f"web root at the plate (s = {s_p:.1f})": (s_p - 1e-9, False, "outer fibre"),
               f"pocket start (s = {s_a:.1f}), pocket floor": (s_a, False, "floor"),
               f"pocket start (s = {s_a:.1f}), rim tip": (s_a, False, "rim"),
               f"screw holes (s = {holes[0][0]:.1f}), net section": (holes[0][0], True, "floor")}
        out = {}
        for name, (s, wh, where) in pts.items():
            _, M = internal(F, s)
            I, A, xc = section(s + (1e-9 if s == s_a else 0.0), wh)
            if where == "outer fibre":
                c = max(xc, H - xc)
            elif where == "floor":
                c = abs(3.0 - xc) if s >= s_a else xc
            else:
                c = xc
            out[name] = float(M) * c / I
        return out

    _rows = ""
    for _lbl, _F in F_cases.items():
        for _k, _sig in stresses(_F).items():
            _rows += f"| {_lbl} ({_F:.1f} N) | {_k} | {_sig:.2f} MPa | {RP02/_sig:.0f} |\n"
    mo.md(f"""
    | Load case | Point | σ_b | safety vs. Rp0.2 |
    |---|---|---|---|
    {_rows}

    Every nominal stress is far below Rp0.2 (safety > 20); the largest sits in the web at the clamp.
    **Strength is not the design criterion for the jaw; stiffness is.**
    The sharp pocket corner in the CAD is a notch (the FEM sees a growing peak there): a milled pocket has a
    corner radius anyway - put R ≥ 1 mm on the drawing.
    """)
    return (stresses,)


@app.cell
def _(mo):
    mo.md(r"""
    ## 5. Deflection

    Three contributions, added up (all linear in F):

    **a) Bending** (Euler-Bernoulli, stepped section), double integration with $w(0) = w'(0) = 0$:
    $$w_b(s) = \int_0^s \int_0^{\xi} \frac{M(\eta)}{E\,I(\eta)}\,d\eta\,d\xi$$

    **b) Shear** (Timoshenko), $\kappa$ = 5/6 for a rectangle - not negligible because the jaw is short ($L/h \approx 4$):
    $$w_s(s) = \int_0^s \frac{V(\eta)}{\kappa\,G\,A(\eta)}\,d\eta$$

    **c) Elastic clamp.** A "rigid clamp" in beam theory means the root section cannot rotate. In reality the
    material behind the root (here the 18.5 mm of jaw in the holder zone) deforms as well. The classic
    correction for a beam built into an elastic body of the same material is Weber's foundation term,
    known from gear-tooth deflection (Weber & Banaschek 1953; restated by Sainsot, Velex & Duverger,
    *J. Mech. Des.* 126 (2004) 748):
    $$w_c = \frac{F}{b\,E}\left[\frac{16.67}{\pi}\left(\frac{a}{h}\right)^2 + \frac{2(1-\nu)}{\pi}\,\frac{a}{h} + 1.534\right]$$
    with $a$ = lever of the resultant, $h$ = 8 mm (root thickness) and $b$ = 25 mm (web width at the root).
    """)
    return


@app.cell
def _(B, E, F_cases, G_mod, H, NU, internal, math, mo, np, s_F, s_grid, section, use_holes):
    kappa = 5 / 6
    I_s = np.array([section(s, use_holes.value)[0] for s in s_grid])
    A_s = np.array([section(s, use_holes.value)[1] for s in s_grid])

    def cumtrapz(y, x):
        return np.concatenate([[0.0], np.cumsum((y[1:] + y[:-1]) / 2 * np.diff(x))])

    def deflection(F):
        V, M = internal(F, s_grid)
        slope = cumtrapz(M / (E * I_s), s_grid)
        w_b = cumtrapz(slope, s_grid)
        w_s = cumtrapz(V / (kappa * G_mod * A_s), s_grid)
        a_h = s_F / H
        w_c_load = F / (B * E) * (16.67 / math.pi * a_h**2 + 2 * (1 - NU) / math.pi * a_h + 1.534)
        theta_c = w_c_load / s_F                           # same correction expressed as a root rotation
        w_c = theta_c * s_grid
        at = lambda w: float(np.interp(s_F, s_grid, w))
        return dict(w_b=at(w_b), w_s=at(w_s), w_c=w_c_load, curve=(w_b, w_b + w_s, w_b + w_s + w_c))

    _rows = ""
    for _lbl, _F in F_cases.items():
        _d = deflection(_F)
        _tot = _d["w_b"] + _d["w_s"] + _d["w_c"]
        _rows += (f"| {_lbl} ({_F:.1f} N) | {_d['w_b']*1000:.2f} | {_d['w_s']*1000:.2f} | {_d['w_c']*1000:.2f} | "
                  f"**{(_d['w_b']+_d['w_s'])*1000:.2f}** | **{_tot*1000:.2f}** |\n")
    mo.md(f"""
    Deflection at the load centre ($s$ = {s_F} mm), in µm:

    | Load case | a) bending | b) shear | c) elastic clamp | rigid clamp (a+b) | elastic clamp (a+b+c) |
    |---|---|---|---|---|---|
    {_rows}
    """)
    return (deflection,)


@app.cell
def _(F_cases, deflection, plt, s_a, s_b, s_grid):
    _F = F_cases["full current"]
    _d = deflection(_F)
    fig_w, _ax = plt.subplots(figsize=(7, 3.2))
    _ax.plot(s_grid, _d["curve"][0] * 1000, color="#94a3b8", lw=1.5, label="bending only")
    _ax.plot(s_grid, _d["curve"][1] * 1000, color="#0f766e", lw=1.5, ls="--", label="+ shear (rigid clamp)")
    _ax.plot(s_grid, _d["curve"][2] * 1000, color="#0f766e", lw=2.5, label="+ elastic clamp")
    _ax.axvspan(s_a, s_b, color="#C2410C", alpha=0.08, label="pad / load zone")
    _ax.set_xlabel("distance below the clamp s [mm]"); _ax.set_ylabel("deflection [µm]")
    _ax.set_title(f"Bending line at F = {_F:.1f} N"); _ax.grid(alpha=0.3); _ax.legend(frameon=False, fontsize=8)
    fig_w.tight_layout()
    fig_w
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 6. Comparison with the FEM (E5, Elias, F = 63.6 N)

    FEM of **this V3 jaw**, read from `Elias/05_FEM/v4_fem_results.json` (written by `v4_check.py`: gmsh C3D10,
    finest mesh, CalculiX, converged); the old mock-jaw values of 11 Nov are the fallback if the file is missing.
    Two FEM support variants exist: **standard** (holder face fixed in x only, dowel holes in y/z) and **rigid clamp**.
    Like must be compared with like: the analytical *rigid* model belongs to neither exactly - the FEM "rigid clamp"
    still lets the jaw material inside the holder zone deform, which is exactly what term c) describes.
    """)
    return


@app.cell
def _(F_cases, deflection, mo, stresses):
    import json as _json
    _F = F_cases["full current"]
    _p = mo.notebook_dir() / ".." / ".." / "Elias" / "05_FEM" / "v4_fem_results.json"
    FEM = _json.load(open(_p)) if _p.exists() else {"sigma_check": 5.71, "w_std": 17.0, "w_rigid": 15.0}
    _sig = stresses(_F)["FEM check z = -53.5 (s = 6.5)"]
    _d = deflection(_F)
    w_rigid_an = (_d["w_b"] + _d["w_s"]) * 1000
    w_el_an = w_rigid_an + _d["w_c"] * 1000

    def dev(an, fem):
        return (an - fem) / fem * 100

    def ok(x):
        return "✓" if abs(x) <= 5 else "✗"

    _r1, _r2, _r3, _r4 = (dev(_sig, FEM["sigma_check"]), dev(w_el_an, FEM["w_rigid"]),
                          dev(w_el_an, FEM["w_std"]), dev(w_rigid_an, FEM["w_rigid"]))
    _bounds = sorted((w_rigid_an, w_el_an))
    reading = (f"the stress at the check section deviates {_r1:+.1f} % ({'within' if abs(_r1) <= 5 else 'outside'} 5 %). "
               f"Deflection: the beam with Weber's elastic clamp deviates {_r2:+.1f} % from the FEM rigid clamp "
               f"({'within' if abs(_r2) <= 5 else 'outside'} 5 %); the two beam variants give {_bounds[0]:.1f} ... "
               f"{_bounds[1]:.1f} µm against {FEM['w_rigid']:.1f} µm in the FEM. Weber's term comes from gear-tooth "
               f"roots and is used here as an estimate of the elastic clamp, not as a proof. The FEM standard support "
               f"(holder face fixed in x only) is the soft bound; the printed jaw (V5) decides which support is real.")
    mo.md(f"""
    | Quantity | analytical (V4) | FEM (E5) | deviation | ≤ 5 % |
    |---|---|---|---|---|
    | bending stress at z = -53.5 | {_sig:.2f} MPa | {FEM['sigma_check']:.2f} MPa | {_r1:+.1f} % | {ok(_r1)} |
    | deflection, beam with rigid clamp (a+b) vs FEM rigid clamp | {w_rigid_an:.1f} µm | {FEM['w_rigid']:.1f} µm | {_r4:+.1f} % | {ok(_r4)} |
    | **deflection, elastic clamp (a+b+c) vs FEM rigid clamp** | **{w_el_an:.1f} µm** | **{FEM['w_rigid']:.1f} µm** | **{_r2:+.1f} %** | **{ok(_r2)}** |
    | deflection, elastic clamp vs FEM standard support | {w_el_an:.1f} µm | {FEM['w_std']:.1f} µm | {_r3:+.1f} % | {ok(_r3)} |

    **Reading:** {reading}

    For the report: compare **like with like** (rigid ↔ rigid + Weber), state the standard-support result as the
    upper bound, and let V5 arbitrate.
    """)
    return


@app.cell
def _(F_cases, deflection, mo):
    _F = F_cases["full current"]
    _d = deflection(_F)
    _w = (_d["w_b"] + _d["w_s"] + _d["w_c"]) * 1000            # µm, aluminium, linear in F and 1/E
    _rows = ""
    for _mat, _E in (("PLA (≈ 3.5 GPa)", 3500.0), ("PETG (≈ 2.1 GPa)", 2100.0)):
        for _Fp in (20.0, 50.0):
            _rows += f"| {_mat} | {_Fp:.0f} N | {_w * (_Fp/_F) * (71700.0/_E) / 1000:.2f} mm |\n"
    mo.md(f"""
    ## 7. Outlook V5 - what to expect on the printed jaw

    Same geometry, printed solid (100 % infill), deflection scales with $F/E$ (the shear term scales with $1/G$,
    close enough for planning). Measured with a dial gauge at the pad centre:

    | Material | Load | expected deflection |
    |---|---|---|
    {_rows}

    A few tenths of a millimetre - easy to measure with a 0.01 mm dial gauge. Use the E-modulus of the actual
    filament (data sheet, printed in the load direction) - print orientation changes it noticeably.
    """)
    return


if __name__ == "__main__":
    app.run()
