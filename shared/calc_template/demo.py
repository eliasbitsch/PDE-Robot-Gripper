import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="Calc template")


@app.cell
def _():
    import marimo as mo
    import numpy as np
    import matplotlib.pyplot as plt
    import pint

    u = pint.UnitRegistry()
    u.formatter.default_format = "~P"
    return mo, np, plt, u


@app.cell
def _(mo):
    mo.md(r"""
    # Calculation template - what marimo can do

    Every cell reruns automatically when a value it depends on changes.
    Move the sliders and watch the numbers, formula and plot update.
    """)
    return


@app.cell
def _(mo):
    mu = mo.ui.slider(0.1, 0.6, step=0.05, value=0.2, label="friction coefficient µ", show_value=True)
    S = mo.ui.slider(1.0, 3.0, step=0.5, value=2.0, label="safety factor S", show_value=True)
    mo.hstack([mu, S], justify="start")
    return S, mu


@app.cell
def _(mo):
    mo.md(r"""
    ## 1. Calculating with units
    Values carry their unit; adding newtons to millimetres raises an error instead of giving a wrong number.
    """)
    return


@app.cell
def _(S, mo, mu, u):
    m = 0.602 * u.kg                 # part B, PA 6
    g = 9.81 * u.m / u.s**2
    a = 2.05 * u.m / u.s**2          # path acceleration
    n = 2                            # friction faces
    F_req = (S.value * m * (g + a) / (n * mu.value)).to(u.N)
    F_max = 63.6 * u.N               # full motor current
    mo.md(f"""
    | Quantity | Value |
    |---|---|
    | weight force $m \\cdot g$ | {(m * g).to(u.N):.2f} |
    | inertia force $m \\cdot a$ | {(m * a).to(u.N):.2f} |
    | **required grip force per jaw** | **{F_req:.1f}** |
    | available at full current | {F_max:.1f} |
    | utilisation | {(F_req / F_max).to("percent"):.0f} |
    """)
    return F_max, F_req


@app.cell
def _(mo):
    mo.md(r"""
    ## 2. Formulas in LaTeX

    Gripping force with friction (VDI 2860):

    $$F_\mathrm{G} = \frac{S \cdot m \cdot (g + a)}{n \cdot \mu}$$

    Tooth root stress after Lewis, with tooth form factor $Y$:

    $$\sigma_F = \frac{F_t}{b \cdot m_n \cdot Y} \le \sigma_{F,\mathrm{zul}}
    \qquad F_t = \frac{2\,T}{d}$$

    Deflection of the jaw as a cantilever with load at distance $a$ from the clamp:

    $$f = \frac{F \, a^3}{3\,E\,I}, \qquad I = \frac{b\,t^3}{12}$$
    """)
    return


@app.cell
def _(F_req, mo, u):
    b_j, t_j = 25 * u.mm, 8 * u.mm
    a_j = 30.5 * u.mm
    E = 72 * u.GPa                   # EN AW-7075
    I = b_j * t_j**3 / 12
    f = (F_req * a_j**3 / (3 * E * I)).to(u.um)
    sigma = (F_req * a_j * (t_j / 2) / I).to(u.MPa)
    mo.md(f"Jaw 25 x 8 mm, lever 30.5 mm, current grip force: deflection **{f:.2f}**, bending stress **{sigma:.1f}**")
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 3. Diagram
    """)
    return


@app.cell
def _(mo):
    mo.mermaid("""
    flowchart LR
        M[Motor 0.20 Nm] --> G[Gearbox i = 7.07]
        G --> B[Bevel pair 1:1]
        B --> P[Pinion m1 z20]
        P --> RL[Rack left] --> JL[Jaw left]
        P --> RR[Rack right] --> JR[Jaw right]
        BR[Brake 0.35 Nm] -.holds.-> G
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 4. Plot
    Required grip force over µ, with the current slider value and the motor limit.
    """)
    return


@app.cell
def _(F_max, F_req, S, mu, np, plt):
    mus = np.linspace(0.1, 0.6, 100)
    F_curve = S.value * 0.602 * (9.81 + 2.05) / (2 * mus)
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(mus, F_curve, color="#0f766e", lw=2, label="required")
    ax.axhline(F_max.magnitude, color="#C2410C", ls="--", label="motor limit")
    ax.fill_between(mus, F_curve, F_max.magnitude, where=F_curve > F_max.magnitude, color="#C2410C", alpha=0.15)
    ax.plot(mu.value, F_req.magnitude, "o", color="black")
    ax.annotate(f"{F_req.magnitude:.1f} N", (mu.value, F_req.magnitude), textcoords="offset points", xytext=(8, 8))
    ax.set_xlabel("µ")
    ax.set_ylabel("grip force per jaw [N]")
    ax.set_ylim(0, 120)
    ax.grid(alpha=0.3)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## 5. Visualisation
    Jaw as a cantilever: bending line, scaled up so it is visible.
    """)
    return


@app.cell
def _(F_req, np, plt):
    _L, _t, _b, _E = 30.5, 8.0, 25.0, 72e3        # mm, MPa
    _I = _b * _t**3 / 12
    _F = F_req.magnitude
    x = np.linspace(0, _L, 60)
    w = _F * x**2 * (3 * _L - x) / (6 * _E * _I)    # mm
    scale = 2.0 / max(w.max(), 1e-9)            # show 2 mm at the tip
    fig2, ax2 = plt.subplots(figsize=(7, 2.4))
    ax2.fill_between(x, -_t / 2, _t / 2, color="#cbd5e1", label="undeformed")
    ax2.fill_between(x, -_t / 2 + w * scale, _t / 2 + w * scale, color="#0f766e", alpha=0.6, label="deformed (scaled)")
    ax2.add_patch(plt.Rectangle((-6, -10), 6, 20, color="#475569"))
    ax2.annotate("", xy=(_L, _t / 2 + w[-1] * scale + 6), xytext=(_L, _t / 2 + w[-1] * scale + 1),
                 arrowprops=dict(arrowstyle="<-", color="#C2410C", lw=2))
    ax2.text(_L + 1, _t / 2 + w[-1] * scale + 4, f"F = {_F:.1f} N\nf = {w[-1] * 1000:.2f} µm", color="#C2410C")
    ax2.set_aspect("equal")
    ax2.set_xlim(-8, _L + 14)
    ax2.axis("off")
    ax2.legend(frameon=False, loc="lower left", fontsize=8)
    fig2
    return


if __name__ == "__main__":
    app.run()
