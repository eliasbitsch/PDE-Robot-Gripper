import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="Jaw FEM")


@app.cell
def _():
    import json
    import os

    import marimo as mo
    import matplotlib.pyplot as plt

    import analytic as A
    import fem_jaw as F
    return A, F, json, mo, os, plt


@app.cell
def _(mo):
    mo.md(r"""
    # Jaw FEM - gmsh + CalculiX

    Right jaw, EN AW-7075 T6. The holder face takes the grip force (normal direction only),
    the two dowels locate it; the grip force acts as uniform pressure on the pad pocket floor.
    Pick a load case and a mesh size, then press **Run**. One run takes a few seconds at 2.5 mm
    and about a minute at 1 mm.
    """)
    return


@app.cell
def _(F, mo):
    case = mo.ui.dropdown({f"{k}: {v:.1f} N": v for k, v in F.LOAD_CASES.items()},
                          value=list(f"{k}: {v:.1f} N" for k, v in F.LOAD_CASES.items())[1], label="load case")
    lc = mo.ui.slider(0.8, 4.0, step=0.1, value=2.0, label="element size [mm]", show_value=True)
    clamp = mo.ui.checkbox(label="rigid clamp (fix the holder face in all directions)")
    go = mo.ui.run_button(label="Run")
    mo.hstack([case, lc, clamp, go], justify="start", gap=1.5, wrap=True)
    return case, clamp, go, lc


@app.cell
def _(F, case, clamp, go, lc, mo):
    mo.stop(not go.value, mo.md("_Press **Run** to compute._"))
    with mo.status.spinner(title="meshing and solving ..."):
        res = F.run(lc.value, case.value, base="notebook", keep_mesh=True, clamp=clamp.value)
        png = F.render(res, F.os.path.join(F.OUT, "notebook.png"), clim=(0, 12))
    return png, res


@app.cell
def _(A, mo, res):
    ref = A.reference(res["force"])
    dev_s = (abs(res["szz_check"]) / ref["sigma_check"] - 1) * 100
    dev_w = (res["ux_load"] / ref["w_total"] - 1) * 100
    mo.md(f"""
    | | FEM | beam theory | deviation |
    |---|---|---|---|
    | bending stress, section z = -53.5 | {abs(res['szz_check']):.2f} MPa | {ref['sigma_check']:.2f} MPa | {dev_s:+.1f} % |
    | deflection at the load centre | {res['ux_load'] * 1000:.2f} µm | {ref['w_total'] * 1000:.2f} µm | {dev_w:+.1f} % |
    | peak von Mises (pocket corner, sharp edge) | {res['vm_peak']:.1f} MPa | - | safety {res['safety']:.0f} vs Rp0.2 |

    {res['elements']:,} elements, {res['nodes']:,} nodes.
    """)
    return


@app.cell
def _(mo, png):
    mo.image(src=png, width=700)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## Mesh convergence
    Saved study from `python fem_jaw.py` (five element sizes, full-current load case).
    """)
    return


@app.cell
def _(A, F, json, mo, os, plt):
    _path = os.path.join(F.OUT, "convergence.json")
    mo.stop(not os.path.exists(_path), mo.md("_No study yet: run `python fem_jaw.py` once (about 7 minutes)._"))
    rows = json.load(open(_path))
    ref0 = A.reference()
    el = [r["elements"] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.4))
    ax[0].plot(el, [-r["szz_check"] for r in rows], "o-", color="#0f766e")
    ax[0].axhline(ref0["sigma_check"], color="#c2410c", ls="--")
    ax[0].set(title="bending stress [MPa]", ylim=(0, 7))
    ax[1].plot(el, [r["ux_load"] * 1000 for r in rows], "o-", color="#0f766e")
    ax[1].axhline(ref0["w_total"] * 1000, color="#c2410c", ls="--")
    ax[1].set(title="deflection [µm]", ylim=(0, 20))
    ax[2].plot(el, [r["vm_peak"] for r in rows], "o-", color="#0f766e")
    ax[2].set(title="peak von Mises [MPa], does not converge", ylim=(0, 25))
    for a in ax:
        a.set_xscale("log"); a.set_xlabel("elements"); a.grid(alpha=0.3)
    fig.tight_layout()
    fig
    return


if __name__ == "__main__":
    app.run()
