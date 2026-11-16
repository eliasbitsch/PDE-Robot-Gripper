import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="Jaw train FEM")


@app.cell
def _():
    import json
    import os

    import marimo as mo
    import matplotlib.pyplot as plt
    import numpy as np

    import fem_train as T
    import load_cases as L
    return L, T, json, mo, np, os, plt


@app.cell
def _(mo):
    mo.md(r"""
    # Jaw train FEM - slide + jaw holder + mock jaw

    Right side of the gripper. The slide is fixed on its contact patch with the ball carriage, slide, holder and
    jaw are bonded, the load acts on the pad area of the jaw. The jaw is a **mock** until Viktoriia's jaw (V3)
    exists; it only carries the pad load into the holder at the right place.

    The last results are shown straight away. **Run** meshes once and solves all five load cases
    (about 5 min at 3 mm, longer when finer - plug the laptop in).
    """)
    return


@app.cell
def _(L, mo):
    mo.md(f"""
    ## Load cases (from `load_cases.py`)

    {L.table()}

    Emergency stop deceleration {L.A_STOP} m/s² is an assumption until V2 delivers the value.
    Criteria: von Mises below Rp0.2 / S, pad tilt in LC2 ≤ {L.TILT_MAX}°.
    """)
    return


@app.cell
def _(mo):
    lc = mo.ui.slider(1.5, 4.0, step=0.5, value=3.0, label="element size [mm]", show_value=True)
    go = mo.ui.run_button(label="Run all load cases")
    mo.hstack([lc, go], justify="start", gap=1.5)
    return go, lc


@app.cell
def _(T, go, json, lc, mo, os):
    path = os.path.join(T.J.OUT, "train_results.json")
    if go.value:
        with mo.status.spinner(title="meshing and solving LC1-LC5 ..."):
            res = T.run_all(lc.value)
    else:
        mo.stop(not os.path.exists(path), mo.md("_No results yet - press **Run**._"))
        res = json.load(open(path))
    return (res,)


@app.cell
def _(mo, res):
    rows = ["| Load case | u pad [µm] | tilt [°] | slide [MPa] / S | holder [MPa] / S | jaw [MPa] / S |", "|---|---|---|---|---|---|"]
    for c in res["cases"]:
        cells = []
        for k in ("slide", "holder", "jaw (mock)"):
            v = c["parts"][k]
            cells.append(f"{v['vm_peak']:.1f} / {v['safety']:.1f}{'' if v['ok'] else ' ⚠'}")
        tilt = f"{c['tilt_deg']:.3f}{'' if c.get('tilt_ok', True) else ' ⚠'}" if "tilt_deg" in c else "-"
        rows.append(f"| {c['case']} | {c['u_pad'] * 1000:.1f} | {tilt} | " + " | ".join(cells) + " |")
    mo.md(f"""
    ## Results ({res['elements']} elements, element size {res['lc']} mm)

    {chr(10).join(rows)}

    Peak stresses exclude a 1.5 mm band at the fixed carriage patch (support singularity).
    """)
    return


@app.cell
def _(L, np, plt, res):
    cases = [c["case"].split()[0] for c in res["cases"]]
    parts = ["slide", "holder", "jaw (mock)"]
    x = np.arange(len(cases))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.4))
    for i, k in enumerate(parts):
        ax1.bar(x + (i - 1) * 0.27, [c["parts"][k]["safety"] for c in res["cases"]], 0.27, label=k)
    ax1.plot(x, [c["s_req"] for c in res["cases"]], "k_", ms=22, mew=2, label="required")
    ax1.set_xticks(x, cases)
    ax1.set_ylabel("safety against Rp0.2")
    ax1.legend(frameon=False, fontsize=8)
    ax1.grid(alpha=0.3, axis="y")
    if "tilt_deg" in res["cases"][0]:
        ax2.bar(x, [abs(c["tilt_deg"]) for c in res["cases"]], color="#0f766e")
        ax2.axhline(L.TILT_MAX, color="#C2410C", ls="--", label=f"limit {L.TILT_MAX}° (LC2)")
        ax2.set_ylabel("pad tilt [°]")
        ax2.legend(frameon=False, fontsize=8)
    ax2.set_xticks(x, cases)
    ax2.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig
    return


@app.cell
def _(T, mo, os):
    png = os.path.join(T.HERE, "figures", "train_lc2_von_mises.png")
    mo.vstack([mo.md("## LC2, von Mises, deformation x300"), mo.image(src=png, width=760)]) if os.path.exists(png) else None
    return


if __name__ == "__main__":
    app.run()
