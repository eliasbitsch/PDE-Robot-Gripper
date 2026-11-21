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
    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**


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
    Criteria: von Mises below Rp0.2 / S; pad tilt: grip safety with the pressure wedge (`tilt_check.py`), the old {L.TILT_MAX}° line is for reference only.
    """)
    return


@app.cell
def _(mo):
    lc = mo.ui.slider(1.0, 4.0, step=0.5, value=2.0, label="element size [mm]", show_value=True)
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
        for _k in ("slide", "holder", "jaw"):
            v = c["parts"][_k]
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
    parts = ["slide", "holder", "jaw"]
    x = np.arange(len(cases))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.4))
    for _i, _k in enumerate(parts):
        ax1.bar(x + (_i - 1) * 0.27, [c["parts"][_k]["safety"] for c in res["cases"]], 0.27, label=_k)
    ax1.plot(x, [c["s_req"] for c in res["cases"]], "k_", ms=22, mew=2, label="required")
    ax1.set_xticks(x, cases)
    ax1.set_ylabel("safety against Rp0.2")
    ax1.legend(frameon=False, fontsize=8)
    ax1.grid(alpha=0.3, axis="y")
    if "tilt_deg" in res["cases"][0]:
        ax2.bar(x, [abs(c["tilt_deg"]) for c in res["cases"]], color="#0f766e")
        ax2.axhline(L.TILT_MAX, color="#C2410C", ls="--", label=f"former limit {L.TILT_MAX}° (reference)")
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


@app.cell
def _(mo):
    mo.md(r"""
    ## Mesh convergence (LC2, governing for the pad tilt)

    Every element size is meshed and solved once and kept (`out/convergence_train.json`), so finer sizes can be
    added later. Converged when the last refinement changes the value by less than 2 %. Strain energy (the work of
    the load) is the classic measure: it rises monotonically towards the exact value. Tilt and deflection come from a
    plane fit over the whole pad zone, so they do not depend on where nodes happen to lie. Peak stresses at sharp
    edges are singular and keep growing; they are reported, not used for convergence.
    """)
    return


@app.cell
def _(mo):
    conv_sizes = mo.ui.multiselect(["4", "3", "2.5", "2", "1.5", "1.2", "1"], value=["4", "3", "2.5", "2", "1.5", "1.2"],
                                   label="element sizes [mm]")
    conv_redo = mo.ui.checkbox(label="recompute sizes that already exist")
    conv_go = mo.ui.run_button(label="Compute missing sizes")
    mo.hstack([conv_sizes, conv_redo, conv_go], justify="start", gap=1.5)
    return conv_go, conv_redo, conv_sizes


@app.cell
def _(conv_go, conv_redo, conv_sizes, mo):
    import convergence as C
    conv_store = C.load()
    if conv_go.value:
        _todo = [float(x) for x in conv_sizes.value if conv_redo.value or x not in conv_store]
        for _lc in sorted(_todo, reverse=True):
            with mo.status.spinner(title=f"element size {_lc:g} mm: meshing and solving LC2 ..."):
                conv_store = C.run_size(_lc, conv_store)
    conv_store = {k: v for k, v in conv_store.items() if k in conv_sizes.value}
    return C, conv_store


@app.cell
def _(C, conv_store, mo):
    mo.stop(len(conv_store) < 2, mo.md("_At least two element sizes needed - press **Compute missing sizes**._"))
    _s = C.summary(conv_store)
    _r1 = ["| size [mm] | elements | time [s] | energy [N·mm] | tilt [°] | pad deflection [µm] | slide / holder / jaw peak [MPa] |",
           "|---|---|---|---|---|---|---|"]
    for _r in _s["rows"]:
        _r1.append(f"| {_r['lc']:g} | {_r['elements']} | {_r['t_mesh'] + _r['t_solve']:.0f} | {_r['energy']:.5f} | "
                   f"{_r['tilt_deg']:.4f} | {_r['u_pad'] * 1000:.2f} | "
                   + " / ".join(f"{_r['vm'][_k]:.1f}" for _k in C.PARTS) + " |")
    _r2 = ["| measure | last refinement | | extrapolated (Richardson) | order p | error of the finest mesh |", "|---|---|---|---|---|---|"]
    for _m in _s["metrics"].values():
        _ext = f"{_m['extrapolated'] * _m['scale']:.4g}" if _m["extrapolated"] is not None else "-"
        _p = f"{_m['order']:.2f}" if _m["order"] else "-"
        _e = f"{_m['error_finest'] * 100:.2f} %" if _m["error_finest"] is not None else "-"
        _r2.append(f"| {_m['label']} | {_m['change_last'] * 100:.2f} % | {'converged' if _m['converged'] else '**not converged**'} | {_ext} | {_p} | {_e} |")
    mo.vstack([mo.md(chr(10).join(_r1)), mo.md(chr(10).join(_r2)), C.plot(conv_store)])
    return


@app.cell
def _(conv_store, mo):
    _keys = sorted(conv_store, key=float, reverse=True)
    conv_pick = mo.ui.multiselect(_keys, value=[_keys[0], _keys[-1]] if len(_keys) > 1 else _keys, label="compare meshes")
    conv_pick
    return (conv_pick,)


@app.cell
def _(T, conv_pick, mo, os):
    _imgs = []
    for _k in sorted(conv_pick.value, key=float, reverse=True):
        _png = os.path.join(T.HERE, "figures", f"conv_lc{_k}.png")
        if os.path.exists(_png):
            _imgs.append(mo.vstack([mo.md(f"**{_k} mm**"), mo.image(src=_png, width=480)]))
    mo.hstack(_imgs, justify="start", wrap=True) if _imgs else mo.md("_No mesh pictures yet._")
    return


if __name__ == "__main__":
    app.run()
