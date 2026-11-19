"""Mesh convergence of the jaw train FEM (LC2, the governing case for the pad tilt).

python convergence.py [sizes ...]   ->  out/convergence_train.json, figures/convergence_train.png, figures/conv_lc*.png
Each element size is meshed and solved once and stored, so a finer size can be added later without redoing the
coarse ones. Converged when the last refinement changes energy, tilt and pad deflection by less than 2 %.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fem_train as T  # noqa: E402

PATH = os.path.join(T.J.OUT, "convergence_train.json")   # convergence.json belongs to fem_jaw.py
SIZES = [4.0, 3.0, 2.5, 2.0, 1.5, 1.2]
TOL_CONV = 0.02
CASE = "LC2"
METRICS = [("energy", "strain energy [N·mm]", 1.0), ("tilt_deg", "pad tilt [°]", 1.0), ("u_pad", "pad deflection [µm]", 1000.0)]
PARTS = ["slide", "holder", "jaw"]


def load():
    return json.load(open(PATH)) if os.path.exists(PATH) else {}


def run_size(lc, store=None):
    store = load() if store is None else store
    r = T.run(lc, only=[CASE], tag=f"conv{lc:g}", png=os.path.join(HERE, "figures", f"conv_lc{lc:g}.png"))
    c = r["cases"][0]
    store[f"{lc:g}"] = dict(lc=lc, nodes=r["nodes"], elements=r["elements"], t_mesh=r["t_mesh"], t_solve=r["t_solve"],
                            energy=c["energy"], tilt_deg=c["tilt_deg"], u_pad=c["u_pad"], ux_pad=c["ux_pad"],
                            vm={k: c["parts"][k]["vm_peak"] for k in PARTS},
                            vm_at={k: c["parts"][k]["at"] for k in PARTS})
    json.dump(store, open(PATH, "w"), indent=1)
    return store


def richardson(h, f):
    """Extrapolated value and observed order from the three finest results (h = element size)."""
    if len(h) < 3:
        return None, None
    (h1, f1), (h2, f2), (h3, f3) = sorted(zip(h, f))[:3]          # h1 finest
    r = h2 / h1
    try:
        p = np.log(abs((f3 - f2) / (f2 - f1))) / np.log(r)
        if not np.isfinite(p) or p <= 0:
            return None, None
        return f1 + (f1 - f2) / (r ** p - 1), float(p)
    except ZeroDivisionError:
        return None, None


def summary(store=None):
    store = load() if store is None else store
    rows = sorted(store.values(), key=lambda r: -r["lc"])
    out = {"rows": rows, "metrics": {}}
    if len(rows) < 2:
        return out
    for key, label, s in METRICS + [(f"vm:{p}", f"peak von Mises {p} [MPa]", 1.0) for p in PARTS]:
        f = [r["vm"][key[3:]] if key.startswith("vm:") else r[key] for r in rows]
        h = [r["lc"] for r in rows]
        change = abs(f[-1] - f[-2]) / abs(f[-1])
        ext, p = richardson(h, f)
        out["metrics"][key] = dict(label=label, scale=s, values=f, change_last=change, converged=bool(change < TOL_CONV),
                                   extrapolated=ext, order=p, error_finest=abs(f[-1] - ext) / abs(ext) if ext else None)
    return out


def plot(store=None, path=None):
    import matplotlib
    if path:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    s = summary(store)
    rows = s["rows"]
    n = [r["elements"] for r in rows]
    fig, ax = plt.subplots(1, 4, figsize=(15, 3.4))
    for a, (key, label, sc) in zip(ax[:3], METRICS):
        m = s["metrics"].get(key)
        if not m:
            continue
        a.plot(n, np.array(m["values"]) * sc, "o-", color="#0f766e")
        if m["extrapolated"] is not None:
            a.axhline(m["extrapolated"] * sc, color="#C2410C", ls="--", lw=1, label="Richardson extrapolation")
            a.legend(frameon=False, fontsize=8)
        a.set_title(f"{label}\nlast step {m['change_last'] * 100:.2f} %", fontsize=9)
    for p in PARTS:
        m = s["metrics"].get(f"vm:{p}")
        if m:
            ax[3].plot(n, m["values"], "o-", label=p)
    ax[3].set_title("peak von Mises [MPa] (singular edges grow)", fontsize=9)
    ax[3].legend(frameon=False, fontsize=8)
    for a in ax:
        a.set_xscale("log")
        a.set_xlabel("elements")
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=130)
    return fig


if __name__ == "__main__":
    sizes = [float(x) for x in sys.argv[1:]] or SIZES
    st = load()
    for lc in sizes:
        if f"{lc:g}" in st:
            continue
        st = run_size(lc, st)
        r = st[f"{lc:g}"]
        print(f"lc {lc:4.1f} mm  {r['elements']:7d} el  {r['t_mesh'] + r['t_solve']:6.0f} s  energy {r['energy']:.5f}  "
              f"tilt {r['tilt_deg']:.4f}  u_pad {r['u_pad'] * 1000:.2f} um  "
              + "  ".join(f"{k} {v:.1f}" for k, v in r["vm"].items()), flush=True)
    plot(st, os.path.join(HERE, "figures", "convergence_train.png"))
    for k, m in summary(st)["metrics"].items():
        ext = f"extrapolated {m['extrapolated'] * m['scale']:.4g} (p {m['order']:.2f})" if m["extrapolated"] is not None else ""
        print(f"{m['label']:40s} last step {m['change_last'] * 100:5.2f} %  {'converged' if m['converged'] else 'NOT converged'}  {ext}")
