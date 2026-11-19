"""Viktoriia's V4 (beam theory of the jaw) against the FEM of her jaw V3 (jaw_v3.step), the <= 5 % check.

python v4_check.py [sizes ...]  ->  out/v4_check.json
Her notebook is executed as it is (marimo app.run), so the comparison always uses her current numbers. FEM: holder
face fixed in x + dowel holes in y/z ("standard") and the face fixed in x, y, z ("rigid clamp", the beam assumption),
full current, the same check points as V4: bending stress on the outer face at z = -53.5, deflection at the load centre.
"""
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fem_jaw as J  # noqa: E402

V4 = os.path.join(HERE, "..", "..", "Viktoriia", "04_Analytical", "v4_jaw.py")
J.STEP = os.path.join(HERE, "jaw_v3.step")
PATH = os.path.join(J.OUT, "v4_check.json")
SHARED = os.path.join(HERE, "v4_fem_results.json")   # committed: V4 reads its FEM reference values from here
SIZES = [2.5, 1.6, 1.0]                             # 0.7 mm needs several GB of scratch, adds nothing
TOL = 5.0                                           # % (lecture 17 Oct)


def analytic(force=J.P.F_JAW_MAX):
    import matplotlib
    matplotlib.use("Agg")
    spec = importlib.util.spec_from_file_location("v4_jaw", V4)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _, defs = mod.app.run()
    d = defs["deflection"](force)
    return dict(force=force, sigma_check=defs["stresses"](force)["FEM check z = -53.5 (s = 6.5)"],
                w_rigid=(d["w_b"] + d["w_s"]) * 1000, w_elastic=(d["w_b"] + d["w_s"] + d["w_c"]) * 1000,
                w_b=d["w_b"] * 1000, w_s=d["w_s"] * 1000, w_c=d["w_c"] * 1000, fem_used_by_v4=defs["FEM"])


def load():
    return json.load(open(PATH)) if os.path.exists(PATH) else {}


def run_size(lc, store=None):
    store = load() if store is None else store
    row = {}
    for sup, clamp in (("standard", False), ("rigid", True), ("section", "section")):
        r = J.run(lc, base=f"v4_{sup}_{lc:g}".replace(".", "_"), clamp=clamp)
        row[sup] = dict(elements=r["elements"], ux=r["ux_load"] * 1000, szz=r["szz_check"], vm_peak=r["vm_peak"],
                        vm_peak_at=r["vm_peak_at"])
    store[f"{lc:g}"] = row
    json.dump(store, open(PATH, "w"), indent=1)
    fine = store[min(store, key=float)]
    json.dump(dict(sigma_check=round(abs(fine["standard"]["szz"]), 3), w_rigid=round(fine["rigid"]["ux"], 2),
                   w_std=round(fine["standard"]["ux"], 2), elements=fine["rigid"]["elements"],
                   lc=float(min(store, key=float)), jaw="Viktoriia V3 (jaw_v3.step)"), open(SHARED, "w"), indent=1)
    return store


def compare(store=None, an=None):
    store = load() if store is None else store
    an = analytic() if an is None else an
    if not store:
        return None
    fine = store[min(store, key=float)]
    dev = lambda a, f: (a - f) / f * 100
    rows = [("bending stress z = -53.5 [MPa]", an["sigma_check"], abs(fine["standard"]["szz"])),   # outer face is in compression
            ("deflection, rigid beam (a+b) vs FEM clamped section at z = -47 [µm]", an["w_rigid"], fine["section"]["ux"]) if "section" in fine else None,
            ("deflection, rigid beam (a+b) vs FEM rigid clamp [µm]", an["w_rigid"], fine["rigid"]["ux"]),
            ("deflection, beam + Weber (a+b+c) vs FEM rigid clamp [µm]", an["w_elastic"], fine["rigid"]["ux"]),
            ("deflection, beam + Weber vs FEM standard support [µm]", an["w_elastic"], fine["standard"]["ux"])]
    return [dict(quantity=q, analytic=a, fem=f, dev=dev(a, f), ok=abs(dev(a, f)) <= TOL) for q, a, f in filter(None, rows)]


if __name__ == "__main__":
    sizes = [float(x) for x in sys.argv[1:]] or SIZES
    st = load()
    for lc in sizes:
        if f"{lc:g}" not in st:
            st = run_size(lc, st)
            print(lc, st[f"{lc:g}"], flush=True)
    for r in compare(st):
        print(f"{r['quantity']:58s} V4 {r['analytic']:7.2f}  FEM {r['fem']:7.2f}  {r['dev']:+6.1f} %  {'ok' if r['ok'] else 'NO'}")
