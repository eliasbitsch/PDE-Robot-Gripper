"""Monte Carlo robustness of the grip: full cycles (built two-stage closing from fully open, lift, transfer on the
real robot path, lower) with scattered inputs.

python robustness.py [n_spec] [n_stress]  ->  out/robustness_cpu.json, figures/robustness_cpu.png

CPU reference of the study (float64, one process per core). The study itself runs on the GPU, robustness_gpu.py
(MuJoCo Warp, about 2 min instead of an hour) and writes out/robustness.json; this script checks it.

spec    inputs inside their documented ranges: does every grip hold?
          part position across the grip axis  +-5 mm uniform      CAD_PLAN point 0
          pad friction                         0.2 ... 0.3         oily design value ... grooved pads (gripper_params)
          grip force                           44.5 N +-5 %        calibration at commissioning (CAD_PLAN tab. 267)
          part mass                            +0 ... +3 %         PA 6 moisture uptake in standard climate 23 C / 50 % RH
                                                                   (data sheet values 2.5 ... 3 %); a lighter part is easier
stress  friction down to 0.08 and grip force down to 50 %, to show where the grip starts to fail
saturated  spec, but the part water-saturated (+9 ... +10 %, e.g. wet from a wash): python robustness.py --saturated
Held = not dropped and slip beyond the model's creep base < SLIP_OK (validate_grip.py).
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np
from scipy.stats import qmc

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim as S  # noqa: E402
import validate_grip as V  # noqa: E402

GP, MM = S.GP, S.MM
F0 = GP.F_GRIP_SET
TOL = 0.005
RANGES = {"spec": dict(offset=(-TOL, TOL), mu=(0.2, 0.3), force=(0.95 * F0, 1.05 * F0), mass=(1.0, 1.03)),
          "stress": dict(offset=(-TOL, TOL), mu=(0.08, 0.3), force=(0.5 * F0, 1.05 * F0), mass=(1.0, 1.03)),
          "saturated": dict(offset=(-TOL, TOL), mu=(0.2, 0.3), force=(0.95 * F0, 1.05 * F0), mass=(1.09, 1.10))}
MU_GRID = np.linspace(0.08, 0.3, 6)


def creep(mu):
    """Creep base of the built closing at this friction (validate_grip.creep_base, centred part, 100 N)."""
    r, _ = V.run("reference", V.F_REF, mu=float(mu), offset=0.0, creep_ref=0.0)
    return r["slip_max"]


def sample(study, n, seed):
    rg = RANGES[study]
    u = qmc.LatinHypercube(d=4, seed=seed).random(n)
    keys = ("offset", "mu", "force", "mass")
    return [dict(study=study, i=i, **{k: float(rg[k][0] + u[i, j] * (rg[k][1] - rg[k][0])) for j, k in enumerate(keys)})
            for i in range(n)]


def one(args):
    s, base_grid = args
    base = float(np.interp(s["mu"], MU_GRID, base_grid))
    t0 = time.time()
    r, _ = V.run(s["study"], s["force"], mu=s["mu"], offset=s["offset"], mass_scale=s["mass"], creep_ref=base)
    keep = ("util_max", "util_median", "util_pad_max", "util_pad_raw", "lost_contact", "t_peak", "phase_peak", "slip_max",
            "slip_excess", "held", "dropped", "a_tcp_max", "seat_mm")
    return dict(s, **{k: r[k] for k in keep}, creep_base=base, sec=time.time() - t0)


def plot(res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    spec = [r for r in res if r["study"] == "spec"]
    stress = [r for r in res if r["study"] == "stress"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    if spec:
        u = np.array([r["util_max"] for r in spec])
        ax[0].hist(u, bins=40, color="#0F8A7A")
        ax[0].axvline(1.0, color="#C2410C", ls="--", lw=1.2, label="friction limit")
        ax[0].set(xlabel="peak friction use over the cycle", ylabel="grips",
                  title=f"{len(spec)} grips inside spec: {sum(r['held'] for r in spec)} held")
        ax[0].legend()
    if stress:
        x = np.array([r["mu"] for r in stress])
        y = np.array([r["force"] for r in stress])
        ok = np.array([r["held"] for r in stress])
        ax[1].scatter(x[ok], y[ok], s=6, c="#0F8A7A", label="held")
        ax[1].scatter(x[~ok], y[~ok], s=6, c="#C2410C", label="slips or drops")
        mm = np.linspace(0.08, 0.3, 100)
        need = GP.B_MASSE * (GP.G + GP.A_PATH_REAL) / (GP.N_BACKEN * mm)       # hand calculation without safety
        ax[1].plot(mm, need, "k--", lw=1, label="hand calculation, S = 1")
        ax[1].add_patch(plt.Rectangle((0.2, 0.95 * F0), 0.1, 0.1 * F0, fill=False, ec="k", lw=1.2, label="spec box"))
        ax[1].set(xlabel="pad friction", ylabel="grip force per jaw [N]", ylim=(0.45 * F0, 1.1 * F0),
                  title=f"stress: {len(stress)} grips, {int((~ok).sum())} fail")
        ax[1].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0))   # the scatter fills the whole plot
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def main(n_spec=10000, n_stress=2000, workers=None):
    workers = workers or max(1, os.cpu_count() - 2)
    t0 = time.time()
    with Pool(workers) as pool:
        base_grid = pool.map(creep, MU_GRID)
        print("creep base per friction:", dict(zip(np.round(MU_GRID, 3).tolist(), np.round(base_grid, 2).tolist())))
        todo = sample("spec", n_spec, 1) + sample("stress", n_stress, 2)
        res = []
        for k, r in enumerate(pool.imap_unordered(one, [(s, base_grid) for s in todo], chunksize=4)):
            res.append(r)
            if (k + 1) % 250 == 0:
                print(f"{k + 1}/{len(todo)}  {time.time() - t0:.0f} s", flush=True)
    res.sort(key=lambda r: (r["study"], r["i"]))             # completion order differs from run to run
    json.dump(dict(ranges=RANGES, slip_ok=V.SLIP_OK, creep_grid=dict(mu=MU_GRID.tolist(), base=list(base_grid)),
                   runtime_s=time.time() - t0, results=res),
              open(os.path.join(S.B.OUT, "robustness_cpu.json"), "w"))
    for st in ("spec", "stress"):
        rr = [r for r in res if r["study"] == st]
        if rr:
            u = np.array([r["util_max"] for r in rr])
            print(f"{st}: {sum(r['held'] for r in rr)}/{len(rr)} held, peak friction use median {np.median(u):.3f}, "
                  f"max {u.max():.3f}, dropped {sum(r['dropped'] for r in rr)}")
    plot(res, os.path.join(HERE, "figures", "robustness_cpu.png"))


def saturated(n=200, workers=6):
    """Extra check outside the standard climate: out/robustness_saturated.json, the main study stays as it is."""
    base = json.load(open(os.path.join(S.B.OUT, "robustness.json")))["creep_grid"]["base"]
    with Pool(workers) as pool:
        res = pool.map(one, [(s, base) for s in sample("saturated", n, 3)])
    json.dump(dict(range=RANGES["saturated"], results=res), open(os.path.join(S.B.OUT, "robustness_saturated_cpu.json"), "w"))
    u = np.array([r["util_max"] for r in res])
    print(f"saturated: {sum(r['held'] for r in res)}/{n} held, peak friction use median {np.median(u):.3f}, "
          f"max {u.max():.3f} (safety {1 / u.max():.2f})")


def _width_case(a):
    dw, off = a
    r, lg = V.run("width", F0, offset=off, width_delta=dw * MM, creep_ref=None)
    t, nl, nr = np.array(lg["t"]), np.array(lg["nL"]), np.array(lg["nR"])
    t_lift = lg["phases"]["lift"]
    pre = t < t_lift                                              # closing only: the lift starts at 0.40 s (E4)
    band = (np.abs(nl - F0) < 0.05 * F0) & (np.abs(nr - F0) < 0.05 * F0)
    # first instant from which both jaws stay within +-5 % of the setpoint until the lift
    t_grip = next((float(t[i]) for i in np.nonzero(pre)[0] if band[i:np.count_nonzero(pre)].all()), None)
    i_l = np.count_nonzero(pre) - 1
    return dict(width_mm=dw, offset_mm=off / MM, f_left=float(nl[i_l]), f_right=float(nr[i_l]), t_grip=t_grip,
                util_max=r["util_max"], held=r["held"], dropped=r["dropped"])


def width(workers=6):
    """Part B wider or thinner across the grip faces (nominal 70 mm, ISO 2768-m +-0.3 mm), centred and 5 mm off
    centre: does the two-stage profile still build the grip force in time? -> out/width_tolerance.json"""
    cases = [(dw, off) for dw in (-2.0, -1.5, -1.25, -1.0, -0.75, -0.5, -0.3, 0.0, 0.3, 0.5, 1.0) for off in (0.0, TOL)]
    with Pool(workers) as pool:
        res = pool.map(_width_case, cases)
    json.dump(res, open(os.path.join(S.B.OUT, "width_tolerance_cpu.json"), "w"), indent=1)
    for r in res:
        tg = f"{r['t_grip'] * 1000:5.0f} ms" if r["t_grip"] is not None else "not before the lift"
        print(f"width {r['width_mm']:+5.2f} mm  offset {r['offset_mm']:3.0f} mm  jaws {r['f_left']:5.1f} / {r['f_right']:5.1f} N "
              f"at the lift, both within 5 % from {tg}  peak use {r['util_max']:.3f}  "
              f"{'held' if r['held'] else 'SLIPS' + (' / DROPS' if r['dropped'] else '')}")


def corners(workers=6):
    """The 16 joint extremes of the spec ranges (Latin hypercube strata do not hit them together), at the start of
    the transfer and with the full E4 timing -> out/robustness_corners.json"""
    rg = RANGES["spec"]
    keys = ("offset", "mu", "force", "mass")
    base = json.load(open(os.path.join(S.B.OUT, "robustness.json")))["creep_grid"]["base"]
    todo = [dict(study="corner", i=i, **{k: rg[k][(i >> j) & 1] for j, k in enumerate(keys)}) for i in range(16)]
    with Pool(workers) as pool:
        res = pool.map(one, [(c, base) for c in todo])
    json.dump(res, open(os.path.join(S.B.OUT, "robustness_corners_cpu.json"), "w"), indent=1)
    for r in sorted(res, key=lambda r: -r["util_max"]):
        print(f"offset {r['offset'] * 1000:+.0f} mm  mu {r['mu']:.2f}  F {r['force']:.1f} N  mass x{r['mass']:.2f}:  peak use "
              f"{r['util_max']:.3f} (S {1 / r['util_max']:.2f})  worst pad {r['util_pad_raw']:.2f}  {'held' if r['held'] else 'SLIPS'}")


if __name__ == "__main__":
    if "--corners" in sys.argv:
        corners()
    elif "--width" in sys.argv:
        width()
    elif "--saturated" in sys.argv:
        saturated()
    else:
        main(*[int(x) for x in sys.argv[1:3]])
