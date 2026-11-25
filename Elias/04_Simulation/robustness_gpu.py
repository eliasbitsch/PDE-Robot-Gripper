"""Monte Carlo grip study on the GPU (MuJoCo Warp): every sample is one world, all worlds step at once.

python robustness_gpu.py [n_spec] [n_stress]   ->  out/robustness.json, figures/robustness.png (the study)
python robustness_gpu.py --corners | --saturated | --width   ->  the extra checks, as robustness.py writes them
python robustness_gpu.py --check [n]            ->  n samples on GPU and CPU (validate_grip.run), side by side

One cycle as validate_grip.run(offset=...) runs it: built two-stage close from fully open, lift and lower on the
loaded time law, transfer through the via to chain 2. Per world: pad friction, grip force limit, part mass and
inertia, part offset across the grip axis. "Held" as on the CPU: both pads in contact and each below its friction
limit at every step, not dropped, movement beyond the creep base < SLIP_OK. MuJoCo Warp computes in float32,
so --check compares it against the float64 CPU model before the study is trusted.
"""
import json
import os
import sys
import time

import mujoco
import numpy as np
import warp as wp
import mujoco_warp as mjw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closing_time as CT  # noqa: E402
import e4_cycle as E  # noqa: E402
import robustness as R  # noqa: E402
import sim as S  # noqa: E402
import validate_grip as V  # noqa: E402

GP, MM = S.GP, S.MM
DT, LOG_EVERY = 0.0005, 10                       # s per step; log every 10 steps = 5 ms, as validate_grip
BATCH = int(os.environ.get("GPU_BATCH", 6144))      # worlds per GPU batch; lower it when other apps hold GPU memory


def plan(m, d):
    """Control trajectory and phase times of one cycle (validate_grip.run with offset, chain 2, no e-stop)."""
    q0, _ = S.q_pick()
    q1, _ = S.q_pick(V.LIFT)
    q2, _ = S.q_place(V.LIFT, +1)
    q3, _ = S.q_place(0.0, +1)
    vmax = tuple(E.SPEED_USE * E.VMAX)
    path = S.transfer_path(V.LIFT, +1)
    law = S.TimeLaw(path, GP.A_PATH_REAL, vmax_deg=vmax, v_lim=E.V_LOADED)
    lu, ld = E.line(q0, q1), E.line(q2, q3)
    law_up = S.TimeLaw(lu, GP.A_PATH_REAL, m=m, d=d, vmax_deg=vmax, v_lim=E.V_LOADED)
    law_dn = S.TimeLaw(ld, GP.A_PATH_REAL, m=m, d=d, vmax_deg=vmax, v_lim=E.V_LOADED)
    close_ref = CT.profile("tolerant", CT.T_FAST_BUILT, CT.V_TOUCH_BUILT)
    t_lift0 = E.T_CLOSE
    t_tr0 = t_lift0 + law_up.T
    t_low0 = t_tr0 + law.T
    t_end = t_low0 + law_dn.T + 0.3
    n = int(t_end / DT)
    ja = m.actuator("jaw").id
    ctrl = np.zeros((n, m.nu), np.float32)
    for k in range(n):
        t = k * DT
        if t < t_lift0:
            q = q0
        elif t < t_tr0:
            q = lu(law_up(t - t_lift0)[0])
        elif t < t_low0:
            q = path(min(max(law(t - t_tr0)[0], 0.0), 1.0))
        else:
            q = ld(law_dn(t - t_low0)[0])
        ctrl[k, :6] = q
        ctrl[k, ja] = close_ref(t)
    return dict(ctrl=ctrl, n=n, q0=q0, t_lift0=t_lift0, t_tr0=t_tr0, t_low0=t_low0, t_win1=t_low0 + law_dn.T,
                t_ref=t_lift0 - 0.02)


@wp.kernel
def set_ctrl(traj: wp.array2d(dtype=float), kk: wp.array(dtype=int), ctrl: wp.array2d(dtype=float)):
    w, j = wp.tid()
    ctrl[w, j] = traj[wp.min(kk[0], traj.shape[0] - 1), j]


@wp.kernel
def advance(kk: wp.array(dtype=int)):
    kk[0] = kk[0] + 1


@wp.kernel
def pad_sum(nacon: wp.array(dtype=int), geom: wp.array(dtype=wp.vec2i), worldid: wp.array(dtype=int),
            frame: wp.array(dtype=wp.mat33), force: wp.array(dtype=wp.spatial_vector), pad_l: int, pad_r: int,
            part: int, fn: wp.array2d(dtype=float), ft: wp.array2d(dtype=wp.vec3)):
    i = wp.tid()
    if i >= nacon[0]:
        return
    g = geom[i]
    side = int(-1)
    if (g[0] == pad_l and g[1] == part) or (g[1] == pad_l and g[0] == part):
        side = 0
    if (g[0] == pad_r and g[1] == part) or (g[1] == pad_r and g[0] == part):
        side = 1
    if side < 0:
        return
    f = wp.spatial_top(force[i])                 # contact frame: normal, tangent 1, tangent 2
    fr = frame[i]
    t1 = wp.vec3(fr[1, 0], fr[1, 1], fr[1, 2])
    t2 = wp.vec3(fr[2, 0], fr[2, 1], fr[2, 2])
    w = worldid[i]
    wp.atomic_add(fn, w, side, f[0])
    wp.atomic_add(ft, w, side, f[1] * t1 + f[2] * t2)


@wp.kernel
def per_world(fn: wp.array2d(dtype=float), ft: wp.array2d(dtype=wp.vec3), mu: wp.array(dtype=float),
              kk: wp.array(dtype=int), k0: int, k1: int, pad_raw: wp.array(dtype=float), lost: wp.array(dtype=int),
              util_now: wp.array(dtype=float)):
    w = wp.tid()
    n = fn[w, 0] + fn[w, 1]
    if kk[0] >= k0 and kk[0] < k1:              # every step from the lift to the end of lowering
        for s in range(2):
            if fn[w, s] <= 1.0e-6:
                lost[w] = 1
            else:
                pad_raw[w] = wp.max(pad_raw[w], wp.length(ft[w, s]) / (mu[w] * fn[w, s]))
    util_now[w] = 0.0
    if n > 1.0e-6:
        util_now[w] = wp.length(ft[w, 0] + ft[w, 1]) / (mu[w] * n)


@wp.kernel
def log_row(util_now: wp.array(dtype=float), xpos: wp.array2d(dtype=wp.vec3), xmat: wp.array2d(dtype=wp.mat33),
            bg: int, bp: int, kk: wp.array(dtype=int), fn: wp.array2d(dtype=float), util_log: wp.array2d(dtype=float),
            rel_log: wp.array2d(dtype=wp.vec3), z_log: wp.array2d(dtype=float), fn_log: wp.array3d(dtype=float)):
    w = wp.tid()
    if kk[0] % 10 != 0 or kk[0] // 10 >= util_log.shape[0]:
        return
    row = kk[0] // 10
    util_log[row, w] = util_now[w]
    rel_log[row, w] = wp.transpose(xmat[w, bg]) * (xpos[w, bp] - xpos[w, bg])
    z_log[row, w] = xpos[w, bp][2]
    fn_log[row, w, 0] = fn[w, 0]
    fn_log[row, w, 1] = fn[w, 1]


def run_batch(m0, pl, samples):
    """samples: dicts with offset [m], mu, force [N per jaw], mass (factor). Returns per-sample raw logs."""
    n = len(samples)
    m = mujoco.MjModel.from_xml_path(os.path.join(S.B.OUT, "cell.xml"))
    m.opt.timestep = DT
    d = mujoco.MjData(m)
    S.set_robot(m, d, pl["q0"], jaw=GP.D_OPEN * MM)
    S.place_part_in_gripper(m, d)
    pb, ja = m.body("part").id, m.actuator("jaw").id
    wide = any(s.get("width", 0.0) for s in samples)
    batch = {"pair_friction": n, "actuator_forcerange": n, "body_mass": n, "body_inertia": n}
    if wide:
        batch.update(geom_size=n, geom_aabb=n, geom_rbound=n)
    wm = mjw.put_model(m, batch_sizes=batch)
    if wide:                                     # part wider (+) or thinner (-) across the grip faces, contact box only
        gp = m.geom("part").id
        gs, ga, gr = wm.geom_size.numpy(), wm.geom_aabb.numpy(), wm.geom_rbound.numpy()
        for w, s in enumerate(samples):
            gs[w, gp, 0] += s.get("width", 0.0) * MM / 2
            ga[w, gp, 1, 0] += s.get("width", 0.0) * MM / 2
            gr[w, gp] = float(np.linalg.norm(gs[w, gp]))
        wm.geom_size.assign(gs)
        wm.geom_aabb.assign(ga)
        wm.geom_rbound.assign(gr)
    pf = wm.pair_friction.numpy()
    fr = wm.actuator_forcerange.numpy()
    bm, bi = wm.body_mass.numpy(), wm.body_inertia.numpy()
    for w, s in enumerate(samples):
        pf[w, :, 0:2] = s["mu"]
        fr[w, ja] = (-2 * s["force"], 2 * s["force"])
        bm[w, pb] = m.body_mass[pb] * s["mass"]
        bi[w, pb] = m.body_inertia[pb] * s["mass"]
    wm.pair_friction.assign(pf)
    wm.actuator_forcerange.assign(fr)
    wm.body_mass.assign(bm)
    wm.body_inertia.assign(bi)
    wd = mjw.put_data(m, d, nworld=n, nconmax=32, njmax=96)     # a cycle needs at most 15 contacts, 46 rows
    qpos = wd.qpos.numpy()
    xg = d.xmat[m.body("gripper_frame").id].reshape(3, 3)[:, 0]
    adr = m.joint("part").qposadr[0]
    for w, s in enumerate(samples):
        qpos[w, adr:adr + 3] += xg * s["offset"]
    wd.qpos.assign(qpos)
    mjw.forward(wm, wd)
    traj = wp.array(pl["ctrl"], dtype=float)
    mu = wp.array(np.array([s["mu"] for s in samples], np.float32), dtype=float)
    nlog = pl["n"] // LOG_EVERY + 1
    util_log = wp.zeros((nlog, n), dtype=float)
    rel_log = wp.zeros((nlog, n), dtype=wp.vec3)
    z_log = wp.zeros((nlog, n), dtype=float)
    fn_log = wp.zeros((nlog, n, 2), dtype=float)
    fn = wp.zeros((n, 2), dtype=float)
    ft = wp.zeros((n, 2), dtype=wp.vec3)
    pad_raw = wp.zeros(n, dtype=float)
    lost = wp.zeros(n, dtype=int)
    util_now = wp.zeros(n, dtype=float)
    naconmax = wd.contact.worldid.shape[0]
    cids = wp.array(np.arange(naconmax, dtype=np.int32), dtype=int)
    cforce = wp.zeros(naconmax, dtype=wp.spatial_vector)
    ids = (m.geom("pad_L").id, m.geom("pad_R").id, m.geom("part").id)
    bg, bpart = m.body("gripper_frame").id, pb
    kk = wp.zeros(1, dtype=int)
    k0, k1 = int(np.ceil(pl["t_lift0"] / DT - 1e-9)), int(np.ceil(pl["t_win1"] / DT - 1e-9))

    def one_step():
        wp.launch(set_ctrl, dim=(n, m.nu), inputs=[traj, kk, wd.ctrl])
        mjw.step(wm, wd)
        fn.zero_()
        ft.zero_()
        mjw.contact_force(wm, wd, cids, False, cforce)
        wp.launch(pad_sum, dim=naconmax, inputs=[wd.nacon, wd.contact.geom, wd.contact.worldid, wd.contact.frame,
                                                 cforce, *ids, fn, ft])
        wp.launch(per_world, dim=n, inputs=[fn, ft, mu, kk, k0, k1, pad_raw, lost, util_now])
        wp.launch(log_row, dim=n, inputs=[util_now, wd.xpos, wd.xmat, bg, bpart, kk, fn, util_log, rel_log, z_log,
                                          fn_log])
        wp.launch(advance, dim=1, inputs=[kk])
    one_step()                                   # compile and warm up outside the capture (step 0)
    # the per-step launches recorded once as a CUDA graph and replayed: no Python per step
    with wp.ScopedCapture() as cap:
        for _ in range(LOG_EVERY):
            one_step()
    for _ in range((pl["n"] - 1 + LOG_EVERY - 1) // LOG_EVERY):
        wp.capture_launch(cap.graph)
    wp.synchronize()
    return dict(util=util_log.numpy(), rel=rel_log.numpy(), z=z_log.numpy(), fn=fn_log.numpy(), pad_raw=pad_raw.numpy(),
                lost=lost.numpy(), t=np.arange(nlog) * LOG_EVERY * DT)


def evaluate(pl, raw, samples, creep):
    """Same reduction as validate_grip.run: 50 ms moving average, window lift ... end of lowering."""
    tt = raw["t"]
    win = (tt > pl["t_lift0"]) & (tt < pl["t_win1"])
    i_ref = int(np.searchsorted(tt, pl["t_ref"] - 1e-9))
    k = max(1, int(0.05 / (LOG_EVERY * DT)))
    out = []
    for w, s in enumerate(samples):
        u_s = np.convolve(raw["util"][:, w], np.ones(k) / k, mode="same")
        u_max = float(u_s[win].max())
        slip = np.linalg.norm(raw["rel"][:, w] - raw["rel"][i_ref, w], axis=1) / MM
        base = creep(s["mu"]) if creep is not None else 0.0
        excess = float(slip[win].max()) - base
        dropped = bool(raw["z"][-1, w] / MM < S.CP.H_PLACE - 50)
        lost = bool(raw["lost"][w])
        held = bool(not dropped and not lost and raw["pad_raw"][w] < V.PAD_LIMIT and excess < V.SLIP_OK)
        t_peak = float(tt[win][np.argmax(u_s[win])])
        out.append(dict(s, util_max=u_max, util_median=float(np.median(raw["util"][win, w])),
                        util_pad_raw=float(raw["pad_raw"][w]), lost_contact=lost, slip_max=float(slip[win].max()),
                        slip_excess=excess, creep_base=base, held=held, dropped=dropped, t_peak=t_peak,
                        phase_peak="lift" if t_peak < pl["t_tr0"] else "transfer" if t_peak < pl["t_low0"] else "lower"))
    return out


def study(samples):
    m, d = S.load()
    pl = plan(m, d)
    # creep base per friction (100 N reference), as robustness.creep
    refs = [dict(offset=0.0, mu=float(mu), force=V.F_REF, mass=1.0) for mu in R.MU_GRID]
    base = [r["slip_max"] for r in evaluate(pl, run_batch(m, pl, refs), refs, None)]
    creep = lambda mu: float(np.interp(mu, R.MU_GRID, base))
    res = []
    for a in range(0, len(samples), BATCH):
        chunk = samples[a:a + BATCH]
        t0 = time.time()
        res += evaluate(pl, run_batch(m, pl, chunk), chunk, creep)
        print(f"{a + len(chunk)}/{len(samples)} worlds, batch {time.time() - t0:.0f} s", flush=True)
    return res, base


def check(n=24):
    """GPU vs CPU on the same samples (spec and stress mixed)."""
    samples = (R.sample("spec", n // 2, 11) + R.sample("stress", n - n // 2, 12))
    res, base = study(samples)
    print(f"creep base GPU {np.round(base, 2).tolist()}")
    rows = []
    for g in res:
        c, _ = V.run("check", g["force"], mu=g["mu"], offset=g["offset"], mass_scale=g["mass"], creep_ref=None)
        rows.append((g["util_max"], c["util_max"], g["util_pad_raw"], c["util_pad_raw"], g["held"], c["held"]))
        print(f"mu {g['mu']:.3f} F {g['force']:5.1f}  util GPU {g['util_max']:.3f} CPU {c['util_max']:.3f}  "
              f"pad GPU {g['util_pad_raw']:.3f} CPU {c['util_pad_raw']:.3f}  held GPU {g['held']} CPU {c['held']}")
    r = np.array(rows, float)
    print(f"util max |GPU - CPU| {np.abs(r[:, 0] - r[:, 1]).max():.4f}, pad {np.abs(r[:, 2] - r[:, 3]).max():.4f}, "
          f"held agree {int((r[:, 4] == r[:, 5]).sum())}/{len(r)}")


def corners():
    """The 16 joint extremes of the spec ranges -> out/robustness_corners.json (as robustness.corners)."""
    rg, keys = R.RANGES["spec"], ("offset", "mu", "force", "mass")
    todo = [dict(study="corner", i=i, **{k: rg[k][(i >> j) & 1] for j, k in enumerate(keys)}) for i in range(16)]
    res, _ = study(todo)
    json.dump(res, open(os.path.join(S.B.OUT, "robustness_corners.json"), "w"), indent=1)
    for r in sorted(res, key=lambda r: -r["util_max"])[:4]:
        print(f"offset {r['offset'] * 1000:+.0f} mm mu {r['mu']:.2f} F {r['force']:.1f} N mass x{r['mass']:.2f}: peak "
              f"{r['util_max']:.3f} (S {1 / r['util_max']:.2f}) worst pad {r['util_pad_raw']:.2f} {'held' if r['held'] else 'SLIPS'}")
    print(f"corners: {sum(r['held'] for r in res)}/16 held")


def saturated(n=200):
    """Part as if water-saturated (+9 ... +10 % mass and inertia) -> out/robustness_saturated.json."""
    todo = R.sample("saturated", n, 3)
    res, _ = study(todo)
    json.dump(dict(range=R.RANGES["saturated"], results=res), open(os.path.join(S.B.OUT, "robustness_saturated.json"), "w"))
    u = np.array([r["util_max"] for r in res])
    print(f"saturated: {sum(r['held'] for r in res)}/{n} held, peak median {np.median(u):.3f}, max {u.max():.3f} "
          f"(S {1 / u.max():.2f})")


def width():
    """Part B 2 mm thinner ... 1 mm wider, centred and 5 mm off centre -> out/width_tolerance.json (robustness.width)."""
    m, d = S.load()
    pl = plan(m, d)
    cases = [dict(width=dw, offset=off, mu=GP.MU_OILY, force=R.F0, mass=1.0)
             for dw in (-2.0, -1.5, -1.25, -1.0, -0.75, -0.5, -0.3, 0.0, 0.3, 0.5, 1.0) for off in (0.0, R.TOL)]
    refs = [dict(offset=0.0, mu=GP.MU_OILY, force=V.F_REF, mass=1.0)]
    base = evaluate(pl, run_batch(m, pl, refs), refs, None)[0]["slip_max"]
    raw = run_batch(m, pl, cases)
    ev = evaluate(pl, raw, cases, lambda mu: base)
    tt = raw["t"]
    pre = tt < pl["t_lift0"]
    i_l = int(np.count_nonzero(pre)) - 1
    out = []
    for w, (c, e) in enumerate(zip(cases, ev)):
        nl, nr = raw["fn"][:, w, 0], raw["fn"][:, w, 1]
        band = (np.abs(nl - R.F0) < 0.05 * R.F0) & (np.abs(nr - R.F0) < 0.05 * R.F0)
        t_grip = next((float(tt[i]) for i in range(i_l + 1) if band[i:i_l + 1].all()), None)
        out.append(dict(width_mm=c["width"], offset_mm=c["offset"] / MM, f_left=float(nl[i_l]), f_right=float(nr[i_l]),
                        t_grip=t_grip, util_max=e["util_max"], held=e["held"], dropped=e["dropped"]))
    json.dump(out, open(os.path.join(S.B.OUT, "width_tolerance.json"), "w"), indent=1)
    for r in out:
        tg = f"{r['t_grip'] * 1000:5.0f} ms" if r["t_grip"] is not None else "not before the lift"
        print(f"width {r['width_mm']:+5.2f} mm offset {r['offset_mm']:3.0f} mm  jaws {r['f_left']:5.1f} / {r['f_right']:5.1f} N "
              f"at the last sample before the lift, both within 5 % from {tg}  {'held' if r['held'] else 'SLIPS'}")


def main(n_spec=10000, n_stress=2000):
    t0 = time.time()
    todo = R.sample("spec", n_spec, 1) + R.sample("stress", n_stress, 2)
    res, base = study(todo)
    res.sort(key=lambda r: (r["study"], r["i"]))
    json.dump(dict(ranges=R.RANGES, slip_ok=V.SLIP_OK, creep_grid=dict(mu=R.MU_GRID.tolist(), base=list(base)),
                   runtime_s=time.time() - t0, device="cuda (MuJoCo Warp, float32)", results=res),
              open(os.path.join(S.B.OUT, "robustness.json"), "w"))
    R.plot(res, os.path.join(HERE, "figures", "robustness.png"))
    for st in ("spec", "stress"):
        rr = [r for r in res if r["study"] == st]
        u = np.array([r["util_max"] for r in rr])
        print(f"{st}: {sum(r['held'] for r in rr)}/{len(rr)} held, peak friction use median {np.median(u):.3f}, "
              f"max {u.max():.3f}, dropped {sum(r['dropped'] for r in rr)}  ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    wp.config.quiet = True
    if "--check" in sys.argv:
        check(int(sys.argv[2]) if len(sys.argv) > 2 else 24)
    elif "--corners" in sys.argv:
        corners()
    elif "--saturated" in sys.argv:
        saturated()
    elif "--width" in sys.argv:
        width()
    else:
        main(*[int(x) for x in sys.argv[1:3]])
