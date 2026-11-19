"""FEM of the jaw train: slide + jaw holder + jaw V3 (right side), all load cases of load_cases.py.

python fem_train.py [lc]        mesh once, solve LC1-LC5, write out/train_results.json and figures/train_*.png
Reuses solver, reader and C3D10 conversion of fem_jaw.py. Contact faces are merged by gmsh (bonded).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import fem_jaw as J  # noqa: E402
import load_cases as L  # noqa: E402
P = J.P

STEP = os.path.join(HERE, "train_v3.step")       # v3_import.py: slide, holder (export_train.py) + Viktoriia's jaw V3
# Supports: the carriage holds the slide in y and z only (it runs along x on the rail); the grip force along x goes
# through the rack into the pinion, i.e. through the rack pin (dia 3, the screws sit in slots) into the slide.
BC = "rack"                                      # "fixed": old model, carriage patch fixed in x too
NAMES = ["slide", "holder", "jaw"]
TOL = 1e-3

# Carriage contact patch on the slide top and pad pocket floor of the right jaw (see export_train.py)
PATCH = (P.CARR_X0 - P.CARRIAGE_L_BODY / 2, P.CARR_X0 + P.CARRIAGE_L_BODY / 2,
         P.RAIL_Y - P.CARRIAGE_W / 2, P.RAIL_Y + P.CARRIAGE_W / 2, P.SLIDE_Z_TOP)
X_FLOOR = P.PAD_X0 + P.PAD_T + P.PAD_PLATE_T
PAD_C = np.array([P.PAD_X0, 0.0, sum(P.PAD_Z) / 2])


def mesh(lc):
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    vols = gmsh.model.occ.importShapes(STEP)
    vols, _ = gmsh.model.occ.fragment(vols, [])              # shared contact faces -> conformal (bonded)
    gmsh.model.occ.synchronize()
    order = sorted(vols, key=lambda v: gmsh.model.getBoundingBox(*v)[0])   # slide (x 0.8), jaw (36), holder (44)
    tags = {"slide": order[0][1], "jaw": order[1][1], "holder": order[2][1]}
    gmsh.option.setNumber("Mesh.MeshSizeMin", lc / 3)
    gmsh.option.setNumber("Mesh.MeshSizeMax", lc)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 2)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.model.mesh.generate(3)
    nt, nc, _ = gmsh.model.mesh.getNodes()
    nodes = {int(t): nc[3 * i:3 * i + 3] for i, t in enumerate(nt)}
    elems, part_of = {}, {}
    for name, vt in tags.items():
        for typ, et, conn in zip(*gmsh.model.mesh.getElements(3, vt)):
            if typ == 11:
                for e, row in zip(et, conn.reshape(-1, 10)):
                    elems[int(e)] = [int(x) for x in row]
                    part_of[int(e)] = name
    tris = []
    for typ, _, conn in zip(*gmsh.model.mesh.getElements(dim=2)):
        if typ == 9:
            tris += [[int(x) for x in r] for r in conn.reshape(-1, 6)]
    gmsh.finalize()
    used = {n for e in elems.values() for n in e}
    return {t: c for t, c in nodes.items() if t in used}, elems, part_of, tris


def supports(nodes):
    xyz, ids = np.array(list(nodes.values())), np.array(list(nodes.keys()))
    x0, x1, y0, y1, z = PATCH
    sel = (np.abs(xyz[:, 2] - z) < TOL) & (xyz[:, 0] > x0 - TOL) & (xyz[:, 0] < x1 + TOL) \
        & (xyz[:, 1] > y0 - TOL) & (xyz[:, 1] < y1 + TOL)
    if not sel.any():
        raise RuntimeError("no nodes on the carriage patch")
    return ids[sel].tolist()


def pad_weights(nodes, tris):
    w, area = {}, 0.0
    for tri in tris:
        c = np.array([nodes[n] for n in tri if n in nodes])
        if len(c) < 6 or not np.all(np.abs(c[:, 0] - X_FLOOR) < TOL):
            continue
        if not (np.all(c[:, 2] > P.PAD_Z[0] - TOL) and np.all(c[:, 2] < P.PAD_Z[1] + TOL)):
            continue
        a = 0.5 * np.linalg.norm(np.cross(c[1] - c[0], c[2] - c[0]))
        area += a
        for n in tri[3:6]:
            w[n] = w.get(n, 0.0) + a / 3.0
    if not w:
        raise RuntimeError("no pad face found")
    return {n: v / area for n, v in w.items()}, area


# The rack is not meshed: pin bore and the boss the rack sits on move as one rigid body (CalculiX *RIGID BODY),
# held in x at the pitch line of the mesh with the pinion, where the rack force really acts.
RACK_REF = (0.0, -P.PINION_D / 2, P.PINION_Z0 + P.PINION_B / 2)     # x at the pinion, pitch line, tooth middle


def rack_nodes(nodes):
    px, py = P.RACK_PIN_XS[1], -(P.PINION_D / 2 + 2.25 + (P.RACK_H - 2.25) / 2)   # = parts.core.rack_pin(1)
    xyz, ids = np.array(list(nodes.values())), np.array(list(nodes.keys()))
    pin = (np.abs(np.hypot(xyz[:, 0] - px, xyz[:, 1] - py) - 1.5) < 0.02) & (xyz[:, 2] < P.SLIDE_Z_TOP + P.RACK_GAP + TOL)
    ry = -(P.PINION_D / 2 - 1.0)
    boss = (np.abs(xyz[:, 2] - (P.SLIDE_Z_TOP + P.RACK_GAP)) < TOL) & (xyz[:, 1] < ry + TOL) & (xyz[:, 1] > ry - P.RACK_H - TOL)
    if not pin.any() or not boss.any():
        raise RuntimeError(f"rack support: pin {pin.sum()}, boss {boss.sum()} nodes")
    return ids[pin | boss].tolist()


def reactions(path, nodes, ref):
    """Sum of the support reactions (RF from the .dat) per node set: force and moment about `ref` (N, N*mm)."""
    out, cur = {}, None
    for line in open(path, errors="replace"):
        if "forces (fx,fy,fz) for set" in line:
            cur = line.split("set")[1].split()[0].strip()
            out[cur] = {"F": np.zeros(3), "M": np.zeros(3)}
            continue
        parts = line.split()
        if cur and len(parts) == 4:
            try:
                n, f = int(parts[0]), np.array([float(v) for v in parts[1:]])
            except ValueError:
                continue
            out[cur]["F"] += f
            out[cur]["M"] += np.cross(np.asarray(nodes[n] if n in nodes else RACK_REF) - ref, f)
        elif cur and parts and not parts[0].lstrip("-").replace(".", "").isdigit():
            cur = None
    return {k: {"F": v["F"].tolist(), "M": v["M"].tolist()} for k, v in out.items()}


def write_inp(path, nodes, elems, part_of, fixed, weights, f, pin=None):
    mats = {"slide": "AL6082", "holder": "AL6082", "jaw": "AL7075"}
    props = {"AL6082": L.MATERIALS["EN AW-6082 T6"], "AL7075": L.MATERIALS["EN AW-7075 T6"]}
    z = ["*HEADING", "Jaw train, group 10", "*NODE"]
    z += [f"{t}, {c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}" for t, c in sorted(nodes.items())]
    for name in NAMES:
        es = sorted(e for e, p in part_of.items() if p == name)
        z.append(f"*ELEMENT, TYPE=C3D10, ELSET=E{NAMES.index(name)}")
        z += [f"{e}, " + ", ".join(map(str, J.to_c3d10(elems[e], nodes))) for e in es]
    z.append("*NSET, NSET=NFIX")
    z += [", ".join(map(str, fixed[i:i + 8])) for i in range(0, len(fixed), 8)]
    for m, pr in props.items():
        z += [f"*MATERIAL, NAME={m}", "*ELASTIC", f"{pr['E']}, {pr['nu']}"]
    for name in NAMES:
        z.append(f"*SOLID SECTION, ELSET=E{NAMES.index(name)}, MATERIAL={mats[name]}")
    if pin:
        ref = max(nodes) + 1
        z += ["*NODE", f"{ref}, {RACK_REF[0]}, {RACK_REF[1]}, {RACK_REF[2]}", f"{ref + 1}, {RACK_REF[0]}, {RACK_REF[1]}, {RACK_REF[2]}"]
        z += ["*NSET, NSET=NRACK"] + [", ".join(map(str, pin[i:i + 8])) for i in range(0, len(pin), 8)]
        z += ["*NSET, NSET=NPIN", str(ref), f"*RIGID BODY, NSET=NRACK, REF NODE={ref}, ROT NODE={ref + 1}"]
    z += ["*STEP", "*STATIC", "*BOUNDARY"]
    z += ["NFIX, 2, 3, 0.0", "NPIN, 1, 1, 0.0"] if pin else ["NFIX, 1, 3, 0.0"]
    z += ["*NODE PRINT, NSET=NFIX, TOTALS=NO", "RF"] + (["*NODE PRINT, NSET=NPIN, TOTALS=NO", "RF"] if pin else [])
    z.append("*CLOAD")
    for n, w in sorted(weights.items()):
        for dof, comp in enumerate(f, 1):
            if comp:
                z.append(f"{n}, {dof}, {comp * w:.8f}")
    z += ["*NODE FILE", "U, S", "*END STEP"]
    with open(path, "w") as fh:
        fh.write("\n".join(z) + "\n")


def run(lc=2.0, only=None, tag="train", png=os.path.join(HERE, "figures", "train_lc2_von_mises.png")):
    """Mesh once and solve the load cases whose name starts with one of `only` (all if None)."""
    import time
    os.makedirs(J.OUT, exist_ok=True)
    t0 = time.time()
    nodes, elems, part_of, tris = mesh(lc)
    t_mesh = time.time() - t0
    fixed = supports(nodes)
    pin = rack_nodes(nodes) if BC == "rack" else None
    carriage_c = np.array([P.CARR_X0, P.RAIL_Y, P.SLIDE_Z_TOP])
    weights, area = pad_weights(nodes, tris)
    _yw = [nodes[n][1] for n in weights]
    y_pad = (min(_yw), max(_yw))
    node_part = {}
    for e, p in part_of.items():
        for n in elems[e]:
            node_part.setdefault(n, p)
    xyz_all = {t: np.asarray(c) for t, c in nodes.items()}
    results = []
    t_solve = 0.0
    for name, fx, fy, fz, s_req, what in L.LOAD_CASES:
        if only and not name.startswith(tuple(only)):
            continue
        base = f"{tag}_" + name.split()[0].lower()
        t1 = time.time()
        write_inp(os.path.join(J.OUT, base + ".inp"), nodes, elems, part_of, fixed, weights, (fx, fy, fz), pin)
        J.solve(base)
        t_solve += time.time() - t1
        b = J.read_frd(os.path.join(J.OUT, base + ".frd"))
        rf = reactions(os.path.join(J.OUT, base + ".dat"), nodes, carriage_c)
        J.cleanup(base)
        U, S = b["DISP"], b["STRESS"]
        ids = list(U)
        xyz = np.array([xyz_all[t] for t in ids])
        u = np.array([U[t][:3] for t in ids])
        vm = np.array([J.mises(S[t][:6]) for t in ids])
        near_fix = np.abs(xyz[:, 2] - PATCH[4]) < 1.5          # support edge singularity, not a design stress
        per_part = {}
        for part in NAMES:
            m = np.array([node_part.get(t) == part for t in ids]) & ~near_fix
            i = int(np.argmax(np.where(m, vm, 0)))
            rp = L.MATERIALS[L.PARTS[part]]["Rp02"]
            per_part[part] = dict(vm_peak=float(vm[i]), at=[round(float(v), 1) for v in xyz[i]],
                                  safety=float(rp / vm[i]), ok=bool(rp / vm[i] >= s_req))
        # pad zone on the pocket floor; plane fit u_x(y, z) over all its nodes, so the measure does not depend on
        # the element size (needed for the convergence study)
        pad = (np.abs(xyz[:, 0] - X_FLOOR) < TOL) & (xyz[:, 2] > P.PAD_Z[0] - TOL) & (xyz[:, 2] < P.PAD_Z[1] + TOL)             & (xyz[:, 1] > y_pad[0] - TOL) & (xyz[:, 1] < y_pad[1] + TOL)
        A = np.c_[np.ones(pad.sum()), xyz[pad, 1], xyz[pad, 2]]
        coef, *_ = np.linalg.lstsq(A, u[pad, 0], rcond=None)
        tilt = float(np.degrees(np.arctan(-coef[2])))
        ux_c = float(coef[0] + coef[2] * PAD_C[2])
        # strain energy = work of the load (N*mm): converges monotonically from below, the classic convergence measure
        energy = 0.5 * sum(w * float(np.dot((fx, fy, fz), U[n][:3])) for n, w in weights.items())
        results.append(dict(case=name, F=[fx, fy, fz], s_req=s_req, ux_pad=ux_c,
                            u_pad=float(np.mean(np.linalg.norm(u[pad], axis=1))), tilt_deg=tilt, energy=energy,
                            tilt_ok=bool(abs(tilt) <= L.TILT_MAX), parts=per_part,
                            carriage=rf.get("NFIX"), rack=rf.get("NPIN")))
        if name.startswith("LC2") and png:
            render(nodes, elems, ids, u, vm, png, fx)
    out = dict(bc=BC, step=os.path.basename(STEP), lc=lc, nodes=len(nodes), elements=len(elems), pad_area=area, support_nodes=len(fixed), cases=results,
               t_mesh=t_mesh, t_solve=t_solve)
    return out


def run_all(lc=2.0):
    out = run(lc)
    with open(os.path.join(J.OUT, "train_results.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    return out


def render(nodes, elems, ids, u, vm, path, force, scale=300.0):
    import pyvista as pv
    pv.OFF_SCREEN = True
    index = {t: i for i, t in enumerate(ids)}
    pts0 = np.array([nodes[t] for t in ids])
    cells = np.array([[4] + [index[n] for n in e[:4]] for e in elems.values()]).ravel()
    types = np.full(len(elems), pv.CellType.TETRA, dtype=np.uint8)
    g0 = pv.UnstructuredGrid(cells, types, pts0)
    g = pv.UnstructuredGrid(cells, types, pts0 + scale * u)
    g.point_data["von Mises [MPa]"] = vm
    p = pv.Plotter(off_screen=True, window_size=(1100, 800))
    p.add_mesh(g0, style="wireframe", color="#9aa5ab", opacity=0.12)
    p.add_mesh(g, scalars="von Mises [MPa]", cmap="turbo", clim=(0, float(np.percentile(vm, 99.5))),
               scalar_bar_args=dict(title="von Mises [MPa]", vertical=True, n_labels=5, fmt="%.0f"))
    p.add_text(f"LC2 F = {force:.1f} N per jaw, deformation x{scale:.0f}", font_size=11)
    p.view_vector((1.0, -1.4, 0.5), viewup=(0, 0, 1))
    p.reset_camera()
    p.screenshot(path)
    p.close()


if __name__ == "__main__":
    lc = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    r = run_all(lc)
    print(f"lc {lc} mm: {r['nodes']} nodes, {r['elements']} elements, pad {r['pad_area']:.0f} mm2, "
          f"{r['support_nodes']} support nodes")
    for c in r["cases"]:
        parts = "  ".join(f"{k} {v['vm_peak']:.1f} MPa (S {v['safety']:.1f}{'' if v['ok'] else ' !'})"
                          for k, v in c["parts"].items())
        print(f"{c['case']:20s} u_pad {c['u_pad'] * 1000:6.1f} um  tilt {c['tilt_deg']:.3f} deg   {parts}")
