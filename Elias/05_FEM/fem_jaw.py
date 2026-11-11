"""FEM of the jaw (E5): STEP -> gmsh (C3D10) -> CalculiX -> results.

Model (right jaw, CAD frame, see gripper_params.py):
  support  outer face x = 44 where the holder sits (z = -47 ... -28.5): u_x = 0
           dowel holes (2 x d4 at z = -32): u_y = u_z = 0
  load     grip force as uniform pressure on the pad pocket floor (x = 39), direction +x
Run: python fem_jaw.py            one run at the default mesh + convergence study
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402

OUT = os.path.join(HERE, "out")
STEP = os.path.join(HERE, "jaw.step")                 # from export_jaw.py, committed so FreeCAD is not needed


def find_ccx():
    """CalculiX ships with FreeCAD; set CCX to point elsewhere."""
    cands = [os.environ.get("CCX", ""),
             os.path.expandvars(r"%LOCALAPPDATA%\Programs\FreeCAD 1.1\bin\ccx.exe"),
             r"C:\Program Files\FreeCAD 1.1\bin\ccx.exe", "/usr/bin/ccx"]
    return next((c for c in cands if c and os.path.exists(c)), "ccx")


CCX = find_ccx()

# EN AW-7075 T6, EN 485-2 plate 6-12.5 mm
E, NU, RP02 = 71700.0, 0.33, 460.0

X_SUPPORT = P.PAD_X0 + P.PAD_T + P.JAW_T             # 44, face on the holder
X_FLOOR = P.PAD_X0 + P.PAD_T + P.PAD_PLATE_T          # 39, pocket floor
Z_HOLDER = P.HOLDER_Z_LOW                             # -47, lower end of the holder contact
Z_LOAD = (P.PAD_Z[0], P.PAD_Z[1])                     # -95 ... -60
Z_CHECK = -53.5                                       # clean section between holder and pocket
LOAD_CASES = {"nominal (mu = 0.2)": P.F_GRIP_OILY, "full current": P.F_JAW_MAX}
TOL = 1e-3


def mesh(lc):
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("jaw")
    vols = gmsh.model.occ.importShapes(STEP)
    # Split the volume at the holder edge so the support face ends exactly at z = -47.
    rect = gmsh.model.occ.addRectangle(30, -20, Z_HOLDER, 20, 40)
    gmsh.model.occ.fragment(vols, [(2, rect)])
    gmsh.model.occ.synchronize()
    loose = [e for e in gmsh.model.getEntities(2) if not gmsh.model.getAdjacencies(2, e[1])[0].size]
    if loose:
        gmsh.model.removeEntities(loose, recursive=True)
    gmsh.option.setNumber("Mesh.MeshSizeMin", lc / 2)
    gmsh.option.setNumber("Mesh.MeshSizeMax", lc)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 12)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.HighOrderOptimize", 2)   # curved C3D10 otherwise get negative jacobians
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.model.mesh.generate(3)

    nt, nc, _ = gmsh.model.mesh.getNodes()
    nodes = {int(t): nc[3 * i:3 * i + 3] for i, t in enumerate(nt)}
    elems = {}
    for typ, tags, conn in zip(*gmsh.model.mesh.getElements(dim=3)):
        if typ == 11:
            for tag, row in zip(tags, conn.reshape(-1, 10)):
                elems[int(tag)] = [int(x) for x in row]
    tris = []
    for typ, tags, conn in zip(*gmsh.model.mesh.getElements(dim=2)):
        if typ == 9:
            tris += [[int(x) for x in r] for r in conn.reshape(-1, 6)]
    gmsh.finalize()
    if not elems:
        raise RuntimeError("no C3D10 mesh")
    used = {n for e in elems.values() for n in e}
    nodes = {t: c for t, c in nodes.items() if t in used}
    return nodes, elems, tris


def boundary(nodes, tris, force):
    xyz = np.array(list(nodes.values()))
    ids = np.array(list(nodes.keys()))
    on_support = (np.abs(xyz[:, 0] - X_SUPPORT) < TOL) & (xyz[:, 2] > Z_HOLDER - TOL)
    r_dowel = np.min([np.hypot(xyz[:, 1] - s * P.DOWEL_Y, xyz[:, 2] - P.DOWEL_Z) for s in (-1, 1)], axis=0)
    in_dowel = (np.abs(r_dowel - P.DOWEL_D / 2) < 0.02) & (xyz[:, 0] < X_SUPPORT + TOL)
    fix_x, fix_yz = ids[on_support].tolist(), ids[in_dowel].tolist()

    weight, area = {}, 0.0
    for tri in tris:
        c = np.array([nodes[n] for n in tri if n in nodes])
        if len(c) < 6 or not np.all(np.abs(c[:, 0] - X_FLOOR) < TOL):
            continue
        if not (np.all(c[:, 2] > Z_LOAD[0] - TOL) and np.all(c[:, 2] < Z_LOAD[1] + TOL)):
            continue
        a = 0.5 * np.linalg.norm(np.cross(c[1] - c[0], c[2] - c[0]))
        area += a
        for n in tri[3:6]:                      # consistent load of a 6-node triangle: corners 0, midsides A/3
            weight[n] = weight.get(n, 0.0) + a / 3.0
    if not weight or not fix_x or not fix_yz:
        raise RuntimeError(f"boundary sets empty: load {len(weight)}, support {len(fix_x)}, dowels {len(fix_yz)}")
    p = force / area
    return fix_x, fix_yz, {n: p * w for n, w in weight.items()}, area


def to_c3d10(n, nodes):
    """gmsh Tet10 -> Abaqus C3D10: last two midside nodes swapped, corners ordered for positive volume."""
    p = [np.asarray(nodes[t]) for t in n[:4]]
    if np.dot(p[1] - p[0], np.cross(p[2] - p[0], p[3] - p[0])) < 0.0:
        n = [n[0], n[1], n[3], n[2], n[4], n[9], n[7], n[6], n[8], n[5]]
    return [n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[9], n[8]]


def write_inp(path, nodes, elems, fix_x, fix_yz, loads, clamp=False):
    def nset(name, ids):
        out = [f"*NSET, NSET={name}"]
        out += [", ".join(str(x) for x in ids[i:i + 8]) for i in range(0, len(ids), 8)]
        return out
    z = ["*HEADING", "Jaw EN AW-7075, group 10", "*NODE"]
    z += [f"{t}, {c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}" for t, c in sorted(nodes.items())]
    z.append("*ELEMENT, TYPE=C3D10, ELSET=EALL")
    z += [f"{t}, " + ", ".join(map(str, to_c3d10(n, nodes))) for t, n in sorted(elems.items())]
    z += nset("NFIXX", fix_x) + nset("NFIXYZ", fix_yz)
    z += ["*MATERIAL, NAME=AL7075", "*ELASTIC", f"{E}, {NU}",
          "*SOLID SECTION, ELSET=EALL, MATERIAL=AL7075", "*STEP", "*STATIC",
          "*BOUNDARY", "NFIXX, 1, 3, 0.0" if clamp else "NFIXX, 1, 1, 0.0", "NFIXYZ, 2, 3, 0.0", "*CLOAD"]
    z += [f"{n}, 1, {f:.8f}" for n, f in sorted(loads.items())]
    z += ["*NODE FILE", "U, S", "*END STEP"]
    with open(path, "w") as fh:
        fh.write("\n".join(z) + "\n")


def solve(base):
    env = dict(os.environ, OMP_NUM_THREADS=str(os.cpu_count() or 4))
    r = subprocess.run([CCX, base], cwd=OUT, env=env, capture_output=True, text=True, timeout=1800)
    log = r.stdout + r.stderr
    with open(os.path.join(OUT, base + ".log"), "w") as fh:
        fh.write(log)
    if "*ERROR" in log or not os.path.exists(os.path.join(OUT, base + ".frd")):
        raise RuntimeError("CalculiX failed:\n" + log[-2000:])


def read_frd(path):
    """Node results from the .frd file. Fixed-width: negative numbers run together, so no split()."""
    blocks, cur, ncomp = {}, None, 0
    for line in open(path, errors="replace"):
        if line.startswith(" -4"):
            cur = blocks.setdefault(line[5:13].strip(), {})
            ncomp = int(line[13:18])
        elif line.startswith(" -1") and cur is not None:
            vals = []
            for i in range(ncomp):
                f = line[13 + 12 * i:25 + 12 * i]
                if not f.strip():
                    break
                vals.append(float(f))
            cur[int(line[3:13])] = vals
        elif line.startswith(" -2"):
            cur = None
    return blocks


def mises(s):
    sxx, syy, szz, sxy, syz, szx = s[:6]
    return np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))


def run(lc=1.5, force=P.F_JAW_MAX, base="jaw", keep_mesh=False, clamp=False):
    """clamp=True fixes the support face in all directions (rigid clamp, the hand-calculation assumption)."""
    os.makedirs(OUT, exist_ok=True)
    nodes, elems, tris = mesh(lc)
    fix_x, fix_yz, loads, area = boundary(nodes, tris, force)
    write_inp(os.path.join(OUT, base + ".inp"), nodes, elems, fix_x, fix_yz, loads, clamp)
    solve(base)
    b = read_frd(os.path.join(OUT, base + ".frd"))
    U, S = b["DISP"], b["STRESS"]
    xyz = np.array([nodes[t] for t in U])
    ids = list(U)
    u = np.array([U[t][:3] for t in ids])
    s = np.array([S[t][:6] for t in ids])
    vm = np.array([mises(r) for r in s])

    # Deflection at the load centre on the outer face, stress on the outer face at the check section.
    zc = 0.5 * (Z_LOAD[0] + Z_LOAD[1])
    sel_u = (np.abs(xyz[:, 0] - X_SUPPORT) < TOL) & (np.abs(xyz[:, 2] - zc) < max(lc, 1.0)) & (np.abs(xyz[:, 1]) < 3)
    sel_s = (np.abs(xyz[:, 0] - X_SUPPORT) < TOL) & (np.abs(xyz[:, 2] - Z_CHECK) < max(lc, 1.0)) & (np.abs(xyz[:, 1]) < 3)
    free = xyz[:, 2] < Z_HOLDER - 2.0           # away from the support singularities
    i_peak = int(np.argmax(np.where(free, vm, 0)))
    res = dict(lc=lc, force=force, nodes=len(nodes), elements=len(elems), load_area=area,
               ux_load=float(np.mean(u[sel_u, 0])), szz_check=float(np.mean(s[sel_s, 2])),
               vm_peak=float(vm[i_peak]), vm_peak_at=[float(v) for v in xyz[i_peak]],
               u_max=float(np.max(np.linalg.norm(u, axis=1))), safety=float(RP02 / vm[i_peak]))
    if keep_mesh:
        res["_mesh"] = (nodes, elems, ids, u, vm)
    return res


def convergence(sizes=(4.0, 2.5, 1.6, 1.0, 0.7), force=P.F_JAW_MAX):
    rows = []
    for lc in sizes:
        r = run(lc, force, base=f"conv_{lc:.1f}".replace(".", "_"))
        rows.append(r)
        print(f"lc {lc:4.1f} mm  {r['elements']:7d} el  ux {r['ux_load'] * 1000:7.3f} um  "
              f"szz {r['szz_check']:7.3f} MPa  vm_peak {r['vm_peak']:6.2f} MPa")
    with open(os.path.join(OUT, "convergence.json"), "w") as fh:
        json.dump(rows, fh, indent=1)
    return rows


if __name__ == "__main__":
    convergence()


def grid(res, scale=1.0):
    """PyVista grid of a run(keep_mesh=True) result; corner nodes only (linear tets are enough to look at)."""
    import pyvista as pv
    nodes, elems, ids, u, vm = res["_mesh"]
    index = {t: i for i, t in enumerate(ids)}
    pts = np.array([nodes[t] for t in ids]) + scale * u
    cells = np.array([[4] + [index[n] for n in e[:4]] for e in elems.values()]).ravel()
    g = pv.UnstructuredGrid(cells, np.full(len(elems), pv.CellType.TETRA, dtype=np.uint8), pts)
    g.point_data["von Mises [MPa]"] = vm
    g.point_data["u_x [um]"] = u[:, 0] * 1000
    return g


def render(res, path, scale=200.0, field="von Mises [MPa]", clim=None):
    import pyvista as pv
    pv.OFF_SCREEN = True
    p = pv.Plotter(off_screen=True, window_size=(1100, 800))
    p.add_mesh(grid(res, 0.0), style="wireframe", color="#9aa5ab", opacity=0.15)
    p.add_mesh(grid(res, scale), scalars=field, cmap="turbo", clim=clim, show_edges=False,
               scalar_bar_args=dict(title=field, vertical=True, n_labels=5, fmt="%.1f"))
    p.add_text(f"F = {res['force']:.1f} N, deformation x{scale:.0f}", font_size=11)
    p.view_vector((1.0, -1.3, 0.45), viewup=(0, 0, 1))
    p.reset_camera()
    p.camera.zoom(0.9)
    p.screenshot(path)
    p.close()
    return path
