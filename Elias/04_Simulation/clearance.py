"""Collision check of the work cycle: smallest distance between robot + gripper and the cell (fence, belt, chains,
carriers, pedestal, floor) along a joint path. Geometry distances by MuJoCo (convex hulls of the meshes, so the
result is on the safe side); the fence and conveyors are not physical in the model, only measured.

python clearance.py  ->  out/clearance.json
"""
import json
import os

import mujoco
import numpy as np

import sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
OBSTACLES = ("fence_left", "fence_right", "belt", "chain_1", "chain_2", "carrier", "carrier_1", "pedestal", "floor")
D_MAX = 0.5                                             # m, distances above this are reported as >= D_MAX


def robot_geoms(m):
    """Mesh geoms of links 2-6 and of the gripper (incl. the jaws); the base and link 1 sit on the pedestal."""
    keep = {m.body(f"link{i}").id for i in range(2, 7)}
    gid = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        root = b
        while root and root not in keep:
            root = m.body_parentid[root]
        if root in keep and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            gid.append(g)
    return gid


def clearance(q_path, m=None, d=None, labels=None):
    """q_path: (n, 6) joint angles in rad. Returns per obstacle the smallest distance [mm], where it happened and
    which robot geom came closest."""
    if m is None:
        m, d = S.load()
    rg = robot_geoms(m)
    obs = {o: m.geom(o).id for o in OBSTACLES if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, o) >= 0}
    best = {o: (D_MAX * 1000, -1, "") for o in obs}
    ft = np.zeros(6)
    for k, q in enumerate(q_path):
        S.set_robot(m, d, q)
        for o, og in obs.items():
            for g in rg:
                dist = mujoco.mj_geomDistance(m, d, g, og, D_MAX, ft)
                if dist * 1000 < best[o][0]:
                    body = m.body(m.geom_bodyid[g]).name
                    best[o] = (dist * 1000, k, body)
    return {o: dict(min_mm=v[0], step=v[1], label=(labels[v[1]] if labels and v[1] >= 0 else v[1]), body=v[2])
            for o, v in best.items()}


def straight_swing(n=60, lift=60.0):
    q1, _ = S.q_pick(lift)
    q2, _ = S.q_place(lift)
    return np.array([q1 + s * (q2 - q1) for s in np.linspace(0, 1, n)])


def cycle_path(n_lift=10, n_tr=80, lift=60.0):
    """The whole cycle as in validate_grip: lift at the pick, transfer through the vias, lower at the place."""
    q0, _ = S.q_pick()
    q1, _ = S.q_pick(lift)
    q2, _ = S.q_place(lift)
    q3, _ = S.q_place()
    p = S.transfer_path(lift)
    u = np.linspace(0, 1, n_tr)
    qs = ([q0 + s * (q1 - q0) for s in np.linspace(0, 1, n_lift)] + [p(x) for x in u]
          + [q2 + s * (q3 - q2) for s in np.linspace(0, 1, n_lift)])
    labels = ([f"lift {i}" for i in range(n_lift)] + [f"transfer u={x:.2f}" for x in u]
              + [f"lower {i}" for i in range(n_lift)])
    return np.array(qs), labels


def main():
    m, d = S.load()
    out = {}
    for name, (path, labels) in (("straight swing (before)", (straight_swing(), None)), ("cycle with vias", cycle_path())):
        res = clearance(path, m, d, labels)
        out[name] = res
        print(name)
        for o, r in res.items():
            print(f"  {o:12s} min {r['min_mm']:7.1f} mm  at {r['label']}  ({r['body']})")
    json.dump(out, open(os.path.join(S.B.OUT, "clearance.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
