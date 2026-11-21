"""Grip force with the twist of the part in the jaws (review 21 Nov 2026).

python grip_check.py   ->  table: normal force per jaw needed per load case, for the old and the current pad

Part B's centre of mass sits dz below the pad centre. An acceleration a_y along the pads (horizontal, in the pad
plane) loads each pad with a shear force m a_y / 2 and a twist m a_y dz / 2; gravity plus vertical acceleration add
the shear m (g + a_z) / 2. Shear and twist share the same friction: the pad slips when the load leaves the limit
surface of a uniformly pressed rectangle (Coulomb, instantaneous centre of rotation), so they are not added up
linearly and not checked one by one. Normal force needed per jaw = (load / limit surface) / mu.
Acceleration in the grip direction x only shifts normal force from one jaw to the other and is not governing here.
"""
import os
import sys

import numpy as np
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_params as P  # noqa: E402

COG_Z_PART = -4.5            # mm, part B centre of mass in its STEP frame (mesh centroid, homogeneous PA 6)


def limit_hull(w, h, n=30, c=None):
    """Friction wrench (fy, fz, t) at slip for mu N = 1 on a w x h pad [m], over many instantaneous centres.
    c: tilted jaw, pressure falls linearly from the top edge to zero at depth c [m] (tilt_check.py); None = uniform."""
    ys, zs = np.meshgrid(np.linspace(-w / 2, w / 2, n), np.linspace(-h / 2, h / 2, n))
    pts_pad = np.c_[ys.ravel(), zs.ravel()]
    if c is not None:
        p = np.clip(1.0 - (h / 2 - pts_pad[:, 1]) / c, 0.0, None)
        pts_pad = np.repeat(pts_pad[p > 0], np.maximum(1, np.round(p[p > 0] * 20).astype(int)), axis=0)
    out = [[np.cos(a), np.sin(a), 0.0] for a in np.linspace(0, 2 * np.pi, 72, endpoint=False)]   # pure sliding
    for r in np.geomspace(1e-4, 50, 60):
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            c = r * np.array([np.cos(a), np.sin(a)])
            d = pts_pad - c
            v = np.c_[-d[:, 1], d[:, 0]] / (np.linalg.norm(d, axis=1)[:, None] + 1e-12)
            for sgn in (1, -1):
                f = -sgn * v.mean(axis=0)
                t = -sgn * np.mean(pts_pad[:, 0] * v[:, 1] - pts_pad[:, 1] * v[:, 0])
                out.append([f[0], f[1], t])
    return ConvexHull(np.array(out))


def normal_force(hull, load, mu):
    A, b = hull.equations[:, :3], hull.equations[:, 3]
    return float(np.max(A @ np.asarray(load, float) / -b)) / mu


def pad(pad_z, pad_w, part_z):
    """NBR face (w, h) [m] and CoG depth under its centre [m] from the pocket z range, width and part offset."""
    h = (pad_z[1] - pad_z[0] - 1.2) / 1000
    w = (pad_w - 1.2) / 1000
    dz = (0.5 * (pad_z[0] + pad_z[1]) - (part_z + COG_Z_PART)) / 1000
    return w, h, dz


CASES = [   # (name, a_y along the pad, a_z vertical, safety factor)
    ("path, hand calculation 4.96 m/s2", P.A_PATH_REAL, 0.0, P.S_SICHERHEIT),
    ("path peak in MuJoCo 5.35 m/s2", 5.35, 0.0, P.S_SICHERHEIT),
    ("e-stop 17.1 m/s2 horizontal", 17.1, 0.0, 1.5),
    ("e-stop 17.1 m/s2 vertical (lowering)", 0.0, 17.1, 1.5),
]


def table(mu=P.MU_OILY):
    pads = {"old 21.8 x 33.8, part at z -92": pad((-95.0, -60.0), 23.0, -92.0),
            "new 80.8 x 19.8, part at z -77": pad(P.PAD_Z, P.PAD_W, P.PART_OFFSET[2])}
    m, g = P.B_MASSE, P.G
    rows = {}
    for tag, (w, h, dz) in pads.items():
        hull = limit_hull(w, h)
        rows[tag] = (dz, [normal_force(hull, S * m / 2 * np.array([ay, g + az, ay * dz]), mu) for _, ay, az, S in CASES])
    print(f"mu {mu}, part {m:.3f} kg; setpoint {P.F_GRIP_SET:.1f} N, current limit (bevel) {P.F_JAW_LIMIT:.1f} N per jaw")
    print(f"{'case':40s} " + " ".join(f"{t[:22]:>24s}" for t in rows))
    print(f"{'CoG under the pad centre [mm]':40s} " + " ".join(f"{v[0] * 1000:24.1f}" for v in rows.values()))
    for i, (name, ay, az, S) in enumerate(CASES):
        print(f"{name + f', S {S}':40s} " + " ".join(f"{v[1][i]:22.1f} N" for v in rows.values()))
    return rows


if __name__ == "__main__":
    table()
