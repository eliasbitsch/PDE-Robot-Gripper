"""Where the SimReady meshes go in the cell: each asset is scaled and moved so that a reference box of the asset
lands on the exact cell geometry of cell_params (Abb. 1). Visual only; collisions use the plain boxes.

placements() -> [(asset, scale (3,), offset (3,), yaw deg)] in metres, world = offset + Rz(yaw)(scale * local)
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "..", "vendor", "simready", "obj")
sys.path.insert(0, os.path.join(HERE, ".."))
import cell_params as CP  # noqa: E402

MM = 0.001


def manifest(asset):
    p = os.path.join(OBJ, asset, "manifest.json")
    return json.load(open(p)) if os.path.exists(p) else None


def fit(ref_lo, ref_hi, tgt_lo, tgt_hi, keep=()):
    """Per axis: scale and offset so [ref_lo, ref_hi] maps onto [tgt_lo, tgt_hi]; axes in keep stay unscaled and
    are centred on the target."""
    s, o = np.ones(3), np.zeros(3)
    for i in range(3):
        if i in keep:
            o[i] = 0.5 * (tgt_lo[i] + tgt_hi[i]) - 0.5 * (ref_lo[i] + ref_hi[i])
        else:
            s[i] = (tgt_hi[i] - tgt_lo[i]) / (ref_hi[i] - ref_lo[i])
            o[i] = tgt_lo[i] - s[i] * ref_lo[i]
    return s, o


def placements():
    out = []
    # buffer belt: the roller field (x 0.05 ... 0.953, top 0.769) onto the belt, 300 x 1400 mm, top at H_PICK
    # turned -90 deg about z: asset length (local y, -2 ... 0) runs along +x, the tall end posts (y ~ 0) outside
    if manifest("conveyor_belt_a05"):
        sx = CP.BELT_W * MM / (0.953 - 0.05)
        sy = (CP.BELT_X[1] - CP.BELT_X[0]) * MM / 2.0
        s = np.array([sx, sy, CP.H_PICK * MM / 0.769])
        o = np.array([CP.BELT_X[1] * MM, CP.BELT_W / 2 * MM + sx * 0.05, 0.0])   # world = o + Rz(-90)(s * local)
        out.append(("conveyor_belt_a05", s, o, -90.0))
    # fence: one mesh panel per side over the enclosure length, 2100 high, inner face on y = +-Y_FENCE
    m = manifest("wire_machineGuard_a04")
    if m:
        lo, hi = np.array(m["bbox"][0]), np.array(m["bbox"][1])
        half_t = 0.5 * (hi[1] - lo[1])
        for side in (1, -1):
            yc = side * (CP.Y_FENCE * MM + half_t)
            s, o = fit(lo, hi, [CP.X_FENCE[0] * MM, yc - half_t, 0.0], [CP.X_FENCE[1] * MM, yc + half_t, CP.H_FENCE * MM], keep=(1,))
            out.append(("wire_machineGuard_a04", s, o, 0.0))
    # hall floor, top face at z = 0
    m = manifest("environment_floor_a01")
    if m:
        lo, hi = np.array(m["bbox"][0]), np.array(m["bbox"][1])
        out.append(("environment_floor_a01", np.ones(3), np.array([0.3, 0.0, -hi[2]]), 0.0))
    # operator terminal outside the fence, next to the buffer belt
    if manifest("digitalSystem_touchTerminal_standing_a01"):
        out.append(("digitalSystem_touchTerminal_standing_a01", np.ones(3),
                    np.array([0.35, -(CP.Y_FENCE * MM + 0.45), 0.0]), 0.0))
    return out


def mjcf(mesh_prefix="sr"):
    """Asset and worldbody snippets for MuJoCo; empty if the assets are not downloaded (fetch.py, usd2obj.py)."""
    assets, geoms, seen = [], [], set()
    for k, (asset, s, o, yaw) in enumerate(placements()):
        m = manifest(asset)
        for i, p in enumerate(m["parts"]):
            name = f"{mesh_prefix}{k}_{i}"
            f = os.path.join(OBJ, asset, p["file"])
            assets.append(f'<mesh name="{name}" file="{f}" scale="{s[0]:.6g} {s[1]:.6g} {s[2]:.6g}" inertia="shell"/>')
            rgba = " ".join(f"{c:.3f}" for c in p["rgba"])
            geoms.append(f'<geom type="mesh" mesh="{name}" pos="{o[0]:.6g} {o[1]:.6g} {o[2]:.6g}" euler="0 0 {yaw:g}" rgba="{rgba}" '
                         f'contype="0" conaffinity="0" group="1"/>')
            seen.add(asset)
    return assets, geoms, sorted(seen)
