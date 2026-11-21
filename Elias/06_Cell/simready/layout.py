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
DOOR_X = (-0.35, 1.05)          # door bay on the operator side, m
CAB_X = -0.75                   # switch cabinet next to the door, outside the fence


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
    # buffer belt: flat belt (derived.py from warehouse_conveyorBelt_a06), belt 900 wide, top 1.781 -> 300, H_PICK
    if manifest("flat_belt"):
        s = np.array([(CP.BELT_X[1] - CP.BELT_X[0]) * MM / 2.0, CP.BELT_W * MM / 0.9, CP.H_PICK * MM / 1.781])
        out.append(("flat_belt", s, np.array([CP.BELT_X[0] * MM, 0.0, 0.0]), 0.0))
    # fence (Abb. 1: 2100 high, inner faces y = +-Y_FENCE): mesh panels wire_machineGuard_a03 (1.3 m, post at its +x
    # end) cut to the bays, on the operator side (-y) the door wire_machineGuard_a01 (1.4 m, with the guard locking
    # box) next to the switch cabinet; the ends are closed by the machine and the buffer housing (cell_parts.py)
    sz = CP.H_FENCE * MM / 1.7
    yf = (CP.Y_FENCE * MM + 0.075)
    xa, xb = CP.X_FENCE[0] * MM, CP.X_FENCE[1] * MM
    if manifest("wire_machineGuard_a03"):
        def bay(x0, x1, y):
            sx = (x1 - x0) / 1.3
            out.append(("wire_machineGuard_a03", np.array([sx, 1.0, sz]), np.array([x1 - sx * 0.075, y, 0.0]), 0.0))
        xm = 0.5 * (xa + xb)
        bay(xa, xm, yf)
        bay(xm, xb, yf)
        bay(xa, DOOR_X[0], -yf)
    if manifest("wire_machineGuard_a01"):
        out.append(("wire_machineGuard_a01", np.array([(DOOR_X[1] - DOOR_X[0]) / 1.396, 1.0, sz]),
                    np.array([0.5 * (DOOR_X[0] + DOOR_X[1]), -yf, 0.0]), 0.0))
    for own in ("machine", "buffer_housing", "chain_conveyors", "fence_fill"):
        if manifest(own):
            out.append((own, np.ones(3), np.zeros(3), 0.0))
    # hall floor, top face at z = 0
    m = manifest("environment_floor_a01")
    if m:
        lo, hi = np.array(m["bbox"][0]), np.array(m["bbox"][1])
        out.append(("environment_floor_a01", np.ones(3), np.array([0.3, 0.0, -hi[2]]), 0.0))
    # OmniCore C30 on the switch cabinet outside the fence, door towards the operator (-y)
    if manifest("controller"):
        out.append(("controller", np.ones(3), np.array([CAB_X, -(CP.Y_FENCE + CP.T_FENCE) * MM - 0.32, 0.0]), 0.0))
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
            if p.get("texture"):
                assets.append(f'<texture name="{name}_t" type="2d" file="{os.path.join(OBJ, asset, p["texture"])}"/>')
                assets.append(f'<material name="{name}_m" texture="{name}_t"/>')
                look = f'material="{name}_m"'
            else:
                look = 'rgba="' + " ".join(f"{c:.3f}" for c in p["rgba"]) + '"'
            geoms.append(f'<geom type="mesh" mesh="{name}" pos="{o[0]:.6g} {o[1]:.6g} {o[2]:.6g}" euler="0 0 {yaw:g}" {look} '
                         f'contype="0" conaffinity="0" group="1"/>')
            seen.add(asset)
    return assets, geoms, sorted(seen)
