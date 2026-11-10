"""Demo sequence shared by the headless check and the FreeCAD play macro."""

LIFT = 60.0
OPEN = 10.0                         # set from gripper_params.D_OPEN by the callers
# (time s, opening travel d mm, gripper z offset mm)
KEYS = [(0.0, "o", LIFT), (0.8, "o", LIFT), (1.8, "o", 0), (2.4, 2, 0), (2.8, 2, 0), (3.8, 2, LIFT),
        (4.6, 2, LIFT), (5.6, 2, 0), (6.0, 2, 0), (6.6, "o", 0), (7.6, "o", LIFT), (8.2, "o", LIFT)]
DURATION = KEYS[-1][0]


def _ease(u):
    return u * u * (3 - 2 * u)


def state(t, d_grip, d_open=None):
    """-> (d, gripper_z, part_z). The part rides along only while the jaws hold it."""
    t = t % DURATION
    o = OPEN if d_open is None else d_open
    keys = [(k, o if d == "o" else d, z) for k, d, z in KEYS]
    for (t0, d0, z0), (t1, d1, z1) in zip(keys, keys[1:]):
        if t0 <= t <= t1:
            u = _ease((t - t0) / (t1 - t0)) if t1 > t0 else 1.0
            d, z = d0 + (d1 - d0) * u, z0 + (z1 - z0) * u
            return d, z, (z if d <= d_grip + 1e-6 else 0.0)
    return o, KEYS[-1][2], 0.0
