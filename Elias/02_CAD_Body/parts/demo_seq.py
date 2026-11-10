"""Demo sequence shared by the headless check and the FreeCAD play macro."""

LIFT = 60.0
# (time s, opening travel d mm, gripper z offset mm)
KEYS = [(0.0, 16, LIFT), (0.8, 16, LIFT), (1.8, 16, 0), (2.4, 2, 0), (2.8, 2, 0), (3.8, 2, LIFT),
        (4.6, 2, LIFT), (5.6, 2, 0), (6.0, 2, 0), (6.6, 16, 0), (7.6, 16, LIFT), (8.2, 16, LIFT)]
DURATION = KEYS[-1][0]


def _ease(u):
    return u * u * (3 - 2 * u)


def state(t, d_grip):
    """-> (d, gripper_z, part_z). The part rides along only while the jaws hold it."""
    t = t % DURATION
    for (t0, d0, z0), (t1, d1, z1) in zip(KEYS, KEYS[1:]):
        if t0 <= t <= t1:
            u = _ease((t - t0) / (t1 - t0)) if t1 > t0 else 1.0
            d, z = d0 + (d1 - d0) * u, z0 + (z1 - z0) * u
            return d, z, (z if d <= d_grip + 1e-6 else 0.0)
    return KEYS[-1][1], KEYS[-1][2], 0.0
