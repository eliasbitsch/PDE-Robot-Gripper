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


# Exploded view: (assembly step from CAD_PLAN.md section 8, offset at full explosion), s = side (-1 left, +1 right).
def explode_rules(s):
    return [
        ("Screw_ISO10642-M4x16_Adapter", 10, (0, 0, 230)), ("Adapter_ISO9409", 10, (0, 0, 190)),
        ("Screw_ISO4762-M3x6_Window", 9, (0, 80, 0)), ("Cable_Gland", 9, (0, 65, 0)), ("Window_Plate", 9, (0, 50, 0)),
        ("Screw_ISO4762-M4x12_Hood", 3, (0, 0, -60)), ("Hood", 3, (0, 0, 140)),
        ("Screw_ISO4762-M4x12_Gearbox", 2, (-40, 0, 60)), ("Screw_ISO4762-M3x10_Bracket", 2, (0, 0, 95)), ("Bracket", 2, (0, 0, 70)),
        ("Bevel_Pinion", 2, (0, 0, 85)), ("Setscrew_ISO4029-M4x4_Wheel", 4, (-25, 0, 40)), ("Bevel_Wheel", 4, (0, 0, 40)),
        ("Bearing_608_upper", 4, (0, 0, 25)), ("Bearing_608_lower", 4, (0, 0, -30)), ("Pinion_Shaft", 4, (0, 0, -60)),
        ("Screw_ISO4762-M3x8_Pad", 8, (45 * s, 0, -200)), ("Washer_", 8, (35 * s, 0, -200)),
        ("PadPlate_", 8, (-10 * s, 0, -235)), ("Pad_", 8, (-10 * s, 0, -235)), ("Jaw_", 8, (12 * s, 0, -200)),
        ("Screw_ISO4762-M3x10_Cover", 7, (0, 0, -175)), ("Cover", 7, (0, 0, -140)),
        ("Plunger_", 6, (40 * s, 0, -120)), ("Dowel_ISO8734-4x12", 6, (-15 * s, 0, -120)),
        ("Screw_ISO4762-M4x25_Holder", 6, (0, 0, -150)), ("Holder_", 6, (0, 0, -120)),
        ("Screw_ISO4762-M3x8_Carriage", 5, (0, 0, -115)), ("Screw_ISO4762-M3x10_Rack", 5, (0, 0, -115)),
        ("Dowel_ISO8734-3x14_Rack", 5, (0, 0, -115)), ("Slide_", 5, (0, 0, -90)), ("Carriage_", 5, (0, 0, -55)),
        ("Rack_", 5, (0, 0, -55)),
        ("Gearbox", 2, (0, 0, 85)), ("Motor", 2, (0, 0, 85)), ("Brake", 2, (0, 0, 85)),
        ("Screw_ISO4762-M3x8_Rail", 1, (0, 0, -60)), ("Rail", 1, (0, 0, -35)),
    ]


def explode(name, side):
    """-> (step, (dx, dy, dz)) for a part name; FreeCAD turns "-" into "_" in labels, so compare normalised."""
    n = name.replace("-", "_")
    for prefix, step, off in explode_rules(side):
        if n.startswith(prefix.replace("-", "_")):
            return step, off
    return 0, (0, 0, 0)
