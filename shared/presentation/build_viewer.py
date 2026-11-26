"""Data for the interactive gripper viewer (public/viewer): python build_viewer.py

-> public/viewer/gripper.glb   every CAD part as its own named node (02_CAD_Body/out/stl), assembled pose, mm
-> public/viewer/parts.json    per part: card, group, joining direction and distance for the exploded view
-> viewer_cards.md             the same cards as text, for review
Every number on a card is read from the BOM, gripper_params or a result file; the joining directions come from the
assembly order in Elias/07_Render/hero_v1.py, so the viewer explodes the way the films assemble.
"""
import ast
import csv
import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
CAD = os.path.join(ROOT, "Elias", "02_CAD_Body")
sys.path.insert(0, CAD)
import gripper_params as P  # noqa: E402

OUT = os.path.join(HERE, "public", "viewer")
J = lambda *p: json.load(open(os.path.join(ROOT, *p), encoding="utf-8"))
BOM = {r["Pos"]: r for r in csv.DictReader(open(os.path.join(CAD, "out", "bom.csv"), encoding="utf-8"), delimiter=";")}
MASS = J("Elias", "02_CAD_Body", "out", "checks.json")["mass_parts_kg"]
DRIVE = J("Elias", "02_CAD_Body", "out", "drive_margin.json")
TRAIN = next(c for c in J("Elias", "05_FEM", "out", "train_results.json")["cases"] if c["case"].startswith("LC2"))
BRAKE = J("Elias", "04_Simulation", "out", "brake_play.json")["cases"]["NBR E 5.5 MPa"]
POWER = J("Elias", "04_Simulation", "out", "power_fail.json")
SHEETS = {d["id"]: d for d in J("Elias", "01_Robot", "datasheet_values.json")}
ROBOT = J("Elias", "01_Robot", "robot_selection.json")


def assembly_order():
    """ASSEMBLY and its direction names, parsed from hero_v1.py (it imports bpy, so it cannot be imported here)"""
    src = open(os.path.join(ROOT, "Elias", "07_Render", "hero_v1.py"), encoding="utf-8").read()
    keep = [n for n in ast.parse(src).body if isinstance(n, ast.Assign) and any(
        isinstance(t, (ast.Name, ast.Tuple)) and ("ASSEMBLY" in ast.dump(t) or "DOWN" in ast.dump(t)) for t in n.targets)]
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), "hero_v1", "exec"), ns)
    return ns["ASSEMBLY"]


def b(pos):
    r = BOM[str(pos)]
    mat = f", {r['Material']}" if r["Material"].strip() not in ("-", "") else ""
    return f"pos. {pos}: {r['Qty']} × {r['Designation']} ({r['Standard / supplier']}{mat})"


g = lambda name: sum(v for k, v in MASS.items() if k.startswith(name)) * 1000
bev = DRIVE["bevels"]
bev_old, bev_new = bev["KHK SMSG1-20 (before 21 Nov)"], bev["KHK SMSG1.5-20 (built)"]
F2 = TRAIN["F"][0]
sheet = lambda k: f"{SHEETS[k]['file']} p. {SHEETS[k]['page']}"

GROUPS = {  # group -> (colour, label)
    "drive": ("#C2410C", "Drive"), "gear": ("#B45309", "Gear train"), "guide": ("#0F766E", "Guidance"),
    "jaw": ("#2563EB", "Jaws and pads"), "body": ("#78716C", "Housing"), "flange": ("#57534E", "Robot interface"),
    "std": ("#A8A29E", "Standard parts"),
}

# cards: (id, group, title, part prefixes, kind/what, why, key values, notebook section)
CARDS = [
    ("housing", "body", "Gear housing", ("Housing",), b(5),
     "Carries everything: rail, both bearings of the pinion shaft and the gearbox. Milled from one block of the "
     "standard machining alloy.",
     [f"length {P.HOUSING_X:.0f} mm (grew from 164 mm: brake and encoder bracket on both sides, kept symmetric)",
      f"wall {P.WALL:.0f} mm - the M3 cover screws sit on the wall centre line",
      f"top plate {P.PLATE_T:.0f} mm (thinned on 11 Nov; the bearings sit in a boss above it)",
      f"at least {P.WALL_MIN} mm of material around every hole, checked for every own part",
      f"{g('Housing'):.0f} g"], "4"),
    ("hood", "body", "Drive hood", ("Hood",), b(4),
     "Closes the drive train and carries the robot adapter. The adapter screws go into wire thread inserts, because "
     "a thread in aluminium that is opened again and again wears out.",
     [b(40), f"{g('Hood'):.0f} g"], "4"),
    ("cover", "body", "Bottom cover", ("Cover",), b(1),
     "Closes the housing from below; the two slots let the jaw holders through. Brush strips in the slots keep oil "
     "and dirt from the oily parts out of the rail.",
     [b(42), "ISO 10642 countersunk screws: the underside stays flat", f"{g('Cover'):.0f} g"], "4"),
    ("window", "body", "Cable window and gland", ("Window_Plate", "Cable_Gland"), b(2) + "; " + b(12),
     "The motor, brake and encoder cable leaves the hood through a sealed gland in the side window.",
     ["SKINTOP M16 × 1.5: strain relief and seal in one part"], "4"),
    ("adapter", "flange", "Robot adapter ISO 9409-1-40-4-M6", ("Adapter_ISO9409",), b(9),
     "The standard flange of the ABB IRB 1300: any tool with this flange fits, and the dowel pin fixes the angle so "
     "the gripper always sits the same way on the robot.",
     [f"Ø {P.ADAPTER_D:.0f} mm, {P.ADAPTER_T:.0f} mm thick, spigot Ø {P.ISO_SPIGOT_D:.0f} × {P.ISO_SPIGOT_H} mm",
      b(43), b(44), "to the hood: 4 × ISO 10642 M4x16 countersunk from the top, into wire thread inserts",
      f"gripper + part {ROBOT['payload']['mass']:.2f} kg of the robot's 7 kg"], "4"),
    ("motor", "drive", "Stepper motor", ("Motor",), b(23),
     "The motor current sets the grip force: limit the current and the gripper can never squeeze harder than "
     "intended. Closed loop: the encoder tells the controller where the jaws are.",
     [f"holding torque {P.MOTOR_T_HOLD} N·m (NEMA 17)",
      f"available at the jaws at full current, all losses: {P.F_JAW_AVAIL:.1f} N per jaw against the setpoint "
      f"{P.F_GRIP_SET:.1f} N", "the current limit is set to the setpoint"], "3"),
    ("brake", "drive", "Permanent-magnet brake", ("Brake",), b(17),
     "On a power failure the grip force would be gone in about 2 ms and the part would fall. The brake holds the "
     "drive; it must be free of play, because every degree of play at the motor shaft releases grip force.",
     [f"holding torque {P.BRAKE_T} N·m",
      f"allowed play at the motor shaft: {BRAKE['play_allowed_deg_S1']:.2f}° before nothing is left against the weight",
      "the spring brake BWA we had first allowed up to 2° → replaced by this zero-backlash brake (review 21 Nov)",
      "brake-first shutdown: the brake closes before the motor current drops (MuJoCo, power_fail.py)"], "6"),
    ("encoder", "drive", "Encoder", ("Encoder",), b(14),
     "Closes the loop for the stepper: the controller knows the motor angle and so the jaw position.",
     [], "4"),
    ("gearbox", "drive", "Planetary gearbox", ("Gearbox", "Bracket"), b(18) + "; " + b(3),
     "Multiplies the small motor torque so the pinion reaches the grip force; the bracket holds the motor-brake "
     "stack beside it.",
     [f"ratio i = {P.GEAR_I}, efficiency η = {P.GEAR_ETA}", f"{g('Gearbox') + g('Bracket'):.0f} g with the bracket"], "3"),
    ("bevel", "gear", "Spiral bevel pair", ("Bevel_Pinion", "Bevel_Wheel"), b(21) + "; " + b(22),
     "Turns the drive by 90°: the motor lies flat along the housing instead of standing on top of it - a short "
     "gripper keeps the payload lever and the wrist torque small.",
     [f"m = 1 pair (before 21 Nov) used {bev_old['rows']['datasheet']['use_surface'] * 100:.0f} % of its allowed "
      f"surface torque {bev_old['t_surf']} N·m - too weak",
      f"built m = 1.5 pair: {bev_new['rows']['datasheet']['use_surface'] * 100:.0f} % of {bev_new['t_surf']} N·m",
      "DIN 988 shim rings under the wheel hub set the mesh at assembly"], "4"),
    ("pinion", "gear", "Pinion shaft with bearings", ("Pinion_Shaft", "Bearing_608", "Circlip", "Key_", "Shim_", "Setscrew"),
     b(8) + "; " + b(13),
     "One pinion drives both racks. Fixed-loose bearing: the upper 608 is held by the two retaining rings, the lower "
     "one can slide, so heat and tolerances cannot jam the shaft.",
     [f"m = {P.MODUL}, z = {P.Z_RITZEL} → d = {P.PINION_D:.0f} mm; z = 20 lies above the undercut limit of 17 teeth",
      "case-hardened 16MnCr5 teeth", b(37), b(38), b(36)], "4"),
    ("racks", "gear", "Racks", ("Rack_",), b(19) + "; " + b(20),
     "Rack and pinion keep the jaws parallel and the force constant over the whole stroke; with one pinion between "
     "two racks the jaws always move together, so the part is centred.",
     [f"m = 1, 10 × 12 mm (DIN 867 profile), screwed through ±{P.RACK_SLOT:.0f} mm slots along the stroke",
      f"stroke {P.D_OPEN:.0f} mm per jaw: pads {2 * (P.PAD_X0 + P.D_OPEN):.0f} mm apart when open, 70 mm part"], "2"),
    ("rail", "guide", "Ball rail and carriages", ("Rail", "Carriage_"), b(15) + "; " + b(11),
     "A preloaded ball rail guides both jaws without play and takes the moments of the jaw lever.",
     [f"size 12: static load safety S0 5.1 - 7.8 (Rexroth asks 5 - 8; size 9 gave only 2.5 - 3.7) ({sheet('rexroth_s0')})",
      f"C0 {P.CARRIAGE_C0:.0f} N, Mt0 {P.CARRIAGE_MT0} N·m, ML0 {P.CARRIAGE_ML0} N·m per carriage ({sheet('rexroth_size12')})",
      f"rail L = {P.RAIL_L:.0f} mm, pitch {P.RAIL_PITCH:.0f} mm"], "4"),
    ("slides", "guide", "Slides", ("Slide_",), b(10),
     "Connect carriage, rack and jaw holder on each side.",
     [f"{P.SLIDE_EDGE} mm wider than the carriage on each side, so the M3 counterbores keep a wall",
      f"{g('Slide_') / 2:.0f} g each"], "4"),
    ("holders", "jaw", "Jaw holders and quick change", ("Holder_", "Thumbscrew_", "Dowel_ISO8734-4x12"),
     b(7) + "; " + b(35) + "; " + b(34),
     "Jaws can be changed by hand: two dowels locate the jaw, a knurled thumb screw from outside holds it against "
     "pulling off - no tool, no loose parts.",
     [f"holder {P.HOLDER_T_TOP:.0f} / {P.HOLDER_T_LOW:.0f} mm thick above / below the cover, {P.HOLDER_W:.0f} mm wide "
      "(2 mm wall beside the screw counterbores)", "to the slide: 2 × ISO 4762 M4x25 per side",
      "the thumb screw replaced a spring plunger that sat in the wrong direction (review 23 Nov)"], "4"),
    ("jaw", "jaw", "T-jaw", ("Jaw_",), b(6),
     "The wide, low pad sits on the flat zone of part B. Its centre of mass is below the pads, so when the robot "
     "accelerates the part wants to twist; the wide pad gives the friction a long lever against that twist.",
     [f"web {P.JAW_T:.0f} × {P.JAW_W:.0f} mm, {P.JAW_W_PAD:.0f} mm wide at the pad",
      f"FEM at full current ({F2:.1f} N): {TRAIN['parts']['jaw']['vm_peak']:.1f} MPa in the jaw, safety "
      f"{TRAIN['parts']['jaw']['safety']:.0f}; pad tilt {TRAIN['tilt_deg']:.3f}°",
      "EN AW-7075 T6: high strength, light - the jaws move with every grip", f"{g('Jaw_') / 2:.0f} g each"], "5"),
    ("pad", "jaw", "Pad plate", ("PadPlate_", "Pad_", "Screw_ISO4762-M3x6_Pad", "Washer_"), b(16),
     "NBR is oil resistant and grips oily PA 6. Vertical grooves let the oil run off the contact face.",
     [f"steel {P.PAD_PLATE_T:.0f} mm + NBR 70 Shore A {P.PAD_T:.0f} mm, pad face {P.PAD_W:.0f} mm wide",
      f"grooves {P.PAD_GROOVE[0]} mm wide, {P.PAD_GROOVE[1]} mm deep, every {P.PAD_GROOVE[2]:.0f} mm",
      f"design friction µ = {P.MU_OILY} (oily part); µ = {P.MU_PAD_EXPECTED} expected with the grooves, to be measured",
      "2 × ISO 4762 M3x6 from the back: the pad can be replaced"], "3"),
    ("screws", "std", "Standard parts", ("Screw_", "Dowel_ISO8734-3x14"),
     "ISO 4762 socket head cap screws, ISO 10642 countersunk screws, ISO 8734 dowel pins, ISO 4029 set screws - all "
     "strength class 8.8 or better",
     "Only standard sizes M3 and M4 (M6 at the robot flange, M5 thumb screws), all off the shelf. Countersunk "
     "screws wherever a face must stay flat (cover, window, adapter); dowel pins wherever a part must come back to "
     "exactly the same place (racks, jaws, flange).",
     [f"{BOM['30']['Qty']} × ISO 4762 M3x8, {BOM['28']['Qty']} × M3x10, {BOM['31']['Qty']} × M4x12, "
      f"{BOM['32']['Qty']} × M4x25", "M3 for plates on 7 mm walls, M4 for the loaded joints (gearbox, hood, holders)",
      "wire thread inserts where screws in aluminium are opened often (hood, top plate)"], "4"),
]


def joining_dirs(meshes):
    """direction each part leaves along when the gripper is taken apart; same rules as the hero film"""
    out, used = {}, set()
    order = assembly_order()
    for gi, (prefixes, how, dist) in enumerate(order):
        mem = [n for n in meshes if n.startswith(prefixes) and n not in used]
        used.update(mem)
        for n in mem:
            m = meshes[n]
            c = m.bounds.mean(axis=0)
            if how in ("in", "out"):
                s = -1 if c[0] < -0.1 else 1
                d = np.array([-s if how == "in" else s, 0, 0], float)
            elif how == "head":
                v = m.vertices - c
                ext = v.max(0) - v.min(0)
                ax = int(np.argmax(ext))
                o = [i for i in range(3) if i != ax]
                r = np.hypot(v[:, o[0]], v[:, o[1]])
                d = np.zeros(3)
                d[ax] = 1.0 if r[v[:, ax] > 0].max(initial=0) > r[v[:, ax] < 0].max(initial=0) else -1.0
            else:
                d = np.array(how, float)
            out[n] = (d.tolist(), 20 + 110 * gi / max(1, len(order) - 1))   # later assembled -> further out
    for n in meshes:                                                         # the housing stays
        out.setdefault(n, ([0, 0, 0], 0.0))
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    stl = os.path.join(CAD, "out", "stl")
    # only the current parts: the stl folder also keeps exports of replaced parts (GN 617 plunger, old tower ...)
    current = J("Elias", "02_CAD_Body", "out", "kinematics.json")["material"]
    meshes = {n: trimesh.load(os.path.join(stl, n + ".stl"), force="mesh") for n in sorted(current)}
    card_of, group_of = {}, {}
    for cid, grp, *_ in CARDS:
        prefixes = next(c[3] for c in CARDS if c[0] == cid)
        for n in meshes:
            if n.startswith(prefixes) and n not in card_of:
                card_of[n], group_of[n] = cid, grp
    for n in meshes:
        group_of.setdefault(n, "std")
        card_of.setdefault(n, "screws")
    dirs = joining_dirs(meshes)
    scene = trimesh.Scene()
    for n, m in meshes.items():
        m = m.copy()
        m.merge_vertices()
        col = GROUPS[group_of[n]][0]
        rgba = [int(col[i:i + 2], 16) for i in (1, 3, 5)] + [255]
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
        scene.add_geometry(m, node_name=n, geom_name=n)
    scene.export(os.path.join(OUT, "gripper.glb"))
    data = dict(
        groups={k: dict(color=v[0], label=v[1]) for k, v in GROUPS.items()},
        cards={c[0]: dict(group=c[1], title=c[2], what=c[4], why=c[5], values=c[6], section=c[7]) for c in CARDS},
        parts={n: dict(card=card_of[n], group=group_of[n], dir=dirs[n][0], dist=dirs[n][1]) for n in meshes},
    )
    json.dump(data, open(os.path.join(OUT, "parts.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    with open(os.path.join(HERE, "viewer_cards.md"), "w", encoding="utf-8") as fh:
        fh.write("# Viewer cards (generated by build_viewer.py - edit the texts there)\n\n")
        for c in CARDS:
            fh.write(f"## {c[2]}  ·  {GROUPS[c[1]][1]}\n\n**What:** {c[4]}\n\n**Why:** {c[5]}\n\n")
            for v in c[6]:
                fh.write(f"- {v}\n")
            fh.write(f"\nNotebook section {c[7]}\n\n")
    print("parts", len(meshes), "cards", len(CARDS), "glb MB", round(os.path.getsize(os.path.join(OUT, "gripper.glb")) / 1e6, 1))


if __name__ == "__main__":
    main()
