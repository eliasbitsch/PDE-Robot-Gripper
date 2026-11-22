# Hero render v1 (product look, first shots, 12 Nov 2026; recovered 21 Nov). Output in out/ (git-ignored).
# blender -b -P hero_v1.py -- [still|save|anim]   (env: FRAME, SAMPLES, RES=1920x1080, OUT)
# then: ffmpeg -framerate 30 -i out/frames/f_%04d.png -c:v libx264 -crf 18 -pix_fmt yuv420p hero_v1.mp4
import bpy, json, math, os, sys
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "02_CAD_Body")
sys.path.insert(0, CAD)
import gripper_params as P  # noqa: E402
KIN = json.load(open(os.path.join(CAD, "out", "kinematics.json")))
MM = 0.001
PINION_X, DRIVE_Z, R_PIN = P.PINION_X, P.DRIVE_Z, KIN["pinion_r_mm"]
D_OPEN, D_GRIP = KIN["d_open"], KIN["d_grip"]
JAWS = ("Jaw_", "PadPlate_", "Pad_", "Screw_ISO4762-M3x6_Pad", "Washer_ISO7089-3_Pad")   # V3 jaws, shown since 21 Nov
SHELL = ("Housing", "Hood", "Cover", "Screw_ISO4762-M3x10_Cover", "Screw_ISO4762-M4x12_Hood", "Adapter",
         "Screw_ISO10642", "Window", "Cable", "Screw_ISO4762-M3x6_Window")
FPS = 30
SHOT_A = (1, 150)     # hero, holders open/close while the gripper turns
SHOT_B = (151, 270)   # drive train close-up, shell hidden

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
mode = args[0] if args else "still"
sc = bpy.context.scene


def reset():
    global sc
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene


def mat(name, base, metal, rough, coat=0.0, aniso=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    b.inputs["Base Color"].default_value = (*base, 1)
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    b.inputs["Coat Weight"].default_value = coat
    b.inputs["Coat Roughness"].default_value = 0.1
    b.inputs["Anisotropic"].default_value = aniso
    return m


def materials():
    return {
        "alu": mat("alu_blasted", (0.80, 0.81, 0.83), 1.0, 0.34),
        "alu2": mat("alu_fine", (0.86, 0.87, 0.88), 1.0, 0.24),
        "orange": mat("anodized_orange", (0.62, 0.13, 0.025), 1.0, 0.30, coat=0.3),
        "steel": mat("steel_polished", (0.62, 0.62, 0.63), 1.0, 0.16),
        "steel_dark": mat("steel_ground", (0.42, 0.42, 0.43), 1.0, 0.26, aniso=0.5),
        "black": mat("black_coated", (0.018, 0.018, 0.02), 0.0, 0.42, coat=0.2),
        "oxide": mat("black_oxide", (0.035, 0.035, 0.038), 1.0, 0.33),
        "plastic": mat("black_plastic", (0.012, 0.012, 0.013), 0.0, 0.35),
    }


def pick(name, material, M):
    if name.startswith(("Holder_",)):
        return M["orange"]
    if name.startswith(("Screw", "Setscrew")):
        return M["oxide"]
    if name.startswith(("Washer", "Dowel")):
        return M["steel"]
    if name.startswith(("Plunger", "Cable")):
        return M["plastic"]
    if name.startswith(("Rack", "Bevel", "Pinion")):
        return M["steel_dark"]
    if name.startswith(("Housing", "Hood", "Cover")):
        return M["alu"]
    return {"EN AW-6082": M["alu2"], "steel": M["steel"], "vendor": M["black"]}.get(material, M["alu"])


def empty(name, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None)
    sc.collection.objects.link(e)
    e.location = loc
    e.parent = parent
    return e


def import_parts(M):
    root = empty("Gripper")
    piv = {"rot": empty("PivotPinion", (PINION_X * MM, 0, 0), root),
           "rotx": empty("PivotBevel", (0, 0, DRIVE_Z * MM), root),
           -1: empty("SideL", parent=root), 1: empty("SideR", parent=root)}
    objs = {}
    for name, material in KIN["material"].items():
        bpy.ops.wm.stl_import(filepath=os.path.join(CAD, "out", "stl", name + ".stl"), global_scale=MM)
        ob = bpy.context.selected_objects[0]
        ob.name = name
        ob.data.materials.append(pick(name, material, M))
        par = piv.get(KIN["moving"].get(name, 0), root)
        mw = ob.matrix_world.copy()
        ob.parent = par
        ob.matrix_world = mw
        objs[name] = ob
    # sharp CAD edges, smooth curved faces
    for ob in objs.values():
        ob.data.shade_smooth()
        mod = ob.modifiers.new("smooth", "SMOOTH_BY_ANGLE") if hasattr(bpy.types, "SmoothByAngleModifier") else None
        if mod is None:
            try:
                ob.data.set_sharp_from_angle(angle=math.radians(32))
            except AttributeError:
                pass
    return root, piv, objs


def ease(f):
    return f * f * (3 - 2 * f)


def animate(root, piv, objs):
    """d(t): open -> grip -> hold -> open per shot; all movers keyed from the same d, so they stay geared."""
    def d_at(frame, start, end):
        t = (frame - start) / (end - start)
        if t < 0.2:
            return D_OPEN
        if t < 0.45:
            return D_OPEN - (D_OPEN - D_GRIP) * ease((t - 0.2) / 0.25)
        if t < 0.65:
            return D_GRIP
        if t < 0.9:
            return D_GRIP + (D_OPEN - D_GRIP) * ease((t - 0.65) / 0.25)
        return D_OPEN

    for start, end in (SHOT_A, SHOT_B):
        for f in range(start, end + 1):
            d = d_at(f, start, end)
            piv[-1].location.x = -d * MM
            piv[1].location.x = d * MM
            piv["rot"].rotation_euler.z = d / R_PIN
            piv["rotx"].rotation_euler.x = d / R_PIN
            for k in (-1, 1):
                piv[k].keyframe_insert("location", index=0, frame=f)
            piv["rot"].keyframe_insert("rotation_euler", index=2, frame=f)
            piv["rotx"].keyframe_insert("rotation_euler", index=0, frame=f)
    # slow turn of the whole gripper in shot A, still in B
    for f, z in ((SHOT_A[0], math.radians(-18)), (SHOT_A[1], math.radians(14)), (SHOT_B[0], 0.0)):
        root.rotation_euler.z = z
        root.keyframe_insert("rotation_euler", index=2, frame=f)
    # shell visible in A, hidden in B
    for name, ob in objs.items():
        if name.startswith(SHELL):
            for f, hide in ((SHOT_A[0], False), (SHOT_B[0], True)):
                ob.hide_render = hide
                ob.keyframe_insert("hide_render", frame=f)


def bounds(objs):
    lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for ob in objs.values():
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            lo, hi = Vector(map(min, lo, w)), Vector(map(max, hi, w))
    return lo, hi


def stage(lo, hi):
    w = bpy.data.worlds.new("World")
    sc.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.004, 0.0042, 0.0046, 1)
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 1.0
    # floor below the floating gripper, dark gloss gives the soft reflection
    zf = lo.z - 0.035
    me = bpy.data.meshes.new("Floor")
    s = 3.0
    me.from_pydata([(-s, -s, zf), (s, -s, zf), (s, s, zf), (-s, s, zf)], [], [(0, 1, 2, 3)])
    fl = bpy.data.objects.new("Floor", me)
    sc.collection.objects.link(fl)
    fl.data.materials.append(mat("floor", (0.03, 0.03, 0.033), 0.0, 0.2))
    c = (lo + hi) / 2

    def area(name, loc, size, energy, shape="RECTANGLE", sy=None, color=(1, 1, 1)):
        ld = bpy.data.lights.new(name, "AREA")
        ld.shape, ld.size, ld.energy, ld.color = shape, size, energy, color
        if sy:
            ld.size_y = sy
        ob = bpy.data.objects.new(name, ld)
        sc.collection.objects.link(ob)
        ob.location = loc
        ob.visible_camera = False
        t = ob.constraints.new("TRACK_TO")
        t.target = tgt
        t.track_axis, t.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
        return ob

    global tgt
    tgt = empty("Target", c)
    prod = bpy.data.collections.new("Product")
    sc.collection.children.link(prod)
    for ob in [o for o in sc.objects if o.type == "MESH" and o.name != "Floor"]:
        prod.objects.link(ob)
    key = area("Key", c + Vector((-0.35, -0.45, 0.55)), 0.7, 9, sy=0.5)
    key.light_linking.receiver_collection = prod   # keeps the key out of the glossy floor
    area("Top", c + Vector((0.0, 0.05, 0.6)), 0.5, 4)
    rim = area("RimL", c + Vector((-0.45, 0.4, 0.12)), 0.05, 40, sy=0.7, color=(0.9, 0.95, 1.0))
    rim.light_linking.receiver_collection = prod
    rim = area("RimR", c + Vector((0.22, 0.55, 0.1)), 0.05, 40, sy=0.7, color=(1.0, 0.96, 0.92))
    rim.light_linking.receiver_collection = prod
    fill = area("Fill", c + Vector((0.5, -0.5, 0.05)), 0.6, 1.2)
    fill.light_linking.receiver_collection = prod
    pool = bpy.data.lights.new("Pool", "SPOT")
    pool.energy, pool.spot_size, pool.spot_blend, pool.shadow_soft_size = 70, math.radians(38), 1.0, 0.15
    po = bpy.data.objects.new("Pool", pool)
    sc.collection.objects.link(po)
    po.location = (c.x, c.y + 0.05, zf + 0.7)
    floorset = bpy.data.collections.new("FloorOnly")
    sc.collection.children.link(floorset)
    floorset.objects.link(fl)
    pool.id_data  # noqa
    po.light_linking.receiver_collection = floorset
    for f, e in ((SHOT_A[0], 70), (SHOT_B[0] - 1, 70), (SHOT_B[0], 12)):
        pool.energy = e
        pool.keyframe_insert("energy", frame=f)
    return c


def camera(name, target, vec, dist, lens, fstop):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.dof.use_dof = True
    cd.dof.aperture_fstop = fstop
    cam = bpy.data.objects.new(name, cd)
    sc.collection.objects.link(cam)
    v = Vector(vec).normalized()
    cam.location = target + v * dist
    ft = empty(name + "_focus", target)
    cd.dof.focus_object = ft
    t = cam.constraints.new("TRACK_TO")
    t.target = ft
    t.track_axis, t.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
    return cam


def render_setup(samples, res):
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for ctype in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = ctype
        except TypeError:
            continue
        prefs.get_devices()
        if any(d.type == ctype for d in prefs.devices):
            for d in prefs.devices:
                d.use = d.type == ctype
            sc.cycles.device = "GPU"
            break
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.cycles.use_adaptive_sampling = True
    sc.view_settings.view_transform = "AgX"
    for cand in ("AgX - Medium High Contrast", "AgX - Base Contrast", "None"):
        try:
            sc.view_settings.look = cand
            break
        except TypeError:
            continue
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.fps = FPS
    sc.frame_start, sc.frame_end = SHOT_A[0], SHOT_B[1]
    sc.render.image_settings.file_format = "PNG"
    sc.render.film_transparent = False


reset()
M = materials()
root, piv, objs = import_parts(M)
lo, hi = bounds(objs)
print("bounds mm", [round(v / MM) for v in lo], [round(v / MM) for v in hi])
c = stage(lo, hi)
camA = camera("CamA", c + Vector((0, 0, 0.004)), (0.85, -1.6, -0.04), 0.98, 85, 5.6)   # pulled back: jaws in frame
camB = camera("CamB", Vector((-42, -5, 2)) * MM, (-0.9, -1.5, 0.75), 0.47, 85, 6.3)
animate(root, piv, objs)
# camera switch via timeline markers
for f, cam in ((SHOT_A[0], camA), (SHOT_B[0], camB)):
    mk = sc.timeline_markers.new(cam.name, frame=f)
    mk.camera = cam
res = tuple(int(v) for v in os.environ.get("RES", "1280x720").split("x"))
render_setup(int(os.environ.get("SAMPLES", "48")), res)

if mode == "anim":
    sc.render.filepath = os.path.join(HERE, "out", "frames", "f_")
    bpy.ops.render.render(animation=True)
    print("anim done")
elif mode == "flight":
    # cuts, dip centres and the big contact sit on beats of the chosen track (Mixkit 720 from 45.76 s, 126 BPM)
    # the approved views (review 21 Nov) as shots (review 22 Nov): every shot starts close and pulls out, is slow at its
    # view, then pushes in on a detail; the cut falls inside that push with a short dip to black. Flowing moves:
    # three-quarter -> top, jaws -> drive. env ENGINE=EEVEE for quick previews, STEP, FSTART, FRAMES
    zp = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1]) * MM
    jaw_t, drive_t = Vector((0, 0, zp)), Vector((-42, -5, 2)) * MM
    flange = Vector((c.x, c.y, hi.z))
    SHOTS = [  # per shot: keys (frame, target, direction, distance, lens, speed at the key); a new shot = a cut
        [(1, c, (0.55, -1.6, -0.2), 0.56, 100, 0.6), (55, c, (0.85, -1.6, -0.04), 0.98, 85, 0.3),
         (102, (c + jaw_t) / 2, (0.95, -1.6, -0.06), 0.68, 85, 1.4)],
        [(103, c, (1.1, -0.9, 0.62), 0.80, 70, 1.3), (145, c, (1.0, -1.0, 0.65), 1.05, 70, 0.35),
         (195, c, (0.08, -0.35, 1.0), 1.05, 70, 0.35), (230, flange, (0.05, -0.3, 1.0), 0.62, 70, 1.4)],
        [(231, c, (-1.0, -0.2, 0.06), 0.74, 85, 1.3), (275, c, (-1.0, -0.12, 0.05), 0.95, 85, 0.35),
         (316, (c + jaw_t) / 2, (-1.0, -0.3, 0.08), 0.6, 95, 1.3)],
        [(317, jaw_t, (0.22, -1.0, 0.10), 0.44, 100, 1.2), (387, jaw_t, (0.45, -1.0, 0.14), 0.35, 100, 0.35),
         (418, (jaw_t + drive_t) / 2, (-0.3, -1.5, 0.5), 0.62, 90, 1.0),   # pull back: do not fly through the parts
         (447, drive_t, (-0.9, -1.5, 0.75), 0.50, 85, 0.5), (515, drive_t, (-0.78, -1.6, 0.7), 0.47, 85, 0.5),
         (600, drive_t, (-0.6, -1.65, 0.62), 0.43, 85, 0.6)]]
    VARIANT = os.environ.get("VARIANT", "")
    if VARIANT == "_cont":
        # one continuous move, slow at every view; a view every 57 frames = 4 beats of the music at 126 BPM
        SHOTS = [[(1, c, (0.55, -1.6, -0.2), 0.56, 100, 0.6), (57, c, (0.85, -1.6, -0.04), 0.98, 85, 0.3),
                  (115, c, (1.0, -1.0, 0.65), 1.05, 70, 0.3), (172, c, (0.08, -0.35, 1.0), 1.05, 70, 0.3),
                  (229, c, (-1.0, -0.12, 0.05), 0.95, 85, 0.3),
                  (300, (c + jaw_t) / 2, (-0.2, -1.0, 0.1), 0.62, 92, 1.0),   # swing round the front, outside the parts
                  (343, jaw_t, (0.4, -1.0, 0.14), 0.36, 100, 0.3), (380, jaw_t, (0.45, -1.0, 0.14), 0.35, 100, 0.3),
                  (411, (jaw_t + drive_t) / 2, (-0.3, -1.5, 0.5), 0.62, 90, 1.0),
                  (440, drive_t, (-0.9, -1.5, 0.75), 0.50, 85, 0.5), (515, drive_t, (-0.78, -1.6, 0.7), 0.47, 85, 0.5),
                  (600, drive_t, (-0.6, -1.65, 0.62), 0.43, 85, 0.6)]]
    # continuous variant: no cuts, but a dip to dark at the fastest point between two views (review 22 Nov)
    DIPS = [86, 143, 200, 300] if VARIANT == "_cont" else None
    JAWS_D = [(41, 74, D_OPEN, D_GRIP), (150, 180, D_GRIP, D_OPEN), (268, 300, D_OPEN, D_GRIP),
              (337, 359, D_GRIP, D_OPEN), (365, 387, D_OPEN, D_GRIP), (450, 475, D_GRIP, D_OPEN),
              (490, 512, D_OPEN, D_GRIP), (528, 552, D_GRIP, D_OPEN), (562, 584, D_OPEN, D_GRIP)]
    END_FADE = (586, 600)
    END = SHOTS[-1][-1][0]
    root.animation_data_clear()
    root.rotation_euler = (0, 0, 0)
    for p_ in piv.values():
        p_.animation_data_clear()
    for f in range(1, END + 1):
        d = D_OPEN
        for f0, f1, a, b in JAWS_D:
            if f >= f0:
                d = a + (b - a) * ease(min((f - f0) / (f1 - f0), 1.0))
        piv[-1].location.x, piv[1].location.x = -d * MM, d * MM
        piv["rot"].rotation_euler.z = piv["rotx"].rotation_euler.x = d / R_PIN
        for k in (-1, 1):
            piv[k].keyframe_insert("location", index=0, frame=f)
        piv["rot"].keyframe_insert("rotation_euler", index=2, frame=f)
        piv["rotx"].keyframe_insert("rotation_euler", index=0, frame=f)
    # shell fades out on the way from the jaws to the drive; own material copies so the inner parts stay opaque
    FADE = (387, 427)
    shell_mats = {}
    for name, ob in objs.items():
        if name.startswith(SHELL):
            if ob.animation_data:
                ob.animation_data_clear()
            m0 = ob.data.materials[0]
            if m0.name not in shell_mats:
                m = m0.copy()
                m.name = m0.name + "_shell"
                if hasattr(m, "surface_render_method"):
                    m.surface_render_method = "DITHERED"
                if hasattr(m, "use_transparent_shadow"):
                    m.use_transparent_shadow = True
                shell_mats[m0.name] = m
            ob.data.materials[0] = shell_mats[m0.name]
            for f, hide in ((1, False), (FADE[1] + 1, True)):   # fully gone: skip it entirely
                ob.hide_render = hide
                ob.keyframe_insert("hide_render", frame=f)
    for m in shell_mats.values():
        a = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED").inputs["Alpha"]
        for f in range(FADE[0], FADE[1] + 1, 2):
            a.default_value = 1 - ease((f - FADE[0]) / (FADE[1] - FADE[0]))
            a.keyframe_insert("default_value", frame=f)
    # top view: the floor pool spot and the top light's floor reflection blow out the frame -> dip around frame 100
    pool = bpy.data.lights["Pool"]
    pool.animation_data_clear()
    for f, e in ((1, 0), (50, 0), (85, 70), (170, 70), (190, 10), (230, 10), (231, 70), (FADE[0], 70), (FADE[1], 12)):
        pool.energy = e
        pool.keyframe_insert("energy", frame=f)
    # reveal (review 22 Nov: more epic): black, one glint sweeps over the hood, then rims, key, fill and top come up
    for name, (f0, f1) in {"RimL": (8, 45), "RimR": (14, 50), "Key": (28, 72), "Fill": (40, 80), "Top": (40, 80)}.items():
        ld = bpy.data.lights[name]
        full = ld.energy
        for f in range(f0, f1 + 1, 2):
            ld.energy = full * ease((f - f0) / (f1 - f0))
            ld.keyframe_insert("energy", frame=f)
        ld.energy = 0.0
        ld.keyframe_insert("energy", frame=1)
    sw = bpy.data.lights.new("Sweep", "AREA")
    sw.shape, sw.size, sw.color = "DISK", 0.03, (1.0, 0.97, 0.93)
    swo = bpy.data.objects.new("Sweep", sw)
    sc.collection.objects.link(swo)
    swo.visible_camera = False
    swo.light_linking.receiver_collection = bpy.data.collections["Product"]
    tt = swo.constraints.new("TRACK_TO")
    tt.target, tt.track_axis, tt.up_axis = tgt, "TRACK_NEGATIVE_Z", "UP_Y"
    for f, x, e in ((1, -0.55, 0), (6, -0.45, 22), (40, 0.4, 22), (56, 0.55, 0)):   # from behind: edges only
        swo.location = c + Vector((x, 0.35, 0.28))
        sw.energy = e
        swo.keyframe_insert("location", frame=f)
        sw.keyframe_insert("energy", frame=f)
    bpy.data.objects["Top"].light_linking.receiver_collection = bpy.data.collections["Product"]
    sc.timeline_markers.clear()
    cam = camera("CamFlight", c, SHOTS[0][0][2], SHOTS[0][0][3], SHOTS[0][0][4], 5.6)
    cam.constraints.clear()
    foc = cam.data.dof.focus_object

    def cr(p0, p1, p2, p3, t):
        return 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3)

    def herm(t, y0, y1, m0, m1, h):
        return ((2 * t ** 3 - 3 * t * t + 1) * y0 + (t ** 3 - 2 * t * t + t) * h * m0
                + (-2 * t ** 3 + 3 * t * t) * y1 + (t ** 3 - t * t) * h * m1)

    cuts, moves = [], []
    for keys in SHOTS:
        # spline through the keys in space; time -> spline parameter by a Hermite warp, slope at a key = mean of the
        # neighbouring segment slopes times the key's speed factor
        F = [k[0] for k in keys]
        pos = [k[1] + Vector(k[2]).normalized() * k[3] for k in keys]
        tgt = [k[1].copy() for k in keys]
        lens = [float(k[4]) for k in keys]
        sl = [1 / (F[i + 1] - F[i]) for i in range(len(F) - 1)]
        m = [sl[0]] + [0.5 * (sl[i - 1] + sl[i]) for i in range(1, len(sl))] + [sl[-1]]
        m = [mi * k[5] for mi, k in zip(m, keys)]       # slow at the views, moving through the cuts
        for f in range(F[0], F[-1] + 1):
            k = min(max(i for i in range(len(F)) if F[i] <= f), len(F) - 2)
            h = F[k + 1] - F[k]
            u = herm((f - F[k]) / h, 0.0, 1.0, m[k], m[k + 1], h)
            ip = lambda arr: cr(arr[max(k - 1, 0)], arr[k], arr[k + 1], arr[min(k + 2, len(arr) - 1)], u)
            cam.location, foc.location = ip(pos), ip(tgt)
            cam.data.lens = lens[k] + (lens[k + 1] - lens[k]) * u
            d = foc.location - cam.location
            cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
            cam.keyframe_insert("location", frame=f)
            cam.keyframe_insert("rotation_euler", frame=f)
            cam.data.keyframe_insert("lens", frame=f)
            foc.keyframe_insert("location", frame=f)
        if keys is not SHOTS[-1]:
            cuts.append(F[-1] + 1)
    # no interpolation across a cut (motion blur would smear the jump)
    def fcurves(act):   # Blender 5 layered actions keep fcurves in channel bags
        if hasattr(act, "fcurves"):
            return list(act.fcurves)
        return [fc for ly in act.layers for st in ly.strips for cb in st.channelbags for fc in cb.fcurves]

    for idb in (cam, cam.data, foc):
        for fc in fcurves(idb.animation_data.action):
            for kp in fc.keyframe_points:
                kp.interpolation = "CONSTANT" if int(kp.co.x) + 1 in cuts else "LINEAR"
    # dip to black across every cut, fade out at the end
    vs = sc.view_settings
    for f in range(1, END + 1):
        e = 0.0
        for cf in (DIPS if DIPS else cuts):
            if cf - 12 <= f < cf + 12:                  # 0.8 s dip; 5 frames felt cheap (review 22 Nov)
                e = -8 * (1 - ease(abs(f - (cf - 0.5)) / 12))
        if f >= END_FADE[0]:
            e = -10 * ease((f - END_FADE[0]) / (END_FADE[1] - END_FADE[0]))
        vs.exposure = e
        vs.keyframe_insert("exposure", frame=f)
    vs.exposure = 0.0
    json.dump({"fps": FPS, "end": END, "cuts": cuts, "jaws": JAWS_D, "fade": FADE, "end_fade": END_FADE, "dips": DIPS or cuts,
               "moves": [(145, 195), (380, 440)]},   # the two flowing camera moves
              open(os.path.join(HERE, "out", f"flight_timeline{VARIANT}.json"), "w"))
    sc.camera = cam
    if os.environ.get("ENGINE", "EEVEE").upper().startswith("EEVEE"):
        for eng in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
            try:
                sc.render.engine = eng
                break
            except TypeError:
                continue
        try:
            sc.eevee.use_raytracing = True
            sc.eevee.taa_render_samples = int(os.environ.get("SAMPLES", "16"))
        except AttributeError:
            pass
    sc.frame_start, sc.frame_end = int(os.environ.get("FSTART", 1)), int(os.environ.get("FRAMES", END))
    sc.frame_step = int(os.environ.get("STEP", "1"))
    sc.render.filepath = os.path.join(HERE, "out", "flight" + VARIANT, "f_")
    bpy.ops.render.render(animation=True)
    print("flight done", sc.render.engine)
elif mode == "explode":
    # shot 5b: partial exploded view until the drive train is free, motor drives three strokes, then closes again.
    # Parts move along their mounting axes with a damped spring (zeta 0.65 -> ~7 % overshoot), outer parts first.
    END = 420
    root.animation_data_clear()
    root.rotation_euler = (0, 0, 0)
    for ob in objs.values():
        ob.animation_data_clear()
        ob.hide_render = False
    sc.timeline_markers.clear()

    def spring(t, zeta=0.65):
        """0 -> 1 over t in [0, 1], settled (~2 %) at t = 1; zeta >= 1 never overshoots."""
        if t <= 0:
            return 0.0
        if t >= 1.3:
            return 1.0
        if zeta >= 1:
            return 1 - (1 + 6 * t) * math.exp(-6 * t)
        w = 4.0 / zeta
        wd = w * math.sqrt(1 - zeta * zeta)
        return 1 - math.exp(-zeta * w * t) * (math.cos(wd * t) + zeta * w / wd * math.sin(wd * t))

    UP, BACK, FRONT = Vector((0, 0, 1)), Vector((0, 1, 0)), Vector((0, -1, 0))
    GROUPS = [  # (name prefixes, offset mm, start frame, frames to settle)
        (("Screw_ISO10642-M4x16_Adapter",), UP * 175, 40, 26),
        (("Screw_ISO10642-M3x8_Window",), UP * 110 + FRONT * 45, 44, 26),
        (("Screw_ISO4762-M4x12_Hood",), UP * 110, 66, 34),   # leave with the hood
        (("Window_Plate", "Cable_Gland"), UP * 110 + FRONT * 28, 56, 30),
        (("Adapter_ISO9409",), UP * 150, 62, 32),
        (("Hood",), UP * 110, 70, 34),
        (("Housing", "Rail", "Screw_ISO4762-M3x8_Rail", "Cover", "Screw_ISO10642-M3x10_Cover"), BACK * 200, 92, 38),
    ]
    OPEN_AT, CLOSE_AT = (40, 135), (330, 395)          # explode window, close-up window
    for prefixes, off, f0, dur in GROUPS:
        c0 = CLOSE_AT[0] + (OPEN_AT[1] - (f0 + dur)) * 0.6   # closing in reverse order, a little faster
        for name, ob in objs.items():
            if not name.startswith(prefixes):
                continue
            base = ob.location.copy()
            for f in range(1, END + 1):
                s = spring((f - f0) / dur) * (1 - spring((f - c0) / (dur * 0.8), zeta=1.0))  # no dive into the body
                ob.location = base + off * MM * s
                ob.keyframe_insert("location", frame=f)
            ob.location = base
    # motor: three strokes forward and back while the drive train is free
    JAWS_D = [(160, 190, D_OPEN, D_GRIP), (200, 230, D_GRIP, D_OPEN), (240, 268, D_OPEN, D_GRIP),
              (276, 304, D_GRIP, D_OPEN), (310, 326, D_OPEN, D_GRIP), (400, 418, D_GRIP, D_OPEN)]
    for p_ in piv.values():
        p_.animation_data_clear()
    for f in range(1, END + 1):
        d = D_OPEN
        for f0, f1, a, b in JAWS_D:
            if f >= f0:
                d = a + (b - a) * ease(min((f - f0) / (f1 - f0), 1.0))
        piv[-1].location.x, piv[1].location.x = -d * MM, d * MM
        piv["rot"].rotation_euler.z = piv["rotx"].rotation_euler.x = d / R_PIN
        for k in (-1, 1):
            piv[k].keyframe_insert("location", index=0, frame=f)
        piv["rot"].keyframe_insert("rotation_euler", index=2, frame=f)
        piv["rotx"].keyframe_insert("rotation_euler", index=0, frame=f)
    # camera: slow orbit; wider while it opens, pushes in on the drive train during the strokes
    mech = Vector((-10, -5, 25)) * MM
    keys = [(1, c, (1.0, -1.35, 0.5), 0.78, 50), (130, (c + mech) / 2, (0.75, -1.45, 0.62), 0.95, 50),
            (320, mech + Vector((-5, 0, -15)) * MM, (0.4, -1.5, 0.22), 0.56, 60), (420, c, (0.15, -1.55, 0.35), 0.80, 55)]
    cam = camera("CamExplode", c, keys[0][2], keys[0][3], keys[0][4], 8.0)
    cam.constraints.clear()
    foc = cam.data.dof.focus_object

    def cr(p0, p1, p2, p3, t):
        return 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3)

    pos = [k[1] + Vector(k[2]).normalized() * k[3] for k in keys]
    tg = [k[1].copy() for k in keys]
    for f in range(1, END + 1):
        k = min(max(i for i in range(len(keys)) if keys[i][0] <= f), len(keys) - 2)
        t = (f - keys[k][0]) / (keys[k + 1][0] - keys[k][0])
        t = 0.5 * t + 0.5 * ease(t)                    # gentle: never stops
        ip = lambda arr: cr(arr[max(k - 1, 0)], arr[k], arr[k + 1], arr[min(k + 2, len(arr) - 1)], t)
        cam.location, foc.location = ip(pos), ip(tg)
        cam.data.lens = keys[k][4] + (keys[k + 1][4] - keys[k][4]) * t
        cam.rotation_euler = (foc.location - cam.location).to_track_quat("-Z", "Y").to_euler()
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_euler", frame=f)
        cam.data.keyframe_insert("lens", frame=f)
        foc.keyframe_insert("location", frame=f)
    sc.camera = cam
    bpy.data.objects["Top"].light_linking.receiver_collection = bpy.data.collections["Product"]
    pool = bpy.data.lights["Pool"]
    pool.animation_data_clear()
    pool.energy = 45
    if os.environ.get("ENGINE", "EEVEE").upper().startswith("EEVEE"):
        for eng in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
            try:
                sc.render.engine = eng
                break
            except TypeError:
                continue
        try:
            sc.eevee.use_raytracing = True
            sc.eevee.taa_render_samples = int(os.environ.get("SAMPLES", "16"))
        except AttributeError:
            pass
    sc.frame_start, sc.frame_end = int(os.environ.get("FSTART", 1)), int(os.environ.get("FRAMES", END))
    sc.frame_step = int(os.environ.get("STEP", "1"))
    sc.render.filepath = os.path.join(HERE, "out", "explode", "f_")
    bpy.ops.render.render(animation=True)
    print("explode done", sc.render.engine)
elif mode == "assemble":
    # shot 4: self-assembly in a white lab (review 22 Nov): groups arrive one after another along their own mounting
    # axis, on paths that stay clear of the parts already in place; the camera goes to where parts are joining.
    # Order follows parts/demo_seq.py STEP_TEXT, except the pinion shaft before the drive (the bevel wheel would
    # otherwise pass through the bevel pinion) and the racks before the slides (rack R floats above slide L).
    # Parts land critically damped (an overshoot would dive into the body); the spring feel comes from a 3 % scale
    # pulse on seating, a tilt that rings out, screws turning in and the assembly bouncing when a big part lands.
    import re
    from mathutils import Matrix, Quaternion
    root.animation_data_clear()
    root.rotation_euler = (0, 0, 0)
    for k_, p_ in piv.items():                          # CAD pose, no rotation: offsets below are world offsets
        p_.animation_data_clear()
        p_.rotation_euler = (0, 0, 0)
        if k_ in (-1, 1):
            p_.location = (0, 0, 0)
    sc.timeline_markers.clear()
    DOWN, UP, FRONT, RIGHT, LEFT = (0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (-1, 0, 0)
    GROUPS = [  # (parts by prefix, where they come from: a direction, "in" = from the centre, "out" = from outside; mm)
        (("Rail",), DOWN, 70), (("Screw_ISO4762-M3x8_Rail",), DOWN, 40),
        (("Pinion_Shaft", "Bearing_608_lower", "Circlip_DIN471"), DOWN, 70),
        (("Bearing_608_upper", "Circlip_DIN472"), UP, 60), (("Shim_", "Key_"), UP, 50), (("Bevel_Wheel",), UP, 60),
        (("Setscrew",), LEFT, 35),
        (("Bracket", "Gearbox", "Motor", "Brake", "Encoder", "Bevel_Pinion", "Screw_ISO4762-M4x12_Gearbox"), RIGHT, 140),
        (("Screw_ISO4762-M3x10_Bracket",), UP, 40),
        (("Hood",), UP, 90), (("Screw_ISO4762-M4x12_Hood",), DOWN, 45),
        (("Carriage_",), DOWN, 60), (("Rack_",), DOWN, 60), (("Slide_",), DOWN, 60),
        (("Screw_ISO4762-M3x8_Carriage", "Screw_ISO4762-M3x10_Rack", "Dowel_ISO8734-3x14_Rack"), DOWN, 40),
        (("Holder_",), DOWN, 60), (("Screw_ISO4762-M4x25_Holder",), DOWN, 50),
        (("Dowel_ISO8734-4x12",), "in", 30), (("Plunger_",), "out", 50),
        (("Cover",), DOWN, 70), (("Screw_ISO10642-M3x10_Cover",), DOWN, 40),
        (("Jaw_",), "in", 28), (("PadPlate_", "Pad_"), "in", 18), (("Screw_ISO4762-M3x6_Pad", "Washer_"), "out", 25),
        (("Window_Plate", "Cable_Gland"), FRONT, 60), (("Screw_ISO10642-M3x8_Window",), FRONT, 40),
        (("Adapter_ISO9409",), UP, 80), (("Screw_ISO10642-M4x16_Adapter",), UP, 45),
    ]
    PLATES = ("Hood", "Cover", "Rail", "Bracket", "Slide_", "Carriage_", "Holder_", "Window_Plate", "Adapter",
              "Jaw_", "PadPlate_", "Pad_")
    FASTENERS = ("Screw", "Setscrew", "Dowel", "Washer", "Key_", "Circlip", "Shim")
    for name, ob in objs.items():
        ob.animation_data_clear()
        ob.hide_render = False
        ctr = sum((Vector(v) for v in ob.bound_box), Vector()) / 8      # pivot on the part's own centre
        ob.data.transform(Matrix.Translation(-ctr))
        ob.location = ob.location + ob.matrix_basis.to_3x3() @ ctr   # objects carry the mm -> m scale
        ob.rotation_mode = "QUATERNION"
    T0, DUR, GAP = 30, 15, 10                          # first arrival, frames to land, hold before the next group
    f, sched, pois, used = T0, [], [], set()
    for prefixes, how, dist in GROUPS:
        members = sorted(n for n in objs if n.startswith(prefixes) and n not in used)
        if not members:
            continue
        used.update(members)
        g0 = f
        for i, n in enumerate(members):
            s_ = -1 if objs[n].location.x < -1e-4 else 1
            d = Vector((-s_, 0, 0)) if how == "in" else Vector((s_, 0, 0)) if how == "out" else Vector(how)
            sched.append((g0 + i * 2, n, objs[n], d * dist * MM))
        f = g0 + 2 * (len(members) - 1) + DUR + GAP
        locs = [objs[n].location for n in members]
        lo_ = Vector([min(v[i] for v in locs) for i in range(3)])
        hi_ = Vector([max(v[i] for v in locs) for i in range(3)])
        pois.append((g0, (lo_ + hi_) / 2, (hi_ - lo_).length + dist * MM, how))
    print("assemble unplaced:", sorted(set(objs) - used))
    END = f + 75
    rng_ = __import__("random").Random(4)
    # metal does not bounce: on seating the whole assembly yields a little in the push direction and turns about the
    # lever to its centre, then settles (review 22 Nov: no scale pulse, no wobble of the seated part)
    nudge_loc = [Vector() for _ in range(END + 2)]
    nudge_rot = [Vector() for _ in range(END + 2)]
    for f0, name, ob, off in sched:
        base = ob.location.copy()
        axis = off.normalized()
        tilt_ax = axis.cross(Vector((0.3, 0.7, 0.2))).normalized()
        tilt = math.radians(rng_.uniform(4, 8))
        seat = f0 + DUR
        for fr in range(1, END + 1):
            t = (fr - f0) / DUR
            s = 0.0 if t <= 0 else min(1.0, 1 - (1 + 6 * t) * math.exp(-6 * t))
            ob.location = base + off * (1 - s)
            ob.hide_render = t <= 0
            if name.startswith(("Screw", "Setscrew")):
                q = Quaternion(axis, (1 - s) * 6 * math.pi)                   # three turns in
            else:
                q = Quaternion(tilt_ax, tilt * (1 - s))                     # straightens out on the way in
            ob.rotation_quaternion = q
            ob.keyframe_insert("location", frame=fr)
            ob.keyframe_insert("rotation_quaternion", frame=fr)
            ob.keyframe_insert("hide_render", frame=fr)
        # the assembly rocks along the joining axis only: plates snap in with a short firm push, screws hardly at all
        push = -axis                                    # direction the part moves when it seats
        amp = 1.4 if name.startswith(PLATES) else 0.05 if name.startswith(FASTENERS) else 0.5
        for u in range(0, 20):
            if seat + u <= END:
                prof = math.exp(-u / 3.5) * math.sin(math.pi * u / 5)   # quick push, one damped swing back
                nudge_loc[seat + u] += push * amp * MM * prof
    # white lab: bright soft world, white floor; flash in from white
    w = sc.world.node_tree.nodes["Background"]
    w.inputs["Color"].default_value = (0.82, 0.83, 0.85, 1)
    w.inputs["Strength"].default_value = 0.9
    fl = bpy.data.objects["Floor"].data.materials[0]
    fb = next(n for n in fl.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    fb.inputs["Base Color"].default_value = (0.8, 0.8, 0.82, 1)
    fb.inputs["Roughness"].default_value = 0.45
    bpy.data.lights["Pool"].animation_data_clear()
    bpy.data.lights["Pool"].energy = 0
    vs = sc.view_settings
    for fr in range(1, 15):
        vs.exposure = 4 * (1 - ease((fr - 1) / 13)) - 0.3
        vs.keyframe_insert("exposure", frame=fr)
    # camera: wide on the floating housing, then to each joint while its parts arrive (looking across the approach
    # axis), wide again for the finished gripper. It holds at a joint and moves in the 14 frames before the next one.
    VIEW = {DOWN: (0.45, -1.0, -0.12), UP: (0.45, -1.0, 0.55), RIGHT: (0.15, -1.0, 0.35), FRONT: (1.0, -0.55, 0.3),
            LEFT: (0.2, -1.0, 0.3), "in": (0.12, -1.0, 0.12), "out": (0.12, -1.0, 0.12)}
    keys = [(1, c, (0.9, -1.3, 0.5), 0.9)]
    for g0, tgt_, size, how in pois:
        dist = min(max(size * 2.2 + 0.12, 0.24), 0.75)
        if (tgt_ - keys[-1][1]).length < 0.015 and abs(dist - keys[-1][3]) < 0.08:
            continue                                   # same joint: hold
        keys.append((max(g0 - 3, keys[-1][0] + 8), tgt_, VIEW[how], dist))
    keys.append((pois[-1][0] + DUR + 12, c, (0.85, -1.4, 0.45), 0.78))
    keys.append((END, c, (0.35, -1.5, 0.35), 0.72))
    cam = camera("CamAssemble", c, keys[0][2], keys[0][3], 50, 8.0)
    cam.constraints.clear()
    foc = cam.data.dof.focus_object
    pos_ = [k[1] + Vector(k[2]).normalized() * k[3] for k in keys]
    STYLE = os.environ.get("STYLE", "")
    root.rotation_mode = "QUATERNION"
    qn = lambda fr: Quaternion(nudge_rot[fr].normalized(), nudge_rot[fr].length) if nudge_rot[fr].length > 0 else Quaternion()
    if STYLE == "turn":
        # variant B (Mac Pro style): the camera stays in front; the gripper turns so the side the parts come from
        # faces the camera at about 45 deg (from below: tilts its underside to the camera). Orientation per group by
        # the shortest rotation, blended by slerp in the 16 frames before the group starts.
        CD = Vector((0.35, -1.0, 0.18)).normalized()   # target -> camera
        UPW = Vector((0, 0, 1))
        RGT = Vector((1.0, 0.35, 0.0)).normalized()    # screen right
        T_DOWN, T_UP, T_SIDE = (CD + -UPW).normalized(), (CD + UPW).normalized(), (CD + RGT).normalized()

        # isometric rule (review 22 Nov): the camera looks along a space diagonal of the gripper (54.7 deg to every axis),
        # the octant with the side the parts come from facing the camera; the gripper's z stays vertical on screen.
        # Of the octants that qualify, the one closest to the previous orientation, so it never spins needlessly.
        U_PERP = (UPW - CD * UPW.dot(CD)).normalized()

        def iso_quat(sx, sy, sz):
            a1 = Vector((sx, sy, sz)).normalized()
            z = Vector((0, 0, 1))
            a2 = (z - a1 * z.dot(a1)).normalized()
            A = Matrix((a1, a2, a1.cross(a2))).transposed()
            B = Matrix((CD, U_PERP, CD.cross(U_PERP))).transposed()
            return (B @ A.transposed()).to_quaternion()

        def orient(how, q_prev):
            d = Vector((0.4, -0.4, -1.0)) if how in ("in", "out") else Vector(how)   # jaws/pads: from below-front
            cands = []
            for sx in (1, -1):
                for sy in (1, -1):
                    for sz in (1, -1):
                        if (abs(d.x) > 0.5 and sx != (1 if d.x > 0 else -1)) or \
                           (abs(d.y) > 0.5 and sy != (1 if d.y > 0 else -1)) or \
                           (abs(d.z) > 0.5 and sz != (1 if d.z > 0 else -1)):
                            continue
                        cands.append(iso_quat(sx, sy, sz))
            return min(cands, key=lambda q: q_prev.rotation_difference(q).angle)

        tk = [(1, iso_quat(1, -1, 1), c, 0.9)]
        prev_how = None
        for g0, tgt_, size, how in pois:
            dist = min(max(size * 2.2 + 0.12, 0.26), 0.75)
            if how == prev_how and (tgt_ - tk[-1][2]).length < 0.04 and len(tk) > 1:
                continue                               # same side, same place (e.g. screws to their plate): hold
            tk.append((max(g0 - 2, tk[-1][0] + 6), orient(how, tk[-1][1]), tgt_, max(dist, tk[-1][3] if how == prev_how else 0)))
            prev_how = how
        tk.append((pois[-1][0] + DUR + 14, iso_quat(1, -1, 1), c, 0.8))     # finished gripper: classic iso from above
        tk.append((END, Quaternion((0, 0, 1), math.radians(35)) @ iso_quat(1, -1, 1), c, 0.74))
        for fr in range(1, END + 1):
            k = max(i for i in range(len(tk)) if tk[i][0] <= fr)
            if k == len(tk) - 1:
                v, k2 = 0.0, k
            else:
                k2 = k + 1
                span = min(16, tk[k2][0] - tk[k][0])
                v = ease(min(1.0, max(0.0, (fr - (tk[k2][0] - span)) / span)))
            qv = tk[k][1].slerp(tk[k2][1], v)
            poi = tk[k][2].lerp(tk[k2][2], v)
            dist = tk[k][3] + (tk[k2][3] - tk[k][3]) * v
            if os.environ.get("NOZOOM"):                # whole gripper always in frame: only the turns, no zoom or pan
                poi, dist = c, 0.82
            root.rotation_quaternion = qv @ qn(fr)
            root.location = qv @ nudge_loc[fr]
            root.keyframe_insert("rotation_quaternion", frame=fr)
            root.keyframe_insert("location", frame=fr)
            t_ = qv @ poi
            cam.location, foc.location = t_ + CD * dist, t_
            cam.rotation_euler = (-CD).to_track_quat("-Z", "Y").to_euler()
            cam.keyframe_insert("location", frame=fr)
            cam.keyframe_insert("rotation_euler", frame=fr)
            foc.keyframe_insert("location", frame=fr)
    else:
        for fr in range(1, END + 1):
            root.rotation_quaternion = qn(fr)
            root.location = nudge_loc[fr]
            root.keyframe_insert("rotation_quaternion", frame=fr)
            root.keyframe_insert("location", frame=fr)
    for fr in range(1, END + 1) if STYLE != "turn" else ():
        k = max(i for i in range(len(keys)) if keys[i][0] <= fr)
        if k == len(keys) - 1:
            p_, t_ = pos_[k], keys[k][1]
        else:
            span = min(14, keys[k + 1][0] - keys[k][0])
            v = ease(min(1.0, max(0.0, (fr - (keys[k + 1][0] - span)) / span)))
            p_, t_ = pos_[k].lerp(pos_[k + 1], v), keys[k][1].lerp(keys[k + 1][1], v)
        cam.location, foc.location = p_, t_
        cam.rotation_euler = (t_ - p_).to_track_quat("-Z", "Y").to_euler()
        cam.keyframe_insert("location", frame=fr)
        cam.keyframe_insert("rotation_euler", frame=fr)
        foc.keyframe_insert("location", frame=fr)
    sc.camera = cam
    if os.environ.get("ENGINE", "EEVEE").upper().startswith("EEVEE"):
        for eng in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
            try:
                sc.render.engine = eng
                break
            except TypeError:
                continue
        try:
            sc.eevee.use_raytracing = True
            sc.eevee.taa_render_samples = int(os.environ.get("SAMPLES", "16"))
        except AttributeError:
            pass
    print("assemble frames", END, "groups", len(pois), "camera keys", len(keys))
    sc.frame_start, sc.frame_end = int(os.environ.get("FSTART", 1)), int(os.environ.get("FRAMES", END))
    sc.frame_step = int(os.environ.get("STEP", "1"))
    sc.render.filepath = os.path.join(HERE, "out", "assemble" + ("_turn" if STYLE == "turn" else "")
                                      + ("_nozoom" if os.environ.get("NOZOOM") else ""), "f_")
    bpy.ops.render.render(animation=True)
    print("assemble done", sc.render.engine)
elif mode == "preview":
    # quick look: short clips at 480p and few samples (env CLIPS="a:40-100,b:170-230", STEP=2 -> 15 fps)
    step = int(os.environ.get("STEP", "2"))
    sc.frame_step = step
    for clip in os.environ.get("CLIPS", "a:40-100,b:170-230").split(","):
        tag, rng = clip.split(":")
        sc.frame_start, sc.frame_end = (int(v) for v in rng.split("-"))
        sc.render.filepath = os.path.join(HERE, "out", "preview", tag, "f_")
        bpy.ops.render.render(animation=True)
        print("clip done", tag)
elif mode == "views":
    # setup review before any video: stills from candidate cameras (name, target, direction, distance, lens, f-stop, frame)
    zp = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1]) * MM
    cands = [("1_hero_open", c, (0.85, -1.6, -0.04), 0.98, 85, 5.6, 10),
             ("2_hero_closing", c, (0.85, -1.6, -0.04), 0.98, 85, 5.6, 75),
             ("3_low_front", c, (0.15, -1.0, -0.14), 0.95, 70, 5.6, 75),
             ("4_three_quarter_high", c, (1.0, -1.0, 0.65), 1.05, 70, 5.6, 75),
             ("5_side", c, (-1.0, -0.12, 0.05), 0.95, 85, 5.6, 75),
             ("6_top", c, (0.08, -0.35, 1.0), 1.05, 70, 5.6, 75),
             ("7_jaws_macro", Vector((0, 0, zp)), (0.35, -1.0, 0.12), 0.36, 100, 4.0, 75),
             ("8_drive_closeup", Vector((-42, -5, 2)) * MM, (-0.9, -1.5, 0.75), 0.47, 85, 6.3, 205)]
    sc.timeline_markers.clear()      # markers would force CamA / CamB per frame
    os.makedirs(os.path.join(HERE, "out", "views"), exist_ok=True)
    for name, tgt, vec, dist, lens, fst, fr in cands:
        cam = camera("V_" + name, tgt, vec, dist, lens, fst)
        sc.frame_set(fr)
        sc.camera = cam
        sc.render.filepath = os.path.join(HERE, "out", "views", name + ".png")
        bpy.ops.render.render(write_still=True)
        print("wrote", sc.render.filepath)
elif mode == "save":
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(HERE, "out", "hero_v1.blend"))
    print("saved")
else:
    for fr in os.environ.get("FRAME", "75").split(","):
        sc.frame_set(int(fr))
        sc.camera = camA if int(fr) < SHOT_B[0] else camB
        sc.render.filepath = os.path.join(HERE, "out", os.environ.get("OUT", "test") + f"_{int(fr):04d}.png")
        bpy.ops.render.render(write_still=True)
        print("wrote", sc.render.filepath)
