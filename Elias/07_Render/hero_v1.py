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
    # the approved views (review 21 Nov) as shots (review 22 Nov): every shot starts close and pulls out, is slow at its
    # view, then pushes in on a detail; the cut falls inside that push with a short dip to black. Flowing moves:
    # three-quarter -> top, jaws -> drive. env ENGINE=EEVEE for quick previews, STEP, FSTART, FRAMES
    zp = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1]) * MM
    jaw_t, drive_t = Vector((0, 0, zp)), Vector((-42, -5, 2)) * MM
    flange = Vector((c.x, c.y, hi.z))
    SHOTS = [  # per shot: keys (frame, target, direction, distance, lens, speed at the key); a new shot = a cut
        [(1, c, (0.75, -1.6, 0.0), 0.76, 85, 1.3), (55, c, (0.85, -1.6, -0.04), 0.98, 85, 0.3),
         (105, (c + jaw_t) / 2, (0.95, -1.6, -0.06), 0.68, 85, 1.4)],
        [(106, c, (1.1, -0.9, 0.62), 0.80, 70, 1.3), (145, c, (1.0, -1.0, 0.65), 1.05, 70, 0.35),
         (195, c, (0.08, -0.35, 1.0), 1.05, 70, 0.35), (230, flange, (0.05, -0.3, 1.0), 0.62, 70, 1.4)],
        [(231, c, (-1.0, -0.2, 0.06), 0.74, 85, 1.3), (275, c, (-1.0, -0.12, 0.05), 0.95, 85, 0.35),
         (320, (c + jaw_t) / 2, (-1.0, -0.3, 0.08), 0.6, 95, 1.3)],
        [(321, jaw_t, (0.22, -1.0, 0.10), 0.44, 100, 1.2), (380, jaw_t, (0.45, -1.0, 0.14), 0.35, 100, 0.35),
         (411, (jaw_t + drive_t) / 2, (-0.3, -1.5, 0.5), 0.62, 90, 1.0),   # pull back: do not fly through the parts
         (440, drive_t, (-0.9, -1.5, 0.75), 0.50, 85, 0.5), (515, drive_t, (-0.78, -1.6, 0.7), 0.47, 85, 0.5),
         (600, drive_t, (-0.6, -1.65, 0.62), 0.43, 85, 0.6)]]
    JAWS_D = [(40, 72, D_OPEN, D_GRIP), (150, 180, D_GRIP, D_OPEN), (268, 300, D_OPEN, D_GRIP),
              (330, 352, D_GRIP, D_OPEN), (358, 380, D_OPEN, D_GRIP), (450, 475, D_GRIP, D_OPEN),
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
    FADE = (380, 420)
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
    for f, e in ((1, 70), (170, 70), (190, 10), (230, 10), (231, 70), (FADE[0], 70), (FADE[1], 12)):
        pool.energy = e
        pool.keyframe_insert("energy", frame=f)
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
        for cf in cuts:
            if cf - 12 <= f < cf + 12:                  # 0.8 s dip; 5 frames felt cheap (review 22 Nov)
                e = -8 * (1 - ease(abs(f - (cf - 0.5)) / 12))
        if f >= END_FADE[0]:
            e = -10 * ease((f - END_FADE[0]) / (END_FADE[1] - END_FADE[0]))
        vs.exposure = e
        vs.keyframe_insert("exposure", frame=f)
    vs.exposure = 0.0
    json.dump({"fps": FPS, "end": END, "cuts": cuts, "jaws": JAWS_D, "fade": FADE, "end_fade": END_FADE,
               "moves": [(145, 195), (380, 440)]},   # the two flowing camera moves
              open(os.path.join(HERE, "out", "flight_timeline.json"), "w"))
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
    sc.render.filepath = os.path.join(HERE, "out", "flight", "f_")
    bpy.ops.render.render(animation=True)
    print("flight done", sc.render.engine)
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
