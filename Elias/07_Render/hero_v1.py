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
