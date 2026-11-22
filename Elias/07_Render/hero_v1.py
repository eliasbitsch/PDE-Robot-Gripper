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

def lab_set(FLOOR_Z=-1.0, CEIL_Z=2.2):
    # lab set after Elias' reference (22 Nov): polished concrete floor 1 m below, ceiling grid with light strips and
    # spots at 3.2 m, glass walls, white machines in the distance; Poly Haven "Autoshop 01" (CC0) as environment for
    # the reflections. Real geometry, so the joint rise of gripper and camera shows parallax. Flash in from white.
    import urllib.request
    hdr = os.path.join(HERE, "out", "env", "autoshop_01_2k.hdr")
    if not os.path.exists(hdr):
        os.makedirs(os.path.dirname(hdr), exist_ok=True)
        req = urllib.request.Request("https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/autoshop_01_2k.hdr",
                                     headers={"User-Agent": "Mozilla/5.0"})
        open(hdr, "wb").write(urllib.request.urlopen(req).read())
    wn = sc.world.node_tree
    bg = wn.nodes["Background"]
    env = wn.nodes.new("ShaderNodeTexEnvironment")
    env.image = bpy.data.images.load(hdr)
    wn.links.new(env.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.8
    # the HDRI only lights and reflects (it shows cars); the camera sees the set, and a plain grey where nothing is
    lp = wn.nodes.new("ShaderNodeLightPath")
    plain = wn.nodes.new("ShaderNodeBackground")
    plain.inputs["Color"].default_value = (0.55, 0.57, 0.6, 1)
    mix = wn.nodes.new("ShaderNodeMixShader")
    out_w = next(n for n in wn.nodes if n.type == "OUTPUT_WORLD")
    wn.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    wn.links.new(bg.outputs["Background"], mix.inputs[1])
    wn.links.new(plain.outputs["Background"], mix.inputs[2])
    wn.links.new(mix.outputs["Shader"], out_w.inputs["Surface"])
    bpy.data.lights["Pool"].animation_data_clear()
    bpy.data.lights["Pool"].energy = 0
    for n_ in ("Key", "Top", "Fill"):                   # the environment lights the room; keep the rims for the edges
        bpy.data.lights[n_].energy *= 0.6

    def box(name, lo_, hi_, m):
        me = bpy.data.meshes.new(name)
        x0, y0, z0 = lo_; x1, y1, z1 = hi_
        v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        me.from_pydata(v, [], [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)])
        ob_ = bpy.data.objects.new(name, me)
        sc.collection.objects.link(ob_)
        ob_.data.materials.append(m)
        return ob_

    def emit(name, strength):
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        nt = m.node_tree
        nt.nodes.remove(next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"))
        e = nt.nodes.new("ShaderNodeEmission")
        e.inputs["Strength"].default_value = strength
        nt.links.new(e.outputs["Emission"], next(n for n in nt.nodes if n.type == "OUTPUT_MATERIAL").inputs["Surface"])
        return m

    fl_ob = bpy.data.objects["Floor"]
    fl_ob.data.vertices.foreach_set("co", [c_ for v in fl_ob.data.vertices for c_ in (v.co.x * 4, v.co.y * 4, FLOOR_Z)])
    fm = fl_ob.data.materials[0]
    fbsdf = next(n for n in fm.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    noise = fm.node_tree.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 6.0
    ramp = fm.node_tree.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.30, 0.31, 0.32, 1)
    ramp.color_ramp.elements[1].color = (0.42, 0.43, 0.44, 1)
    fm.node_tree.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    fm.node_tree.links.new(ramp.outputs["Color"], fbsdf.inputs["Base Color"])
    fbsdf.inputs["Roughness"].default_value = 0.12             # polished concrete
    beam = mat("beam", (0.85, 0.86, 0.87), 0.0, 0.5)
    strip, spot = emit("strip", 6.0), emit("spot", 25.0)
    for i in range(-4, 5):                                      # ceiling grid, 1.2 m pitch, strips on every other beam
        box(f"BeamX{i}", (-6, i * 1.2 - 0.04, CEIL_Z), (6, i * 1.2 + 0.04, CEIL_Z + 0.12), beam)
        box(f"BeamY{i}", (i * 1.2 - 0.04, -6, CEIL_Z + 0.12), (i * 1.2 + 0.04, 6, CEIL_Z + 0.2), beam)
        if i % 2 == 0:
            box(f"Strip{i}", (-5.5, i * 1.2 - 0.025, CEIL_Z - 0.01), (5.5, i * 1.2 + 0.025, CEIL_Z), strip)
        for j in range(-4, 5):
            if (i + j) % 2:
                box(f"Spot{i}_{j}", (j * 1.2 + 0.3, i * 1.2 + 0.25, CEIL_Z - 0.06), (j * 1.2 + 0.36, i * 1.2 + 0.31, CEIL_Z), spot)
    glass = bpy.data.materials.new("glass")
    glass.use_nodes = True
    gb = next(n for n in glass.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    # see-through by alpha, not transmission: EEVEE refracts the world probe (the HDRI with its cars), not the set
    gb.inputs["Alpha"].default_value = 0.14
    gb.inputs["Roughness"].default_value = 0.03
    gb.inputs["Base Color"].default_value = (0.8, 0.9, 0.93, 1)
    if hasattr(glass, "surface_render_method"):
        glass.surface_render_method = "BLENDED"
    frame_m = mat("frame", (0.6, 0.62, 0.64), 1.0, 0.3)
    for k_, (x0, x1, y0, y1) in enumerate(((-6, 6, 2.6, 2.62), (-3.2, -3.18, -6, 6))):   # glass walls behind and left
        box(f"Glass{k_}", (x0, y0, FLOOR_Z), (x1, y1, CEIL_Z), glass)
        n_ = 10
        for q in range(n_ + 1):
            u = q / n_
            px_, py_ = x0 + (x1 - x0) * u, y0 + (y1 - y0) * u
            box(f"Mullion{k_}_{q}", (px_ - 0.02, py_ - 0.02, FLOOR_Z), (px_ + 0.02, py_ + 0.02, CEIL_Z), frame_m)
    wall = mat("wall", (0.78, 0.79, 0.8), 0.0, 0.6)    # room walls beyond the glass, so the camera sees only the set
    for k_, (lo_w, hi_w) in enumerate((((-7, 6.4, FLOOR_Z), (7, 6.6, CEIL_Z + 0.4)), ((-6.6, -7, FLOOR_Z), (-6.4, 7, CEIL_Z + 0.4)),
                                       ((6.4, -7, FLOOR_Z), (6.6, 7, CEIL_Z + 0.4)), ((-7, -7, CEIL_Z + 0.4), (7, 7, CEIL_Z + 0.5)))):
        box(f"Wall{k_}", lo_w, hi_w, wall)
    for k_, x in enumerate((-4.8, -2.4, 0.0, 2.4, 4.8)):                  # window bands in the back wall, like the reference
        box(f"WinBand{k_}", (x - 0.9, 6.35, FLOOR_Z + 0.9), (x + 0.9, 6.4, CEIL_Z - 0.3), emit(f"winband{k_}", 2.5))
    white = mat("machine", (0.86, 0.87, 0.88), 0.0, 0.35)
    dark = mat("machine_glass", (0.05, 0.07, 0.09), 0.0, 0.1)
    for k_, (x, y) in enumerate(((-1.6, 4.2), (0.6, 4.6), (2.6, 4.0), (-4.6, 1.4), (-4.8, -1.2))):
        box(f"Machine{k_}", (x - 0.55, y - 0.5, FLOOR_Z), (x + 0.55, y + 0.5, FLOOR_Z + 1.9), white)
        box(f"MachineWin{k_}", (x - 0.35, y - 0.52, FLOOR_Z + 0.8), (x + 0.35, y - 0.5, FLOOR_Z + 1.5), dark)


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
    # shot 4: self-assembly in a white lab. Reviews 22 Nov: parts arrive group by group along their own mounting axis
    # (screws from their head side) on paths clear of the parts in place; the gripper turns so the camera looks
    # isometrically at the joint, the side the parts come from facing it (a bit steeper from below). Consecutive groups
    # at the same place whose directions fit one view form a shot: inside a shot nothing moves but the parts. Strict
    # order: parts land, short hold, then the camera turns or zooms, then the next parts start.
    # Order: parts/demo_seq.py STEP_TEXT, except pinion shaft before the drive (the bevel wheel would pass through the
    # bevel pinion), racks before slides (rack R floats above slide L), cover before the holders (it would pass
    # through them), window plate before the cable gland; the drive threads on like a sandwich.
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
    GROUPS = [  # (parts by prefix, where they come from: a direction, "in" = from the centre, "out" = from outside,
                #  "head" = along the screw axis from its head side; travel in mm)
        (("Rail",), DOWN, 70), (("Screw_ISO4762-M3x8_Rail",), "head", 40),
        (("Pinion_Shaft", "Bearing_608_lower", "Circlip_DIN471"), DOWN, 70),
        (("Bearing_608_upper", "Circlip_DIN472"), UP, 60), (("Shim_", "Key_"), UP, 50), (("Bevel_Wheel",), UP, 60),
        (("Setscrew",), LEFT, 35),
        (("Bracket", "Gearbox", "Bevel_Pinion"), RIGHT, 140),
        (("Motor",), RIGHT, 110), (("Brake",), RIGHT, 90), (("Encoder",), RIGHT, 70),
        (("Screw_ISO4762-M4x12_Gearbox",), "head", 40),   # after the sandwich: no turn back and forth
        (("Screw_ISO4762-M3x10_Bracket",), "head", 40),
        (("Hood",), UP, 90), (("Screw_ISO4762-M4x12_Hood",), "head", 45),
        (("Carriage_",), DOWN, 60), (("Rack_",), DOWN, 60), (("Slide_",), DOWN, 60),
        (("Screw_ISO4762-M3x8_Carriage", "Screw_ISO4762-M3x10_Rack"), "head", 40), (("Dowel_ISO8734-3x14_Rack",), DOWN, 40),
        (("Cover",), DOWN, 70), (("Screw_ISO10642-M3x10_Cover",), "head", 40),
        (("Holder_",), DOWN, 60), (("Screw_ISO4762-M4x25_Holder",), "head", 50),
        (("Dowel_ISO8734-4x12",), "in", 30), (("Plunger_",), "out", 50),
        (("Jaw_",), "in", 28), (("PadPlate_", "Pad_"), "in", 18), (("Screw_ISO4762-M3x6_Pad", "Washer_"), "head", 25),
        (("Window_Plate",), FRONT, 60), (("Screw_ISO10642-M3x8_Window",), "head", 40), (("Cable_Gland",), FRONT, 60),
        (("Adapter_ISO9409",), UP, 80), (("Screw_ISO10642-M4x16_Adapter",), "head", 45),
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

    def head_dir(ob):
        """screw axis pointing to the head: the half (along the longest extent) with the larger radius"""
        vs = [v.co for v in ob.data.vertices]
        dims = [max(v[i] for v in vs) - min(v[i] for v in vs) for i in range(3)]
        ax = dims.index(max(dims))
        o1, o2 = [i for i in range(3) if i != ax]
        r = lambda v: math.hypot(v[o1], v[o2])
        pos = max((r(v) for v in vs if v[ax] > 0), default=0)
        neg = max((r(v) for v in vs if v[ax] < 0), default=0)
        d = Vector((0, 0, 0))
        d[ax] = 1.0 if pos > neg else -1.0
        return d

    groups, used = [], set()                           # (members, {name: from-direction}, travel m, poi, size)
    for prefixes, how, dist in GROUPS:
        members = sorted(n for n in objs if n.startswith(prefixes) and n not in used)
        if not members:
            continue
        used.update(members)
        dirs = {}
        for n in members:
            s_ = -1 if objs[n].location.x < -1e-4 else 1
            dirs[n] = (Vector((-s_, 0, 0)) if how == "in" else Vector((s_, 0, 0)) if how == "out"
                       else head_dir(objs[n]) if how == "head" else Vector(how))
        locs = [objs[n].location for n in members]
        lo_ = Vector([min(v[i] for v in locs) for i in range(3)])
        hi_ = Vector([max(v[i] for v in locs) for i in range(3)])
        groups.append((members, dirs, dist * MM, (lo_ + hi_) / 2, (hi_ - lo_).length + dist * MM))
    print("assemble unplaced:", sorted(set(objs) - used))

    def constraints(dirs):
        """axis -> sign the camera side must have; an axis the members disagree on (L/R pairs) stays free"""
        out = {}
        for ax in range(3):
            signs = {1 if d[ax] > 0.5 else -1 for d in dirs.values() if abs(d[ax]) > 0.5}
            if len(signs) == 1:
                out[ax] = signs.pop()
            elif len(signs) == 2 and ax == 0:          # symmetric pair along the grip axis: look from below-front
                out.setdefault(1, -1)
                out.setdefault(2, -1)
        return out

    # shots: consecutive groups within 7 cm whose view constraints agree
    shots = []
    for g in groups:
        cg = constraints(g[1])
        if shots:
            sh = shots[-1]
            ok = (g[3] - sh["anchor"]).length < 0.06 and all(sh["cons"].get(a, s) == s for a, s in cg.items())
            if ok:                                     # distance to the shot's first group, so a shot cannot creep
                sh["groups"].append(g)
                sh["cons"].update(cg)
                pts = [objs[n].location for gg in sh["groups"] for n in gg[0]]
                sh["poi"] = sum(pts, Vector()) / len(pts)
                continue
        shots.append({"groups": [g], "cons": dict(cg), "poi": g[3].copy(), "anchor": g[3].copy()})

    CD = Vector((0.35, -1.0, 0.18)).normalized()       # target -> camera, fixed
    UPW = Vector((0, 0, 1))
    U_PERP = (UPW - CD * UPW.dot(CD)).normalized()

    def iso_quat(sx, sy, sz):
        a1 = Vector((sx, sy, sz)).normalized()
        a2 = (UPW - a1 * UPW.dot(a1)).normalized()
        A = Matrix((a1, a2, a1.cross(a2))).transposed()
        B = Matrix((CD, U_PERP, CD.cross(U_PERP))).transposed()
        return (B @ A.transposed()).to_quaternion()

    def orient(cons, q_prev):
        cands = []
        for sx in (1, -1):
            for sy in (1, -1):
                for sz in (1, -1):
                    if any(cons.get(a, s) != s for a, s in ((0, sx), (1, sy), (2, sz))):
                        continue
                    cands.append(iso_quat(sx, sy, sz * (1.5 if cons.get(2) == -1 else 1.0)))   # steeper from below
        return min(cands, key=lambda q: q_prev.rotation_difference(q).angle)

    DUR, HOLD, TRANS, STAG = 9, 7, 13, 1                # land, hold after landing, camera move, stagger in a group
    q0 = iso_quat(1, -1, 1)
    states = [(1, q0, c, 0.9, TRANS)]                   # (frame the camera arrives, orientation, target, distance, move frames)
    sched = []
    f = 14
    gidx = 0
    for sh in shots:
        pts = [objs[n].location for g in sh["groups"] for n in g[0]]
        lo_ = Vector([min(v[i] for v in pts) for i in range(3)])
        hi_ = Vector([max(v[i] for v in pts) for i in range(3)])
        size = (hi_ - lo_).length + max(g[2] for g in sh["groups"])
        dist = min(max(size * 1.6 + 0.2, 0.36), 0.8)
        if abs(dist - states[-1][3]) < 0.2 * states[-1][3]:
            dist = states[-1][3]                        # no small zoom steps
        q = orient(sh["cons"], states[-1][1])
        f += TRANS
        states.append((f, q, sh["poi"], dist, TRANS))
        f += 4
        for gi, (members, dirs, travel, _, _) in enumerate(sh["groups"]):
            for i, n in enumerate(members):
                sched.append((f + i * STAG, n, objs[n], dirs[n] * travel, gidx, dirs[n], travel))
            gidx += 1
            last = gi == len(sh["groups"]) - 1        # the camera waits for the full landing; groups in a shot overlap
            f += STAG * (len(members) - 1) + (DUR + HOLD if last else int(DUR * 0.5))
        print("shot", len(states) - 1, sh["cons"], [g[0][0] for g in sh["groups"]])
    f += 24
    states.append((f, q0, c, 0.8, 24))                 # finished gripper: classic iso from above
    # finale (22 Nov): everything flies apart in reverse order along its joining axis, stacked like a sandwich
    # (later parts further out on the same axis), the camera pulls out to show all parts in the air, a slow turn,
    # then everything snaps back at once like pulled by a strong magnet (accelerating, not easing in).
    G = gidx
    E0 = f + 10
    EX_STEP, EX_DUR, SNAP, STAGE = 2, 14, 10, 0.45
    # exploded offsets, stacked like a sandwich: a part goes out along its joining axis (jaws, pads and holder dowels
    # outward, as when the gripper opens) until it clears, with 12 mm air, every earlier part that flies the same way
    # and overlaps it sideways. Without this, motor/brake/encoder ended up on top of each other and screws inside
    # their plates (review 22 Nov).
    bpy.context.view_layer.update()

    def wbox(ob):
        pts = [ob.matrix_world @ Vector(v) for v in ob.bound_box]
        return Vector([min(q[i] for q in pts) for i in range(3)]), Vector([max(q[i] for q in pts) for i in range(3)])

    OUTWARD = ("Jaw_", "PadPlate_", "Pad_", "Dowel_ISO8734-4x12")
    expl, placed = {}, []
    for f0, n, ob, off, gi, d, travel in sched:        # assembly order: inner parts first
        ed = -d if n.startswith(OUTWARD) else d
        ax = max(range(3), key=lambda i: abs(ed[i]))
        sg = 1 if ed[ax] > 0 else -1
        lo, hi = wbox(ob)
        near, far = (lo[ax], hi[ax]) if sg > 0 else (-hi[ax], -lo[ax])
        o = max(0.035, travel * 0.5)
        for ax2, sg2, lo2, hi2, far2 in placed:
            if ax2 == ax and sg2 == sg and all(lo[i] < hi2[i] - 0.002 and lo2[i] < hi[i] - 0.002 for i in range(3) if i != ax):
                o = max(o, far2 + 0.012 - near)
        placed.append((ax, sg, lo, hi, far + o))
        u = Vector((0, 0, 0))
        u[ax] = sg
        expl[n] = (E0 + (G - 1 - gi) * EX_STEP, u * o)
    pts = []
    for f0, n, ob, off, gi, d, travel in sched:
        lo, hi = wbox(ob)
        pts += [lo + expl[n][1], hi + expl[n][1]]
    ex_lo = Vector([min(q[i] for q in pts) for i in range(3)])
    ex_hi = Vector([max(q[i] for q in pts) for i in range(3)])
    dist_ex = max(1.3, (ex_hi - ex_lo).length * 1.5)
    dist_mid = 0.5 * (dist_ex + 0.85)
    EX_END = E0 + G * EX_STEP + EX_DUR
    S1 = EX_END + 75                                   # a full turn in the exploded view
    S2 = S1 + SNAP + 36                                # first snap to a stage, a further turn, then the final snap
    END = S2 + SNAP + 50
    print("finale frames: explode", E0, EX_END, "stage", S1, "final snap", S2, "end", END)
    states.append((EX_END, q0, c, dist_ex, EX_END - f))
    states.append((S1, q0, c, dist_ex, 10))
    states.append((S1 + SNAP, q0, c, dist_mid, SNAP))
    states.append((S2, q0, c, dist_mid, 10))
    states.append((S2 + SNAP, q0, c, 0.85, SNAP))
    states.append((END, q0, c, 0.8, 50))

    def yaw_extra(fr):                                 # turns of the finale, on top of the shot orientation
        y = 2 * math.pi * ease(min(1.0, max(0.0, (fr - EX_END) / (S1 - EX_END))))
        y += (2 * math.pi / 3) * ease(min(1.0, max(0.0, (fr - S1 - SNAP) / (S2 - S1 - SNAP))))
        y += math.radians(30) * ease(min(1.0, max(0.0, (fr - S2 - SNAP) / (END - S2 - SNAP))))
        return y

    def explode_factor(fr, e0):
        if fr < S1:
            return ease(min(1.0, max(0.0, (fr - e0) / EX_DUR)))
        if fr < S1 + SNAP:                             # magnet: accelerating into the stage
            return 1.0 - (1.0 - STAGE) * ((fr - S1) / SNAP) ** 3
        if fr < S2:
            return STAGE
        if fr < S2 + SNAP:
            return STAGE * (1.0 - ((fr - S2) / SNAP) ** 3)
        return 0.0
    print("assemble shots", len(shots), "groups", len(groups))

    nudge_loc = [Vector() for _ in range(END + 2)]
    nudge_rot = [Vector() for _ in range(END + 2)]     # axis * angle in the gripper frame
    rng_ = __import__("random").Random(4)
    for f0, name, ob, off, gi, d, travel in sched:
        base = ob.location.copy()
        e0, eoff = expl[name]
        axis = off.normalized()
        tilt_ax = axis.cross(Vector((0.3, 0.7, 0.2))).normalized()
        tilt = math.radians(rng_.uniform(4, 8))
        seat = f0 + DUR
        for fr in range(1, END + 1):
            t = (fr - f0) / DUR
            s = 0.0 if t <= 0 else min(1.0, 1 - (1 + 7 * t) * math.exp(-7 * t)) if t < 1 else 1.0
            e = explode_factor(fr, e0)
            ob.location = base + off * (1 - s) + eoff * e
            ob.hide_render = t <= 0
            if name.startswith(("Screw", "Setscrew")):
                q = Quaternion(axis, ((1 - s) + e * 0.66) * 6 * math.pi)   # three turns in, two out and back
            else:
                q = Quaternion(tilt_ax, tilt * (1 - s))                     # straightens out on the way in
            ob.rotation_quaternion = q
            ob.keyframe_insert("location", frame=fr)
            ob.keyframe_insert("rotation_quaternion", frame=fr)
            ob.keyframe_insert("hide_render", frame=fr)
        # the assembly yields along the joining axis and tilts about the lever from the joint to its centre: plates
        # snap in with a firm push and a clear tilt, screws hardly at all (reviews 22 Nov)
        amp = 1.4 if name.startswith(PLATES) else 0.05 if name.startswith(FASTENERS) else 0.5
        lever = base - c
        tq = (-axis).cross(lever)
        tilt_amp = math.radians(2.0 if name.startswith(PLATES) else 0.03 if name.startswith(FASTENERS) else 0.9)
        tq = tq.normalized() * tilt_amp * min(1.0, lever.length / 0.04) if tq.length > 1e-6 else Vector()
        for u in range(0, 12):
            if seat + u <= END:
                prof = math.exp(-u / 2.8) * math.sin(math.pi * u / 4.5)
                nudge_loc[seat + u] += -axis * amp * MM * prof
                nudge_rot[seat + u] += tq * prof
    for t_snap, k_ in ((S1 + SNAP, 0.5), (S2 + SNAP, 1.0)):   # a common jolt at the stage and when everything is home
        for u in range(0, 14):
            prof = k_ * math.exp(-u / 3) * math.sin(math.pi * u / 5)
            nudge_loc[t_snap + u] += Vector((0, 0, -1)) * 2.5 * MM * prof
            nudge_rot[t_snap + u] += Vector((1, 0.4, 0)).normalized() * math.radians(0.8) * prof
    lab_set()
    RISE = 1.1                                       # gripper and camera rise together 1.1 m over the shot
    vs = sc.view_settings
    for fr in range(1, 15):
        vs.exposure = 4 * (1 - ease((fr - 1) / 13)) - 0.2
        vs.keyframe_insert("exposure", frame=fr)
    # camera fixed in front; the gripper turns. Between two states the move takes the TRANS frames before arrival.
    cam = camera("CamAssemble", c, CD, 0.9, 50, 3.5)   # shallower depth of field: the room goes soft
    cam.constraints.clear()
    foc = cam.data.dof.focus_object
    root.rotation_mode = "QUATERNION"
    for fr in range(1, END + 1):
        j = max(i for i in range(len(states)) if states[i][0] <= fr)
        if j == len(states) - 1:
            v, j2 = 0.0, j
        else:
            j2 = j + 1
            span = min(states[j2][4], states[j2][0] - states[j][0])
            v = ease(min(1.0, max(0.0, (fr - (states[j2][0] - span)) / span)))
        qv = Quaternion((0, 0, 1), yaw_extra(fr)) @ states[j][1].slerp(states[j2][1], v)
        poi = states[j][2].lerp(states[j2][2], v)
        dist = states[j][3] + (states[j2][3] - states[j][3]) * v
        if os.environ.get("NOZOOM"):                    # whole gripper always in frame: only the turns
            poi, dist = c, 0.82
        nr = nudge_rot[fr]
        root.rotation_quaternion = qv @ (Quaternion(nr.normalized(), nr.length) if nr.length > 0 else Quaternion())
        rise = Vector((0, 0, RISE * ease(fr / END)))      # gripper and camera rise together through the room
        root.location = qv @ nudge_loc[fr] + rise
        root.keyframe_insert("rotation_quaternion", frame=fr)
        root.keyframe_insert("location", frame=fr)
        t_ = qv @ poi + rise
        cam.location, foc.location = t_ + CD * dist, t_
        cam.rotation_euler = (-CD).to_track_quat("-Z", "Y").to_euler()
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
    print("assemble frames", END)
    sc.frame_start, sc.frame_end = int(os.environ.get("FSTART", 1)), int(os.environ.get("FRAMES", END))
    sc.frame_step = int(os.environ.get("STEP", "1"))
    sc.render.filepath = os.path.join(HERE, "out", "assemble" + ("_nozoom" if os.environ.get("NOZOOM") else ""), "f_")
    bpy.ops.render.render(animation=True)
    print("assemble done", sc.render.engine)
elif mode == "robot":
    # shot 6 (review 22 Nov): IRB 1300 in the lab, bowed down as if asleep; the gripper snaps onto the flange along
    # the tool axis, the cable draws itself from the gland to the forearm and clicks in, the robot rises in one move,
    # turns to the camera with the gripper level like a head, tilts it, blinks with the jaws and waves with axis 6.
    # Robot links: ABB STEP as STL in the robot frame at home pose (04_Simulation/meshes), axes from cell_params.
    from mathutils import Matrix, Quaternion
    sys.path.insert(0, os.path.join(HERE, "..", "06_Cell"))
    import cell_params as CP  # noqa: E402
    MESHES = os.path.join(HERE, "..", "04_Simulation", "meshes")
    sc.timeline_markers.clear()
    root.animation_data_clear()
    for p_ in piv.values():
        p_.animation_data_clear()
    FLOOR = -1.0
    BASE = Vector((0, 0, FLOOR + 0.5))                 # 500 mm pedestal as in the cell
    # dark reveal look of the hero film (review 22 Nov): black glossy floor, rim lights, a pool of light under the robot
    fl_ob = bpy.data.objects["Floor"]
    fl_ob.data.vertices.foreach_set("co", [c_ for v in fl_ob.data.vertices for c_ in (v.co.x * 3, v.co.y * 3, FLOOR)])
    pool_ = bpy.data.lights["Pool"]
    pool_.animation_data_clear()
    pool_.energy, pool_.spot_size = 900, math.radians(45)
    bpy.data.objects["Pool"].location = (0.35, 0.0, FLOOR + 2.6)
    white = mat("abb_white", (0.88, 0.89, 0.9), 0.0, 0.3, coat=0.4)
    grey = mat("abb_grey", (0.22, 0.23, 0.24), 0.0, 0.45)
    ped = bpy.data.meshes.new("Pedestal")
    w_ = 0.17
    vv = [(-w_, -w_, FLOOR), (w_, -w_, FLOOR), (w_, w_, FLOOR), (-w_, w_, FLOOR),
          (-w_, -w_, BASE.z), (w_, -w_, BASE.z), (w_, w_, BASE.z), (-w_, w_, BASE.z)]
    ped.from_pydata(vv, [], [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)])
    pedo = bpy.data.objects.new("Pedestal", ped)
    sc.collection.objects.link(pedo)
    pedo.data.materials.append(grey)

    def keep_parent(child, parent):
        mw = child.matrix_world.copy()
        child.parent = parent
        child.matrix_world = mw

    chain = CP.joint_chain()
    joints, prev = [], None
    for i, (axis, centre) in enumerate(chain):
        j = bpy.data.objects.new(f"A{i + 1}", None)
        sc.collection.objects.link(j)
        j.location = BASE + Vector(centre) * MM
        bpy.context.view_layer.update()
        if prev is not None:
            keep_parent(j, prev)
        joints.append((j, [abs(a) > 0.5 for a in axis].index(True)))
        prev = j
    bpy.context.view_layer.update()
    for i in range(7):
        bpy.ops.wm.stl_import(filepath=os.path.join(MESHES, f"robot_link{i}.stl"), global_scale=MM)
        ob = bpy.context.selected_objects[0]
        ob.name = f"RobotLink{i}"
        ob.location = BASE
        ob.data.materials.append(grey if i in (0, 6) else white)
        ob.data.shade_smooth()
        try:
            ob.data.set_sharp_from_angle(angle=math.radians(30))
        except AttributeError:
            pass
        bpy.context.view_layer.update()
        if i > 0:
            keep_parent(ob, joints[i - 1][0])
    mount = bpy.data.objects.new("ToolMount", None)
    sc.collection.objects.link(mount)
    mount.location = BASE + Vector(CP.flange_home()) * MM
    mount.rotation_euler = (0, math.radians(-90), 0)    # gripper -z (jaws) along the tool axis +x
    bpy.context.view_layer.update()
    keep_parent(mount, joints[5][0])
    root.parent = mount
    root.matrix_parent_inverse = Matrix.Identity(4)
    root.rotation_euler = (0, 0, 0)
    ZTOP = hi.z                                         # flange face of the adapter in the gripper frame
    anchor = bpy.data.objects.new("CableAnchor", None)  # cable socket on the side of the upper arm
    sc.collection.objects.link(anchor)
    anchor.location = BASE + Vector((CP.A2_X + 120, -78, CP.A2_Z + CP.L2 - 60)) * MM
    bpy.context.view_layer.update()
    keep_parent(anchor, joints[2][0])

    SLEEP = [0, 30, 45, 0, 40, 0]                      # bowed: upper arm forward, forearm and wrist down
    UP = [-25, -12, -14, 0, 26, 0]                     # upright, gripper level
    LOOK = [-40, 4, -22, 0, 18, 14]                    # facing the camera, gripper level, tilted like a head
    G0, SEAT = 24, 38                                   # gripper flight
    C0, C1, PLUG = 66, 88, 92                           # cable draws, plug clicks
    R0, R1 = 106, 170                                   # rise
    L0, L1 = 186, 216                                   # turn to the camera
    W0, W1 = 272, 326                                   # wave
    CUTS = (61, 101, 181)                               # shots: flange, cable, rise, look
    END = 350

    def lerp_q(a, b, u):
        return [x + (y - x) * u for x, y in zip(a, b)]

    def pose(fr):
        if fr < R0:
            q = list(SLEEP)
            if SEAT <= fr < SEAT + 12:                  # the arm gives a little when the gripper snaps on
                q[4] += 1.6 * math.exp(-(fr - SEAT) / 3) * math.sin(math.pi * (fr - SEAT) / 5)
            return q
        if fr < R1:
            u = (fr - R0) / (R1 - R0)
            return lerp_q(SLEEP, UP, ease(u) + 0.04 * math.sin(math.pi * u) ** 2 * (u > 0.7))   # a slight stretch
        if fr < L0:
            return list(UP)
        if fr < L1:
            return lerp_q(UP, LOOK, ease((fr - L0) / (L1 - L0)))
        q = list(LOOK)
        if L1 <= fr < L1 + 24:                          # curious lean towards the camera and back
            q[1] += 5 * math.sin(math.pi * (fr - L1) / 24)
        if W0 <= fr < W1:
            u = (fr - W0) / (W1 - W0)
            env = math.sin(math.pi * u)
            q[5] += 28 * env * math.sin(4 * math.pi * u)
            q[0] += 6 * env * math.sin(4 * math.pi * u)
        return q

    for fr in range(1, END + 1):
        q = pose(fr)
        for (j, ax), a in zip(joints, q):
            e = [0.0, 0.0, 0.0]
            e[ax] = math.radians(a)
            j.rotation_euler = e
            j.keyframe_insert("rotation_euler", index=ax, frame=fr)
        # gripper: hidden before its flight, then pulled onto the flange like a magnet (accelerating)
        t = min(1.0, max(0.0, (fr - G0) / (SEAT - G0)))
        off = 0.22 * (1 - t ** 3)
        root.location = (0, 0, -ZTOP * MM - off)
        root.keyframe_insert("location", frame=fr)
        # jaws: open; blink twice after the turn
        d = D_OPEN
        for b0 in (234, 254):
            if b0 <= fr < b0 + 16:
                d = D_OPEN - (D_OPEN - D_GRIP) * math.sin(math.pi * (fr - b0) / 16)
        piv[-1].location.x, piv[1].location.x = -d * MM, d * MM
        piv["rot"].rotation_euler.z = piv["rotx"].rotation_euler.x = d / R_PIN
        for k in (-1, 1):
            piv[k].keyframe_insert("location", index=0, frame=fr)
        piv["rot"].keyframe_insert("rotation_euler", index=2, frame=fr)
        piv["rotx"].keyframe_insert("rotation_euler", index=0, frame=fr)
    for ob in objs.values():
        for fr, h in ((1, True), (G0, False)):
            ob.hide_render = h
            ob.keyframe_insert("hide_render", frame=fr)

    # cable: a poly curve through the gland exit and the arm socket, re-shaped every frame, drawn on by bevel factor
    cd_ = bpy.data.curves.new("Cable", "CURVE")
    cd_.dimensions = "3D"
    cd_.bevel_depth, cd_.bevel_resolution = 0.0045, 4
    NP = 24
    sp = cd_.splines.new("POLY")
    sp.points.add(NP - 1)
    cable = bpy.data.objects.new("Cable", cd_)
    sc.collection.objects.link(cable)
    cable.data.materials.append(mat("cable", (0.015, 0.015, 0.017), 0.0, 0.45))
    plug = bpy.data.meshes.new("Plug")
    bm_n, r_, h_ = 16, 0.009, 0.03
    pv = [(r_ * math.cos(2 * math.pi * k / bm_n), r_ * math.sin(2 * math.pi * k / bm_n), z) for z in (0, -h_) for k in range(bm_n)]
    pf = [(k, (k + 1) % bm_n, bm_n + (k + 1) % bm_n, bm_n + k) for k in range(bm_n)]
    pf += [tuple(range(bm_n)), tuple(range(2 * bm_n - 1, bm_n - 1, -1))]
    plug.from_pydata(pv, [], pf)
    plugo = bpy.data.objects.new("Plug", plug)
    sc.collection.objects.link(plugo)
    plugo.data.materials.append(mat("plug", (0.05, 0.05, 0.055), 1.0, 0.3))
    gland = objs[next(n for n in objs if n.startswith("Cable_Gland"))]

    def bez(p0, p1, p2, p3, u):
        return (1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * p1 + 3 * (1 - u) * u * u * p2 + u ** 3 * p3

    cam_pts = {}
    for fr in range(1, END + 1):
        sc.frame_set(fr)
        gm = gland.matrix_world
        out_g = (root.matrix_world.to_3x3() @ Vector((0, -1, 0))).normalized()
        p0 = gm.translation + out_g * 0.012
        am = anchor.matrix_world
        out_a = (am.to_3x3() @ Vector((0, -1, 0))).normalized()
        p3 = am.translation
        sag = Vector((0, 0, -0.06))
        p1 = p0 + out_g * 0.09 + sag
        p2 = p3 + out_a * 0.09 + sag
        for k in range(NP):
            pt = bez(p0, p1, p2, p3, k / (NP - 1))
            sp.points[k].co = (pt.x, pt.y, pt.z, 1)
            sp.points[k].keyframe_insert("co", frame=fr)
        u = min(1.0, max(0.0, (fr - C0) / (C1 - C0)))
        cd_.bevel_factor_end = ease(u)
        cd_.keyframe_insert("bevel_factor_end", frame=fr)
        v_ = min(1.0, max(0.0, (fr - (PLUG - 6)) / 6))
        plugo.matrix_world = Matrix.Translation(p3 + out_a * (0.002 + 0.03 * (1 - v_ ** 3))) @ \
            out_a.to_track_quat("Z", "Y").to_matrix().to_4x4()
        plugo.keyframe_insert("location", frame=fr)
        plugo.keyframe_insert("rotation_euler", frame=fr)
        plugo.hide_render = fr < PLUG - 6
        plugo.keyframe_insert("hide_render", frame=fr)
        tool = (root.matrix_world.to_3x3() @ Vector((0, 0, -1))).normalized()
        cam_pts[fr] = (mount.matrix_world.translation.copy(), p0.lerp(p3, 0.5), tool, root.matrix_world.translation.copy())

    # lights: the studio rig is light-linked to the Product collection and set up around the gripper: add the robot
    # and the cable to that collection and scale the rig up around the robot
    prod = bpy.data.collections["Product"]
    for ob in [o for o in sc.objects if o.name.startswith(("RobotLink", "Cable", "Plug"))]:
        if ob.name not in prod.objects:
            prod.objects.link(ob)
    rob_c = BASE + Vector((0.45, 0, 0.6))
    tgt.location = rob_c
    for n_ in ("Key", "Top", "RimL", "RimR", "Fill"):
        lo_ = bpy.data.objects[n_]
        lo_.location = rob_c + (lo_.location - c) * 3.5
        bpy.data.lights[n_].energy *= 10
        bpy.data.lights[n_].size *= 2.5
    # camera: four shots cut with short dips to dark, each a slow drift (flight style): flange, cable, the rise from
    # low and wide, then in front of the gripper when it looks at us
    def nrm(*v):
        return Vector(v).normalized()

    fl, cab, tool_l, grip_l = cam_pts[SEAT][0], cam_pts[PLUG][1], cam_pts[L1 + 10][2], cam_pts[L1 + 10][3]
    face = (tool_l + tool_l.cross(Vector((0, 0, 1))).normalized() * 0.75 + Vector((0, 0, 0.25))).normalized()
    up_c = rob_c + Vector((0, 0, 0.25))
    SHOTS_R = [  # (first frame, last frame, (target, direction, distance, lens) at start, ... at end)
        (1, CUTS[0] - 1, (fl, nrm(0.35, -1, 0.15), 0.75, 70), (fl, nrm(0.15, -1, 0.2), 0.58, 70)),
        (CUTS[0], CUTS[1] - 1, (cab, nrm(0.0, -1, 0.35), 0.62, 60), (cab, nrm(0.2, -1, 0.3), 0.52, 60)),
        (CUTS[1], CUTS[2] - 1, (up_c, nrm(0.9, -0.55, 0.0), 2.3, 30), (up_c, nrm(0.72, -0.72, 0.05), 2.0, 30)),
        (CUTS[2], END, (grip_l, face, 1.5, 70), (grip_l, face, 1.2, 70)),
    ]
    cam = camera("CamRobot", fl, SHOTS_R[0][2][1], 0.75, 70, 3.2)
    cam.constraints.clear()
    foc = cam.data.dof.focus_object
    for f0, f1, k0, k1 in SHOTS_R:
        for fr in range(f0, f1 + 1):
            u = (fr - f0) / max(1, f1 - f0)
            u = 0.5 * u + 0.5 * ease(u)                # drifts, never stops
            t_ = k0[0].lerp(k1[0], u)
            cd = k0[1].lerp(k1[1], u).normalized()
            dist = k0[2] + (k1[2] - k0[2]) * u
            cam.data.lens = k0[3] + (k1[3] - k0[3]) * u
            cam.location, foc.location = t_ + cd * dist, t_
            cam.rotation_euler = (-cd).to_track_quat("-Z", "Y").to_euler()
            cam.keyframe_insert("location", frame=fr)
            cam.keyframe_insert("rotation_euler", frame=fr)
            cam.data.keyframe_insert("lens", frame=fr)
            foc.keyframe_insert("location", frame=fr)
    # out of black at the start, a short dip to dark across every cut
    vs = sc.view_settings
    for fr in range(1, END + 1):
        e = -8 * (1 - ease(min(1.0, fr / 24)))
        for cf in CUTS:
            if cf - 8 <= fr < cf + 8:
                e = min(e, -7 * (1 - ease(abs(fr - (cf - 0.5)) / 8)))
        vs.exposure = e
        vs.keyframe_insert("exposure", frame=fr)
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
    print("robot frames", END)
    sc.frame_start, sc.frame_end = int(os.environ.get("FSTART", 1)), int(os.environ.get("FRAMES", END))
    sc.frame_step = int(os.environ.get("STEP", "1"))
    sc.render.filepath = os.path.join(HERE, "out", "robot", "f_")
    bpy.ops.render.render(animation=True)
    print("robot done", sc.render.engine)
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
