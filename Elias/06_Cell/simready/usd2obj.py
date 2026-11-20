"""SimReady USD -> one OBJ per material + manifest (colour, bounding box) for the MuJoCo cell. Run in Blender:

blender -b --factory-startup --python usd2obj.py -- <usd> <out_dir> <decimate_ratio>

Scene instancing is switched off on import (instanced rollers would collapse to the origin, see warehouse-twin).
Colour per material = mean of its albedo texture (or the base colour), good enough for renders of the cell.
"""
import json
import os
import sys

import bpy
import numpy as np

usd, out, ratio = sys.argv[sys.argv.index("--") + 1:][:3]
ratio = float(ratio)
os.makedirs(out, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.usd_import(filepath=usd, support_scene_instancing=False)
bpy.ops.object.select_all(action="DESELECT")
meshes = [o for o in bpy.data.objects if o.type == "MESH"]
for o in meshes:
    o.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
bpy.ops.object.make_single_user(object=True, obdata=True)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def colour(mat):
    if mat is None or not mat.use_nodes:
        return [0.6, 0.6, 0.6, 1.0]
    for n in mat.node_tree.nodes:
        if n.type == "BSDF_PRINCIPLED":
            inp = n.inputs["Base Color"]
            if inp.is_linked and inp.links[0].from_node.type == "TEX_IMAGE" and inp.links[0].from_node.image:
                img = inp.links[0].from_node.image
                try:
                    img.scale(32, 32)
                    px = np.array(img.pixels[:], dtype=np.float32).reshape(-1, 4)
                    return [float(v) for v in px[:, :3].mean(axis=0)] + [1.0]
                except Exception:
                    pass
            c = list(inp.default_value)
            return [float(c[0]), float(c[1]), float(c[2]), 1.0]
    return [0.6, 0.6, 0.6, 1.0]


# split by material: one object per material, then join per material
groups = {}
for o in meshes:
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    if len(o.data.materials) > 1:
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.separate(type="MATERIAL")
        bpy.ops.object.mode_set(mode="OBJECT")
for o in [o for o in bpy.data.objects if o.type == "MESH"]:
    m = o.data.materials[0] if o.data.materials else None
    groups.setdefault(m.name if m else "none", (m, []))[1].append(o)

parts, lo, hi = [], np.full(3, 1e9), np.full(3, -1e9)
for i, (name, (mat, objs)) in enumerate(sorted(groups.items())):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    if ratio < 1.0 and len(ob.data.polygons) > 2000:
        mod = ob.modifiers.new("dec", "DECIMATE")
        mod.ratio = ratio
        bpy.ops.object.modifier_apply(modifier="dec")
    v = np.array([ob.matrix_world @ p.co for p in ob.data.vertices])
    if len(v) < 4:
        continue
    lo, hi = np.minimum(lo, v.min(axis=0)), np.maximum(hi, v.max(axis=0))
    fn = f"part{i:02d}.obj"
    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True)
    bpy.ops.wm.obj_export(filepath=os.path.join(out, fn), export_selected_objects=True, export_materials=False,
                          export_uv=False, forward_axis="Y", up_axis="Z", apply_modifiers=True)
    parts.append(dict(file=fn, material=name, rgba=colour(mat), faces=len(ob.data.polygons)))
json.dump(dict(source=usd, parts=parts, bbox=[lo.tolist(), hi.tolist()]), open(os.path.join(out, "manifest.json"), "w"), indent=1)
print("RESULT", out, len(parts), "parts", "bbox", np.round(lo, 3).tolist(), np.round(hi, 3).tolist(),
      "faces", sum(p["faces"] for p in parts))
