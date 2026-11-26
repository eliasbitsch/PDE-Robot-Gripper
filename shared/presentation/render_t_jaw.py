"""Still of the final T-jaw (jaw L with pad plate and NBR pad) from the committed STEP files.

python render_t_jaw.py  ->  shared/presentation/media/t_jaw.png
"""
import os

import cadquery as cq
import numpy as np
import pyvista as pv

HERE = os.path.dirname(os.path.abspath(__file__))
JAWS = os.path.join(HERE, "..", "..", "Viktoriia", "03_CAD_Jaws")
OUT = os.path.join(HERE, "media")
PARTS = [("jaw_L.step", "#c2410c"), ("pad_plate_L.step", "#737a80"), ("pad_NBR_L.step", "#151515")]


def mesh(path):
    shape = cq.importers.importStep(path).val()
    verts, tris = shape.tessellate(0.05, 0.2)
    pts = np.array([(v.x, v.y, v.z) for v in verts])
    faces = np.hstack([[3, *t] for t in tris])
    return pv.PolyData(pts, faces).compute_normals(split_vertices=True, feature_angle=30)


def main():
    os.makedirs(OUT, exist_ok=True)
    pl = pv.Plotter(off_screen=True, window_size=(1600, 1200))
    pl.set_background("white")
    meshes = [mesh(os.path.join(JAWS, name)) for name, _ in PARTS]
    for m, (_, color) in zip(meshes, PARTS):
        pl.add_mesh(m, color=color, smooth_shading=True, specular=0.3)
    pl.enable_anti_aliasing("ssaa")
    # T outline lies in the y-z plane: look mostly along x, a little from the side and above.
    c = np.array(pv.MultiBlock(meshes).combine().center)
    d = 400.0
    pl.camera_position = [tuple(c + np.array([-d, -0.55 * d, 0.35 * d])), tuple(c), (0, 0, 1)]
    pl.reset_camera()
    pl.camera.zoom(1.1)
    pl.screenshot(os.path.join(OUT, "t_jaw.png"))


if __name__ == "__main__":
    main()
