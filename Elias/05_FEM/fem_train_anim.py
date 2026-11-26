"""Animated FEM of the T-jaw train (LC2): load ramps up, the train bends (scaled), camera turns once.

python fem_train_anim.py  ->  figures/train_lc2_anim.mp4, figures/train_lc2_anim.gif
"""
import os
import shutil
import subprocess

import numpy as np

import fem_train as T

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
N, FPS, SCALE = 150, 30, 300.0


def capture():
    cache = os.path.join(T.J.OUT, "train_lc2_anim.npz")
    if os.path.exists(cache):
        z = np.load(cache, allow_pickle=True)
        return {k: z[k].item() if z[k].dtype == object and z[k].shape == () else z[k] for k in z.files}
    got = {}

    def keep(nodes, elems, ids, u, vm, path, force, scale=SCALE):
        got.update(nodes=nodes, elems=elems, ids=ids, u=u, vm=vm, force=force)

    T.render = keep
    T.run(only=["LC2"], png="unused")
    np.savez(cache, nodes=np.array(got["nodes"], dtype=object), elems=np.array(got["elems"], dtype=object),
             ids=np.array(got["ids"]), u=got["u"], vm=got["vm"], force=got["force"])
    return got


def animate(r):
    import pyvista as pv
    pv.OFF_SCREEN = True
    ids = r["ids"]
    index = {t: i for i, t in enumerate(ids)}
    pts0 = np.array([r["nodes"][t] for t in ids])
    cells = np.array([[4] + [index[n] for n in e[:4]] for e in r["elems"].values()]).ravel()
    types = np.full(len(r["elems"]), pv.CellType.TETRA, dtype=np.uint8)
    g = pv.UnstructuredGrid(cells, types, pts0.copy())
    vmax = float(np.percentile(r["vm"], 99.5))
    g.point_data["von Mises [MPa]"] = r["vm"] * 0
    p = pv.Plotter(off_screen=True, window_size=(1280, 720))
    p.set_background("white")
    p.add_mesh(pv.UnstructuredGrid(cells, types, pts0), style="wireframe", color="#9aa5ab", opacity=0.10)
    p.add_mesh(g, scalars="von Mises [MPa]", cmap="turbo", clim=(0, vmax), smooth_shading=True,
               scalar_bar_args=dict(title="von Mises [MPa]", vertical=True, n_labels=5, fmt="%.0f", color="black"))
    txt = p.add_text("", position="upper_left", font_size=13, color="black")
    p.view_vector((1.0, -1.4, 0.5), viewup=(0, 0, 1))
    p.reset_camera()
    p.camera.zoom(1.15)
    pos0 = np.array(p.camera.position)
    foc = np.array(p.camera.focal_point)
    tmp = os.path.join(FIG, "_anim")
    os.makedirs(tmp, exist_ok=True)
    for k in range(N):
        t = k / (N - 1)
        s = min(1.0, t / 0.45)
        s = s * s * (3 - 2 * s)                                  # ease in/out of the load ramp, then hold
        g.points[:] = pts0 + SCALE * s * r["u"]
        g.point_data["von Mises [MPa]"][:] = r["vm"] * s
        g.Modified()
        ang = np.radians(-50 * t)                                # slow turn about the vertical axis
        d = pos0 - foc
        rot = np.array([[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
        p.camera.position = foc + rot @ d
        txt.SetText(2, f"T-jaw drive train, full motor force\nF = {r['force'] * s:.1f} N per jaw   "
                       f"(deformation x{SCALE:.0f})")
        p.render()
        p.screenshot(os.path.join(tmp, f"f_{k:04d}.png"))
    p.close()
    mp4, gif = os.path.join(FIG, "train_lc2_anim.mp4"), os.path.join(FIG, "train_lc2_anim.gif")
    FF = [shutil.which("ffmpeg"), "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(tmp, "f_%04d.png")]
    subprocess.run(FF + ["-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", mp4], check=True)
    subprocess.run(FF + ["-vf", "fps=15,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse",
                         gif], check=True)
    for f in os.listdir(tmp):
        os.remove(os.path.join(tmp, f))
    os.rmdir(tmp)
    print("wrote", mp4, gif)


if __name__ == "__main__":
    animate(capture())
