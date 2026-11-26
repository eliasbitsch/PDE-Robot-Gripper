"""Placement map laid over an orthographic side view of the MuJoCo cell (same x/z frame in mm).

python placement_overlay.py  ->  figures/placement_overlay.png
"""
import os
import sys

import matplotlib
import mujoco
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "04_Simulation"))
import robot_selection as RS  # noqa: E402
import sim as S  # noqa: E402

W, H = 1600, 1300
CX, CZ, VIEW_H = -50.0, 1050.0, 2100.0          # mm: view centre and visible height


def side_view(m, d):
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    # the free camera ignores the orthographic flag: a far telephoto camera is near enough (<2 % scale error)
    dist = 60.0
    m.vis.global_.fovy = float(np.degrees(2 * np.arctan(VIEW_H / 2000 / dist)))
    m.vis.map.zfar = 200.0
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for i in range(m.ngeom):                   # near side of the fence would hide the robot base
        if m.geom_bodyid[i] == 0 and i != floor and d.geom_xpos[i, 1] < -0.25:
            m.geom_rgba[i, 3] = 0.0
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (CX / 1000, 0.0, CZ / 1000)
    cam.distance, cam.azimuth, cam.elevation = dist, 90.0, 0.0
    r.update_scene(d, cam)
    img = r.render()
    r.close()
    return img


def main():
    m, d = S.load()
    q, _ = S.q_pick()
    S.set_robot(m, d, q)
    S.place_part_in_gripper(m, d)
    img = side_view(m, d)
    xs, hs, grid, rows = RS.placement()
    cur = next(r for r in rows if r["x"] == RS.CP_BASE0[0] and r["h"] == RS.CP_BASE0[2])
    half_w = VIEW_H * W / H / 2
    fig, ax = plt.subplots(figsize=(W / 200, H / 200), dpi=200)
    ax.imshow(img, extent=(CX - half_w, CX + half_w, CZ - VIEW_H / 2, CZ + VIEW_H / 2))
    g = np.ma.masked_invalid(grid)
    im = ax.pcolormesh(xs, hs, g, cmap="RdYlGn", vmin=-5, vmax=40, shading="nearest", alpha=0.6)
    ax.contour(xs, hs, np.nan_to_num(grid, nan=-10), levels=[RS.MARGIN_DEG], colors="k", linewidths=1.2)
    ax.plot(cur["x"], cur["h"], "o", ms=12, mfc="none", mec="#C2410C", mew=3)
    ax.annotate("chosen: pedestal 500 mm", (cur["x"], cur["h"]), (cur["x"] - 900, cur["h"] - 330), fontsize=12,
                color="#C2410C", weight="bold", bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.9),
                arrowprops=dict(arrowstyle="->", color="#C2410C", lw=1.8))
    for x, z, s in ((900, 1080, "buffer belt"), (-900, 1350, "workpiece carrier")):
        ax.text(x, z, s, ha="center", fontsize=11, weight="bold", color="#1c1917",
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85))
    ax.set_xlim(CX - half_w, CX + half_w)
    ax.set_ylim(0, CZ + VIEW_H / 2)
    ax.set_xlabel("x in the cell [mm]")
    ax.set_ylabel("height [mm]")
    cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01)
    cb.set_label("smallest axis margin [deg]")
    ax.set_title("Where can the robot base sit?\nColour = how comfortably the robot reaches every point of the cycle")
    fig.tight_layout()
    out = os.path.join(HERE, "figures", "placement_overlay.png")
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
