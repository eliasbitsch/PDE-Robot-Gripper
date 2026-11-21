"""E3, part drawing of the gripper jaw (Viktoriia's V3), A3, 2:1, German texts.

python part_drawing.py  (after fc.sh part_export.py)  ->  part_drawing_jaw.pdf, out/sheet_jaw.png
Dimensions from Viktoriia/03_CAD_Jaws/jaws.py; geometry lines from the STEP (out/jaw_views.json).
"""
import json
import math
import os
import sys

import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Circle, PathPatch, Polygon
from matplotlib.path import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "Viktoriia", "03_CAD_Jaws"))
import drawing_sheet as DS  # noqa: E402

S = 2.0                                                       # 2:1
J = json.load(open(os.path.join(HERE, "out", "jaw_views.json")))
AXES = {"inner": (np.array([0, -1.0, 0]), np.array([0, 0, 1.0])), "A-A": (np.array([1.0, 0, 0]), np.array([0, 0, 1.0])),
        "outer": (np.array([0, 1.0, 0]), np.array([0, 0, 1.0]))}
CENTER = {"inner": (95, 178), "A-A": (178, 178), "outer": (262, 178)}
X_IN, X_OUT, YH, Z_BOT, Z_TOP = 38.0, 46.0, 12.5, -98.0, -28.5
ZM = 0.5 * (Z_BOT + Z_TOP)
POCKET = (3.0, 23.0, -95.0, -60.0)                            # depth, width, z0, z1
SCREW_Z, HOLE, CB = (-67.5, -87.5), 3.4, (6.5, 1.0)
DOWEL = (4.0, 6.5, 5.5, -32.0)                                # d, depth, y, z
PLUNGER = (5.2, 6.0, -39.0)


def fix(view):
    M = np.array(J["views"][view]["M"])
    pts = np.array([[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100], [50, 70, 30]], float)
    uv = np.c_[pts, np.ones(len(pts))] @ M
    h, v = AXES[view]
    return np.linalg.lstsq(np.c_[uv, np.ones(len(uv))], np.c_[pts @ h, pts @ v], rcond=None)[0]


def P(view, x, y, z):
    """Model point -> sheet mm."""
    h, v = AXES[view]
    p = np.array([x, y, z], float)
    c = np.array([0.5 * (X_IN + X_OUT), 0.0, ZM])
    return np.array(CENTER[view]) + S * np.array([(p - c) @ h, (p - c) @ v])


def lines(sh, view):
    X = fix(view)
    c = np.array([0.5 * (X_IN + X_OUT), 0.0, ZM])
    h, v = AXES[view]
    off = np.array(CENTER[view]) - S * np.array([c @ h, c @ v])
    keys = (("visible", DS.LW_THICK, None),) if view == "A-A" else (("visible", DS.LW_THICK, None), ("hidden", DS.LW_THIN, (4, 2)))
    for key, lw, dash in keys:                     # no hidden edges in the section
        for pl in J["views"][view][key]:
            a = (np.c_[np.array(pl), np.ones(len(pl))] @ X) * S + off
            kw = dict(dashes=dash) if dash else {}
            sh.ax.plot(a[:, 0], a[:, 1], color="black", lw=DS.mm(lw), solid_capstyle="round", **kw)


def centre_line(sh, a, b):
    sh.ax.plot([a[0], b[0]], [a[1], b[1]], color="black", lw=DS.mm(DS.LW_THIN), dashes=(10, 2, 1.5, 2))


def leader(sh, a, b, text, ha="left"):
    sh.line([a[0], b[0]], [a[1], b[1]], DS.LW_THIN)
    sh.ax.add_patch(Circle(a, 0.5, color="black"))
    w = 1.9 * len(text)
    xe = b[0] + (w if ha == "left" else -w)
    sh.line([b[0], xe], [b[1], b[1]], DS.LW_THIN)
    sh.text(b[0] + (0.5 if ha == "left" else -0.5), b[1] + 1.0, text, 3.5, ha=ha)


def surface(sh, x, y, text, rot=0):
    """ISO 21920 surface symbol (material removal required), tip at (x, y)."""
    t = math.radians(rot)
    R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    pts = np.array([[-2.5, 4.3], [0, 0], [5.0, 8.6], [14, 8.6]]) @ R.T + [x, y]
    sh.line(pts[:, 0], pts[:, 1], DS.LW_THIN)
    tri = np.array([[-2.5, 4.3], [2.5, 4.3]]) @ R.T + [x, y]
    sh.line(tri[:, 0], tri[:, 1], DS.LW_THIN)
    tp = np.array([6.0, 9.4]) @ R.T + [x, y]
    sh.text(tp[0], tp[1], text, 3.0, rotation=rot)


def datum(sh, x, y, letter, dx=0, dy=-8):
    sh.ax.add_patch(Polygon([[x - 2, y], [x + 2, y], [x, y - 3]] if dy < 0 else [[x - 2, y], [x + 2, y], [x, y + 3]],
                            closed=True, color="black"))
    bx, by = x + dx, y + dy
    sh.line([x, bx], [y - (3 if dy < 0 else -3), by + (3.5 if dy < 0 else -3.5)], DS.LW_THIN)
    sh.ax.add_patch(DS.Rectangle((bx - 3.5, by - 3.5), 7, 7, fill=False, lw=DS.mm(DS.LW_THIN)))
    sh.text(bx, by - 1.3, letter, 3.5, ha="center")


def fcf(sh, x, y, sym, tol, ref=None):
    """Feature control frame: symbol | tolerance | datum."""
    cells = [7, 14] + ([7] if ref else [])
    xx = x
    for i, w in enumerate(cells):
        sh.ax.add_patch(DS.Rectangle((xx, y), w, 7, fill=False, lw=DS.mm(DS.LW_THIN)))
        cx = xx + w / 2
        if i == 0:
            if sym == "flat":
                sh.line([cx - 2.4, cx + 1.2, cx + 2.4, cx - 1.2, cx - 2.4], [y + 2.2, y + 2.2, y + 4.8, y + 4.8, y + 2.2], DS.LW_THIN)
            else:
                sh.line([cx, cx], [y + 1.8, y + 5.2], DS.LW_THIN)
                sh.line([cx - 2, cx + 2], [y + 1.8, y + 1.8], DS.LW_THIN)
        elif i == 1:
            sh.text(cx, y + 2.0, tol, 3.5, ha="center")
        else:
            sh.text(cx, y + 2.0, ref, 3.5, ha="center")
        xx += w
    return x, y


def main():
    sh = DS.Sheet(title="Greiferbacke rechts (links spiegelbildlich), Konstruktion V3", number="PDE-G10-E3-002")
    for v in ("inner", "A-A", "outer"):
        lines(sh, v)
    # hatching of the section
    verts, codes = [], []
    for loops in J["faces_AA"]:
        for loop in loops:
            pts = [P("A-A", x, 0, z) for x, z in loop]
            verts += pts + [pts[0]]
            codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
    sh.ax.add_patch(PathPatch(Path(verts, codes), facecolor="none", edgecolor="black", lw=0, hatch="////"))
    # centre lines
    centre_line(sh, P("inner", 0, 0, Z_TOP + 3), P("inner", 0, 0, Z_BOT - 3))
    centre_line(sh, P("outer", 0, 0, Z_TOP + 3), P("outer", 0, 0, Z_BOT - 3))
    for z in SCREW_Z:
        centre_line(sh, P("A-A", X_IN - 2, 0, z), P("A-A", X_OUT + 2, 0, z))
        centre_line(sh, P("inner", 0, -3, z), P("inner", 0, 3, z))
        centre_line(sh, P("outer", 0, -5, z), P("outer", 0, 5, z))
    centre_line(sh, P("A-A", X_OUT - PLUNGER[1] - 1.5, 0, PLUNGER[2]), P("A-A", X_OUT + 2, 0, PLUNGER[2]))
    for y in (-DOWEL[2], DOWEL[2]):
        centre_line(sh, P("outer", 0, y, DOWEL[3] + 4), P("outer", 0, y, DOWEL[3] - 4))
    centre_line(sh, P("outer", 0, -DOWEL[2] - 4, DOWEL[3]), P("outer", 0, DOWEL[2] + 4, DOWEL[3]))
    # section line A-A in the inner view (plane y = 0, seen from -y: arrows towards +y = left on the sheet)
    for z, sg in ((Z_TOP + 12, 1), (Z_BOT - 13, -1)):
        a = P("inner", 0, 0, z)
        sh.line([a[0], a[0]], [a[1], a[1] - sg * 5], DS.LW_THICK)
        sh.ax.annotate("", xy=(a[0] - 7, a[1]), xytext=(a[0], a[1]),
                       arrowprops=dict(arrowstyle="-|>", lw=DS.mm(DS.LW_THICK), color="black", mutation_scale=7))
        sh.text(a[0] - 9, a[1] - 1.5, "A", 5, ha="right", weight="bold")
    # titles
    for v, t in (("inner", "Ansicht Innenseite (Belagtasche)"), ("A-A", "A-A"), ("outer", "Ansicht Außenseite (Anlage am Halter)")):
        p = P(v, 0.5 * (X_IN + X_OUT), 0, Z_TOP)
        sh.text(CENTER[v][0], p[1] + 16, t, 4.2 if v == "A-A" else 3.5, ha="center", weight="bold" if v == "A-A" else "normal")
    # --- dimensions, inner view
    a, b = P("inner", 0, YH, Z_BOT), P("inner", 0, -YH, Z_BOT)
    DS.dim_h(sh, a[0], b[0], a[1] - 10, a[1], b[1], "25", above=False)
    a, b = P("inner", 0, YH, Z_BOT), P("inner", 0, YH, Z_TOP)
    DS.dim_v(sh, a[1], b[1], a[0] - 10, a[0], b[0], "69,5")
    a, b = P("inner", 0, POCKET[1] / 2, POCKET[3]), P("inner", 0, -POCKET[1] / 2, POCKET[3])
    DS.dim_h(sh, a[0], b[0], P("inner", 0, 0, Z_TOP)[1] + 6, a[1], b[1], "23")
    t, p1, p0 = P("inner", 0, -YH, Z_TOP), P("inner", 0, -YH, POCKET[3]), P("inner", 0, -YH, POCKET[2])
    DS.dim_v(sh, p1[1], t[1], t[0] + 8, p1[0], t[0], "31,5", outside=True)
    DS.dim_v(sh, p0[1], p1[1], t[0] + 16, p0[0], p1[0], "35", outside=True)
    s1, s2 = P("inner", 0, -YH, SCREW_Z[0]), P("inner", 0, -YH, SCREW_Z[1])
    DS.dim_v(sh, s1[1], t[1], t[0] + 24, s1[0], t[0], "39", outside=True)
    DS.dim_v(sh, s2[1], t[1], t[0] + 32, s2[0], t[0], "59", outside=True)
    leader(sh, P("inner", 0, POCKET[1] / 2 - 0.3, POCKET[3] - 0.3), P("inner", 0, YH + 9, Z_TOP + 3) + [0, 0], "4x R1", ha="right")
    # --- dimensions, section A-A
    a, b = P("A-A", X_IN, 0, Z_BOT), P("A-A", X_OUT, 0, Z_BOT)
    DS.dim_h(sh, a[0], b[0], a[1] - 10, a[1], b[1], "8", above=False)
    a, b = P("A-A", X_IN, 0, POCKET[2] - 0.0), P("A-A", X_IN + POCKET[0], 0, POCKET[2])
    DS.dim_h(sh, a[0], b[0], a[1] - 4, a[1], b[1], "3", above=False)
    leader(sh, P("A-A", X_OUT - 0.5, 0, SCREW_Z[0] + CB[0] / 2), P("A-A", X_OUT + 10, 0, SCREW_Z[0] + 10), "2x Ø3,4 durch, Senkung Ø6,5 x 1")
    leader(sh, P("A-A", X_OUT - 3, 0, PLUNGER[2] + PLUNGER[0] / 2), P("A-A", X_OUT + 10, 0, PLUNGER[2] + 9), "Ø5,2 x 6 tief")
    leader(sh, P("A-A", X_IN + 0.3, 0, Z_BOT + 0.3), P("A-A", X_IN - 12, 0, Z_BOT - 8), "0,5 x 45°", ha="right")
    datum(sh, *P("A-A", X_OUT, 0, ZM - 18), "A", dx=8, dy=0.01)
    fx, fy = P("A-A", X_OUT, 0, ZM - 30)[0] + 6, P("A-A", X_OUT, 0, ZM - 30)[1]
    fcf(sh, fx, fy, "flat", "0,02")
    tip = P("A-A", X_OUT, 0, ZM - 26)
    sh.line([fx, tip[0] + 3, tip[0] + 3], [fy + 3.5, fy + 3.5, tip[1]], DS.LW_THIN)
    sh.ax.annotate("", xy=(tip[0], tip[1]), xytext=(tip[0] + 3, tip[1]),
                   arrowprops=dict(arrowstyle="-|>", lw=DS.mm(DS.LW_THIN), color="black", mutation_scale=6))
    surface(sh, *P("A-A", X_OUT, 0, ZM + 8), "Ra 1,6", rot=-90)
    surface(sh, *P("A-A", X_IN + POCKET[0], 0, -70.0), "Ra 1,6", rot=90)
    # --- dimensions, outer view
    a, b = P("outer", 0, -DOWEL[2], DOWEL[3]), P("outer", 0, DOWEL[2], DOWEL[3])
    DS.dim_h(sh, a[0], b[0], P("outer", 0, 0, Z_TOP)[1] + 6, a[1], b[1], "11 ±0,02")
    t = P("outer", 0, YH, Z_TOP)
    d1, pl = P("outer", 0, YH, DOWEL[3]), P("outer", 0, YH, PLUNGER[2])
    DS.dim_v(sh, d1[1], t[1], t[0] + 8, d1[0], t[0], "3,5", outside=True)
    DS.dim_v(sh, pl[1], t[1], t[0] + 16, pl[0], t[0], "10,5", outside=True)
    shelf = P("outer", 0, YH + 26, DOWEL[3] - 18)
    leader(sh, P("outer", 0, DOWEL[2] + 1.4, DOWEL[3] - 1.4), shelf, "2x Ø4 H7 x 6,5 tief")
    fcf(sh, shelf[0], shelf[1] - 9, "perp", "0,02", "A")                     # directly under the hole callout
    surface(sh, shelf[0] + 32, shelf[1] - 9, "Ra 0,8")
    # notes
    x0, y0 = DS.W - 10 - 180, 55
    notes = ["Werkstoff: EN AW-7075 T6 (AlZn5,5MgCu), Plattenmaterial 10 mm",
             "Rohmasse ca. 45 g, Fertigmasse 31 g",
             "Taschenecken R1 (Fräserradius), Taschenboden eben",
             "Außenkanten 0,5 x 45°, Oberkante nur entgratet (Restwand über Ø4 >= 1,5)",
             "Bezug A = Anlagefläche am Backenhalter",
             "Oberflächen: allgemein Ra 3,2; Anlagefläche und Taschenboden Ra 1,6",
             "Rechte Backe dargestellt, linke spiegelbildlich (PDE-G10-E3-002-L)",
             "Belagplatte (Pos. 17) mit 2x ISO 4762 M3x6 von außen"]
    sh.text(x0, y0 + 46, "Hinweise", 3.8, weight="bold")
    for i, s in enumerate(notes):
        sh.text(x0, y0 + 41 - i * 4.6, s, 3.3)
    DS.AUTHORS = ("V. Ovdiienko", "E. Bitsch")
    sh.title_block(1, 1, scale="2:1", doc_type="Einzelteilzeichnung")
    with PdfPages(os.path.join(HERE, "part_drawing_jaw.pdf")) as pdf:
        sh.save(pdf, os.path.join(HERE, "out", "sheet_jaw.png"))
    print("ok")


if __name__ == "__main__":
    main()
