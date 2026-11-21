"""E3, step 2: assembly drawing (A3, 1:2, first-angle projection) and parts list as one PDF.

python drawing_sheet.py  ->  assembly_drawing.pdf (+ PNG previews in out/)

Line data from drawing_export.py (out/views.json), parts list from 02_CAD_Body/out/bom.csv: the drawing cannot drift
from the model. Hidden edges are left out (assembly drawing practice), cut faces are hatched per part.
"""
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Circle, PathPatch, Polygon, Rectangle
from matplotlib.path import Path
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "02_CAD_Body")
sys.path.insert(0, CAD)
import gripper_params as P  # noqa: E402

W, H = 420.0, 297.0
SCALE = 0.5                                        # 1:2
LW_THICK, LW_THIN = 0.5, 0.25                      # mm, ISO 128 line widths
PT = 72 / 25.4                                     # points per mm
FONT = "DejaVu Sans"
DATE = "2026-11-21"
DRAWING_NO = "PDE-G10-E3-001"
TITLE = "Robot gripper for part B"
SUBTITLE = "Assembly, electric drive (rack and pinion)"
AUTHORS = ("E. Bitsch", "V. Ovdiienko")

V = json.load(open(os.path.join(HERE, "out", "views.json")))


def _iso_axes():
    d = np.array([1.0, -1.0, 1.0]) / math.sqrt(3)                 # towards the viewer
    v = np.array([0, 0, 1.0]) - d[2] * d
    v /= np.linalg.norm(v)
    h = np.cross(-d, v)
    return h, v


# sheet axes (right, up) of every view in model coordinates, first-angle arrangement
AXES = {"A-A": (np.array([1.0, 0, 0]), np.array([0, 0, 1.0])),
        "top": (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])),
        "left": (np.array([0, -1.0, 0]), np.array([0, 0, 1.0])),
        "iso": _iso_axes()}


def _uv_fix(view):
    """Affine map from TechDraw's own 2D frame to the sheet axes, fitted on marker points."""
    M = np.array(V["views"][view]["M"])
    pts = np.array([[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100], [50, 70, 30]], float)
    uv = np.c_[pts, np.ones(len(pts))] @ M
    h, v = AXES[view]
    tgt = np.c_[pts @ h, pts @ v]
    X = np.linalg.lstsq(np.c_[uv, np.ones(len(uv))], tgt, rcond=None)[0]
    return X


FIX = {k: _uv_fix(k) for k in V["views"]}
for _k, _vw in V["views"].items():
    for _key in ("visible", "hidden"):
        _vw[_key] = [(np.c_[np.array(pl), np.ones(len(pl))] @ FIX[_k]).tolist() for pl in _vw[_key]]


def mm(lw):
    return lw * PT


def to2d(view, p):
    h, v = AXES[view]
    p = np.asarray(p, float)
    return np.array([p @ h, p @ v])


class Sheet:
    def __init__(self, title_suffix=""):
        self.fig = plt.figure(figsize=(W / 25.4, H / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, W)
        self.ax.set_ylim(0, H)
        self.ax.set_aspect("equal")
        self.ax.axis("off")
        self.frame()

    def line(self, xs, ys, lw=LW_THIN, **kw):
        self.ax.plot(xs, ys, color="black", lw=mm(lw), solid_capstyle="round", **kw)

    def text(self, x, y, s, size=3.5, **kw):
        self.ax.text(x, y, s, fontsize=size * PT * 0.72, family=FONT, **kw)

    def frame(self):
        self.ax.add_patch(Rectangle((20, 10), W - 30, H - 20, fill=False, lw=mm(0.7)))
        for x in np.linspace(20, W - 10, 9)[1:-1]:                 # centring marks / grid ticks
            if x < W - 10 - 180:
                self.line([x, x], [10, 15], 0.35)
            self.line([x, x], [H - 15, H - 10], 0.35)

    def title_block(self, sheet, of, scale="1:2", doc_type="Assembly drawing"):
        x0, y0, w, h = W - 10 - 180, 10, 180, 40
        ax = self.ax
        ax.add_patch(Rectangle((x0, y0), w, h, fill=False, lw=mm(0.7)))
        rows = [y0 + 10, y0 + 20, y0 + 30]
        for y in rows:
            self.line([x0, x0 + w], [y, y], 0.35)
        for x in (x0 + 45, x0 + 90, x0 + 135):
            self.line([x, x], [y0, y0 + 30], 0.35)
        f = lambda x, y, lab, val, sv=3.5: (self.text(x + 1.5, y + 6.6, lab, 1.8, color="0.3"),
                                            self.text(x + 1.5, y + 1.8, val, sv))
        f(x0, y0 + 30, "Title", f"{TITLE} - {SUBTITLE}", 3.2)
        f(x0, y0 + 20, "Created by", AUTHORS[0])
        f(x0 + 45, y0 + 20, "Co-author", AUTHORS[1])
        f(x0 + 90, y0 + 20, "Document type", doc_type)
        f(x0 + 135, y0 + 20, "Group / course", "Group 10, PDE MRE WS26")
        f(x0, y0 + 10, "Drawing number", DRAWING_NO)
        f(x0 + 45, y0 + 10, "Date / rev.", f"{DATE}  A")
        f(x0 + 90, y0 + 10, "Scale / projection", f"{scale}   first angle")
        f(x0 + 135, y0 + 10, "Sheet", f"{sheet} / {of}")
        f(x0, y0, "General tolerances", "ISO 2768-mK")
        f(x0 + 45, y0, "Surfaces", "ISO 21920, Ra 3.2 unless noted")
        f(x0 + 90, y0, "Edges", "ISO 13715, -0.3 unless noted")
        f(x0 + 135, y0, "Org.", "FH Technikum Wien")
        self.projection_symbol(x0 + 170, y0 + 36.5)

    def projection_symbol(self, x, y):
        """ISO 5456-2 first angle: truncated cone, circles on the right."""
        s = 0.9
        self.line([x - 12 * s, x - 4 * s, x - 4 * s, x - 12 * s, x - 12 * s], [y - 1.5 * s, y - 3 * s, y + 3 * s, y + 1.5 * s, y - 1.5 * s], 0.35)
        self.ax.add_patch(Circle((x + 3 * s, y), 3 * s, fill=False, lw=mm(0.35)))
        self.ax.add_patch(Circle((x + 3 * s, y), 1.5 * s, fill=False, lw=mm(0.35)))

    def save(self, pdf, png):
        pdf.savefig(self.fig)
        self.fig.savefig(png, dpi=110)
        plt.close(self.fig)


def place(view, center, align=None):
    """Paper transform for a view: paper = o + SCALE * uv; centred on center, optionally sharing an axis offset."""
    pts = np.array([p for pl in V["views"][view]["visible"] for p in pl])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    o = np.array(center) - SCALE * 0.5 * (lo + hi)
    if align:
        for ax, val in align.items():
            o[ax] = val
    return o, lo, hi


def draw_view(sh, view, o, lw=LW_THICK):
    for pl in V["views"][view]["visible"]:
        a = np.array(pl) * SCALE + o
        sh.ax.plot(a[:, 0], a[:, 1], color="black", lw=mm(lw), solid_capstyle="round", solid_joinstyle="round")


def hatch_section(sh, o):
    angles = {}
    k = 0
    for f in V["faces_AA"]:
        name = f["part"]
        if name not in angles:
            angles[name] = ("////", "\\\\\\\\", "///", "\\\\\\")[k % 4]
            k += 1
        verts, codes = [], []
        for loop in f["loops"]:
            pts = [to2d("A-A", (x, 0.0, z)) * SCALE + o for x, z in loop]
            verts += pts + [pts[0]]
            codes += [Path.MOVETO] + [Path.LINETO] * (len(pts) - 1) + [Path.CLOSEPOLY]
        sh.ax.add_patch(PathPatch(Path(verts, codes), facecolor="none", edgecolor="black", lw=0, hatch=angles[name]))


def dim_h(sh, x0, x1, y, y_ref0, y_ref1, txt, above=True):
    """Horizontal dimension at height y with extension lines from y_ref."""
    for x, yr in ((x0, y_ref0), (x1, y_ref1)):
        sh.line([x, x], [yr + (1 if y > yr else -1), y + (2 if y > yr else -2)], LW_THIN)
    sh.ax.annotate("", xy=(x0, y), xytext=(x1, y), arrowprops=dict(arrowstyle="<|-|>", lw=mm(LW_THIN), color="black",
                                                                   mutation_scale=6, shrinkA=0, shrinkB=0))
    sh.text(0.5 * (x0 + x1), y + (1.0 if above else -4.2), txt, 3.5, ha="center")


def dim_v(sh, y0, y1, x, x_ref0, x_ref1, txt, outside=False):
    for y, xr in ((y0, x_ref0), (y1, x_ref1)):
        sh.line([xr + (1 if x > xr else -1), x + (2 if x > xr else -2)], [y, y], LW_THIN)
    sh.ax.annotate("", xy=(x, y0), xytext=(x, y1), arrowprops=dict(arrowstyle="<|-|>", lw=mm(LW_THIN), color="black",
                                                                   mutation_scale=6, shrinkA=0, shrinkB=0))
    if outside:
        sh.text(x + 1.2, 0.5 * (y0 + y1), txt, 3.5, ha="left", va="center", rotation=90)
    else:
        sh.text(x - 1.2, 0.5 * (y0 + y1), txt, 3.5, ha="right", va="center", rotation=90)


def balloons(sh, view, o, positions, box):
    """Balloons evenly spaced on the rectangle around the view, in the angular order of their anchors; the ring is
    turned so that the leaders are shortest (no overlapping balloons, few crossings)."""
    an = V["views"][view]["anchors"]
    x0, y0, x1, y1 = box
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    items = []
    for p in positions:
        cand = an.get(str(p))
        if cand:
            a = to2d(view, cand[0][1]) * SCALE + o
            items.append((math.atan2(a[1] - cy, a[0] - cx), p, a))
    if not items:
        return
    items.sort()
    n = len(items)
    hw, hh = x1 - x0, y1 - y0
    per = 2 * (hw + hh)

    def on_rect(t):                                   # t in [0, 1): from the right middle, counter-clockwise
        s = (t % 1.0) * per
        for (ax_, ay), (bx, by), L in (((x1, cy), (x1, y1), hh / 2), ((x1, y1), (x0, y1), hw), ((x0, y1), (x0, y0), hh),
                                       ((x0, y0), (x1, y0), hw), ((x1, y0), (x1, cy), hh / 2)):
            if s <= L:
                r = s / L
                return ax_ + r * (bx - ax_), ay + r * (by - ay)
            s -= L
        return x1, cy
    best = None
    for k in range(120):
        off = k / 120
        pts = [on_rect(off + i / n) for i in range(n)]
        L = sum(math.hypot(px - a[0], py - a[1]) for (px, py), (_, _, a) in zip(pts, items))
        if best is None or L < best[0]:
            best = (L, pts)
    for (bx, by), (_, p, a) in zip(best[1], items):
        sh.line([a[0], bx], [a[1], by], LW_THIN)
        sh.ax.add_patch(Circle((a[0], a[1]), 0.5, color="black"))
        sh.ax.add_patch(Circle((bx, by), 3.6, facecolor="white", edgecolor="black", lw=mm(LW_THIN), zorder=5))
        sh.text(bx, by - 1.3, str(p), 3.5, ha="center", zorder=6)


def sheet1(pdf):
    sh = Sheet()
    # A-A (front, seen from -y): centre upper left; left view to the right on the same height; top view below
    oA, loA, hiA = place("A-A", (120, 200))
    oL, loL, hiL = place("left", (265, 192), align={1: oA[1]})
    oT, loT, hiT = place("top", (120, 68), align={0: oA[0]})
    oI, loI, hiI = place("iso", (350, 205))
    for v, o in (("A-A", oA), ("left", oL), ("top", oT), ("iso", oI)):
        draw_view(sh, v, o)
    hatch_section(sh, oA)
    boxA = (oA + SCALE * loA - [30, 28]).tolist() + (oA + SCALE * hiA + [30, 16]).tolist()
    boxT = (oT + SCALE * loT - [16, 14]).tolist() + (oT + SCALE * hiT + [16, 12]).tolist()
    sh.text(*(oA + SCALE * np.array([0.5 * (loA[0] + hiA[0]), hiA[1]]) + [0, 22]), "A-A", 5, ha="center", weight="bold")
    sh.text(*(oL + SCALE * np.array([0.5 * (loL[0] + hiL[0]), hiL[1]]) + [0, 8]), "view from the left", 3.5, ha="center")
    sh.text(*(oI + SCALE * np.array([0.5 * (loI[0] + hiI[0]), hiI[1]]) + [0, 6]), "isometric view", 3.5, ha="center")
    # section line A-A in the top view (plane y = 0, seen from -y: arrows point to +y)
    ya = (to2d("top", (0, 0, 0)) * SCALE + oT)[1]
    xl, xr = oT[0] + SCALE * loT[0] - 6, oT[0] + SCALE * hiT[0] + 6
    sh.ax.plot([xl, xr], [ya, ya], color="black", lw=mm(LW_THIN), dashes=(8, 2, 1, 2))
    for x in (xl, xr):
        sh.line([x, x + (3 if x == xl else -3)], [ya, ya], LW_THICK)
        sh.ax.annotate("", xy=(x, ya + 6), xytext=(x, ya), arrowprops=dict(arrowstyle="-|>", lw=mm(LW_THICK), color="black", mutation_scale=7))
        sh.text(x, ya + 7, "A", 5, ha="center", weight="bold")
    # main dimensions in A-A
    def pA(x, z):
        return to2d("A-A", (x, 0, z)) * SCALE + oA
    xh = P.HOUSING_X / 2
    z_top, z_bot = 76.0 + P.ISO_SPIGOT_H, P.JAW_Z_BOTTOM
    a, b = pA(-xh, -26), pA(xh, -26)
    dim_h(sh, a[0], b[0], SCALE * loA[1] + oA[1] - 9, a[1], b[1], f"{P.HOUSING_X:.0f}", above=False)
    t, bt = pA(-xh - 2, z_top), pA(-xh - 2, z_bot)
    dim_v(sh, bt[1], t[1], oA[0] + SCALE * loA[0] - 9, bt[0], t[0], f"{z_top - z_bot:.1f}")
    f_, tc = pA(xh, 76.0), pA(xh, 76.0 - 150.5)
    dim_v(sh, tc[1], f_[1], oA[0] + SCALE * hiA[0] + 9, f_[0], f_[0], "150.5 (flange - TCP)", outside=True)
    pl, pr = pA(-(P.PAD_X0 + P.D_GRIP), (P.PAD_Z[0] + P.PAD_Z[1]) / 2), pA(P.PAD_X0 + P.D_GRIP, (P.PAD_Z[0] + P.PAD_Z[1]) / 2)
    sh.ax.annotate("", xy=(pl[0], pl[1]), xytext=(pr[0], pr[1]), arrowprops=dict(arrowstyle="<|-|>", lw=mm(LW_THIN),
                   color="black", mutation_scale=6, shrinkA=0, shrinkB=0))
    sh.text(0.5 * (pl[0] + pr[0]), pl[1] + 1.0, f"{2 * (P.PAD_X0 + P.D_GRIP):.0f}", 3.5, ha="center")
    sh.text(0.5 * (pl[0] + pr[0]), pl[1] - 4.5, "(66 ... 86)", 2.8, ha="center")
    # balloons: everything the section shows, the rest in the top view
    posA = sorted(int(p) for p in V["views"]["A-A"]["anchors"])
    rest = sorted(int(p) for p in V["views"]["top"]["anchors"] if int(p) not in posA)
    balloons(sh, "A-A", oA, posA, boxA)
    balloons(sh, "top", oT, rest, boxT)
    # fits and notes
    x0, y0 = W - 10 - 180, 55
    notes = ["Fits", "608-2Z bearings: housing bore 22 H7, shaft 8 k6 (upper fixed, lower floating)",
             "Jaw on holder: dowels ISO 8734 4 m6, holes 4 H7; plunger GN 617-5 into 5.2",
             "Robot flange ISO 9409-1-40-4-M6: spigot 25 h7, locating pin 6 H7",
             "Bevel wheel on shaft: 8 H7/k6 + key DIN 6885 A 2x2x8; pinion hub 8 H7",
             "Notes", "Shown at the grip position (pads 70 mm apart); stroke 2 x 10 mm",
             "Mass 2.92 kg; parts list on sheet 2", "Screws: ISO 4762 / ISO 10642, 8.8, medium-strength threadlocker"]
    for i, s in enumerate(notes):
        sh.text(x0, y0 + 40 - i * 4.6, s, 3.5 if s not in ("Fits", "Notes") else 3.8,
                weight="bold" if s in ("Fits", "Notes") else "normal")
    sh.title_block(1, 2)
    sh.save(pdf, os.path.join(HERE, "out", "sheet1.png"))


def sheet2(pdf):
    sh = Sheet()
    rows = list(csv.reader(open(os.path.join(CAD, "out", "bom.csv"), encoding="utf-8"), delimiter=";"))
    head, body = rows[0], rows[1:]
    cols = [(22, "Pos", 10), (32, "Qty", 9), (41, "Designation", 150), (191, "Standard / supplier", 92), (283, "Material", 78), (361, "Mass [g]", 20)]
    top = H - 22
    sh.text(22, top + 3, "Parts list (ISO 7573) - assembly " + DRAWING_NO, 5, weight="bold")
    rh = min(5.0, (top - 60) / (len(body) + 1))
    y = top - rh
    for x, lab, _ in cols:
        sh.text(x + 1, y + 1.2, lab, 3.2, weight="bold")
    sh.line([20, W - 10], [y, y], 0.5)
    for r in body:
        y -= rh
        for (x, _, wdt), val in zip(cols, r):
            s = val if len(val) * 1.75 < wdt else val[: int(wdt / 1.75) - 1] + "."
            sh.text(x + 1, y + 1.2, s, 2.8)
        sh.line([20, W - 10], [y, y], 0.18)
    for x, _, _ in cols[1:]:
        sh.line([x, x], [y, top], 0.18)
    sh.text(22, y - 6, "Own parts: material and treatment as listed; bought parts: catalogue numbers as listed. "
            "Cabinet parts (controller, power supply, buffer module) see out/bom_cabinet.csv, not part of this assembly.", 3.0)
    sh.title_block(2, 2, scale="-", doc_type="Parts list")
    sh.save(pdf, os.path.join(HERE, "out", "sheet2.png"))


if __name__ == "__main__":
    with PdfPages(os.path.join(HERE, "assembly_drawing.pdf")) as pdf:
        sheet1(pdf)
        sheet2(pdf)
    print("ok")
