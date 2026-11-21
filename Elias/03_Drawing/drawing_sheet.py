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
sys.path.insert(0, HERE)
import gripper_params as P  # noqa: E402

W, H = 420.0, 297.0
SCALE = 0.5                                        # 1:2
LW_THICK, LW_THIN = 0.5, 0.25                      # mm, ISO 128 line widths
PT = 72 / 25.4                                     # points per mm
FONT = "DejaVu Sans"
DATE = "2026-11-21"
DRAWING_NO = "PDE-G10-E3-001"
TITLE = "Robotergreifer für Bauteil B"
SUBTITLE = "Baugruppe, elektrischer Antrieb (Zahnstange/Ritzel)"
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
    def __init__(self, title=f"{TITLE} - {SUBTITLE}", number=DRAWING_NO, size=(W, H)):
        self.title, self.number = title, number
        global W, H
        W, H = size
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

    def title_block(self, sheet, of, scale="1:2", doc_type="Baugruppenzeichnung"):
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
        f(x0, y0 + 30, "Benennung", self.title, 3.2)
        f(x0, y0 + 20, "Erstellt", AUTHORS[0])
        f(x0 + 45, y0 + 20, "Mitarbeit", AUTHORS[1])
        f(x0 + 90, y0 + 20, "Dokumentart", doc_type)
        f(x0 + 135, y0 + 20, "Gruppe / LV", "Gruppe 10, PDE MRE WS26")
        f(x0, y0 + 10, "Zeichnungsnummer", self.number)
        f(x0 + 45, y0 + 10, "Datum / Index", f"{DATE}  A")
        f(x0 + 90, y0 + 10, "Maßstab / Projektion", f"{scale}   Methode 1")
        f(x0 + 135, y0 + 10, "Blatt", f"{sheet} / {of}")
        f(x0, y0, "Allgemeintoleranzen", "ISO 2768-mK")
        f(x0 + 45, y0, "Oberflächen", "ISO 21920, Ra 3,2 wenn n. a.")
        f(x0 + 90, y0, "Werkstückkanten", "ISO 13715, -0,3 wenn n. a.")
        f(x0 + 135, y0, "Organisation", "FH Technikum Wien")
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


def _spread(want, gap, lo, hi):
    """Values sorted high to low, moved as little as possible so neighbours keep at least gap, inside [lo, hi]."""
    y = list(want)
    for i in range(1, len(y)):
        y[i] = min(y[i], y[i - 1] - gap)
    if y and y[-1] < lo:
        y[-1] = lo
        for i in range(len(y) - 2, -1, -1):
            y[i] = max(y[i], y[i + 1] + gap)
    return [min(v, hi) for v in y]


def balloons(sh, view, o, positions, box, gap=10.0):
    """Each balloon sits where the ray from the view centre through its anchor meets the rectangle around the view;
    then the balloons are pushed apart along the rectangle (keeping their order) until they keep the spacing gap.
    Nearly radial leaders in angular order do not cross."""
    an = V["views"][view]["anchors"]
    x0, y0, x1, y1 = box
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    hw, hh = x1 - x0, y1 - y0
    c = 8.0                                          # corners stay free: no balloon pair half on each side of a corner
    segs = [((x1, y0 + c), (x1, y1 - c)), ((x1 - c, y1), (x0 + c, y1)), ((x0, y1 - c), (x0, y0 + c)),
            ((x0 + c, y0), (x1 - c, y0))]            # ccw from bottom right
    lens = [hh - 2 * c, hw - 2 * c, hh - 2 * c, hw - 2 * c]
    per = sum(lens)

    def at(t):
        s_ = (t % 1.0) * per
        for ((ax_, ay), (bx, by)), L in zip(segs, lens):
            if s_ <= L:
                r = s_ / L
                return ax_ + r * (bx - ax_), ay + r * (by - ay)
            s_ -= L
        return segs[0][0]

    def t_of_ray(a):
        """Point where the ray from the centre through a meets the rectangle, as the nearest spot on the side pieces."""
        dx, dy = a[0] - cx, a[1] - cy
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            dx = 1e-3
        k = min((hw / 2) / abs(dx) if dx else 1e9, (hh / 2) / abs(dy) if dy else 1e9)
        px, py = cx + k * dx, cy + k * dy
        best, acc = None, 0.0
        for ((ax_, ay), (bx, by)), L in zip(segs, lens):
            r = max(0.0, min(1.0, ((px - ax_) * (bx - ax_) + (py - ay) * (by - ay)) / (L * L)))
            qx, qy = ax_ + r * (bx - ax_), ay + r * (by - ay)
            dist = math.hypot(px - qx, py - qy)
            if best is None or dist < best[0]:
                best = (dist, (acc + r * L) / per)
            acc += L
        return best[1]
    items = []
    for p in positions:
        cand = an.get(str(p))
        if cand:
            a = to2d(view, cand[0][1]) * SCALE + o
            items.append([t_of_ray(a), p, a])
    if not items:
        return
    items.sort()
    n, g = len(items), gap / per
    want = [it[0] for it in items]
    # open the circle at the largest gap, then the closest positions with spacing >= g keeping the order:
    # q_i = t_i - i g must not decrease -> isotonic regression (pool adjacent violators) of want_i - i g
    gaps = [((want[(i + 1) % n] - want[i]) % 1.0) for i in range(n)]
    k = (max(range(n), key=lambda i: gaps[i]) + 1) % n
    order = list(range(k, n)) + list(range(k))
    w = [want[i] for i in order]
    for i in range(1, n):
        while w[i] < w[i - 1]:
            w[i] += 1.0
    blocks = []                                      # [sum, count]
    for i, v in enumerate(w):
        blocks.append([v - i * g, 1])
        while len(blocks) > 1 and blocks[-2][0] / blocks[-2][1] > blocks[-1][0] / blocks[-1][1]:
            s2, c2 = blocks.pop()
            blocks[-1][0] += s2
            blocks[-1][1] += c2
    q = [b_[0] / b_[1] for b_ in blocks for _ in range(b_[1])]
    tt = [q[i] + i * g for i in range(n)]
    t = [0.0] * n
    for pos, i in enumerate(order):
        t[i] = tt[pos]
    spots = [at(ti) for ti in t]

    def cross(p1, p2, p3, p4):
        d = lambda a_, b_, c_: (b_[0] - a_[0]) * (c_[1] - a_[1]) - (b_[1] - a_[1]) * (c_[0] - a_[0])
        return d(p1, p2, p3) * d(p1, p2, p4) < 0 and d(p3, p4, p1) * d(p3, p4, p2) < 0
    for _ in range(6):                               # untangle: swap the balloon spots of crossing leaders
        changed = False
        for i in range(n):
            for j in range(i + 1, n):
                ai, aj = items[i][2], items[j][2]
                if cross(ai, spots[i], aj, spots[j]) and not cross(ai, spots[j], aj, spots[i]):
                    spots[i], spots[j] = spots[j], spots[i]
                    changed = True
        if not changed:
            break
    for (bx, by), (_, p, a) in zip(spots, items):
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
    boxA = (oA + SCALE * loA - [26, 24]).tolist() + (oA + SCALE * hiA + [26, 14]).tolist()
    boxT = (oT + SCALE * loT - [18, 12]).tolist() + (oT + SCALE * hiT + [18, 8]).tolist()
    sh.text(*(oA + SCALE * np.array([0.5 * (loA[0] + hiA[0]), hiA[1]]) + [0, 22]), "A-A", 5, ha="center", weight="bold")
    sh.text(*(oL + SCALE * np.array([0.5 * (loL[0] + hiL[0]), hiL[1]]) + [0, 8]), "Ansicht von links", 3.5, ha="center")
    sh.text(*(oI + SCALE * np.array([0.5 * (loI[0] + hiI[0]), hiI[1]]) + [0, 6]), "Isometrische Ansicht", 3.5, ha="center")
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
    dim_v(sh, bt[1], t[1], oA[0] + SCALE * loA[0] - 9, bt[0], t[0], f"{z_top - z_bot:.1f}".replace(".", ","))
    f_, tc = pA(xh, 76.0), pA(xh, 76.0 - 150.5)
    dim_v(sh, tc[1], f_[1], oA[0] + SCALE * hiA[0] + 9, f_[0], f_[0], "150,5 (Flansch - TCP)", outside=True)
    pl, pr = pA(-(P.PAD_X0 + P.D_GRIP), (P.PAD_Z[0] + P.PAD_Z[1]) / 2), pA(P.PAD_X0 + P.D_GRIP, (P.PAD_Z[0] + P.PAD_Z[1]) / 2)
    sh.ax.annotate("", xy=(pl[0], pl[1]), xytext=(pr[0], pr[1]), arrowprops=dict(arrowstyle="<|-|>", lw=mm(LW_THIN),
                   color="black", mutation_scale=6, shrinkA=0, shrinkB=0))
    xt = pl[0] + 0.3 * (pr[0] - pl[0])               # off the centre line, where the leader of pos. 1 runs
    sh.text(xt, pl[1] + 1.0, f"{2 * (P.PAD_X0 + P.D_GRIP):.0f}", 3.5, ha="center")
    sh.text(xt, pl[1] - 4.5, "(66 ... 86)", 2.8, ha="center")
    # balloons: everything the section shows, the rest in the top view
    posA = sorted(int(p) for p in V["views"]["A-A"]["anchors"])
    rest = sorted(int(p) for p in V["views"]["top"]["anchors"] if int(p) not in posA)
    balloons(sh, "A-A", oA, posA, boxA)
    balloons(sh, "top", oT, rest, boxT)
    # fits and notes
    x0, y0 = W - 10 - 180, 55
    notes = ["Passungen", "Lager 608-2Z: Gehäusebohrung 22 H7, Welle 8 k6 (oben Fest-, unten Loslager)",
             "Backe auf Halter: Zylinderstifte ISO 8734 4 m6 in 4 H7; Arretierbolzen GN 617-5 in 5,2",
             "Roboterflansch ISO 9409-1-40-4-M6: Zentrierbund 25 h7, Passstift 6 H7",
             "Kegelrad auf Welle: 8 H7/k6 + Passfeder DIN 6885 A 2x2x8; Ritzelnabe 8 H7",
             "Hinweise", "Dargestellt in Greifstellung (Beläge 70 mm); Hub 2 x 10 mm",
             "Masse 2,92 kg; Stückliste auf Blatt 2", "Schrauben ISO 4762 / ISO 10642, 8.8, mittelfest gesichert"]
    for i, s in enumerate(notes):
        sh.text(x0, y0 + 40 - i * 4.6, s, 3.5 if s not in ("Passungen", "Hinweise") else 3.8,
                weight="bold" if s in ("Passungen", "Hinweise") else "normal")
    sh.title_block(1, 2)
    sh.save(pdf, os.path.join(HERE, "out", "sheet1.png"))


def sheet2(pdf):
    sh = Sheet()
    rows = list(csv.reader(open(os.path.join(CAD, "out", "bom.csv"), encoding="utf-8"), delimiter=";"))
    head, body = rows[0], rows[1:]
    import de
    body = [[r[0], r[1], de.designation(r[2]), de.supplier(r[3]), de.material(r[4]), r[5]] for r in body]
    cols = [(22, "Pos.", 10), (32, "Menge", 12), (45, "Benennung", 146), (191, "Norm / Hersteller", 92), (283, "Werkstoff", 78), (361, "Masse [g]", 20)]
    top = H - 22
    sh.text(22, top + 3, "Stückliste (ISO 7573) zur Baugruppe " + DRAWING_NO, 5, weight="bold")
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
    sh.text(22, y - 6, "Eigenteile: Werkstoff und Behandlung wie angegeben; Kaufteile: Bestellnummern wie angegeben. "
            "Schaltschrankteile (Regler, Netzteil, Puffermodul) siehe gesonderte Liste, nicht Teil dieser Baugruppe.", 3.0)
    sh.title_block(2, 2, scale="-", doc_type="Stückliste")
    sh.save(pdf, os.path.join(HERE, "out", "sheet2.png"))


if __name__ == "__main__":
    with PdfPages(os.path.join(HERE, "assembly_drawing.pdf")) as pdf:
        sheet1(pdf)
        sheet2(pdf)
    print("ok")
