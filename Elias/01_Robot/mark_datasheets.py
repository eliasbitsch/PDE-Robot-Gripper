"""Mark the datasheet values the design uses: yellow highlight on the value, red frame around tables and diagrams.

python mark_datasheets.py  ->  <vendor>/marked/*_marked.pdf (vendor PDFs stay out of the public repo),
                               out/datasheets/*.png (cropped marks, local), datasheet_values.json (committed:
                               value, source, page and why we use it - feeds the iteration diagram)
"""
import json
import os
import sys

import fitz

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "02_CAD_Body"))
import gripper_params as P  # noqa: E402

CELL_V = os.path.join(ROOT, "06_Cell", "vendor")
CAD_V = os.path.join(ROOT, "02_CAD_Body", "vendor")
OUT = os.path.join(HERE, "out", "datasheets")
YELLOW, RED = (1.0, 0.9, 0.1), (0.85, 0.1, 0.1)

# (id, file, page, value, why, marks); marks: ("row", needle, k) highlights the k-th line containing needle across
# the table, ("text", needle) highlights every hit, ("box", (x0, y0, x1, y1)) frames a region in PDF points.
SHEETS = [
    ("irb_load_diagram", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 47,
     "IRB 1300-7/1.4 load diagram: 7 kg up to Z = 0.10 m, 3 kg up to Z = 0.22 m",
     "payload check E1: gripper + part B must lie inside the curve for its mass",
     [("box", (167, 118, 533, 501))]),
    ("irb_inertia", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 51,
     "Ja5 = m((Z + 0.09)^2 + L^2) + max(Jox, Joy) <= 0.5 kg m2, Ja6 = m L^2 + Joz <= 0.2 kg m2",
     "payload check E1: inertia of gripper + part about axes 5 and 6",
     [("row", "IRB 1300-7/1.4", 0), ("row", "IRB 1300-7/1.4", 1)]),
    ("irb_wrist_torque", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 53,
     "max wrist torque axis 4/5 13 N m, axis 6 6.9 N m (at 7 kg)",
     "payload check E1: static torque of the offset CoG",
     [("row", "IRB 1300-7/1.4", 0)]),
    ("irb_tcp_acc", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 54,
     "max TCP acceleration 66 m/s2 (controlled), 82 m/s2 (e-stop) at nominal load",
     "upper bound; our path needs 4.96 m/s2, the e-stop 12.1 m/s2 (axis 1 at 1.0 m), 22 m/s2 at the part with all axes braking",
     [("row", "IRB 1300-7/1.4", 0)]),
    ("irb_axis_speed", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 66,
     "axis speeds 249 / 180 / 247 / 500 / 415 / 720 deg/s (OmniCore C30)",
     "cycle time E1: MoveJ time pick -> place from the largest axis travel",
     [("row", "IRB 1300-7/1.4", 0)]),
    ("irb_stop_cat0", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 71,
     "emergency stop configured as category 0 on delivery",
     "V2 / E4: the e-stop load case is the uncontrolled category 0 stop",
     [("text", "Category 0"), ("text", "Emergency stop")]),
    ("irb_stop_data", os.path.join(CELL_V, "irb1300_product_spec.pdf"), 91,
     "category 0 stop: axis 1 55.25 deg in 0.40 s, axis 2 33.39 deg in 0.37 s, axis 3 35.66 deg in 0.27 s",
     "V2 § 2 and MuJoCo e-stop: axis decelerations -> 12.1 m/s2 at our TCP (axis 1), e_stop.py all axes",
     [("row", "55.25", 0), ("row", "33.39", 0), ("row", "35.66", 0)]),
    ("rexroth_s0", os.path.join(CAD_V, "rexroth_ksf_mini_R999001206.pdf"), 9,
     "static load safety S0 5 - 8 for normal machine design",
     "rail size 9 gave S0 2.5 - 3.7 -> changed to size 12 (5.1 - 7.8)",
     [("row", "Normale Auslegung", 0)]),
    ("rexroth_size12", os.path.join(CAD_V, "rexroth_ksf_mini_R999001206.pdf"), 19,
     "carriage SLS size 12: C0 5630 N, Mt0 33.5 N m, ML0 29.2 N m",
     "carriage check V2 § 6 / gripper_params CARRIAGE_*",
     [("row", "5630", 0)]),
    ("rexroth_rail12", os.path.join(CAD_V, "rexroth_ksf_mini_R999001206.pdf"), 21,
     "rail size 12: 12 x 7.8 mm, pitch T = 25 mm",
     "CAD rail geometry (RAIL_W, RAIL_H, RAIL_PITCH)",
     [("row", "7,8", 0)]),
    ("khk_smsg", os.path.join(CAD_V, "khk_smsg_p260.pdf"), 1,
     "SMSG1-20R/L: allowable torque 1.17 N m (bending), 0.97 N m (surface), teeth induction hardened",
     "bevel pair after V2 § 5; m 1 pair replaced by SMSG1.5-20 (4.1 / 3.47 N m) on 21 Nov, drive_margin.py",
     [("row", "0.97", 0), ("first", "Teeth induction hardened")]),
]


def _row_rect(page, needle, k):
    hits = page.search_for(needle)
    r = hits[k]
    return fitz.Rect(48, r.y0 - 1.5, page.rect.width - 48, r.y1 + 1.5)


def mark(entry):
    key, path, pno, value, why, marks = entry
    if not os.path.exists(path):
        print(f"missing {path}")
        return None
    doc = fitz.open(path)
    page = doc[pno - 1]
    rects = []
    for m in marks:
        if m[0] == "row":
            rects.append(_row_rect(page, m[1], m[2]))
            a = page.add_highlight_annot(rects[-1])
        elif m[0] in ("text", "first"):
            for r in page.search_for(m[1])[:1 if m[0] == "first" else None]:
                rects.append(r)
                a = page.add_highlight_annot(r)
        else:
            rects.append(fitz.Rect(m[1]))
            a = page.add_rect_annot(rects[-1])
            a.set_border(width=2)
            a.set_colors(stroke=RED)
            a.update()
            continue
        a.set_colors(stroke=YELLOW)
        a.update()
    clip = rects[0]
    for r in rects[1:]:
        clip |= r
    clip = (clip + (-30, -40, 30, 30)) & page.rect
    label = f"used: {why}"[:110]
    page.draw_rect(fitz.Rect(clip.x0 + 2, clip.y0 + 3, clip.x0 + 6 + 3.6 * len(label), clip.y0 + 15), color=None, fill=(1, 1, 1))
    page.insert_text((clip.x0 + 4, clip.y0 + 12), label, fontsize=7, color=RED)
    os.makedirs(OUT, exist_ok=True)
    page.get_pixmap(dpi=150, clip=clip).save(os.path.join(OUT, f"{key}.png"))
    doc.close()
    return dict(id=key, file=os.path.basename(path), page=pno, value=value, why=why)


def main():
    values = [v for v in (mark(e) for e in SHEETS) if v]
    # one marked copy per source file with all its marks
    for path in sorted({e[1] for e in SHEETS if os.path.exists(e[1])}):
        doc = fitz.open(path)
        for e in SHEETS:
            if e[1] != path:
                continue
            page = doc[e[2] - 1]
            for m in e[5]:
                if m[0] == "box":
                    a = page.add_rect_annot(fitz.Rect(m[1]))
                    a.set_border(width=2)
                    a.set_colors(stroke=RED)
                else:
                    for r in ([_row_rect(page, m[1], m[2])] if m[0] == "row" else page.search_for(m[1])[:1 if m[0] == "first" else None]):
                        a = page.add_highlight_annot(r)
                        a.set_colors(stroke=YELLOW)
                a.update()
            page.insert_text((50, 40), f"[{e[0]}] used: {e[4]}"[:120], fontsize=8, color=RED)
        out = os.path.join(os.path.dirname(path), "marked", os.path.basename(path).replace(".pdf", "_marked.pdf"))
        os.makedirs(os.path.dirname(out), exist_ok=True)
        doc.save(out, garbage=3, deflate=True)
        print(out)
    json.dump(values, open(os.path.join(HERE, "datasheet_values.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(f"{len(values)} values -> datasheet_values.json, crops in {OUT}")


if __name__ == "__main__":
    main()
