"""Cost estimate of one gripper (prototype, lot size 1): python cost_estimate.py -> out/cost_estimate.json, .csv

Bought parts: distributor prices excl. VAT found online on 26 Nov 2026 ("catalogue"), the price of a comparable
part where the exact one is not listed ("benchmark"), or our estimate where the maker gives prices only on request
("estimate"). Own parts: raw material from the CAD stock size plus machining time x hourly rate; the time model
is below and deliberately simple: setups (CAM, clamping, first part) + material removal + finishing per setup.
No VAT, no assembly labour, no surface treatment (anodising) - listed as open in the notes.
"""
import csv
import json
import os

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
BOM = {r["Pos"]: r for r in csv.DictReader(open(os.path.join(HERE, "out", "bom.csv"), encoding="utf-8"), delimiter=";")}
CAB = list(csv.DictReader(open(os.path.join(HERE, "out", "bom_cabinet.csv"), encoding="utf-8"), delimiter=";"))
BOM_MAP = json.load(open(os.path.join(HERE, "out", "bom_map.json")))

RATE = 85.0                     # EUR/h, 3-axis milling, middle of 70 - 120 EUR/h (job-shop guide values, DE 2026)
MRR = 10.0                      # cm3/min effective removal in aluminium incl. finishing passes (small parts, 3-axis)
T_SETUP = 0.5                   # h per clamping: CAM, fixture, zero point, first-part check
MATERIAL = {                    # EUR/kg cut-to-size small quantities, density g/cm3
    "EN AW-6082 T6": (9.0, 2.70), "EN AW-7075 T6": (22.0, 2.81), "16MnCr5 case hardened": (6.0, 7.85),
}
OWN = {  # pos -> (clampings, extra EUR for outside work, note)
    "1": (2, 0, "plate: contour, pockets, countersinks from both sides"),
    "2": (1, 0, "small plate"),
    "3": (3, 0, "bracket: three sides with holes"),
    "4": (4, 0, "hood: 86 % of the block is removed, four sides"),
    "5": (5, 0, "housing: bearing seats, rail face, pockets, five sides"),
    "6": (3, 0, "T-jaw from plate"),
    "7": (3, 0, "holder"),
    "8": (2, 70, "turned + teeth hobbed (outside, 40 EUR) + case hardened (outside, 30 EUR)"),
    "9": (2, 0, "turned flange, ISO 9409 bolt circle"),
    "10": (2, 0, "slide: two sides"),
}
SRC = {
    "rs_carriage": "RS Components, Rexroth R044229401 size 12 carriage, EUR 105.73 (long version assumed +20 %)",
    "rs_rail": "RS Components, Rexroth R0445 size 12 rail 200 mm, GBP 74.89 (cut to 125 mm)",
    "farnell_motor": "Farnell, Nanotec ST4118X1404-A (same frame, other winding) EUR 47.98",
    "khk": "KHK catalogue (traceparts mirror), SMSG1.5-20R CAD 82.01 = about EUR 55",
    "rack": "C45 module 1 rack 10 x 12 x 500 mm from about EUR 20 (eBay) - 30 EUR (250 mm, trade); one bar for both racks",
    "digikey_psu": "DigiKey, Mean Well SDR-120-24 EUR 48.45",
    "digikey_buffer": "DigiKey, PULS UF20.241 EUR 288.78",
    "nanotec": "Nanotec lists prices on request only (and the brake / encoder only with a motor): our estimate",
}
BOUGHT = {  # pos -> (EUR each, kind, source)
    "11": (127.0, "benchmark", SRC["rs_carriage"]), "15": (80.0, "benchmark", SRC["rs_rail"]),
    "13": (5.0, "estimate", "608-2Z brand bearing"), "14": (70.0, "estimate", SRC["nanotec"]),
    "17": (110.0, "estimate", SRC["nanotec"]), "18": (120.0, "estimate", SRC["nanotec"]),
    "23": (50.0, "benchmark", SRC["farnell_motor"]),
    "19": (17.5, "benchmark", SRC["rack"]), "20": (17.5, "benchmark", SRC["rack"]),
    "21": (75.0, "catalogue", SRC["khk"] + " + 20 EUR hub turning"),
    "22": (75.0, "catalogue", SRC["khk"] + " + 20 EUR hub turning and keyway"),
    "12": (4.0, "estimate", "Lapp SKINTOP STR-M16"), "16": (40.0, "estimate", "rubber-metal pad, vulcanised to drawing, prototype"),
    "40": (1.2, "estimate", "HELICOIL plus M4"), "41": (1.0, "estimate", "HELICOIL plus M3"),
    "42": (5.0, "estimate", "brush strip, cut to length"),
}
STD = {"ISO 10642 M3": 0.15, "ISO 10642 M4": 0.2, "ISO 4029": 0.25, "ISO 4762 M3": 0.12, "ISO 4762 M4": 0.18,
       "ISO 4762 M6": 0.3, "ISO 8734": 0.35, "DIN 653": 2.5, "DIN 6885": 0.6, "DIN 471": 0.2, "DIN 472": 0.3,
       "DIN 988": 6.0}          # EUR each, catalogue level (DIN 988: a set of shim rings)
CABINET = {  # cabinet pos -> (EUR, kind, source)
    "1": (250.0, "estimate", SRC["nanotec"]), "2": (48.45, "catalogue", SRC["digikey_psu"]),
    "3": (288.78, "catalogue", SRC["digikey_buffer"]), "4": (15.0, "estimate", "1-pole C6 breaker"),
    "5": (8.0, "estimate", "fuse terminal"), "6": (125.0, "estimate", "robot-rated hybrid cable, about 25 EUR/m x 5 m"),
    "7": (10.0, "estimate", "control cable 3 m"), "8": (1.5, "estimate", "terminal block"),
}


def main():
    rows = []
    for pos, r in BOM.items():
        qty = int(r["Qty"])
        if pos in OWN:
            clamps, extra, note = OWN[pos]
            name = next(n for n, v in BOM_MAP.items() if str(v) == pos)
            m = trimesh.load(os.path.join(HERE, "out", "stl", name + ".stl"), force="mesh")
            stock = [x + (6 if x > 15 else 3) for x in m.extents]           # mm allowance per side pair
            v_stock, v_part = stock[0] * stock[1] * stock[2] / 1000, abs(m.volume) / 1000
            eur_kg, rho = MATERIAL[r["Material"]]
            mat = v_stock * rho / 1000 * eur_kg
            hours = clamps * T_SETUP + (v_stock - v_part) / MRR / 60
            rows.append(dict(pos=pos, qty=qty, part=r["Designation"], group="own part", kind="estimate",
                             material_eur=round(mat, 2), hours=round(hours, 2), extra_eur=extra,
                             each_eur=None, source=f"stock {' x '.join(f'{x:.0f}' for x in stock)} mm, "
                                                   f"{clamps} clampings, {note}"))
        elif pos in BOUGHT:
            eur, kind, src = BOUGHT[pos]
            grp = "drive and electrics" if pos in ("14", "17", "18", "23") else "bought mechanical"
            rows.append(dict(pos=pos, qty=qty, part=r["Designation"], group=grp, kind=kind, each_eur=eur, source=src))
        else:
            key = next((k for k in STD if r["Designation"].startswith(k)), None)
            eur = STD[key] if key else 0.3
            rows.append(dict(pos=pos, qty=qty, part=r["Designation"], group="standard part", kind="catalogue",
                             each_eur=eur, source="standard part, catalogue level"))
    for r in CAB:
        eur, kind, src = CABINET[r["Pos"]]
        rows.append(dict(pos="C" + r["Pos"], qty=int(r["Qty"]), part=r["Designation"], group="cabinet", kind=kind,
                         each_eur=eur, source=src))
    json.dump(dict(rate_eur_h=RATE, mrr_cm3_min=MRR, setup_h=T_SETUP, rows=rows,
                   notes=["prices excl. VAT, lot size 1, no assembly labour",
                          "no surface treatment: anodising of the aluminium parts would add about 10 - 20 EUR per part",
                          "Nanotec parts: prices on request only - estimates; ask Nanotec for a quote of the set"]),
              open(os.path.join(HERE, "out", "cost_estimate.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    with open(os.path.join(HERE, "out", "cost_estimate.csv"), "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh, delimiter=";")
        w.writerow(["Pos", "Qty", "Part", "Group", "Kind", "Each EUR", "Material EUR", "Hours", "Extra EUR", "Total EUR", "Source"])
        tot = 0.0
        for r in rows:
            if r["group"] == "own part":
                t = r["qty"] * (r["material_eur"] + r["hours"] * RATE + r["extra_eur"])
            else:
                t = r["qty"] * r["each_eur"]
            tot += t
            w.writerow([r["pos"], r["qty"], r["part"], r["group"], r["kind"], r.get("each_eur") or "", r.get("material_eur", ""),
                        r.get("hours", ""), r.get("extra_eur", ""), f"{t:.2f}", r["source"]])
    print(f"total {tot:.0f} EUR at {RATE:.0f} EUR/h")


if __name__ == "__main__":
    main()
