"""German texts for the submitted drawings (the model and its BOM keep English designations)."""
import re

DESIGNATION = {
    "Ball carriage SLS size 12, long, preload": "Kugelführungswagen SLS Gr. 12, lang, vorgespannt",
    "Bottom cover": "Bodendeckel",
    "Brush seal strip along the two holder slots in the cover (oily parts)": "Bürstendichtleiste an den Halterschlitzen im Deckel",
    "Cable gland SKINTOP STR-M 16x1.5": "Kabelverschraubung SKINTOP STR-M 16x1,5",
    "Cable window plate": "Kabelfensterplatte",
    "Deep groove ball bearing 608-2Z": "Rillenkugellager 608-2Z",
    "Drive bracket": "Antriebswinkel",
    "Drive hood": "Antriebshaube",
    "Gear housing": "Getriebegehäuse",
    "Indexing plunger GN 617-5-A": "Arretierbolzen GN 617-5-A",
    "Jaw holder": "Backenhalter",
    "Jaw": "Greiferbacke",
    "Magnetic encoder NME2 (on the brake), for the external closed-loop controller": "Magnetischer Drehgeber NME2 (an der Bremse)",
    "Miniature ball rail size 12, L = 125, T1 = 12.5": "Miniatur-Kugelschiene Gr. 12, L = 125, T1 = 12,5",
    "Pad plate, rubber-metal (steel 3 + NBR 70 Sh A 3)": "Belagplatte Gummi-Metall (Stahl 3 + NBR 70 Shore A 3)",
    "Parallel key DIN 6885 A 2 x 2 x 8 (bevel wheel)": "Passfeder DIN 6885 A 2 x 2 x 8 (Kegelrad)",
    "Pinion shaft m = 1, z = 20": "Ritzelwelle m = 1, z = 20",
    "Planetary gearbox GP42-S1-7-SR": "Planetengetriebe GP42-S1-7-SR",
    "Rack m = 1, 10 x 12, cut to 56.5": "Zahnstange m = 1, 10 x 12, abgelängt",
    "Retaining ring DIN 471 8 x 0.8 (fixed bearing, shaft)": "Sicherungsring DIN 471 8 x 0,8 (Festlager, Welle)",
    "Retaining ring DIN 472 22 x 1 (fixed bearing, bore)": "Sicherungsring DIN 472 22 x 1 (Festlager, Bohrung)",
    "Robot adapter ISO 9409-1-40-4-M6": "Roboteradapter ISO 9409-1-40-4-M6",
    "Shim ring DIN 988 8 x 14 x 0.1 / 0.2 / 0.5 (set the bevel mesh)": "Passscheibe DIN 988 8 x 14 x 0,1/0,2/0,5 (Zahneingriff)",
    "Slide": "Schlitten",
    "Spiral miter gear m = 1, z = 20, left hand, rebored 8 H7 + key 2, hub faced to mounting dist. 16":
        "Spiral-Kegelrad m = 1, z = 20, links, Bohrung 8 H7 + Nut, Nabe auf Einbaumaß 16",
    "Spiral miter gear m = 1, z = 20, right hand, rebored 8 H7, hub faced to mounting dist. 16":
        "Spiral-Kegelrad m = 1, z = 20, rechts, Bohrung 8 H7, Nabe auf Einbaumaß 16",
    "Permanent-magnet brake BRAKE-BKE-0,4-5,0, 24 V, zero backlash, on a 3 mm adapter plate": "Permanentmagnetbremse BRAKE-BKE-0,4-5,0, 24 V, spielfrei, auf 3-mm-Adapterplatte",
    "Stepper motor ST4118S1404-B, long B shaft through brake and encoder": "Schrittmotor ST4118S1404-B, durchgehende B-Welle",
    "Wire thread insert HELICOIL plus M3 x 1.5 d, top plate for the drive bracket screws": "Gewindeeinsatz HELICOIL plus M3 x 1,5 d (Antriebswinkel)",
    "Wire thread insert HELICOIL plus M4 x 1.5 d, hood seat for the adapter screws": "Gewindeeinsatz HELICOIL plus M4 x 1,5 d (Adapter)",
}
SUPPLIER = {"standard part": "Normteil", "own part, milled": "Eigenteil, gefräst", "own part, turned": "Eigenteil, gedreht",
            "own part, cut": "Eigenteil, geschnitten", "own part, cut + milled": "Eigenteil, geschnitten + gefräst",
            "own part, turned + hobbed": "Eigenteil, gedreht + wälzgefräst", "bought to drawing": "Kaufteil nach Zeichnung",
            "e.g. Mink LKB, cut to length": "z. B. Mink LKB, abgelängt"}
MATERIAL = {"steel": "Stahl", "steel 8.8": "Stahl 8.8", "steel 45H": "Stahl 45H", "spring steel": "Federstahl",
            "16MnCr5 case hardened": "16MnCr5 einsatzgehärtet", "S45C, teeth induction hardened": "S45C, Zähne induktionsgehärtet",
            "PA / steel": "PA / Stahl"}


def designation(s):
    if s in DESIGNATION:
        return DESIGNATION[s]
    m = re.match(r"ISO (4762|10642|4029|8734) (.*)", s)
    if m:
        kind = {"4762": "Zylinderschraube", "10642": "Senkschraube", "4029": "Gewindestift", "8734": "Zylinderstift"}[m.group(1)]
        return f"{kind} ISO {m.group(1)} - {m.group(2).replace('.', ',')}"
    return s


def supplier(s):
    return SUPPLIER.get(s, s)


def material(s):
    return MATERIAL.get(s, s)
