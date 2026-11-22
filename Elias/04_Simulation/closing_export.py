"""Force and opening over time for the explainer film (scene 5): python closing_export.py -> out/closing_logs.json.

Cases: direct close in 0.12 s, the built tolerant close (fast 0.05 s, slow 20 mm/s) centred and with the part 5 mm
off centre. Same model and profiles as closing_time.py.
"""
import json
import os

import numpy as np

import closing_time as C

CASES = {"direct": ("direct", 0.12, 0.0, C.V_TOUCH), "tolerant": ("tolerant", 0.05, 0.0, 0.020),
         "tolerant_off5": ("tolerant", 0.05, C.TOL, 0.020)}

out = {}
for key, (name, tf, off, vt) in CASES.items():
    r, lg = C.run(name, tf, offset=off, v_touch=vt)
    t = np.array(lg["t"])
    keep = np.searchsorted(t, np.arange(t[0], t[-1], 0.0005))     # 0.5 ms grid, enough for the impact spike
    out[key] = dict(result=r, **{k: [round(float(np.array(v)[i]), 4) for i in keep] for k, v in lg.items()})
    print(key, f"peak {r['f_peak']:.1f} N, grip {r['t_grip'] * 1000:.0f} ms, pushed {r['part_push_mm']:.2f} mm")
json.dump(out, open(os.path.join(C.S.B.OUT, "closing_logs.json"), "w"))
