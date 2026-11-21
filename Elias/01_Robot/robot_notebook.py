import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")


@app.cell
def _():
    import json
    import os
    import marimo as mo
    HERE = str(mo.notebook_dir())
    R = json.load(open(os.path.join(HERE, "robot_selection.json")))
    DS = json.load(open(os.path.join(HERE, "datasheet_values.json"), encoding="utf-8"))
    return DS, HERE, R, json, mo, os


@app.cell
def _(mo):
    mo.md(r"""
    # E1 - Robot selection and placement: ABB IRB 1300-7/1.4
    **Robot gripper, part B (PA 6) - group 10 - Ovdiienko / Bitsch**


    Requirement from the assignment: a robot suited to the working area, the 5 s cycle and the part mass. All numbers
    below come from `robot_selection.py` (run it after a CAD change); the datasheet values are marked in the vendor
    PDFs by `mark_datasheets.py` (yellow = value used, red frame = diagram).
    """)
    return


@app.cell
def _(R, mo):
    _p = R["payload"]
    _rows = "\n".join(f"| {c['check']} | {c['value']:.4g} | {c['limit']:.4g} | {c['use'] * 100:.0f} % | {'✓' if c['use'] <= 1 else '✗'} |"
                      for c in R["checks"])
    mo.md(f"""
## 1. Payload

Gripper {_p['m_gripper']:.3f} kg (CAD build: own parts from volume, Nanotec parts from their data sheets) + part B
{_p['m_part']:.3f} kg = **{_p['mass']:.3f} kg**, centre of gravity **Z = {_p['Z'] * 1e3:.1f} mm** below the flange,
**L = {_p['L'] * 1e3:.1f} mm** off the axis. Inertia about the CoG from the CAD solids, each scaled to its mass.

| check (spec 3HAC070393-001 rev. M) | value | limit | use | |
|---|---|---|---|---|
{_rows}

The wrist torque about axis 4/5 is the highest use (tool horizontal, path acceleration included); everything else
is below 50 %.
""")
    return


@app.cell
def _(HERE, mo, os):
    mo.hstack([mo.image(src=os.path.join(HERE, "figures", "load_diagram.png"), width=420)]
              + ([mo.image(src=os.path.join(HERE, "out", "datasheets", "irb_load_diagram.png"), width=420)]
                 if os.path.exists(os.path.join(HERE, "out", "datasheets", "irb_load_diagram.png")) else []),
              justify="start", wrap=True)
    return


@app.cell
def _(R, mo):
    _c, _b = R["placement"]["current"], R["placement"]["best"]
    mo.md(f"""
## 2. Cycle time

The enclosure is only 1200 mm wide (Abb. 1): a plain joint swing pick → place would take the forearm 318 mm
through the fence (`04_Simulation/clearance.py`). One via in the middle of the swing, where the arm points at the
fence (wrist pulled in to {R['transfer']['via_r']:.0f} mm at {R['transfer']['via_z']:.0f} mm, axis 1 keeps turning): ≥ 96 mm to the fence,
≥ 20° axis margin. Along this path the fastest time law with the TCP acceleration ≤ 4.96 m/s² (what the grip
force is sized for) and every axis ≤ 80 % of its speed takes **{R['transfer']['t_loaded']:.2f} s**; the empty way back
{R['transfer']['t_empty']:.2f} s. With lifting, lowering, closing and opening the measured cycle is **4.48 s of 5 s**
(`04_Simulation/e4_cycle.py`). (Plain MoveJ without the fence: {_c['t_move']:.2f} s.)
The robot allows {R['a_tcp_max']:.0f} m/s² at the TCP.

## 3. Placement

Grid over the base offset x and the pedestal height. At every waypoint (pick, place, each also 60 mm lifted) the
closed-form IK must exist, every axis must stay ≥ 10° from its limit, and the arm must not be stretched beyond 95 %.
{R['placement']['n_ok']} of {R['placement']['n']} grid points pass.

| placement | min. axis margin | arm stretched | axis 5 at the waypoints | MoveJ |
|---|---|---|---|---|
| **chosen: x {_c['x']:.0f} mm, pedestal {_c['h']:.0f} mm** | {_c['margin']:.1f}° | {_c['stretch'] * 100:.0f} % | {_c['a5']} | {_c['t_move']:.2f} s |
| lowest feasible pedestal: x {_b['x']:.0f}, {_b['h']:.0f} mm | {_b['margin']:.1f}° | {_b['stretch'] * 100:.0f} % | {_b['a5']} | {_b['t_move']:.2f} s |

The chosen placement keeps the arm least stretched (best accuracy, furthest from the singularity) with 20° margin;
a lower pedestal is possible but only buys a more stretched arm. Axis 5, which forced the pedestal with the
IRB 1600 (±115°), stays below 91° of its ±130°.
""")
    return


@app.cell
def _(HERE, mo, os):
    mo.image(src=os.path.join(HERE, "figures", "placement_map.png"), width=640)
    return


@app.cell
def _(R, mo):
    _rows = "\n".join(f"| {c['name']} | {c['reach']} | {'✓' if c['reach_ok'] else '✗'} | {c['payload']:.1f} | {'✓' if c['payload_ok'] else '✗'} | "
                      f"{c['a1']} | {c['v_tcp']:.2f} | {c['ctrl']} | {c['src']} |" for c in R["candidates"])
    mo.md(f"""
## 4. Candidates

Required reach from the chosen base to the farthest waypoint: **{R['req_reach_mm']:.0f} mm**. TCP speed estimated
from axis 1 alone at that radius.

| robot | reach [mm] | | payload [kg] | | axis 1 [°/s] | v_TCP [m/s] | controller | source |
|---|---|---|---|---|---|---|---|---|
{_rows}

All industrial candidates qualify; the UR10e is short on reach and speed. **IRB 1300-7/1.4** because it is the
current ABB generation (OmniCore, 220 × 220 mm footprint, 27 % shorter cycle times than the IRB 1600 per ABB), has
the fastest axis 1 of the list, and its load diagram is met with ≥ 30 % margin everywhere.
""")
    return


@app.cell
def _(DS, HERE, mo, os):
    _items = []
    for _d in DS:
        _png = os.path.join(HERE, "out", "datasheets", f"{_d['id']}.png")
        _md = mo.md(f"**{_d['value']}**  \n{_d['file']}, p. {_d['page']} - *used for:* {_d['why']}")
        _items.append(mo.vstack([_md, mo.image(src=_png, width=620)]) if os.path.exists(_png) else _md)
    mo.vstack([mo.md("## 5. Datasheet values used (marked in the vendor PDFs)")] + _items)
    return


if __name__ == "__main__":
    app.run()
