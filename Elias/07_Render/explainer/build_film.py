"""Writes the HyperFrames scenes of the explainer film from the simulation results, so every number on screen comes
from the repo: python build_film.py  (then: cd film && npx hyperframes preview / render).

Inputs: 04_Simulation/out/e4_cycle.json (segments, cycle time), out/e4_collision_log.csv (distances every 10 ms),
out/cell_clean_frames.json (slow factor of the cycle video), out/closing_results.json.
"""
import csv
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "..", "04_Simulation", "out")
FILM = os.path.join(HERE, "film")

BG, INK, ACC, SAF, LOAD, MU = "#0E1013", "#E8E6E1", "#3CC9B4", "#F08A4B", "#8E949C", "#5AA9F0"
S5_LEAD, S5_CURVES = 4.5, 19.1           # closing videos, then the Manim curves (s5_curves.mp4)
S9_VIDEO, S9_HOLD = 11.0, 2.5

STYLE = f"""
    #root {{ position: absolute; inset: 0; background: {BG}; font-family: Inter, sans-serif; color: {INK}; overflow: hidden; }}
    .cap {{ position: absolute; left: 86px; top: 64px; font-size: 40px; font-weight: 400; letter-spacing: -0.01em; }}
    .sub {{ position: absolute; left: 86px; top: 116px; font-size: 28px; color: {LOAD}; }}
"""


def scene(cid, body, script, extra_style=""):
    return f"""<template>
  <style>{STYLE}{extra_style}
  </style>
  <div id="root" data-composition-id="{cid}" data-width="1920" data-height="1080">
{body}
  </div>
  <script>
    const tl = gsap.timeline({{ paused: true }});
{script}
    window.__timelines["{cid}"] = tl;
  </script>
</template>
"""


def s5():
    clo = json.load(open(os.path.join(SIM, "closing_results.json")))
    tol = next(c for c in clo["cases"] if c["profile"] == "tolerant" and c["offset_mm"] > 0 and c["v_touch"] > 0.015
               and not c["blocked"])
    dire = next(c for c in clo["cases"] if c["profile"] == "direct" and abs(c["t_fast"] - 0.12) < 1e-6)
    body = f"""
    <div id="s5-lead" class="clip" data-start="0" data-duration="{S5_LEAD + 0.5}">
      <div class="cap">Closing on the part</div>
      <div class="sub">MuJoCo, slowed down 10&times;</div>
      <div id="s5-l" class="pane" style="left:56px">
        <div class="ph" style="color:{SAF}">direct</div>
        <div class="ps">full speed until contact, closed in {dire['t_fast'] * 1000:.0f} ms</div>
      </div>
      <div id="s5-r" class="pane" style="left:984px">
        <div class="ph" style="color:{ACC}">two-stage (built)</div>
        <div class="ps">part placed {tol['offset_mm']:.0f} mm off centre</div>
      </div>
    </div>
    <video id="s5-vl" class="clip vid" src="assets/closing_direct.mp4" muted playsinline data-start="0"
           data-duration="{S5_LEAD + 0.5}" style="left:56px"></video>
    <video id="s5-vr" class="clip vid" src="assets/closing_tolerant.mp4" muted playsinline data-start="0"
           data-duration="{S5_LEAD + 0.5}" style="left:984px"></video>
    <video id="s5-curves" class="clip" src="assets/s5_curves.mp4" muted playsinline data-start="{S5_LEAD}"
           data-duration="{S5_CURVES}" style="position:absolute;left:0;top:0;width:1920px;height:1080px;z-index:5"></video>"""
    style = f"""
    .pane {{ position: absolute; top: 210px; width: 880px; }}
    .ph {{ font-size: 34px; }}
    .ps {{ font-size: 28px; color: {LOAD}; margin-top: 6px; }}
    .vid {{ position: absolute; top: 330px; width: 880px; height: 495px; object-fit: cover; border-radius: 6px; z-index: 2; }}
"""
    script = f"""
    tl.fromTo("#s5-lead .cap, #s5-lead .sub", {{ opacity: 0, y: 8 }}, {{ opacity: 1, y: 0, duration: 0.5, ease: "power2.out", stagger: 0.1 }}, 0.1);
    tl.fromTo(".pane", {{ opacity: 0, y: 10 }}, {{ opacity: 1, y: 0, duration: 0.5, ease: "power2.out", stagger: 0.12 }}, 0.25);
    tl.fromTo("#s5-curves", {{ opacity: 0 }}, {{ opacity: 1, duration: 0.45, ease: "power1.inOut" }}, {S5_LEAD});"""
    return scene("s5", body, script, style), S5_LEAD + S5_CURVES


def s9():
    """The whole cell from high and wide (also the task of scene 1), then the camera follows one part from the
    buffer belt through grip, transfer and place onto chain 1 until its carrier goes into the machine."""
    import sys
    sys.path.insert(0, os.path.join(HERE, "..", "..", "02_CAD_Body"))
    import gripper_params as GP
    ec = json.load(open(os.path.join(SIM, "e4_cycle.json")))
    cyc = ec["chain_1"]
    vid = json.load(open(os.path.join(SIM, "cell_clean_frames.json")))
    fr_t = [f["t"] for f in vid["frames"]]
    c0, T, takt = vid["follow_cycle_start"], cyc["cycle_time"], cyc["budget"]
    vdur = len(fr_t) / vid["fps"]
    v_follow = next(i for i, x in enumerate(fr_t) if x >= c0 - 1.1) / vid["fps"]   # camera reaches the pick
    VAF = [(f["v"], f["a"], f["f"]) for f in vid["frames"]]
    t_mech = c0 + takt + 4.4                                   # cell_video.SEE_MECH: next grip with the shell hidden
    v_mech = next(i for i, x in enumerate(fr_t) if x >= t_mech - 0.2) / vid["fps"]
    v_card = next(i for i, x in enumerate(fr_t) if x >= c0 + T) / vid["fps"]
    segs = [(s["segment"], s["t_start"], s["duration"]) for s in cyc["segments"]]
    rows = list(csv.DictReader(open(os.path.join(SIM, "e4_collision_log_chain1.csv")), delimiter=";"))
    show = [("belt", "belt [mm]", "buffer belt"), ("carrier_1", "carrier_1 [mm]", "workpiece carrier"),
            ("chain_1", "chain_1 [mm]", "chain"), ("fence_right", "fence_right [mm]", "fence")]
    t = [float(r["t [s]"]) for r in rows]
    dist = {k: [float(r[c]) for r in rows] for k, c, _ in show}
    mins = {k: (min(v), t[v.index(min(v))]) for k, v in dist.items()}
    seg_html = "".join(
        f'<div class="seg" id="s9-seg{i}" style="left:{s0 / takt * 100:.3f}%;width:{d / takt * 100:.3f}%"></div>'
        for i, (_, s0, d) in enumerate(segs))
    rows_html = "".join(
        f'<div class="row" id="s9-row-{k}"><span class="rn">{lab}</span><span class="rv" id="s9-v-{k}">&ndash;</span></div>'
        for k, _, lab in show)
    body = f"""
    <video id="s9-vid" class="clip" src="assets/cell_cycle.mp4" muted playsinline data-start="0" data-duration="{vdur:.2f}"
           style="position:absolute;left:0;top:0;width:1920px;height:1080px;object-fit:cover"></video>
    <img id="s9-still" class="clip" src="assets/cell_cycle_last.png" data-start="{vdur - 0.05:.2f}" data-duration="{S9_HOLD + 0.05}"
         style="position:absolute;left:0;top:0;width:1920px;height:1080px;object-fit:cover" />
    <div id="s9-ui" class="clip" data-start="0" data-duration="{vdur + S9_HOLD:.2f}">
      <div class="shade"></div>
      <div class="plate"></div>
      <div id="s9-a"><div class="cap">The cell</div>
        <div class="sub">part B, {GP.B_MASSE:.3f} kg: buffer belt &rarr; robot &rarr; two carrier chains, takt {GP.TAKTZEIT:.1f} s</div></div>
      <div id="s9-b"><div class="cap">Following one part</div>
        <div class="sub">MuJoCo, real robot path, slowed down at grip and place</div></div>
      <div id="s9-c"><div class="cap">Inside the gripper</div>
        <div class="sub">next cycle, shell hidden: motor, brake, bevel pair, pinion, two racks</div></div>
      <div class="clock" id="s9-clock"></div>
      <div class="panel"><div class="ph">gripper</div>
        <div class="row"><span class="rn"><i class="sw" style="background:#FFD133"></i>velocity</span><span class="rv" id="s9-vv">&ndash;</span></div>
        <div class="row"><span class="rn"><i class="sw" style="background:#FF8019"></i>acceleration</span><span class="rv" id="s9-va">&ndash;</span></div>
        <div class="row"><span class="rn"><i class="sw" style="background:#EB1F14"></i>grip force / jaw</span><span class="rv" id="s9-vf">&ndash;</span></div>
        <div class="ph" style="margin-top:18px">closest approach</div>{rows_html}</div>
      <div class="strip"><div class="stop"><span id="s9-sname">waiting at the pick</span><span class="tk">takt {takt:.1f} s</span></div>
        <div class="bar">{seg_html}<div class="res" style="left:{T / takt * 100:.3f}%;width:{(takt - T) / takt * 100:.3f}%"></div>
          <div class="head" id="s9-head"></div></div>
        <div class="ticks"><span>0 s</span><span class="tc" style="left:{T / takt * 100:.3f}%">cycle {T:.2f} s</span></div>
      </div>
      <div class="card" id="s9-card"><div class="big"><b>{T:.2f} s</b> of {takt:.1f} s takt</div>
        <div class="small">{takt - T:.2f} s reserve &middot; {'no contact' if not ec['any_contact'] else 'CONTACT'}</div></div>
    </div>"""
    style = f"""
    .shade {{ display: none; }}
    #s9-c {{ opacity: 0; }}
    #s9-ui .sub {{ color: rgba(235,235,245,.6); }}
    #s9-ui .cap {{ font-weight: 600; letter-spacing: -0.015em; }}
    .plate {{ position: absolute; left: 56px; top: 40px; width: 960px; height: 136px; background: rgba(28,28,30,.42); backdrop-filter: blur(28px) saturate(160%); -webkit-backdrop-filter: blur(28px) saturate(160%); border: 1px solid rgba(255,255,255,.10); box-shadow: inset 0 1px 0 rgba(255,255,255,.08); border-radius: 18px; }}
    .clock {{ position: absolute; right: 86px; top: 64px; font-size: 34px; font-variant-numeric: tabular-nums; }}
    .panel {{ position: absolute; right: 86px; top: 200px; width: 500px; padding: 24px 28px; background: rgba(28,28,30,.42); backdrop-filter: blur(28px) saturate(160%); -webkit-backdrop-filter: blur(28px) saturate(160%); border: 1px solid rgba(255,255,255,.10); box-shadow: inset 0 1px 0 rgba(255,255,255,.08); border-radius: 18px; }}
    .panel .ph {{ font-size: 20px; font-weight: 600; color: rgba(235,235,245,.55); text-transform: uppercase; letter-spacing: .06em; margin-bottom: 8px; }}
    .row {{ white-space: nowrap; display: flex; justify-content: space-between; font-size: 28px; padding: 9px 0; border-top: 1px solid rgba(255,255,255,.07); }}
    .sw {{ display: inline-block; width: 14px; height: 14px; border-radius: 3px; margin-right: 12px; vertical-align: 1px; }}
    .rv {{ font-variant-numeric: tabular-nums; color: #FFFFFF; font-weight: 500; }}
    .rn {{ color: rgba(235,235,245,.72); }}
    .strip {{ position: absolute; left: 86px; width: 1180px; bottom: 70px; padding: 20px 28px 28px; background: rgba(28,28,30,.42); backdrop-filter: blur(28px) saturate(160%); -webkit-backdrop-filter: blur(28px) saturate(160%); border: 1px solid rgba(255,255,255,.10); box-shadow: inset 0 1px 0 rgba(255,255,255,.08); border-radius: 18px; }}
    .stop {{ display: flex; justify-content: space-between; font-size: 26px; margin-bottom: 12px; }}
    .stop .tk {{ color: {LOAD}; font-size: 24px; }}
    .bar {{ position: relative; height: 14px; background: rgba(255,255,255,.06); border-radius: 7px; }}
    .seg {{ position: absolute; top: 0; height: 14px; background: rgba(255,255,255,.18); border-right: 2px solid {BG}; }}
    .res {{ position: absolute; top: 0; height: 14px; background: repeating-linear-gradient(45deg, rgba(142,148,156,.25) 0 6px, transparent 6px 12px); }}
    .head {{ position: absolute; top: -6px; left: 0; width: 3px; height: 26px; background: {INK}; }}
    .ticks {{ position: relative; height: 28px; margin-top: 8px; font-size: 24px; color: {LOAD}; }}
    .ticks span {{ position: absolute; transform: translateX(-50%); white-space: nowrap; }}
    .ticks span:first-child {{ transform: none; }}
    .ticks span.tc {{ transform: translateX(-100%); }}
    .card {{ position: absolute; right: 86px; top: 760px; width: 500px; padding: 26px 28px; background: rgba(28,28,30,.42); backdrop-filter: blur(28px) saturate(160%); -webkit-backdrop-filter: blur(28px) saturate(160%); border: 1px solid rgba(255,255,255,.10); box-shadow: inset 0 1px 0 rgba(255,255,255,.08); border-radius: 18px; text-align: center; opacity: 0; }}
    .card .big {{ font-size: 46px; white-space: nowrap; }}
    .card .big b {{ color: #FFFFFF; font-weight: 700; }}
    .card .big {{ color: rgba(235,235,245,.72); }}
    .card .small {{ font-size: 26px; color: {LOAD}; margin-top: 10px; }}
"""
    data = dict(t=t, d={k: dist[k] for k in dist}, segs=segs, T=T, takt=takt, c0=c0, F=fr_t, fps=vid["fps"], VAF=VAF,
                mins={k: v for k, v in mins.items()})
    script = f"""
    const D = {json.dumps(data, separators=(",", ":"))};
    const keys = Object.keys(D.d);
    function at(fi) {{
      const fj = Math.max(0, Math.min(D.F.length - 1, Math.round(fi)));
      const tr = D.F[fj];
      const vaf = D.VAF[fj];
      document.getElementById("s9-vv").textContent = vaf[0].toFixed(2) + " m/s";
      document.getElementById("s9-va").textContent = vaf[1].toFixed(1) + " m/s²";
      document.getElementById("s9-vf").textContent = vaf[2] > 0 ? vaf[2].toFixed(1) + " N" : "open";
      const tc = tr - D.c0;                       // time in the followed cycle
      const i = Math.max(0, Math.min(D.t.length - 1, Math.round(tc / 0.01)));
      document.getElementById("s9-clock").textContent = tc < 0 || tc > D.takt ? "" : "t = " + tc.toFixed(2) + " s";
      document.getElementById("s9-head").style.left = (100 * Math.min(D.takt, Math.max(0, tc)) / D.takt) + "%";
      let si = 0;
      D.segs.forEach((s, k) => {{ if (tc >= s[1]) si = k; }});
      if (tc > D.T) si = -1;
      document.getElementById("s9-sname").textContent = tc < 0 ? "waiting at the pick" : (si < 0 ? "on the carrier, into the machine" : D.segs[si][0]);
      D.segs.forEach((s, k) => {{ document.getElementById("s9-seg" + k).style.background = tc < 0 ? "rgba(255,255,255,.18)" : (k === si ? "#FFFFFF" : (k < si || si < 0 ? "rgba(255,255,255,.5)" : "rgba(255,255,255,.18)")); }});
      keys.forEach((k) => {{                      // running minimum over the cycle: ends on the closest approach
        const el = document.getElementById("s9-v-" + k);
        if (tc < 0) {{ el.innerHTML = "&ndash;"; return; }}
        let m = 1e9;
        for (let j = 0; j <= i; j++) m = Math.min(m, D.d[k][j]);
        el.textContent = m >= 500 ? "> 500 mm" : Math.round(m) + " mm";
        const near = Math.abs(tc - D.mins[k][1]) < 0.12;
        document.getElementById("s9-row-" + k).style.color = near ? "{SAF}" : "{INK}";
      }});
    }}
    const clk = {{ f: 0 }};
    tl.fromTo(clk, {{ f: 0 }}, {{ f: D.F.length - 1, duration: D.F.length / D.fps, ease: "none", onUpdate: () => at(clk.f) }}, 0);
    tl.fromTo("#s9-a", {{ opacity: 0, y: 8 }}, {{ opacity: 1, y: 0, duration: 0.6, ease: "power2.out" }}, 0.2);
    tl.fromTo("#s9-a", {{ opacity: 1 }}, {{ opacity: 0, duration: 0.4, ease: "power1.in" }}, {v_follow - 0.4:.2f});
    tl.fromTo("#s9-b, .clock", {{ opacity: 0, y: 8 }}, {{ opacity: 1, y: 0, duration: 0.5, ease: "power2.out" }}, {v_follow:.2f});
    tl.fromTo(".panel, .strip", {{ opacity: 0, y: 12 }}, {{ opacity: 1, y: 0, duration: 0.6, ease: "power2.out", stagger: 0.1 }}, {v_follow + 0.2:.2f});
    tl.fromTo("#s9-card", {{ opacity: 0, scale: 0.96 }}, {{ opacity: 1, scale: 1, duration: 0.6, ease: "power2.out" }}, {v_card + 0.3:.2f});
    tl.to("#s9-card, #s9-b, .strip, .clock", {{ opacity: 0, duration: 0.3, ease: "power1.in" }}, {v_mech - 0.3:.2f});
    tl.fromTo("#s9-c", {{ opacity: 0, y: 8 }}, {{ opacity: 1, y: 0, duration: 0.5, ease: "power2.out" }}, {v_mech:.2f});"""
    return scene("s9", body, script, style), vdur + S9_HOLD


def index(scenes):
    hosts, t0 = [], 0.0
    for cid, dur in scenes:
        hosts.append(f'      <div id="{cid}" data-composition-id="{cid}" data-composition-src="compositions/{cid}.html" '
                     f'data-start="{t0:.2f}" data-duration="{dur:.2f}" data-track-index="1" data-track-kind="graphics" '
                     f'data-width="1920" data-height="1080"></div>')
        t0 += dur
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=1920, height=1080" />
    <script src="https://cdn.jsdelivr.net/npm/gsap@3.15.0/dist/gsap.min.js"></script>
    <style>
      * {{ margin: 0; padding: 0; box-sizing: border-box; }}
      html, body {{ margin: 0; width: 1920px; height: 1080px; overflow: hidden; background: {BG}; }}
      #root {{ position: relative; width: 100%; height: 100%; }}
    </style>
  </head>
  <body>
    <div id="root" data-composition-id="main" data-start="0" data-duration="{t0:.2f}" data-width="1920" data-height="1080">
{chr(10).join(hosts)}
    </div>
    <script>
      const tl = gsap.timeline({{ paused: true }});
      window.__timelines["main"] = tl;
    </script>
  </body>
</html>
"""


if __name__ == "__main__":
    os.makedirs(os.path.join(FILM, "compositions"), exist_ok=True)
    out = []
    for fn in (s5, s9):
        html, dur = fn()
        cid = fn.__name__
        open(os.path.join(FILM, "compositions", f"{cid}.html"), "w", encoding="utf-8").write(html)
        out.append((cid, dur))
    open(os.path.join(FILM, "index.html"), "w", encoding="utf-8").write(index(out))
    print("scenes", out)
