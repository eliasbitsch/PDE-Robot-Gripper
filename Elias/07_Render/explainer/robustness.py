"""Explainer scene: Monte Carlo robustness of the grip (Manim CE).

manim -r 854,480 --fps 30 robustness.py Robustness
Data: 04_Simulation/out/robustness.json (python robustness.py), full cycles in the validated MuJoCo model.
"""
import json
import os

import numpy as np
from manim import (DOWN, LEFT, RIGHT, UP, Axes, Create, DashedLine, Dot, FadeIn, FadeOut, LaggedStart, Rectangle,
                   Scene, Tex, VGroup, Write, linear)

from style import ACC, INK, LOAD, MM, MS2, N, SAF, SIM_OUT, P, tex

R = json.load(open(os.path.join(SIM_OUT, "robustness.json")))
SPEC = [r for r in R["results"] if r["study"] == "spec"]
STRESS = [r for r in R["results"] if r["study"] == "stress"]
RG = R["ranges"]["spec"]
_CORNERS = os.path.join(SIM_OUT, "robustness_corners.json")
CORNERS = json.load(open(_CORNERS)) if os.path.exists(_CORNERS) else []
if len({(c["offset"], c["mu"], c["force"], c["mass"]) for c in CORNERS}) != 16:
    CORNERS = []                                       # only shown as "16 tolerance corners" when all 16 are there
F0 = P.F_GRIP_SET


class Robustness(Scene):
    def construct(self):
        cap = Tex(rf"{len(SPEC):,} grips, scattered inputs", color=INK, font_size=30).to_corner(UP + LEFT, buff=0.45)
        self.play(FadeIn(cap, shift=0.1 * RIGHT), run_time=0.6)

        # what is scattered, each from its source
        f_lo, f_hi = RG["force"][0], RG["force"][1]
        rows = VGroup(
            Tex(rf"part position", rf"$\pm {RG['offset'][1] * 1000:.0f}{MM}$", color=INK, font_size=26),
            Tex(rf"pad friction", rf"${RG['mu'][0]:.1f} \dots {RG['mu'][1]:.1f}$", color=INK, font_size=26),
            Tex(rf"grip force", rf"${f_lo:.1f} \dots {f_hi:.1f}{N}$", color=INK, font_size=26),
            Tex(rf"part mass", rf"$0 \ldots {(RG['mass'][1] - 1) * 100:.0f}\,\%$ heavier", color=INK, font_size=26),
        )
        for r in rows:
            r[0].set_color(LOAD)
            r[1].next_to(r[0], RIGHT, buff=0.3)
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.22).to_edge(LEFT, buff=0.7).shift(0.3 * UP)
        sub = Tex(rf"each one a full cycle: close, lift, swing at ${P.A_PATH_REAL:.2f}{MS2}$, set down", color=LOAD,
                  font_size=22).next_to(rows, DOWN, buff=0.45).align_to(rows, LEFT)
        self.play(LaggedStart(*[FadeIn(r, shift=0.15 * RIGHT) for r in rows], lag_ratio=0.3), run_time=1.6)
        self.play(FadeIn(sub), run_time=0.5)

        # histogram of the peak friction use, built up as the grips come in
        u = np.array([r["util_max"] for r in SPEC])
        held = np.array([r["held"] for r in SPEC])
        ax = Axes(x_range=[0, 1.1, 0.25], y_range=[0, 1, 1], x_length=5.6, y_length=3.0,
                  axis_config=dict(color=LOAD, stroke_width=2, include_tip=False, font_size=20),
                  x_axis_config=dict(numbers_to_include=[0, 0.25, 0.5, 0.75, 1.0], decimal_number_config=dict(num_decimal_places=2)),
                  y_axis_config=dict(include_ticks=False)).to_edge(RIGHT, buff=0.6).shift(0.2 * DOWN)
        for lab in ax.get_x_axis().numbers.submobjects:
            lab.set_color(LOAD)
        xl = Tex(r"peak friction use over the cycle", color=LOAD, font_size=22).next_to(ax.x_axis, DOWN, buff=0.35)
        lim = DashedLine(ax.c2p(1, 0), ax.c2p(1, 1.05), color=SAF, stroke_width=2.5, dash_length=0.07)
        lim_l = Tex(r"slips", color=SAF, font_size=22).next_to(lim, UP, buff=0.08)
        self.play(Create(ax), FadeIn(xl), Create(lim), FadeIn(lim_l), run_time=0.9)
        edges = np.arange(0.0, 1.1001, 0.02)
        steps = [k for k in (500, 2000, 5000) if k < len(u)] + [len(u)]
        hmax = np.histogram(u, edges)[0].max()

        def bars(k):
            h = np.histogram(u[:k], edges)[0] / hmax
            return VGroup(*[Rectangle(width=ax.x_length * 0.02 / 1.1 * 0.9, height=max(1e-3, hi * ax.y_length),
                                      stroke_width=0, fill_color=ACC, fill_opacity=0.9)
                            .move_to(ax.c2p(e + 0.01, 0), aligned_edge=DOWN) for e, hi in zip(edges[:-1], h)])
        hist = bars(steps[0])
        count = Tex(rf"{held[:steps[0]].sum():,} of {steps[0]:,} held", color=INK, font_size=26).next_to(ax, UP, buff=0.2).align_to(ax, LEFT)
        self.play(FadeIn(hist), FadeIn(count), run_time=0.5)
        for k in steps[1:]:
            nxt = Tex(rf"{held[:k].sum():,} of {k:,} held", color=INK, font_size=26).move_to(count, aligned_edge=LEFT)
            self.play(hist.animate.become(bars(k)), FadeOut(count, run_time=0.2), FadeIn(nxt, run_time=0.3), run_time=0.7)
            count = nxt
        verdict = rf"all {len(u):,} held" if held.all() else rf"{held.sum():,} of {len(u):,} held"
        line = rf"{verdict}, highest sampled friction use {u.max():.3f}"
        if CORNERS:
            uc = max(c["util_max"] for c in CORNERS)
            ok = "all held" if all(c["held"] for c in CORNERS) else "not all held"
            line += rf"; 16 tolerance corners: {ok}, lowest safety {1 / uc:.2f}"
        top = Tex(line, color=INK, font_size=22).to_edge(DOWN, buff=0.3)
        self.play(FadeIn(top, shift=0.1 * UP), run_time=0.6)
        self.wait(1.6)

        # stress: push friction and force far below spec until grips fail
        self.play(FadeOut(VGroup(rows, sub, hist, ax, xl, lim, lim_l, count, top)), run_time=0.6)
        cap2 = Tex(rf"{len(STRESS):,} more grips, friction and force pushed far below spec", color=INK,
                   font_size=30).move_to(cap, aligned_edge=LEFT)
        ax2 = Axes(x_range=[0.06, 0.32, 0.05], y_range=[20, 50, 10], x_length=7.6, y_length=3.9,
                   axis_config=dict(color=LOAD, stroke_width=2, include_tip=False, font_size=20),
                   x_axis_config=dict(numbers_to_include=[0.1, 0.15, 0.2, 0.25, 0.3], decimal_number_config=dict(num_decimal_places=2)),
                   y_axis_config=dict(numbers_to_include=[20, 30, 40, 50])).shift(0.15 * DOWN + 0.4 * LEFT)
        for lab in ax2.get_x_axis().numbers.submobjects + ax2.get_y_axis().numbers.submobjects:
            lab.set_color(LOAD)
        xl2 = Tex(r"pad friction $\mu$", color=LOAD, font_size=22).next_to(ax2.x_axis, DOWN, buff=0.35)
        yl2 = tex(r"F\;[\mathrm{N}]", font_size=22, color=LOAD).next_to(ax2.y_axis, UP, buff=0.1)
        self.play(FadeOut(cap), FadeIn(cap2), Create(ax2), FadeIn(xl2), FadeIn(yl2), run_time=0.9)
        order = np.random.default_rng(3).permutation(len(STRESS))
        dots = [Dot(ax2.c2p(STRESS[i]["mu"], STRESS[i]["force"]), radius=0.026,
                    color=ACC if STRESS[i]["held"] else SAF) for i in order]
        batches = [dots[a:a + 250] for a in range(0, len(dots), 250)]
        for b in batches:
            self.play(FadeIn(VGroup(*b)), run_time=0.22, rate_func=linear)
        n_fail = sum(not r["held"] for r in STRESS)
        key = VGroup(VGroup(Dot(color=ACC, radius=0.05), Tex(r"held", color=INK, font_size=22)).arrange(RIGHT, buff=0.12),
                     VGroup(Dot(color=SAF, radius=0.05), Tex(rf"slips or drops ({n_fail})", color=INK, font_size=22))
                     .arrange(RIGHT, buff=0.12)).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
        key.next_to(ax2, RIGHT, buff=0.35).align_to(ax2, UP)
        self.play(FadeIn(key), run_time=0.5)

        # hand calculation without safety: 2 mu F = m (g + a)
        need = lambda mu: P.B_MASSE * (P.G + P.A_PATH_REAL) / (P.N_BACKEN * mu)
        mf = P.B_MASSE * (P.G + P.A_PATH_REAL) / P.N_BACKEN                # mu F on the curve
        hc = ax2.plot(need, x_range=[mf / 50, mf / 20], color=INK, stroke_width=2.5)   # inside the 20 ... 50 N frame
        hc_l = tex(r"2\mu F = m\,(g+a)", font_size=24).next_to(ax2.c2p(mf / 49, 49), RIGHT, buff=0.12)
        self.play(Create(hc), FadeIn(hc_l), run_time=1.0)
        box = Rectangle(width=ax2.c2p(RG["mu"][1], 0)[0] - ax2.c2p(RG["mu"][0], 0)[0],
                        height=ax2.c2p(0, RG["force"][1])[1] - ax2.c2p(0, RG["force"][0])[1], color=INK, stroke_width=3)
        box.move_to(ax2.c2p(np.mean(RG["mu"]), np.mean(RG["force"])))
        box_l = Tex(r"spec", color=INK, font_size=22).next_to(box, UP, buff=0.08)
        mf_fail = max((r["mu"] * r["force"] for r in STRESS if not r["held"]), default=None)
        mf_spec = min(r["mu"] * r["force"] for r in SPEC)
        self.play(Create(box), FadeIn(box_l), run_time=0.8)
        tail = rf"strongest sampled failure ${mf_fail:.1f}{N}$" if mf_fail is not None else "no sampled failure"
        end = Tex(rf"friction capacity $\mu F$ per jaw: weakest sampled grip in spec ${mf_spec:.1f}{N}$, {tail}",
                  color=INK, font_size=24).to_edge(DOWN, buff=0.3)
        self.play(FadeIn(end, shift=0.1 * UP), run_time=0.6)
        self.wait(2.0)
