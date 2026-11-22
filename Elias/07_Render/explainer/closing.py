"""Explainer scene 5: why the gripper closes in two stages (Manim CE).

manim -r 854,480 --fps 30 closing.py Closing
Data: 04_Simulation/out/closing_logs.json (python closing_export.py), same model as closing_time.py.
The plots sit in the lower part of the frame; the MuJoCo closing videos go above them in HyperFrames.
"""
import json
import os

import numpy as np
from manim import (DOWN, LEFT, RIGHT, UP, Axes, Create, DashedLine, Dot, FadeIn, FadeOut, ReplacementTransform, Scene,
                   Tex, ValueTracker, VGroup, always_redraw, linear, Write)

from style import ACC, INK, LOAD, MM, MMS, N, SAF, SIM_OUT, P, n, tex

LOG = json.load(open(os.path.join(SIM_OUT, "closing_logs.json")))
T_MAX = 0.45                                           # s shown on the time axis
F_SET = P.F_GRIP_SET
T_REQ = json.load(open(os.path.join(SIM_OUT, "closing_results.json")))["t_req"]   # s, required closing time


def series(key, field="f"):
    d = LOG[key]
    t = np.array(d["t"])
    y = np.maximum(np.array(d["fL"]), np.array(d["fR"])) if field == "f" else np.array(d[field])   # the harder-hit pad
    m = (t >= 0) & (t <= T_MAX)
    return t[m] * 1000, y[m]


class Closing(Scene):
    def panel(self, x, title, sub, color):
        ax = Axes(x_range=[0, T_MAX * 1000, 100], y_range=[0, 175, 50], x_length=5.4, y_length=2.7,
                  axis_config=dict(color=LOAD, stroke_width=2, include_tip=False, font_size=20),
                  x_axis_config=dict(numbers_to_include=[0, 100, 200, 300, 400]),
                  y_axis_config=dict(numbers_to_include=[0, 50, 100, 150])).move_to(x * RIGHT + 1.15 * DOWN)
        for lab in ax.get_x_axis().numbers.submobjects + ax.get_y_axis().numbers.submobjects:
            lab.set_color(LOAD)
        xl = tex(r"t" + r"\;[\mathrm{ms}]", font_size=22, color=LOAD).next_to(ax.x_axis, DOWN, buff=0.32)
        yl = tex(r"F" + r"\;[\mathrm{N}]", font_size=22, color=LOAD).next_to(ax.y_axis, UP, buff=0.1)
        hd = VGroup(Tex(title, color=color, font_size=28), Tex(sub, color=LOAD, font_size=21)).arrange(DOWN, buff=0.08)
        hd.next_to(ax, UP, buff=0.45)
        setl = DashedLine(ax.c2p(0, F_SET), ax.c2p(T_MAX * 1000, F_SET), color=LOAD, stroke_width=1.5, dash_length=0.06)
        return ax, VGroup(ax, xl, yl, setl), hd, setl

    def construct(self):
        cap = Tex(r"Closing on the part", color=INK, font_size=30).to_corner(UP + LEFT, buff=0.45)
        self.play(FadeIn(cap, shift=0.1 * RIGHT), run_time=0.6)
        rd = LOG["direct"]["result"]
        rt = LOG["tolerant"]["result"]
        ax1, g1, h1, s1 = self.panel(-3.4, r"direct", rf"full speed until contact, closed in {rd['t_fast'] * 1000:.0f} ms", SAF)
        ax2, g2, h2, s2 = self.panel(3.4, r"two-stage (built)",
                                      rf"fast to just before contact, then ${n(rt['v_touch'] * 1000)}{MMS}$", ACC)
        set_lab = tex(rf"{F_SET:.1f}{N}", font_size=20, color=LOAD).next_to(s2.get_end(), UP, buff=0.05).align_to(s2, RIGHT)
        self.play(FadeIn(g1), FadeIn(g2), FadeIn(h1), FadeIn(h2), FadeIn(set_lab), run_time=0.9)

        # both curves drawn together against one clock (slow motion: 450 ms in 6 s)
        tt = ValueTracker(0.0)
        t1, f1 = series("direct")
        t2, f2 = series("tolerant")

        def curve(ax, t, f, col):
            def make():
                k = max(2, int(np.searchsorted(t, tt.get_value())))
                return ax.plot_line_graph(t[:k], np.minimum(f[:k], 175), line_color=col, add_vertex_dots=False,
                                          stroke_width=3)
            return always_redraw(make)

        c1, c2 = curve(ax1, t1, f1, SAF), curve(ax2, t2, f2, ACC)
        clock = always_redraw(lambda: tex(rf"t = {tt.get_value():.0f}" + r"\,\mathrm{ms}", font_size=24, color=LOAD)
                              .to_corner(UP + RIGHT, buff=0.45))
        self.add(c1, c2, clock)
        self.play(tt.animate.set_value(T_MAX * 1000), run_time=6.0, rate_func=linear)

        # peaks
        i1 = int(np.argmax(f1))
        p1 = Dot(ax1.c2p(t1[i1], min(f1[i1], 175)), color=SAF, radius=0.05)
        l1 = tex(rf"{rd['f_peak']:.1f}{N}", font_size=30, color=SAF).next_to(p1, RIGHT, buff=0.12)
        l1b = Tex(r"impact on part and pads", color=LOAD, font_size=20).next_to(l1, DOWN, buff=0.06).align_to(l1, LEFT)
        i2 = int(np.argmax(f2))
        p2 = Dot(ax2.c2p(t2[i2], f2[i2]), color=ACC, radius=0.05)
        l2 = tex(rf"{rt['f_peak']:.1f}{N}", font_size=30, color=ACC).next_to(p2, UP, buff=0.12).shift(0.4 * LEFT)
        self.play(FadeIn(p1, scale=2), Write(l1), FadeIn(l1b), run_time=0.8)
        self.play(FadeIn(p2, scale=2), Write(l2), run_time=0.8)
        ratio = Tex(rf"{rd['f_peak'] / rt['f_peak']:.1f}$\times$ lower peak force, the grip force is reached "
                    rf"after {rt['t_grip'] * 1000:.0f} ms (allowed {T_REQ * 1000:.0f} ms)", color=INK, font_size=24)
        ratio.to_edge(DOWN, buff=0.3)
        self.play(FadeIn(ratio, shift=0.1 * UP), run_time=0.7)
        self.wait(1.6)

        # the part 5 mm off centre: the slow stage pushes it to the middle
        ro = LOG["tolerant_off5"]["result"]
        tp, push = series("tolerant_off5", "push")
        ax3 = Axes(x_range=[0, T_MAX * 1000, 100], y_range=[0, 6, 2], x_length=5.4, y_length=2.7,
                   axis_config=dict(color=LOAD, stroke_width=2, include_tip=False, font_size=20),
                   x_axis_config=dict(numbers_to_include=[0, 100, 200, 300, 400]),
                   y_axis_config=dict(numbers_to_include=[0, 2, 4, 6])).move_to(ax2)
        for lab in ax3.get_x_axis().numbers.submobjects + ax3.get_y_axis().numbers.submobjects:
            lab.set_color(LOAD)
        yl3 = tex(r"s" + r"\;[\mathrm{mm}]", font_size=22, color=LOAD).next_to(ax3.y_axis, UP, buff=0.1)
        h3 = VGroup(Tex(r"part placed 5\,mm off centre", color=ACC, font_size=28),
                    Tex(r"how far the jaws push it", color=LOAD, font_size=21)).arrange(DOWN, buff=0.08).move_to(h2)
        self.remove(c2)
        self.play(FadeOut(VGroup(p2, l2, set_lab, s2, ratio)), ReplacementTransform(g2[0], ax3),
                  ReplacementTransform(g2[2], yl3), ReplacementTransform(h2, h3), run_time=0.9)
        tt.set_value(0)
        c3 = curve(ax3, tp, push, ACC)
        self.add(c3)
        self.play(tt.animate.set_value(T_MAX * 1000), run_time=4.0, rate_func=linear)
        l3 = tex(rf"{ro['part_push_mm']:.1f}{MM}", font_size=30, color=ACC).next_to(ax3.c2p(T_MAX * 1000, ro['part_push_mm']), UP, buff=0.1).shift(0.5 * LEFT)
        note = Tex(rf"centred gently at ${n(rt['v_touch'] * 1000)}{MMS}$, peak still {ro['f_peak']:.1f}\,N", color=INK,
                   font_size=24).to_edge(DOWN, buff=0.3)
        self.play(Write(l3), FadeIn(note, shift=0.1 * UP), run_time=0.8)
        self.wait(2.0)

