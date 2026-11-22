"""Explainer scene 3: grip force per jaw, built term by term (Manim CE).

manim -r 854,480 --fps 30 grip_force.py GripForce      # preview
manim -r 1920,1080 --fps 30 grip_force.py GripForce    # final
Numbers: V2 §§ 1, 2a and 04_Simulation/out/grip_results.json (setpoint 44.5 N per jaw).
"""
from manim import (DOWN, LEFT, RIGHT, UP, ORIGIN, Arrow, Create, FadeIn, FadeOut, GrowArrow, MathTex, RoundedRectangle,
                   Rectangle, Scene, SurroundingRectangle, Tex, TransformMatchingTex, VGroup, Write, config, Indicate,
                   LaggedStart, ReplacementTransform)

import json
import os

from style import ACC, INK, LOAD, MS2, MU, PAD, PART, SAF, SIM_OUT, KG, P, n, tex

M, G, A, A0, S, MU_V = P.B_MASSE, P.G, P.A_PATH_REAL, P.A_BAHN, P.S_SICHERHEIT, P.MU_OILY
F = P.F_GRIP_SET
assert abs(F - S * M * (G + A) / (P.N_BACKEN * MU_V)) < 1e-9



class GripForce(Scene):
    def construct(self):
        cap = Tex(r"Grip force per jaw", color=INK, font_size=30).to_corner(UP + LEFT, buff=0.45)
        self.play(FadeIn(cap, shift=0.1 * RIGHT), run_time=0.6)

        # free body: part B between the two pads; friction holds weight plus inertia
        part = RoundedRectangle(width=1.7, height=2.0, corner_radius=0.12, stroke_color=INK, stroke_width=3,
                                fill_color=PART, fill_opacity=1)
        pads = VGroup(*[Rectangle(width=0.2, height=1.5, stroke_width=0, fill_color=PAD, fill_opacity=1)
                        .next_to(part, s, buff=0.02) for s in (LEFT, RIGHT)])
        body = VGroup(part, pads).shift(4.0 * LEFT + 0.2 * DOWN)
        fn = VGroup(Arrow(pads[0].get_left() + 1.0 * LEFT, pads[0].get_left(), buff=0, color=INK, stroke_width=5),
                    Arrow(pads[1].get_right() + 1.0 * RIGHT, pads[1].get_right(), buff=0, color=INK, stroke_width=5))
        # friction acts on the part at its contact faces: drawn just inside the part, along the pads
        fr = VGroup(*[Arrow(p.get_center() + s * 0.32 + 0.6 * DOWN, p.get_center() + s * 0.32 + 0.6 * UP, buff=0,
                            color=MU, stroke_width=5) for p, s in zip(pads, (RIGHT, LEFT))])
        ld = Arrow(part.get_center() + 0.3 * UP, part.get_center() + 1.7 * DOWN, buff=0, color=LOAD, stroke_width=6)
        l_fn = tex(r"F", font_size=34).next_to(fn[0], UP, buff=0.08)
        l_fr = tex(r"\mu F", font_size=34, color=MU).next_to(part, UP, buff=0.12).align_to(fr[1], RIGHT)
        l_ld = tex(r"m\,(g+a)", font_size=34, color=LOAD).next_to(ld, DOWN, buff=0.08)
        self.play(FadeIn(body, shift=0.2 * UP), run_time=0.7)
        self.play(LaggedStart(GrowArrow(fn[0]), GrowArrow(fn[1]), lag_ratio=0.2), FadeIn(l_fn), run_time=0.7)
        self.play(GrowArrow(ld), FadeIn(l_ld), run_time=0.6)
        self.play(LaggedStart(GrowArrow(fr[0]), GrowArrow(fr[1]), lag_ratio=0.2), FadeIn(l_fr), run_time=0.7)

        # balance with safety, then solved for F
        x0 = 1.6 * RIGHT + 1.3 * UP
        e1 = tex(r"2", r"\mu", r"F", r"\ge", r"S", r"\,m\,(g+a)", font_size=46).move_to(x0)
        e1[1].set_color(MU), e1[4].set_color(SAF)
        why = Tex(r"two pads hold weight and inertia, with safety $S$", color=LOAD, font_size=26).next_to(e1, DOWN, buff=0.3)
        self.play(Write(e1), FadeIn(why), run_time=1.4)
        self.wait(0.6)
        e2 = tex(r"F", r"=", r"{", r"S", r"\,m\,(g+a)", r"\over", r"2", r"\mu", r"}", font_size=52).move_to(x0)
        e2[3].set_color(SAF), e2[7].set_color(MU)
        self.play(TransformMatchingTex(e1, e2), FadeOut(why), run_time=1.2)
        self.wait(0.4)

        # the inputs, each with why it is that value
        rows = VGroup(
            Tex(rf"$\mu = {n(MU_V)}$", r"\; pad on an oily part (worst case)", color=INK, font_size=28),
            Tex(rf"$S = {n(S)}$", r"\; quasi-static safety (VDI 2860)", color=INK, font_size=28),
            Tex(rf"$m = {M:.3f}{KG}$", r"\; part B", color=INK, font_size=28),
            Tex(rf"$a = {n(A)}{MS2}$", rf"\; real robot path (straight line: ${n(A0)}{MS2}$)", color=INK, font_size=28),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.16).next_to(e2, DOWN, buff=0.55).align_to(e2, LEFT).shift(0.6 * LEFT)
        rows[0][0].set_color(MU), rows[1][0].set_color(SAF), rows[3][0].set_color(ACC)
        for r in rows:
            r[1].set_color(LOAD)
        self.play(LaggedStart(*[FadeIn(r, shift=0.15 * RIGHT) for r in rows], lag_ratio=0.35), run_time=2.0)
        self.play(Indicate(rows[3][0], color=ACC, scale_factor=1.12), run_time=0.8)

        # numbers in, result out
        e3 = tex(r"F", r"=", r"{", n(S), rf"\cdot {M:.3f}{KG}\cdot\left({n(G)}{MS2}+{n(A)}{MS2}\right)", r"\over",
                 str(P.N_BACKEN), rf"\cdot {n(MU_V)}", r"}", font_size=42).move_to(x0)
        e3[3].set_color(SAF), e3[7].set_color(MU)
        self.play(TransformMatchingTex(e2, e3), run_time=1.2)
        self.wait(0.5)
        e4 = tex(r"F", r"=", f"{F:.1f}", r"\,\mathrm{N}", font_size=64).move_to(x0)
        e4[2:].set_color(ACC)
        box = SurroundingRectangle(e4, color=ACC, buff=0.2, corner_radius=0.08, stroke_width=3)
        per = Tex(r"per jaw, the setpoint of the current limit", color=LOAD, font_size=26).next_to(box, DOWN, buff=0.22)
        self.play(ReplacementTransform(e3, e4), run_time=0.9)
        self.play(Create(box), FadeIn(per), run_time=0.7)
        l_res = tex(f"{F:.1f}" + r"\,\mathrm{N}", font_size=34, color=ACC).next_to(fn[0], UP, buff=0.08)
        self.play(ReplacementTransform(l_fn, l_res), Indicate(fn, color=ACC, scale_factor=1.05), run_time=0.9)
        # the dynamic check lives in the Monte Carlo study (04_Simulation/robustness.py); the film only states its result
        rb = [r for r in json.load(open(os.path.join(SIM_OUT, "robustness.json")))["results"] if r["study"] == "spec"]
        u_max = max(r["util_max"] for r in rb)
        note = Tex(rf"\mbox{{{len(rb):,} simulated grips with scattered inputs: all held, peak friction use "
                   rf"{u_max:.2f} $<$ 1}}", color=LOAD, font_size=24).to_edge(DOWN, buff=0.45)
        self.play(FadeIn(note, shift=0.1 * UP), run_time=0.7)
        self.wait(2.0)
