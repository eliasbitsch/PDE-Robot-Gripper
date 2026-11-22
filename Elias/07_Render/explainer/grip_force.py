"""Explainer scene 3: grip force per jaw, built term by term (Manim CE).

manim -r 854,480 --fps 30 grip_force.py GripForce      # preview
manim -r 1920,1080 --fps 30 grip_force.py GripForce    # final
Numbers: V2 §§ 1, 2a and 04_Simulation/out/grip_results.json (setpoint 44.5 N per jaw).
"""
from manim import (DOWN, LEFT, RIGHT, UP, ORIGIN, Arrow, Create, FadeIn, FadeOut, GrowArrow, MathTex, RoundedRectangle,
                   Rectangle, Scene, SurroundingRectangle, Tex, TransformMatchingTex, VGroup, Write, config, Indicate,
                   LaggedStart, ReplacementTransform)

config.background_color = "#FAFAF8"
INK, MU, SAF, ACC, LOAD = "#1F2328", "#1F6FB2", "#C2410C", "#0F8A7A", "#6B7280"
M, G, A, S, MU_V = 0.602, 9.81, 4.96, 2.0, 0.2
F = S * M * (G + A) / (2 * MU_V)


def tex(*parts, **kw):
    kw.setdefault("color", INK)
    return MathTex(*parts, **kw)


class GripForce(Scene):
    def construct(self):
        cap = Tex(r"Grip force per jaw", color=INK, font_size=30).to_corner(UP + LEFT, buff=0.45)
        self.play(FadeIn(cap, shift=0.1 * RIGHT), run_time=0.6)

        # free body: part B between the two pads; friction holds weight plus inertia
        part = RoundedRectangle(width=1.7, height=2.0, corner_radius=0.12, stroke_color=INK, stroke_width=3,
                                fill_color="#FFFFFF", fill_opacity=1)
        pads = VGroup(*[Rectangle(width=0.2, height=1.5, stroke_width=0, fill_color=INK, fill_opacity=1)
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
            Tex(r"$\mu = 0.2$", r"\; pad on an oily part (worst case)", color=INK, font_size=28),
            Tex(r"$S = 2$", r"\; quasi-static safety (VDI 2860)", color=INK, font_size=28),
            Tex(r"$m = 0.602$ kg", r"\; part B", color=INK, font_size=28),
            Tex(r"$a = 4.96$ m/s$^2$", r"\; real robot path (straight line: 2.05)", color=INK, font_size=28),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.22).next_to(e2, DOWN, buff=0.55).align_to(e2, LEFT).shift(0.6 * LEFT)
        rows[0][0].set_color(MU), rows[1][0].set_color(SAF), rows[3][0].set_color(ACC)
        for r in rows:
            r[1].set_color(LOAD)
        self.play(LaggedStart(*[FadeIn(r, shift=0.15 * RIGHT) for r in rows], lag_ratio=0.35), run_time=2.0)
        self.play(Indicate(rows[3][0], color=ACC, scale_factor=1.12), run_time=0.8)

        # numbers in, result out
        e3 = tex(r"F", r"=", r"{", r"2", r"\cdot 0.602\,(9.81+4.96)", r"\over", r"2", r"\cdot 0.2", r"}",
                 font_size=52).move_to(x0)
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
        self.wait(1.5)
