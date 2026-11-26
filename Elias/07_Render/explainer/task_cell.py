"""Explainer scene 1: the task and the cell, as a side elevation to scale (Manim CE).

manim -r 854,480 --fps 30 task_cell.py TaskCell      # preview
manim -r 1920,1080 --fps 30 task_cell.py TaskCell    # final
Numbers: 06_Cell/cell_params.py (layout, IK), 02_CAD_Body/gripper_params.py (part mass, takt),
04_Simulation/out/e4_cycle.json (cycle segments).
"""
import json
import math
import os
import sys

import numpy as np
from manim import (DOWN, LEFT, RIGHT, UP, Arc, Create, DashedVMobject, Dot, FadeIn, FadeOut, GrowFromEdge, Line,
                   LaggedStart, MoveAlongPath, Polygon, Rectangle, Scene, Tex, VGroup, Write, rate_functions)

from style import ACC, INK, LOAD, MU, PART, SAF, SIM_OUT, KG, P, tex

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "06_Cell"))
import cell_params as CP  # noqa: E402

CYC = json.load(open(os.path.join(SIM_OUT, "e4_cycle.json")))
SEG = CYC["chain_2"]["segments"]
T_CYC, TAKT = CYC["cycle_time"], P.TAKTZEIT
assert abs(TAKT - CYC["budget"]) < 1e-9

FLANGE_Z = 83.0 - P.ISO_SPIGOT_H                       # flange face in the gripper frame (CAD bounds, as hero_v1.py)
S = 1 / 430.0                                          # scene units per mm
X0, Z0 = -0.3, -2.45                                   # where cell x = 0, z = 0 lands on screen


def pt(x_mm, z_mm):
    return np.array([X0 + x_mm * S, Z0 + z_mm * S, 0.0])


def dim_v(x_mm, z0, z1, label, side=RIGHT, color=INK):
    """vertical dimension line with end ticks and a label"""
    a, b = pt(x_mm, z0), pt(x_mm, z1)
    ln = Line(a, b, color=color, stroke_width=2)
    ticks = VGroup(*[Line(p + 0.08 * LEFT, p + 0.08 * RIGHT, color=color, stroke_width=2) for p in (a, b)])
    lab = tex(label, font_size=26, color=color).next_to(ln, side, buff=0.1)
    return VGroup(ln, ticks, lab)


def dim_h(x0, x1, z_mm, label, color=INK):
    a, b = pt(x0, z_mm), pt(x1, z_mm)
    ln = Line(a, b, color=color, stroke_width=2)
    ticks = VGroup(*[Line(p + 0.08 * DOWN, p + 0.08 * UP, color=color, stroke_width=2) for p in (a, b)])
    lab = tex(label, font_size=26, color=color).next_to(ln, UP, buff=0.08)
    return VGroup(ln, ticks, lab)


def robot_at(q):
    """planar side view of the IRB 1300 at joint angles q (axis 1 = 0): base, upper arm, forearm, wrist, flange"""
    bx, bz = CP.BASE[0], CP.BASE[2]
    a2, a3, a5 = (math.radians(v) for v in (q[1], q[2], q[4]))
    sh = (bx + CP.A2_X, bz + CP.A2_Z)
    el = (sh[0] + CP.L2 * math.sin(a2), sh[1] + CP.L2 * math.cos(a2))
    th = a2 + a3                                       # forearm: horizontal at home, turned down by a2 + a3

    def rot(x, z, t):
        return (x * math.cos(t) + z * math.sin(t), -x * math.sin(t) + z * math.cos(t))
    dx, dz = rot(CP.L3X, CP.C4, th)
    wr = (el[0] + dx, el[1] + dz)
    fx, fz = rot(CP.L_FL, 0.0, th + a5)
    fl = (wr[0] + fx, wr[1] + fz)
    arm = VGroup(Line(pt(*sh), pt(*el), color=INK, stroke_width=9), Line(pt(*el), pt(*wr), color=INK, stroke_width=7),
                 Line(pt(*wr), pt(*fl), color=INK, stroke_width=5))
    joints = VGroup(*[Dot(pt(*p), radius=0.06, color=INK) for p in (sh, el, wr)])
    # base and the rotating link up to the shoulder axis, which sits A2_X in front of axis 1
    base = Polygon(pt(bx - 110, bz), pt(bx + 110, bz), pt(bx + 95, bz + 260), pt(sh[0] + 75, sh[1] - 40),
                   pt(sh[0] + 30, sh[1] + 70), pt(bx - 60, sh[1] + 40), pt(bx - 95, bz + 260),
                   color=INK, stroke_width=2, fill_color=PART, fill_opacity=1)
    gw = 100                                           # gripper body and jaws, below the flange
    grip = VGroup(Rectangle(width=gw * S, height=110 * S, color=ACC, stroke_width=2, fill_color=PART, fill_opacity=1)
                  .move_to(pt(fl[0], fl[1] - 55)),
                  *[Line(pt(fl[0] + s_ * 52, fl[1] - 110), pt(fl[0] + s_ * 52, fl[1] - (FLANGE_Z - P.JAW_Z_BOTTOM)),
                         color=ACC, stroke_width=3) for s_ in (-1, 1)])
    return VGroup(base, arm, joints, grip), fl


class TaskCell(Scene):
    def construct(self):
        cap = Tex(r"The task", color=INK, font_size=30).to_corner(UP + LEFT, buff=0.45)
        self.play(FadeIn(cap, shift=0.1 * RIGHT), run_time=0.5)

        # the cell to scale: floor, pedestal, buffer belt, workpiece carrier on its chain, fence
        floor = Line(pt(-1500, 0), pt(2250, 0), color=LOAD, stroke_width=2)
        ped = Rectangle(width=340 * S, height=CP.BASE[2] * S, color=INK, stroke_width=2, fill_color=PART,
                        fill_opacity=1).move_to(pt(CP.BASE[0], CP.BASE[2] / 2))
        belt = VGroup(Rectangle(width=(CP.BELT_X[1] - CP.BELT_X[0]) * S, height=40 * S, color=INK, stroke_width=2,
                                fill_color=PART, fill_opacity=1)
                      .move_to(pt((CP.BELT_X[0] + CP.BELT_X[1]) / 2, CP.H_PICK - 20)),
                      *[Line(pt(x, 0), pt(x, CP.H_PICK - 40), color=LOAD, stroke_width=2)
                        for x in (CP.BELT_X[0] + 60, CP.BELT_X[1] - 60)])
        car_w = CP.CARRIER[0]
        carrier = VGroup(Rectangle(width=car_w * S, height=CP.CARRIER[2] * S, color=MU, stroke_width=2, fill_color=PART,
                                   fill_opacity=1).move_to(pt(CP.X_PLACE, CP.H_PLACE - CP.CARRIER[2] / 2)),
                         Line(pt(CP.CHAIN_X[0], CP.H_PLACE - 45), pt(CP.CHAIN_X[1], CP.H_PLACE - 45), color=LOAD,
                              stroke_width=4),
                         Line(pt(CP.X_PLACE, 0), pt(CP.X_PLACE, CP.H_PLACE - 60), color=LOAD, stroke_width=2))
        fence = DashedVMobject(Polygon(pt(CP.X_FENCE[0], 0), pt(CP.X_FENCE[0], CP.H_FENCE), pt(CP.X_FENCE[1], CP.H_FENCE),
                                       pt(CP.X_FENCE[1], 0), color=SAF, stroke_width=2), num_dashes=60)
        self.play(Create(floor), run_time=0.5)
        self.play(GrowFromEdge(ped, DOWN), FadeIn(belt, shift=0.1 * UP), FadeIn(carrier, shift=0.1 * UP), run_time=0.9)
        zp = 0.5 * (P.PAD_Z[0] + P.PAD_Z[1])
        tcp_ab = zp - (P.PART_OFFSET[2] - 40.0)        # TCP above the part's bottom face (46 mm)
        q_pick, bad = CP.ik_tool_down((CP.X_PICK, 0.0, CP.H_PICK + tcp_ab + (FLANGE_Z - zp) + CP.L_FL))
        assert not bad, bad
        rob, fl = robot_at(q_pick)
        self.play(FadeIn(rob, shift=0.1 * UP), run_time=0.7)
        part = Rectangle(width=95 * S, height=80 * S, color=INK, stroke_width=2, fill_color=INK, fill_opacity=0.9) \
            .move_to(pt(CP.X_PICK, CP.H_PICK + 40))
        self.play(FadeIn(part, scale=0.6), run_time=0.4)

        # dimensions, one after the other
        d_belt = dim_v(CP.BELT_X[1] - 80, 0, CP.H_PICK, rf"{CP.H_PICK:.0f}\,\mathrm{{mm}}", RIGHT)
        d_car = dim_v(CP.X_PLACE + car_w / 2 + 90, 0, CP.H_PLACE, rf"{CP.H_PLACE:.0f}\,\mathrm{{mm}}", RIGHT, MU)
        d_ped = dim_v(CP.BASE[0] + 230, 0, CP.BASE[2], rf"{CP.BASE[2]:.0f}", RIGHT, LOAD)
        d_dx = dim_h(CP.X_PLACE, CP.X_PICK, CP.H_FENCE + 230, rf"{CP.X_PICK - CP.X_PLACE:.0f}\,\mathrm{{mm}}", ACC)
        d_fence = dim_v(CP.X_FENCE[0] - 110, 0, CP.H_FENCE, rf"{CP.H_FENCE:.0f}\,\mathrm{{mm}}", LEFT, SAF)
        self.play(LaggedStart(*[Write(d) for d in (d_belt, d_car, d_ped)], lag_ratio=0.45), run_time=1.8)
        self.play(Create(fence), Write(d_fence), run_time=1.0)
        self.play(Write(d_dx), run_time=0.7)

        # what moves: part B from the belt to the carrier
        lab = VGroup(Tex(r"part B", color=INK, font_size=28),
                     tex(rf"m = {P.B_MASSE:.3f}{KG}", font_size=28)).arrange(DOWN, buff=0.06, aligned_edge=LEFT)
        lab.move_to(pt(CP.X_FENCE[1] + 480, CP.H_PICK + 600))     # outside the fence, right of the part
        pointer = Line(lab.get_left() + 0.08 * LEFT + 0.1 * DOWN, part.get_right() + 0.03 * RIGHT, color=INK, stroke_width=1.5)
        self.play(FadeIn(lab, shift=0.1 * DOWN), Create(pointer), run_time=0.7)
        start, end = pt(CP.X_PICK, CP.H_PICK + 40), pt(CP.X_PLACE, CP.H_PLACE + 40)
        path = Arc(start_angle=math.pi / 5, angle=math.pi * 3 / 5, radius=1.0)   # flat, like the transfer
        path.put_start_and_end_on(start, end)
        trail = DashedVMobject(path.copy().set_color(ACC).set_stroke(width=2), num_dashes=40)
        self.play(Create(trail), FadeOut(pointer), lab.animate.set_opacity(0.4), run_time=0.8)
        self.play(MoveAlongPath(part, path), rate_func=rate_functions.ease_in_out_cubic, run_time=1.5)
        enc = Tex(rf"enclosure ${2 * CP.Y_FENCE:.0f} \times {CP.H_FENCE:.0f}\,\mathrm{{mm}}$", color=SAF, font_size=24) \
            .next_to(pt(CP.X_FENCE[0], CP.H_FENCE), UP + RIGHT, buff=0.1)
        self.play(FadeIn(enc), run_time=0.5)

        # the takt: one cycle against the 5 s budget, segments from the simulated cycle
        W = 9.0
        x_l = -W / 2
        y = -3.35
        tick = lambda t: x_l + W * t / TAKT
        bar = Rectangle(width=W, height=0.26, color=LOAD, stroke_width=1.5).move_to([0, y, 0])
        cols = {True: ACC, False: LOAD}
        blocks = VGroup()
        for s in SEG:
            c = SAF if s["segment"] in ("close on the belt", "open") else cols[s["loaded"]]
            r = Rectangle(width=W * s["duration"] / TAKT, height=0.26, stroke_width=0, fill_color=c, fill_opacity=0.85)
            r.move_to([tick(s["t_start"] + s["duration"] / 2), y, 0])
            blocks.add(r)
        t_lab = tex(r"0", font_size=22, color=LOAD).next_to([tick(0), y - 0.13, 0], DOWN, buff=0.06)
        e_lab = tex(rf"{TAKT:.0f}\,\mathrm{{s}}\ \text{{takt}}", font_size=22, color=INK).next_to([tick(TAKT), y - 0.13, 0], DOWN, buff=0.06)
        self.play(FadeIn(bar), FadeIn(t_lab), FadeIn(e_lab), run_time=0.4)
        self.play(LaggedStart(*[GrowFromEdge(b, LEFT) for b in blocks], lag_ratio=0.85), run_time=2.2,
                  rate_func=rate_functions.linear)
        mark = Line([tick(T_CYC), y - 0.22, 0], [tick(T_CYC), y + 0.22, 0], color=INK, stroke_width=3)
        c_lab = tex(rf"{T_CYC:.2f}\,\mathrm{{s}}", font_size=24).next_to(mark, UP, buff=0.06)
        res = tex(rf"{TAKT - T_CYC:.2f}\,\mathrm{{s}}\ \text{{reserve}}", font_size=22, color=ACC).next_to(bar, RIGHT, buff=0.2)
        legend = VGroup(*[VGroup(Rectangle(width=0.22, height=0.14, stroke_width=0, fill_color=c, fill_opacity=0.85),
                                 Tex(t, color=LOAD, font_size=20)).arrange(RIGHT, buff=0.08)
                          for c, t in ((SAF, "grip / release"), (ACC, "with part"), (LOAD, "empty"))]) \
            .arrange(RIGHT, buff=0.35).next_to(bar, LEFT, buff=0.3)
        legend.scale(0.95).next_to(bar, UP, buff=0.12).align_to(bar, LEFT)
        self.play(Create(mark), FadeIn(c_lab), FadeIn(legend), run_time=0.6)
        self.play(FadeIn(res, shift=0.05 * UP), run_time=0.5)
        self.wait(2.0)
