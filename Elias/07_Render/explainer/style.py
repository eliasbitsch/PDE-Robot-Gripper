"""Shared look of the explainer scenes: theme colours, number and unit formatting, design inputs."""
import os
import sys

from manim import MathTex, config

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_OUT = os.path.join(HERE, "..", "..", "04_Simulation", "out")
sys.path.insert(0, os.path.join(HERE, "..", "..", "02_CAD_Body"))
import gripper_params as P  # noqa: E402,F401

# THEME=dark (default, matches the films) or light (report figures)
if os.environ.get("THEME", "dark") == "light":
    BG, INK, MU, SAF, ACC, LOAD, PART, PAD = ("#FAFAF8", "#1F2328", "#1F6FB2", "#C2410C", "#0F8A7A", "#6B7280",
                                              "#FFFFFF", "#1F2328")
else:
    BG, INK, MU, SAF, ACC, LOAD, PART, PAD = ("#0E1013", "#E8E6E1", "#5AA9F0", "#F08A4B", "#3CC9B4", "#8E949C",
                                              "#191C21", "#5A606B")
config.background_color = BG

# units in LaTeX, fractions instead of slashes (math mode)
MS2 = r"\,\dfrac{\mathrm{m}}{\mathrm{s}^2}"     # dfrac: full size also inside a fraction or in text lines
MMS = r"\,\dfrac{\mathrm{mm}}{\mathrm{s}}"
KG, N, MM, MS = r"\,\mathrm{kg}", r"\,\mathrm{N}", r"\,\mathrm{mm}", r"\,\mathrm{ms}"


def n(v, d=2):
    return f"{v:.{d}f}".rstrip("0").rstrip(".")


def tex(*parts, **kw):
    kw.setdefault("color", INK)
    return MathTex(*parts, **kw)
