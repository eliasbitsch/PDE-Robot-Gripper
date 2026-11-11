"""Hand-calculation reference for the jaw FEM (cross-check only; the graded proof is Viktoriia's V4).

Cantilever clamped at the holder edge z = -47, grip force spread uniformly over the pad pocket
(z = -60 ... -95). Stepped section: full 25 x 8 above the pocket, 25 x 5 web + two 1 x 3 rims in the pocket.
Holes are neglected. Euler-Bernoulli plus a Timoshenko shear term (kappa = 5/6).
"""
import numpy as np

import fem_jaw as F

B, T = 25.0, 8.0
POCKET_DEPTH, RIM = 3.0, 1.0
G = F.E / (2 * (1 + F.NU))
KAPPA = 5 / 6


def section(z):
    """Second moment I (bending about y, fibre coordinate x), area A and outer-face distance c."""
    if F.Z_LOAD[0] <= z <= F.Z_LOAD[1]:
        parts = [(B, T - POCKET_DEPTH, POCKET_DEPTH + (T - POCKET_DEPTH) / 2),   # web, x measured from the inner face
                 (2 * RIM, POCKET_DEPTH, POCKET_DEPTH / 2)]
    else:
        parts = [(B, T, T / 2)]
    A = sum(w * h for w, h, _ in parts)
    xc = sum(w * h * x for w, h, x in parts) / A
    I = sum(w * h ** 3 / 12 + w * h * (x - xc) ** 2 for w, h, x in parts)
    return I, A, T - xc


def reference(force=F.P.F_JAW_MAX, n=2000):
    z0 = F.Z_HOLDER
    L = z0 - F.Z_LOAD[0]                          # 48 mm from the clamp to the pocket bottom
    s = np.linspace(0, L, n)                      # distance below the clamp
    a, b = z0 - F.Z_LOAD[1], z0 - F.Z_LOAD[0]     # loaded span 13 ... 48
    q = force / (b - a)
    # Shear force and moment at s from the load beyond s.
    V = q * np.clip(b - np.maximum(s, a), 0, None)
    lo = np.maximum(s, a)
    M = np.where(s < b, q * ((b - s) ** 2 - np.clip(lo - s, 0, None) ** 2) / 2, 0.0)
    I = np.array([section(z0 - x)[0] for x in s])
    A = np.array([section(z0 - x)[1] for x in s])
    kappa_b = M / (F.E * I)
    slope = np.concatenate([[0], np.cumsum((kappa_b[1:] + kappa_b[:-1]) / 2 * np.diff(s))])
    w_b = np.concatenate([[0], np.cumsum((slope[1:] + slope[:-1]) / 2 * np.diff(s))])
    gam = V / (KAPPA * G * A)
    w_s = np.concatenate([[0], np.cumsum((gam[1:] + gam[:-1]) / 2 * np.diff(s))])

    s_load = z0 - 0.5 * (F.Z_LOAD[0] + F.Z_LOAD[1])     # 30.5 mm, load centre
    s_chk = z0 - F.Z_CHECK
    I_chk, _, c_chk = section(F.Z_CHECK)
    M_chk = np.interp(s_chk, s, M)
    return dict(
        force=force,
        w_bending=float(np.interp(s_load, s, w_b)),
        w_total=float(np.interp(s_load, s, w_b + w_s)),
        sigma_check=float(M_chk * c_chk / I_chk),
        sigma_clamp=float(M[0] * section(z0)[2] / section(z0)[0]),
        curve=(s, w_b + w_s),
    )


if __name__ == "__main__":
    r = reference()
    print(f"F = {r['force']:.1f} N: w(load) {r['w_bending'] * 1000:.2f} um (bending), {r['w_total'] * 1000:.2f} um (+shear), "
          f"sigma_check {r['sigma_check']:.2f} MPa, sigma_clamp {r['sigma_clamp']:.2f} MPa")
