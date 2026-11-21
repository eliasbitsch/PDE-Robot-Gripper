"""Drive model: Nanotec ST4118S1404-B on a GP42 1:7.07, pinion d = 20, two racks.

Torque-speed curve from the electrical datasheet values (R, L, holding torque, 50 pole pairs) at the controller
supply voltage; the Nanotec curve at 24 V should be laid over it once downloaded (product page, not in the PDF).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "02_CAD_Body"))
import gripper_params as GP  # noqa: E402

# ST4118S1404-B datasheet (vendor/nanotec_st4118s1404b.pdf)
R_PH, L_PH, I_RATED = 2.0, 3.0e-3, 1.4          # ohm, H, A per phase
T_DETENT = 6.0e-3                                 # N*m
J_ROTOR = 3.8e-6                                  # kg*m^2
POLE_PAIRS = 50                                   # 1.8 deg step
K_M = GP.MOTOR_T_HOLD / I_RATED                   # N*m/A = V*s/rad
# Not on the datasheets, estimated: GP42 1-stage input side, BKE armature disc (~10 g, d 28), bevel pair + pinion shaft
J_GEAR_IN, J_BRAKE = 1.5e-6, 1.0e-6
J_OUT = 2 * 0.5 * 0.028 * 0.012 ** 2 + 0.5 * 0.035 * 0.005 ** 2   # at the pinion: m 1.5 bevel gears (CAD 26 + 29 g) + shaft
# moving mass per jaw: slide, holder, jaw, ball carriage, rack (bom.csv)
M_JAW = (73 + 29 + 31 + 51 + 65) / 1000.0
F_FRICTION = 2.5                                  # N per jaw: carriage seals + brush seal along the slot (estimate)
ETA = GP.GEAR_ETA * 0.92                          # planetary x rack-pinion (cad/params.py ETA_GETRIEBE)

R_P = GP.PINION_D / 2 / 1000                      # m
I_G = GP.GEAR_I
J_MOTOR_SIDE = J_ROTOR + J_GEAR_IN + J_BRAKE + J_OUT / I_G ** 2
# kinetic energy 0.5 J w^2 with w = d_dot * i / r: as a linear mass on the jaw coordinate (both jaws together)
M_REFLECTED = J_MOTOR_SIDE * (I_G / R_P) ** 2
ARMATURE_PER_JAW = M_REFLECTED / 2                # MuJoCo armature on each jaw slide (both move with d)


def torque_avail(w, v_supply=24.0):
    """Peak motor torque at shaft speed w [rad/s]: current limited by the drive, or by the supply against
    back-EMF and winding impedance (phasor sum, current in phase with the back-EMF): V^2 = (IR + E)^2 + (I w_e L)^2."""
    e = K_M * w
    x = POLE_PAIRS * w * L_PH
    a, b, c = R_PH ** 2 + x ** 2, 2 * R_PH * e, e ** 2 - v_supply ** 2
    disc = b * b - 4 * a * c
    i = I_RATED if c < 0 and disc > 0 and (-b + math.sqrt(disc)) / (2 * a) > I_RATED else \
        max(0.0, (-b + math.sqrt(max(disc, 0.0))) / (2 * a))
    return max(0.0, K_M * min(i, I_RATED) - T_DETENT)


def load_torque():
    """Motor-side torque to push both jaws against friction."""
    return 2 * F_FRICTION * R_P / (I_G * ETA) + T_DETENT


def jaw_speed(w):
    return w / I_G * R_P                          # m/s per jaw


def min_time(travel, v_contact=0.0, usable=0.7, v_supply=24.0, dt=1e-5):
    """Fastest move of `travel` [m] per jaw with `usable` x the pull-out torque (closed-loop margin), arriving at
    v_contact. Forward integration (accelerate) meets backward integration (brake), both on the torque curve."""
    j = J_MOTOR_SIDE + 2 * M_JAW * (R_P / I_G) ** 2
    th_end = travel / R_P * I_G
    def integrate(sign_load, w0):
        th, w, out = 0.0, w0, [(0.0, w0)]
        while th < th_end and w < 2000:
            acc = (usable * torque_avail(w, v_supply) - sign_load * load_torque()) / j
            if acc <= 1e-3:
                break
            w += acc * dt
            th += w * dt
            out.append((th, w))
        return out
    fwd = integrate(+1, 0.0)
    bwd = integrate(-1, v_contact / R_P * I_G)       # braking: friction helps
    import numpy as np
    thf, wf = np.array(fwd).T
    thb, wb = np.array(bwd).T
    grid = np.linspace(0, th_end, 4000)
    w_up = np.interp(grid, thf, wf, right=wf[-1])
    w_dn = np.interp(th_end - grid, thb, wb, right=wb[-1])
    w = np.maximum(np.minimum(w_up, w_dn), 1e-6)
    t = np.concatenate([[0.0], np.cumsum(np.diff(grid) / (0.5 * (w[1:] + w[:-1])))])
    return dict(t=float(t[-1]), w_max=float(w.max()), v_max=float(jaw_speed(w.max())), profile=(t, grid, w))


if __name__ == "__main__":
    print(f"reflected inertia {M_REFLECTED:.2f} kg on the jaw coordinate vs moving jaws {2 * M_JAW:.2f} kg")
    for rpm in (0, 300, 600, 900, 1200, 1500):
        w = rpm * 2 * math.pi / 60
        print(f"  {rpm:5d} rpm  {torque_avail(w):.3f} N*m  jaw {jaw_speed(w) * 1000:5.1f} mm/s")
    r = min_time((GP.D_OPEN - GP.D_GRIP) / 1000)
    print(f"min time 8 mm to standstill: {r['t'] * 1000:.0f} ms, v_max {r['v_max'] * 1000:.0f} mm/s")
