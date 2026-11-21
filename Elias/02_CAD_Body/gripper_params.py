"""Gripper geometry. Part, cell and force values come from cad/params.py."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cad"))
from params import *  # noqa: E402,F401,F403

# Drive variant: "electric" (stepper + brake + gearbox + bevel pair, tag v1-electric) or "pneumatic" (compact cylinder
# + safety spring, 21 Nov 2026). Set GRIPPER_DRIVE=pneumatic to build the second version; outputs go to out_pneumatic.
DRIVE = os.environ.get("GRIPPER_DRIVE", "electric")

# Catalogue data, sources in vendor/SOURCES.md. Lengths in mm, torques in N*m.
# Rack m=1 steel (Maedler; profile per DIN 867): cross-section and pitch line height from the base
RACK_W, RACK_H, RACK_PITCH_LINE = 10.0, 12.0, 11.0
PINION_D, PINION_B = MODUL * Z_RITZEL, 10.0

# Bosch Rexroth miniature ball rail size 12 (S0 >= 5 at the 44 N setpoint; size 9 gave 2.5 ... 3.7) (catalogue R999001206, 2023-08, p. 19/21)
# carriage SLS long, R0444 213 01 (size 12, accuracy H, preload); rail R0445 203 31
RAIL_W, RAIL_H, RAIL_PITCH = 12.0, 7.8, 25.0
RAIL_HOLE_D, RAIL_CBORE_D, RAIL_SCREW = 3.5, 6.0, "M3"
RAIL_CBORE_DEPTH = 4.0                       # M3 head sinks in, 4.2 mm thread in the plate
CARRIAGE_W, CARRIAGE_L, CARRIAGE_L_BODY = 27.0, 47.5, 34.5
CARRIAGE_H, CARRIAGE_GAP = 13.0, 3.3          # rail base to carriage top, rail base to carriage underside
CARRIAGE_HOLES_E1, CARRIAGE_HOLES_E2, CARRIAGE_THREAD = 20.0, 20.0, "M3x3.5"   # E1 along the rail
CARRIAGE_C0, CARRIAGE_MT0, CARRIAGE_ML0 = 5630.0, 33.5, 29.2   # N, N*m, N*m
CARRIAGE_MASS = 0.051                         # kg

# Drive, Nanotec
MOTOR_FRAME = 42.3                              # NEMA 17 square
MOTOR_L = 34.0                                  # ST4118S, 1.2 in on the datasheet (measure in STEP)
MOTOR_T_HOLD = 0.20                             # 28.3 oz-in
GEAR_I, GEAR_ETA, GEAR_L = 7.07, 0.90, 38.0     # GP42-S1-7-SR (same housing as -5)
BRAKE_T = 0.35                                  # BRAKE-BWA-0,35-5, 24 V
T_PINION_STATIC = MOTOR_T_HOLD * GEAR_I * GEAR_ETA
T_PINION_BRAKE = BRAKE_T * GEAR_I
F_JAW_MAX = T_PINION_STATIC / (2 * PINION_D / 2 / 1000)   # N per jaw at full current, gearbox loss only: FEM upper bound
ETA_BEVEL, ETA_RP = 0.98, 0.97                  # V2 § 10
F_JAW_AVAIL = F_JAW_MAX * ETA_BEVEL * ETA_RP    # N per jaw the motor really delivers at full current (all losses)
# Part may be dirty or oily (feedback 10 Nov). Design value for NBR on oily PA 6: 0.2. Brackets: steel on steel 0.1
# (Schunk's value for its payload ratings), oiled steel 0.05 ... 0.1, oil-resistant NBR gripper pads 0.3 ... 0.4
# (HKI product information), dry knurled NBR > 1 (PFA). The pads are grooved so oil drains off the contact; the
# higher friction this should give (MU_PAD_EXPECTED) is a reserve for hard e-stops, to be measured in V5, not used
# in the design proof (review 21 Nov).
MU_OILY = 0.2
MU_PAD_EXPECTED = 0.3
PAD_GROOVE = (1.5, 1.0, 4.0)        # mm: width, depth, pitch of the vertical oil-drain grooves in the NBR
F_GRIP_OILY = S_SICHERHEIT * B_MASSE * (G + A_BAHN) / (N_BACKEN * MU_OILY)   # required by the hand calculation (2.05 m/s2)
A_PATH_REAL = 4.96                  # m/s2, cap in the robot program (WorldAccLim \On := 4.96 on the loaded moves, review 1.5)
F_GRIP_SET = S_SICHERHEIT * B_MASSE * (G + A_PATH_REAL) / (N_BACKEN * MU_OILY)   # operating setpoint, also covers the e-stop (V2)

# Ganter GN 617-5-A indexing plunger (steel, knob, no lock nut), ganternorm.com dimension table
PLUNGER_PIN_D, PLUNGER_THREAD, PLUNGER_KNOB_D = 5.0, "M10x1", 21.0
PLUNGER_L1, PLUNGER_L3, PLUNGER_L5_MIN, PLUNGER_SW = 45.0, 17.0, 15.0, 12.0
PLUNGER_E, PLUNGER_L2 = 13.8, 5.0                # knob height, pin protrusion
PLUNGER_SPRING = (7.0, 17.0)                     # N, start/end

# ---------------------------------------------------------------- layout
# Frame: z = 0 is the underside of the housing top plate, X = grip axis, Y = 0 through jaws and flange axis.
# d = opening travel per jaw from the closed end stop; pad face at |x| = PAD_X0 + d.
PAD_X0 = 33.0                       # closed end stop, 2 mm past the grip faces
D_GRIP = GREIF_ABSTAND / 2 - PAD_X0  # 2.0
D_OPEN = 10.0                       # open: 86 mm between pads; jaws end above the lug, faces are 70 apart
CARR_X0 = 24.6                      # carriage centre at d = 0 (carriages 1.7 mm apart at the stop)
RAIL_Y = -36.0                      # carriages 1.5 mm clear of the racks, which run at the same height
RAIL_L, RAIL_T1, RAIL_NB = 125.0, 12.5, 5   # L = (nB-1)*T + 2*T1, Rexroth allows custom T1
RAIL_BASE_Z = 0.0                   # rail straight on the plate (size 12 carriage is 3 mm higher); aligned to a straightedge

PLATE_T = 7.0                       # top plate (lighter since 11 Nov; bearings sit in the boss above it)
WALL = 7.0                          # M3 cover screws on the wall centre line
HOUSING_X = 192.0                   # 164 + 2 x 14 (15 Nov): NME2 encoder behind the brake, kept symmetric
HOUSING_XR = (-HOUSING_X / 2, HOUSING_X / 2)
HOUSING_Y = (-60.5, 38.0)           # +y face flush with the hood and the flange edge; -y carries rail and slides
POCKET_Z = -23.0                    # underside of the body = top of the bottom cover
COVER_T = 3.0

SLIDE_T = 8.0
SLIDE_Z_TOP = RAIL_BASE_Z - CARRIAGE_H   # slide sits under the hanging carriage
SLIDE_CBORE_CARR = 3.0              # M3x8 ends at the carriage thread bottom (N3 = 3)
RACK_GAP = 1.0                      # racks float 1 mm above the opposite slide
RACK_Z0 = SLIDE_Z_TOP + RACK_GAP    # rack and pinion bottom
RACK_TEETH = 18                     # 18 * pi = 56.5 mm
RACK_REL = -16.0                    # rack outer end beyond its carriage centre (room for slots + pin)
RACK_SCREW_X, RACK_PIN_X, RACK_SLOT = (18.0, 32.5), 25.0, 1.0   # at d = 0; slots +-1 mm along X
# rack position is set at assembly (jaws symmetric), then pinned: ISO 8734 3 m6 x 14, hole drilled/reamed at assembly
SLIDE_TRIM_X = 12.0                 # slides keep |x| > 12 near y = 0 (pinion hub)

PINION_HUB_D, PINION_HUB_L, PINION_BORE = 16.0, 8.0, 8.0
SETSCREW_Z = -16.0                  # ISO 4029 M4x4 in the hub, 90 deg from the key
GLAND = ("Lapp SKINTOP STR-M 16x1.5", 16.0, 22.0, 18.0)  # thread, hex/body dia, length outside (approx.)
WINDOW_PLATE_T = 3.0
CABLE_SIDE = -1                     # cable window on the -y hood face above the housing step; +y stays one plain face

PAD_T = 3.0                         # NBR 70 Shore A, vulcanised onto the pad plate (oil resistant)
PAD_PLATE_T = 3.0                   # steel backing plate, sits in a pocket in the jaw, 2 x M3 from the back
PAD_SCREW_Y = 28.0                  # two M3 side by side at the pad centre height
JAW_T, JAW_W = 8.0, 25.0            # web from the quick-change seat down to the pad plate
# Pad (review 21 Nov): part B's centre of mass sits below the grip faces, so a path acceleration along y twists the
# part in the jaws; the friction has to carry that moment too. A wide, low pad (on the flat zone between the lug and
# the roof chamfer) keeps the CoG 10.5 mm under the pad centre and gives the friction a long lever (grip_check.py).
PAD_W = 82.0                        # pocket width along y (NBR face 80.8)
JAW_W_PAD = 88.0                    # jaw width at the pad (T-shaped jaw)
# Part B (STEP frame): grip faces x = +-35, top z = 40, lug x -60..-35 up to z = -10 on the -X side.
PART_OFFSET = (0.0, 1.9, -77.0)     # part top 11 mm under the cover (short jaws), centre of mass near y = 0
JAW_Z_BOTTOM = -84.0                # 3 mm above the lug
PAD_Z = (-81.5, -60.5)              # part z -4.5..16.5: flat on both sides over y +-46
JAW_PAD_TOP = PAD_Z[1] + 4.0        # top of the wide jaw plate
JAW_Z_TOP = -28.5                   # jaw starts just under the cover

# Jaw quick change: holder on the slide reaches through the cover; jaw sits on its inner face.
HOLDER_T_TOP, HOLDER_T_LOW = 10.0, 14.0  # X thickness above / below the cover
HOLDER_W = 38.0                     # 2 mm wall beside the screw counterbores
HOLDER_Z_LOW = -47.0
HOLDER_SCREW_Y = 13.0
HOLDER_SCREW_X = 5.5                # from the jaw contact face: counterbore keeps 1.5 mm to it
DOWEL_D, DOWEL_L, DOWEL_Y, DOWEL_Z = 4.0, 12.0, 5.5, -32.0   # ISO 8734 4 m6 x 12
PLUNGER_Z = -39.0

# Round motor housing around the NEMA 17 stack (square diagonal 59.8), connector pod on +X for the window plate
TOWER_D, TOWER_WALL, LID_T, LID_HOLE_D = 72.0, 4.0, 8.0, 54.0
TOWER_BOSS_H = 16.0
GEAR_SCREW_CBORE = 4.4                # M4 heads flush in the plate underside (gearbox and tower screws)
WALL_MIN = 1.5                      # checked for every hole of every own part
TOWER_BOSS = [(29.0, 0.0), (-29.0, 0.0), (0.0, 29.0)]   # three screws from below
BRAKE_TURN = -35.0                  # brake leads point into the cable window
POD_X, POD_Y = (28.0, 40.0), (-24.0, 28.0)              # flat connector pod, face at x = 40
ADAPTER_D, ADAPTER_SCREW_PCD = 76.0, 62.0                # ISO 10642 M4 countersunk from the top
HOUSING_R, EDGE_CHAMFER, COVER_CHAMFER = 12.0, 1.5, 0.0
SLIDE_EDGE = 2.5                    # slide wider than the carriage so the M3 counterbores keep a wall
GEAR_FLANGE_Z = PLATE_T             # gearbox output flange on top of the plate
ADAPTER_T, ISO_PCD, ISO_SPIGOT_D, ISO_SPIGOT_H = 10.0, 40.0, 25.0, 4.5   # ISO 9409-1-40-4-M6 (ABB IRB 1300)
BACKLASH_SHIFT = 0.05               # racks 0.05 mm further out -> about 0.036 mm flank backlash

# ---------------------------------------------------------------- lying drive (since 10 Nov, variant 4)
# Motor, gearbox and brake lie along X under a flat hood; a bevel pair 1:1 turns the drive down onto a pinion shaft
# at x = PINION_X. The racks have different lengths so the jaws, the hood and the flange all stay centred.
PINION_X = -56.0
DRIVE_Z = 36.0                      # drive axis height: room for the m 1.5 bevel wheel above the upper bearing
PINION_Z0 = -11.0                   # pinion teeth z -11..-1, 1 mm above the rack bosses on the slides
# KHK SMSG1.5-20R / SMSG1.5-20L: ground spiral miter gears m 1.5, S45C teeth induction hardened, allowable torque
# 4.1 N*m (bending) / 3.47 N*m (surface). The m 1 pair (1.17 / 0.97 N*m) carried the setpoint only at K_A 1 and with
# no tolerance: with K_A 1.25 and the scatter of the current setting it was loaded to 108 ... 144 % (drive_margin.py,
# review 1.7, 21 Nov). Bore 8 as bought (+ key 2 on the wheel), hubs turned down to 16 (the wheel hub sits inside the
# DIN 472 ring), the wheel hub faced to mounting distance 19. Modelled with straight teeth, same pitch cone and hub.
BEVEL_M, BEVEL_Z, BEVEL_B, BEVEL_HUB_D, BEVEL_HUB_L = 1.5, 20, 8.0, 16.0, 5.0
BEVEL_T_ALLOW, BEVEL_T_BEND = 3.47, 4.1     # N*m, KHK surface durability / bending (catalogue, 600 rpm)
K_A_DRIVE = 1.25                            # application factor (V2)
# Current limit = the highest grip force the controller may be set to: the motor at full current (the bevel pair,
# with K_A and +5 % calibration scatter, would allow about 130 N per jaw).
F_JAW_LIMIT = min(F_JAW_AVAIL, BEVEL_T_ALLOW / K_A_DRIVE * ETA_BEVEL * ETA_RP / (2 * PINION_D / 2 / 1000) / 1.05)
assert F_GRIP_SET <= F_JAW_LIMIT <= F_JAW_AVAIL, (F_GRIP_SET, F_JAW_LIMIT, F_JAW_AVAIL)
BEARING = (8.0, 22.0, 7.0)          # DIN 625 608-2Z
BEARING_LOWER_Z, BEARING_UPPER_Z = 0.0, 9.5
BOSS_D = 32.0
BOSS_TOP = BEARING_UPPER_Z + BEARING[2] + 2.0   # 2 mm above the upper bearing for the DIN 472 groove
SHOULDER_D = 15.0                   # housing shoulder under the outer rings only (608 inner ring OD ~12, outer ring ID ~18.6)
SHIM_T = 0.5                        # DIN 988 8x14 under the bevel wheel hub, 0.1-0.5 picked at assembly to set the mesh
COLLAR_D = 10.5                     # shaft collar under the lower inner ring (loose bearing, inner ring k6 on the shaft)
KEY_B, KEY_H, KEY_T1, KEY_T2, KEY_L = 2.0, 2.0, 1.2, 1.0, 12.0  # DIN 6885 A 2x2x12 for d = 6...8
SHAFT_TOP = 30.0                     # carries the 12 mm key inside the m 1.5 wheel
BEVEL_WHEEL_DZ = -0.25              # 0.25 mm lower than nominal: interference-free mesh with a little backlash (fitted)
GEAR_SHAFT_L = 22.6                 # GP42 output shaft beyond the flange (STEP)
BRACKET_T, BRACKET_FOOT_Y, BRACKET_FOOT_W = 6.0, 25.25, 6.5   # feet beside the gearbox, M3
HOOD_Y, HOOD_WALL, HOOD_R = 38.0, 3.0, 12.0     # hood y = +-38: symmetric, flush with the flange (D76) and the +y face
BRAKE_TURN_X = 95.0                 # brake leads stay inside the hood (the STEP has open lead stubs)
HOOD_TOP, HOOD_TOP_T, HOOD_SEAT = 65.0, 5.0, 3.0   # 3 mm seat ring under the adapter carries the M4 threads
HOOD_SCREW_X, HOOD_SCREW_Y = 69.5, (-25.0, 25.0)   # M4 from below into bosses in the hood corners
RACK_SPAN = {1: (-72.5, 37.5), -1: (-61.98, -18.0)}     # x at d = 0
RACK_TEETH_SIDE = {1: 35, -1: 14}
RACK_SCREW_XS = {1: (18.0, 32.5), -1: (-23.0, -39.0)}
RACK_PIN_XS = {1: 25.0, -1: -31.0}


# ---------------------------------------------------------------- pneumatic variant (21 Nov 2026)
# Double-acting compact cylinder Festo ADN-16-10-A-P-A (536220, ISO 21287): air closes with the force set on a
# pressure regulator (like the current limit of the electric drive), air opens. A compression spring adds to the
# closing force and alone holds the part at standstill when the air fails (fail-safe). The cylinder sits in a flat
# riser on the top plate; a driver on its rod reaches through a slot in the plate into the right rack, the pinion
# keeps the left jaw in step. Racks, slides, rail, jaws and the bearing boss stay as in the electric version.
CYL_BORE, CYL_ROD = 16.0, 8.0                    # mm, ADN-16 (rod thread M6)
CYL_BODY = (39.7, 29.0)                          # body length, square [verify against the Festo drawing]
CYL_ROD_OUT = 8.0                                # rod beyond the front face when retracted, 6 mm of it in the driver
CYL_MASS = 0.089                                 # kg, Festo data sheet 536220
CYL_Y, CYL_Z0 = -32.0, 10.0                      # axis y, body underside (3 mm above the plate)
CYL_FRONT_X = 2.0                                # front face at d = 0
P_SUPPLY = 6.0                                   # bar
A_EXTEND = 3.14159 / 4 * CYL_BORE ** 2           # mm2, opens
A_RETRACT = A_EXTEND - 3.14159 / 4 * CYL_ROD ** 2   # mm2, closes
ETA_CYL = 0.9                                    # seal friction
DRV_X0, DRV_T = CYL_FRONT_X + 2.0, 10.0          # driver plate on the rod
DRV_Y, DRV_Z = (-40.0, 15.0), (12.0, 37.0)
PIN_X = DRV_X0 + DRV_T / 2                       # driver pin into the right rack (ISO 8734 5 x 20)
PIN_D = 5.0
SPRING_Y = 8.0                                   # safety spring beside the cylinder, on a guide pin in the driver
# DIN EN 13906-1, patented spring wire DH: d 1.8, Dm 12, 12.4 active coils, L0 46 -> c ~ 5 N/mm [V2 to check]
SPRING_D, SPRING_DM, SPRING_N, SPRING_L0, SPRING_RATE = 1.8, 12.0, 12.4, 46.0, 5.0
SPRING_L_GRIP = 36.0                             # installed length at d = D_GRIP
GUIDE_D = 9.0
RISER_WALL, RISER_TOP_T, ISO_BOSS_T = 3.0, 3.0, 9.0   # the ISO 9409 pattern is machined into the riser top
RISER_XR, RISER_Y = (-76.0, 74.0), (-50.0, 38.0)       # only over boss, cylinder, spring and flange
RISER_H = CYL_Z0 + CYL_BODY[1] + 1.0 + ISO_BOSS_T - PLATE_T   # plate top to flange face

if DRIVE == "pneumatic":
    HOUSING_XR = (-84.0, 76.0)                   # holder slots end at |x| 65, the cover screws stay outside them
    HOUSING_X = HOUSING_XR[1] - HOUSING_XR[0]
    F_SPRING_GRIP = SPRING_RATE * (SPRING_L0 - SPRING_L_GRIP)            # N, both jaws together, at d = D_GRIP
    F_SPRING_OPEN = F_SPRING_GRIP + SPRING_RATE * (D_OPEN - D_GRIP)
    F_SPRING_CLOSED = F_SPRING_GRIP - SPRING_RATE * D_GRIP
    P_GRIP = (2 * F_GRIP_SET - F_SPRING_GRIP) / (A_RETRACT * ETA_CYL) * 10        # bar on the closing side
    F_JAW_MAX = (F_SPRING_GRIP + P_SUPPLY / 10 * A_RETRACT) / 2          # regulator fails: full supply, no losses
    F_JAW_LIMIT = F_JAW_MAX
    F_JAW_AVAIL = (F_SPRING_GRIP + P_SUPPLY / 10 * A_RETRACT * ETA_CYL) / 2
    F_OPEN_CYL = P_SUPPLY / 10 * A_EXTEND * ETA_CYL                      # N, must beat the spring when opening
    F_HOLD_AIR_LOSS = F_SPRING_GRIP / 2                                  # N per jaw, spring alone
    assert F_OPEN_CYL > F_SPRING_OPEN * 1.2, (F_OPEN_CYL, F_SPRING_OPEN)
    assert F_HOLD_AIR_LOSS >= 1.5 * B_MASSE * G / (N_BACKEN * MU_OILY), F_HOLD_AIR_LOSS
    assert 0 < P_GRIP < P_SUPPLY, P_GRIP
