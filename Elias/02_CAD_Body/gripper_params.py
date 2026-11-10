"""Gripper geometry. Part, cell and force values come from cad/params.py."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cad"))
from params import *  # noqa: E402,F401,F403

# Catalogue data, sources in vendor/SOURCES.md. Lengths in mm, torques in N*m.
# Rack m=1 steel (Maedler; profile per DIN 867): cross-section and pitch line height from the base
RACK_W, RACK_H, RACK_PITCH_LINE = 10.0, 12.0, 11.0
PINION_D, PINION_B = MODUL * Z_RITZEL, 10.0

# Bosch Rexroth miniature ball rail size 9 (catalogue R999001206, 2023-08, p. 19/21)
# carriage SLS long, R0444 913 01 (size 9/M3, accuracy H, preload); rail R0445 9..
RAIL_W, RAIL_H, RAIL_PITCH = 9.0, 5.5, 20.0
RAIL_HOLE_D, RAIL_CBORE_D, RAIL_SCREW = 3.5, 6.0, "M3"
RAIL_CBORE_DEPTH = 3.2
CARRIAGE_W, CARRIAGE_L, CARRIAGE_L_BODY = 20.0, 41.4, 31.3
CARRIAGE_H, CARRIAGE_GAP = 10.0, 2.8          # rail base to carriage top, rail base to carriage underside
CARRIAGE_HOLES_E1, CARRIAGE_HOLES_E2, CARRIAGE_THREAD = 16.0, 15.0, "M3x3"   # E1 along the rail
CARRIAGE_C0, CARRIAGE_MT0, CARRIAGE_ML0 = 3150.0, 14.5, 14.0   # N, N*m, N*m
CARRIAGE_MASS = 0.026                         # kg

# Drive, Nanotec
MOTOR_FRAME = 42.3                              # NEMA 17 square
MOTOR_L = 34.0                                  # ST4118S, 1.2 in on the datasheet (measure in STEP)
MOTOR_T_HOLD = 0.20                             # 28.3 oz-in
GEAR_I, GEAR_ETA, GEAR_L = 7.07, 0.90, 38.0     # GP42-S1-7-SR (same housing as -5)
BRAKE_T = 0.35                                  # BRAKE-BWA-0,35-5, 24 V
T_PINION_STATIC = MOTOR_T_HOLD * GEAR_I * GEAR_ETA
T_PINION_BRAKE = BRAKE_T * GEAR_I
F_JAW_MAX = T_PINION_STATIC / (2 * PINION_D / 2 / 1000)   # N per jaw at full current, FEM worst case
# Part may be dirty or oily (feedback 10 Nov): design with a lubricated friction coefficient, source to cite in V2.
MU_OILY = 0.2
F_GRIP_OILY = S_SICHERHEIT * B_MASSE * (G + A_BAHN) / (N_BACKEN * MU_OILY)

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
CARR_X0 = 21.5                      # carriage centre at d = 0 (carriages 1.6 mm apart at the stop)
RAIL_Y = -32.5                      # carriages clear of the racks, which run at the same height
RAIL_L, RAIL_T1, RAIL_NB = 110.0, 15.0, 5   # L = (nB-1)*T + 2*T1, Rexroth allows custom T1
RAIL_BASE_Z = -3.0                  # rail sits on a 3 mm land = reference shoulder

PLATE_T = 10.0                      # top plate
WALL = 7.0                          # M3 cover screws on the wall centre line
HOUSING_X = 164.0                   # long enough for the lying drive and the long rack
HOUSING_Y = (-53.5, 44.0)
POCKET_Z = -23.0                    # underside of the body = top of the bottom cover
COVER_T = 4.0

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

PAD_T = 3.0                         # NBR 70 Shore A, vulcanised onto the pad plate (oil resistant)
PAD_PLATE_T = 3.0                   # steel backing plate, sits in a pocket in the jaw, 2 x M3 from the back
PAD_SCREW_DZ = 10.0
JAW_T, JAW_W = 8.0, 25.0
# Part B (STEP frame): grip faces x = +-35, top z = 40, lug x -60..-35 up to z = -10 on the -X side.
PART_OFFSET = (0.0, 1.9, -92.0)     # part top 25 mm under the cover, centre of mass near y = 0
JAW_Z_BOTTOM = -98.0                # 4 mm above the lug
PAD_Z = (-95.0, -60.0)              # part z -3..32: flat on both sides
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
DRIVE_Z = 32.5                      # drive axis height: NEMA 17 (42.3 square) clears the top plate
PINION_Z0 = -11.0                   # pinion teeth z -11..-1, 1 mm above the rack bosses on the slides
BEVEL_Z, BEVEL_B, BEVEL_HUB_D, BEVEL_HUB_L = 20, 5.0, 16.0, 6.0   # Maedler bevel gears m = 1, i = 1
BEARING = (8.0, 22.0, 7.0)          # DIN 625 608-2Z
BEARING_LOWER_Z, BEARING_UPPER_Z = 0.0, 9.5
BOSS_D = 32.0
SHAFT_TOP = 27.0
BEVEL_WHEEL_DZ = -0.25              # 0.25 mm lower than nominal: interference-free mesh with a little backlash (fitted)
GEAR_SHAFT_L = 22.6                 # GP42 output shaft beyond the flange (STEP)
BRACKET_T, BRACKET_FOOT_Y = 6.0, 25.5          # feet 8 wide, beside the gearbox
HOOD_Y, HOOD_LIP_Y, HOOD_WALL, HOOD_R = 35.0, 39.0, 4.0, 14.0
BRAKE_TURN_X = 95.0                 # brake leads stay inside the hood (the STEP has open lead stubs)
HOOD_TOP, HOOD_TOP_T = 63.0, 8.0
RACK_SPAN = {1: (-72.5, 37.5), -1: (-61.98, -18.0)}     # x at d = 0
RACK_TEETH_SIDE = {1: 35, -1: 14}
RACK_SCREW_XS = {1: (18.0, 32.5), -1: (-23.0, -39.0)}
RACK_PIN_XS = {1: 25.0, -1: -31.0}
