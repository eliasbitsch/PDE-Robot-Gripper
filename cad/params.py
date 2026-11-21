"""Zentrale Parameter - Gruppe 10, Bauteil B (PA 6).

EINZIGE Quelle der Wahrheit. Alle Skripte (Auslegung, CAD, FEM, Simulation)
importieren von hier, damit eine Iteration nur an einer Stelle passiert.

Quellen:
  Angabe WS26.pdf, Abb.1  -> Anlagengeometrie, Taktzeit
  Angabe WS26.pdf, Tab.1  -> Werkstoff PA 6, Ra 1,6
  scripts/analyze_B.py    -> Bauteilgeometrie aus Bauteil_B.STEP
"""
from math import hypot, pi

# ---------------------------------------------------------------- Organisation
GRUPPE = 10
BAUTEIL = "B"
AUTOREN = ["Bitsch Elias", "Ovdiienko Viktoriia"]

# ---------------------------------------------------------------- Bauteil B
# aus analyze_B.py (Bauteil_B.STEP)
B_BBOX = (95.0, 100.0, 80.0)        # mm
B_VOL = 528.5e-6                    # m^3  (528,5 cm^3)
B_COG = (-3.0, -1.9, -4.5)          # mm, Schwerpunkt rel. STEP-Ursprung

# Tab.1: PA 6, Ra 1,6  -> NICHT Stahl. Aeltere Rechnung mit 4,149 kg war falsch.
B_RHO = 1140.0                      # kg/m^3   PA 6 (Datenblatt 1,12...1,15)
B_MASSE = B_VOL * B_RHO             # kg  -> 0,602
B_RA = 1.6                          # um, geforderte Oberflaeche -> Weichbacken

# Greifflaechen: Paar 1/2 aus analyze_B.py, Normale +-X
GREIF_RICHTUNG = "X"
GREIF_ABSTAND = 70.0                # mm, Backenabstand am Bauteil
GREIF_FLAECHE = 6200.0              # mm^2, kleinere der beiden Flaechen
# Schwerpunktversatz quer zur Greifrichtung -> Kippmoment auf die Zange
GREIF_EXZENTRIZITAET = 4.5          # mm (COG_z)

# ---------------------------------------------------------------- Anlage (Abb.1)
H_ENTNAHME = 1.000                  # m  Pufferband
H_ABLAGE = 1.300                    # m  Werkstuecktraeger
DIST_HORIZONTAL = 1.800             # m  Entnahme <-> Ablage
KETTEN_ABSTAND = 0.600              # m  zwei parallele Foerderketten
EINHAUSUNG_HOEHE = 2.100            # m
TAKTZEIT = 5.0                      # s je Bauteil

# ---------------------------------------------------------------- Physik
G = 9.81                            # m/s^2

# ---------------------------------------------------------------- Zeitbudget
# Aufteilung der Taktzeit; Handhabungszeiten nach Herstellerangaben Parallelgreifer
T_GREIFEN = 0.30                    # s  Zange schliessen
T_LOESEN = 0.20                     # s  Zange oeffnen
T_FEIN = 0.50                       # s  Feinpositionierung Entnahme + Ablage
T_VERFAHREN = TAKTZEIT - (T_GREIFEN + T_LOESEN + T_FEIN)   # s  hin + zurueck
T_EINWEG = T_VERFAHREN / 2.0

# Trapezprofil mit Beschleunigungsphase t_a = t/3 (Faustwert Industrieroboter)
WEG_RAEUMLICH = hypot(DIST_HORIZONTAL, H_ABLAGE - H_ENTNAHME)   # m
_TA = T_EINWEG / 3.0
A_BAHN = WEG_RAEUMLICH / (_TA * (T_EINWEG - _TA))   # m/s^2
V_BAHN = A_BAHN * _TA                               # m/s

# ---------------------------------------------------------------- Grip force
# Updated 21 Nov 2026 (review, section 2) to the current design. The authoritative design values are in
# Elias/02_CAD_Body/gripper_params.py; the names below are kept for the early scripts in cad/, fem/, sim/, docs/.
S_SICHERHEIT = 2.0                  # safety against slip (VDI 2860)
N_BACKEN = 2                        # friction faces of a parallel gripper
MU_BELAG = 0.20                     # NBR pad on oily PA 6, design value (gripper_params MU_OILY); 0.5 was dry, superseded

# F_N = S * m * (g + a) / (n * mu)
F_GREIF = S_SICHERHEIT * B_MASSE * (G + A_BAHN) / (N_BACKEN * MU_BELAG)   # N per jaw, 35.7 (hand calculation, 2.05 m/s2)

# ---------------------------------------------------------------- Gripper
M_GREIFER = 3.04                    # kg, from the CAD build (02_CAD_Body/out/checks.json, 21 Nov)
FLANSCH_NORM = "ISO 9409-1-40-4-M6"  # ABB IRB 1300 flange
GREIFER_TYP = "Own design, rack and pinion, closed-loop stepper with brake"
FREIGANG = 1.5                      # mm clearance per side (early sizing)
GREIFER_HUB = 10.0                  # mm stroke per jaw (gripper_params D_OPEN), 86 mm open over the 70 mm grip faces

MODUL = 1.0                         # mm, z = 20 above the limit 17: no profile shift
Z_RITZEL = 20
B_ZAHN = 10.0                       # mm pinion face width (gripper_params PINION_B)
R_TEILKREIS = MODUL * Z_RITZEL / 2.0                 # mm
ETA_GETRIEBE = 0.97                 # rack and pinion (V2 § 10); the bevel pair 0.98 is separate (gripper_params ETA_BEVEL)

# Drive: Nanotec ST4118S1404-B + GP42-S1-7-SR + BRAKE-BKE-0,4-5,0 under the hood, KHK bevel pair 1:1 to the pinion
MOTOR_TYP = "Stepper ST4118S1404-B, planetary GP42-S1-7-SR, permanent-magnet brake BKE-0,4-5,0"
MOTOR_M_NENN = 0.20                 # N*m holding torque
GETRIEBE_I = 7.07
GETRIEBE_ETA = 0.90
MOT_L_GETRIEBE = 38.0               # mm
MOT_L_MOTOR = 30.5                  # mm
MOT_L_GESAMT = MOT_L_GETRIEBE + MOT_L_MOTOR
MOT_FLANSCH = 42.0                  # mm, NEMA 17

M_ABTRIEB = MOTOR_M_NENN * GETRIEBE_I * GETRIEBE_ETA          # N*m at the pinion
BREMSE_M_NOETIG = 2 * F_GREIF * R_TEILKREIS / 1000.0 / ETA_GETRIEBE   # N*m at the pinion
BREMSE_M_NENN = 0.4                 # N*m at the motor (BKE), x 7.07 at the pinion
BREMSE_L = 28.9                     # mm
BREMSE_D = 39.0                     # mm
GREIFER_F_MAX = M_ABTRIEB * 1000.0 / R_TEILKREIS / 2.0 * ETA_GETRIEBE   # N per jaw
F_ZANGE_AUSLEGUNG = GREIFER_F_MAX   # load case of the early jaw sizing

# ---------------------------------------------------------------- Robot
ROBOTER = "ABB IRB 1300-7/1.4"
R_ROBOTER = 1400.0                  # mm reach
M_TRAGLAST = 7.0                    # kg rated payload

# ---------------------------------------------------------------- Early jaw sizing (superseded by V3/V4/FEM)
# Cantilever from the early concept; kept so cad/ and fem/ still run. The real jaw is Viktoriia's V3.
ZANGE_WERKSTOFF = "EN AW-7075 T6"
ZANGE_E = 71000.0                   # MPa
ZANGE_NU = 0.33
ZANGE_RHO = 2800.0                  # kg/m^3
ZANGE_RP02 = 460.0                  # MPa
ZANGE_S_ZUL = 2.0
ZANGE_SIGMA_ZUL = ZANGE_RP02 / ZANGE_S_ZUL   # MPa
ZANGE_B = 25.0                      # mm
ZANGE_FILLET = 3.0                  # mm
BACKE_LAENGE = 60.0                 # mm
BACKE_BREITE = ZANGE_B - 10.0       # mm
BACKE_TIEFE = 1.5                   # mm
LUFT_BAUTEIL = 10.0                 # mm
ZANGE_L = LUFT_BAUTEIL + (B_BBOX[2] - BACKE_LAENGE) / 2.0 + BACKE_LAENGE
ZANGE_A_LAST = ZANGE_L - BACKE_LAENGE / 2.0
# Thickness from stiffness: jaw deflection no larger than the pad compression (free-block estimate of the early
# sizing; the bonded pad is much stiffer, see Elias/05_FEM/tilt_check.py)
NBR_E = 5.5                         # MPa, NBR 70 Shore A (Gent from the hardness)
NBR_T = 3.0                         # mm, as built (V3)
_f_belag = F_ZANGE_AUSLEGUNG / (BACKE_LAENGE * BACKE_BREITE) * NBR_T / NBR_E
_h_noetig = (F_ZANGE_AUSLEGUNG * ZANGE_A_LAST ** 2 * (3 * ZANGE_L - ZANGE_A_LAST)
             / (6 * ZANGE_E * _f_belag) * 12 / ZANGE_B) ** (1.0 / 3.0)
from math import ceil as _ceil  # noqa: E402
ZANGE_H = float(_ceil(_h_noetig))   # mm
ZANGE_F_BELAG = _f_belag            # mm

if __name__ == "__main__":
    print("GRUPPE %d - BAUTEIL %s (PA 6)" % (GRUPPE, BAUTEIL))
    print("  Masse              %.3f kg   (G = %.2f N)" % (B_MASSE, B_MASSE * G))
    print("  Greifabstand       %.1f mm auf %.0f mm2" % (GREIF_ABSTAND, GREIF_FLAECHE))
    print()
    print("BEWEGUNG")
    print("  Verfahrzeit je Weg %.2f s  (von %.1f s Taktzeit)" % (T_EINWEG, TAKTZEIT))
    print("  Weg                %.3f m" % WEG_RAEUMLICH)
    print("  Beschleunigung     %.2f m/s2  = %.2f g" % (A_BAHN, A_BAHN / G))
    print("  Spitzengeschw.     %.2f m/s" % V_BAHN)
    print()
    print("GREIFKRAFT")
    print("  mu = %.2f, S = %.1f, %d Backen" % (MU_BELAG, S_SICHERHEIT, N_BACKEN))
    print("  F_N                %.1f N je Backe" % F_GREIF)
    print("  Flaechenpressung   %.3f N/mm2 auf %.0f mm2" % (F_GREIF / GREIF_FLAECHE, GREIF_FLAECHE))
    print()
    print("GREIFERZANGE (%s)" % ZANGE_WERKSTOFF)
    print("  Kragarm %.0f x %.0f x %.0f mm, sigma_zul = %.0f MPa"
          % (ZANGE_L, ZANGE_B, ZANGE_H, ZANGE_SIGMA_ZUL))
