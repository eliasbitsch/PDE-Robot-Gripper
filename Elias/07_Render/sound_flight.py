# Sound for the flight preview: deep sub bed, low whooshes on the camera moves, motor whir, rack ticks and metal
# clacks on the jaw contacts. Effects from earshot's bank (synthesized, not recordings), bed and whir made here.
# run with earshot's venv:  <earshot>/.venv/Scripts/python.exe sound_flight.py [video.mp4]
import json, os, sys
import numpy as np
import soundfile as sf
from scipy.signal import butter, sosfilt
from earshot.forge import bank

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "sound")
KIN = json.load(open(os.path.join(HERE, "..", "02_CAD_Body", "out", "kinematics.json")))
SR, FPS, LEN = 48000, 30, 9.0
N = int(SR * LEN)
mix = np.zeros((N, 2))
rng = np.random.default_rng(7)


def t_of(frame):
    return (frame - 1) / FPS


def place(y, t_onset, gain_db, align=0.0):
    """y onset (or the point `align` s into y) lands on t_onset."""
    y = y if y.ndim == 2 else np.stack([y, y], 1)
    i = int(round((t_onset - align) * SR))
    a, b = max(i, 0), min(i + len(y), N)
    if b > a:
        mix[a:b] += y[a - i:b - i] * 10 ** (gain_db / 20)


def sfx(sid, t, gain_db, at="onset", **kn):
    y, m = bank.render(sid, checks=False, **kn)
    place(y, t, gain_db, align=m["peak_s"] if at == "peak" else m["onset_s"])


def lp(x, hz, order=4):
    return sosfilt(butter(order, hz, "lowpass", fs=SR, output="sos"), x)


def bp(x, lo, hi):
    return sosfilt(butter(2, [lo, hi], "bandpass", fs=SR, output="sos"), x)


# jaw timeline, same formula as hero_v1.animate(): close 0.2-0.45, hold, open 0.65-0.9 of each shot
SHOTS = ((1, 150), (151, 270))
moves = []                                   # (t0, t1, closing)
for s, e in SHOTS:
    f = lambda u: s + u * (e - s)
    moves += [(t_of(f(0.2)), t_of(f(0.45)), True), (t_of(f(0.65)), t_of(f(0.9)), False)]
travel = KIN["d_open"] - KIN["d_grip"]       # mm per jaw
T = np.arange(N) / SR

# 1. bed: D sub drone, slow swell towards the reveal of the drive at the second grip (shell gone)
grip2 = moves[2][1]
env = np.clip(T / 1.2, 0, 1) * (0.55 + 0.45 * np.exp(-((T - grip2) / 2.2) ** 2)) * np.clip((LEN - T) / 0.6, 0, 1)
drone = sum(a * np.sin(2 * np.pi * f * T + p) for f, a, p in ((36.71, 0.55, 0), (73.42, 0.75, 1.1),
                                                              (110.0, 0.22, 0.4), (146.8, 0.10, 2.0)))
drone *= 1 + 0.08 * np.sin(2 * np.pi * 0.23 * T)
air = lp(rng.standard_normal(N), 420) * 0.25
env *= 1 - 0.8 * np.clip((T - (grip2 - 0.4)) / 0.15, 0, 1) * np.clip((grip2 - T) / 0.02, 0, 1)  # stop-down
bed = np.tanh(1.4 * (drone + air)) * env
place(np.stack([bed, np.roll(bed, 48)], 1), 0.0, -17)

# 2. cold open: one deep hit on the first frame
sfx("impact/deep", 0.02, -6, pitch="D1")

# 3. camera moves: a low whoosh peaking mid-segment (fastest point of the eased move)
WPS = (1, 55, 100, 145, 195, 270)
woosh = ("pro_whoosh_big/long", "pro_whoosh_big/swell", "pro_whoosh_big/windy", "pro_whoosh_big/long",
         "pro_whoosh_big/swell")
for k, sid in enumerate(woosh):
    a, b = t_of(WPS[k]), t_of(WPS[k + 1])
    d = min(b - a, 1.8)
    sfx(sid, (a + b) / 2, -13 if k != 4 else -16, at="peak", dur=d, apex=d * 0.55, pitch="D1")

# 4. mechanism: motor whir with speed, a tick per mm of rack travel, a metal clack at contact / end stop
for t0, t1, closing in moves:
    n = int((t1 - t0) * SR)
    u = np.linspace(0, 1, n)
    v = 6 * u * (1 - u) / 1.5                  # smoothstep velocity, peak 1
    ph = 2 * np.cumsum(np.pi * (70 + 210 * v) / SR)
    w = (np.sin(ph) + 0.5 * np.sin(2 * ph) + 0.3 * np.sin(3.01 * ph)) + 0.4 * bp(rng.standard_normal(n), 600, 2400)
    w = np.tanh(w) * v ** 0.8 * np.clip(u / 0.05, 0, 1) * np.clip((1 - u) / 0.05, 0, 1)
    place(np.stack([w, w], 1), t0, -27)
    pos = travel * (u * u * (3 - 2 * u))
    for mm in range(1, int(travel)):
        i = np.searchsorted(pos, mm)
        sfx("click/thock", t0 + i / SR, -24 - 4 * (mm % 2), dur=0.06)
        sfx("hover_tick/detent", t0 + i / SR, -24)          # steel tooth on top of the body
    if closing:                               # jaws hit the part: layered metal clack, the second one big
        big = t1 > 5
        sfx("click/thock", t1, -8)
        sfx("click/snap", t1, -9)                         # bright steel edge of the clack
        sfx("pro_click/mouse", t1 + 0.004, -12)
        sfx("hover_tick/detent", t1 + 0.012, -16)
        sfx("tap/knuckle", t1, -10, pitch="D3")
        sfx("pro_pop_in/deep", t1, -10)
        sfx("pro_impact/short" if not big else "pro_impact/deep", t1, -9 if not big else -4, pitch="D2" if not big else "D1")
    else:                                     # back at the end stop: softer
        sfx("click/thock", t1, -14)
        sfx("click/snap", t1, -20)
        sfx("tap/table", t1, -17, pitch="D3")

# 5. shell fades out (frames 178-210): a reverse swell into the second contact
sfx("pro_whoosh_big/reverse", t_of(208), -15, at="peak", dur=1.4, apex=1.1, pitch="D2")

# master: gentle limiter, -1 dBFS sample peak, fade out the last 0.25 s
mix = sosfilt(butter(2, 32, "highpass", fs=SR, output="sos"), mix, axis=0)
mix *= np.clip((LEN - T) / 0.25, 0, 1)[:, None]
mix /= np.abs(mix).max()
DRIVE = 1.7
mix = np.tanh(mix * DRIVE) / np.tanh(DRIVE)
mix = lp(mix.T, 16000, 6).T                   # AAC turned a few ultrasonic tick spikes into +5 dBTP overshoot
from scipy.signal import resample_poly
mix *= 10 ** (-1.5 / 20) / np.abs(resample_poly(mix, 4, 1, axis=0)).max()   # true-peak ceiling -1.5 dBTP
os.makedirs(OUT, exist_ok=True)
wav = os.path.join(OUT, "flight_mix.wav")
sf.write(wav, mix, SR, subtype="PCM_24")
json.dump({"fps": FPS, "events": [{"t": round(t, 3), "kind": "contact" if c else "stop"} for _, t, c in moves]
           + [{"t": round(t_of(f), 3), "kind": "waypoint"} for f in WPS[1:-1]]},
          open(os.path.join(OUT, "cues.json"), "w"), indent=1)
print("wrote", wav)
if len(sys.argv) > 1:
    vid = sys.argv[1]
    dst = vid.replace(".mp4", "_sound.mp4")
    os.system(f'ffmpeg -y -v error -i "{vid}" -i "{wav}" -c:v copy -c:a aac -b:a 256k -shortest "{dst}"')
    print("wrote", dst)
