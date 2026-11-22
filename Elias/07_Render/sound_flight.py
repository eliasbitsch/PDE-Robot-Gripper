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
TL = json.load(open(os.path.join(HERE, "out", "flight_timeline.json")))   # written by hero_v1.py -- flight
SR, FPS = 48000, TL["fps"]
LEN = TL["end"] / FPS
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


moves = [(t_of(f0), t_of(f1), b < a) for f0, f1, a, b in TL["jaws"]]   # (t0, t1, closing)
travel = KIN["d_open"] - KIN["d_grip"]       # mm per jaw
T = np.arange(N) / SR
big = t_of(TL["fade"][0])                    # contact at the jaw macro, where the shell starts to fade

# 1. music bed: clean pad in D minor (Dm9 - Bbmaj7 - Gm9 - Asus4), bass an octave down, soft 120 BPM pulse; no
#    saturation and nothing pitched below its native range (v3 sounded slowed down and crackly)
def saw(f, n):
    t = np.arange(n) / SR
    y = np.zeros(n)
    for det in (-0.004, 0.0, 0.004):
        ph = rng.uniform(0, 2 * np.pi)
        for h in range(1, int(3500 / f) + 1):
            y += np.sin(2 * np.pi * f * (1 + det) * h * t + ph * h) / h
    return y / 3


NOTE = lambda name: 440 * 2 ** ((("C C# D D# E F F# G G# A A# B".split().index(name[:-1])) + 12 * (int(name[-1]) - 4) - 9) / 12)
CHORDS = [("D2", "D3 F3 A3 C4 E4"), ("A#1", "A#2 D3 F3 A3 C4"), ("G1", "G2 A#2 D3 F3 A3"), ("A1", "A2 D3 E3 G3 A3")]
seg = LEN / len(CHORDS)
pad = np.zeros(N)
for i, (bass, notes) in enumerate(CHORDS):
    a0, a1 = max(0.0, i * seg - 0.8), min(LEN, (i + 1) * seg + 0.8)
    n = int((a1 - a0) * SR)
    u = np.arange(n) / SR
    env = np.clip(u / 0.9, 0, 1) * np.clip((a1 - a0 - u) / 0.9, 0, 1)
    y = sum(saw(NOTE(x), n) for x in notes.split()) / 5 + 0.45 * np.sin(2 * np.pi * NOTE(bass) * u)
    i0 = int(a0 * SR)
    pad[i0:i0 + n] += (y * env)[:N - i0]
pad = lp(pad, 2800, 2)
pad = sosfilt(butter(2, 55, "highpass", fs=SR, output="sos"), pad)
pad *= (0.85 + 0.15 * np.cos(2 * np.pi * 4 * T))            # 8ths at 120 BPM
big = t_of(TL["fade"][0])                    # contact at the jaw macro, where the shell starts to fade
swell = np.clip(T / 1.5, 0, 1) * (0.6 + 0.4 * np.exp(-((T - big) / 3.0) ** 2))
swell *= 1 - 0.85 * np.clip((T - (big - 0.4)) / 0.15, 0, 1) * np.clip((big - T) / 0.02, 0, 1)   # stop-down
for f in TL["cuts"]:                         # dip with the picture
    swell *= 1 - 0.5 * np.exp(-((T - t_of(f)) / 0.12) ** 2)
pad *= swell * np.clip((LEN - T) / 0.5, 0, 1)
place(np.stack([pad, np.roll(pad, 24)], 1), 0.0, -14)

# 2. cold open; every cut: a suck into the dip, a soft hit when the picture comes back
sfx("pro_impact/deep", 0.02, -6)
for f in TL["cuts"]:
    sfx("whoosh_fast/reverse", t_of(f), -16, at="peak")
    sfx("pro_impact/air", t_of(f) + 0.12, -14)

# 3. the two flowing camera moves: a whoosh peaking mid-move
for k, (f0, f1) in enumerate(TL["moves"]):
    a, b = t_of(f0), t_of(f1)
    d = min(b - a, 2.0)
    sfx(("whoosh_big/swell", "whoosh_big/deep")[k], (a + b) / 2, -14, at="peak", dur=d, apex=d * 0.55)

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
        hit = abs(t1 - big) < 0.05 or t1 > LEN - 0.7
        sfx("click/thock", t1, -8)
        sfx("click/snap", t1, -9)                         # bright steel edge of the clack
        sfx("pro_click/mouse", t1 + 0.004, -12)
        sfx("hover_tick/detent", t1 + 0.012, -16)
        sfx("tap/knuckle", t1, -10)
        sfx("pro_pop_in/deep", t1, -10)
        sfx("pro_impact/deep" if hit else "pro_impact/short", t1, -5 if hit else -11)
    else:                                     # back at the end stop: softer
        sfx("click/thock", t1, -14)
        sfx("click/snap", t1, -20)
        sfx("tap/table", t1, -17)

# 5. a reverse swell into the big contact
sfx("pro_whoosh_big/reverse", big, -16, at="peak", dur=1.4, apex=1.1)

# master: gentle limiter, -1 dBFS sample peak, fade out the last 0.25 s
mix = sosfilt(butter(2, 35, "highpass", fs=SR, output="sos"), mix, axis=0)
mix *= np.clip((LEN - T) / 0.25, 0, 1)[:, None]
mix /= np.abs(mix).max()
DRIVE = 2.6
mix = np.tanh(mix * DRIVE) / np.tanh(DRIVE)
mix = lp(mix.T, 16000, 6).T                   # AAC turned a few ultrasonic tick spikes into +5 dBTP overshoot
from scipy.signal import resample_poly
mix *= 10 ** (-1.5 / 20) / np.abs(resample_poly(mix, 4, 1, axis=0)).max()   # true-peak ceiling -1.5 dBTP
os.makedirs(OUT, exist_ok=True)
wav = os.path.join(OUT, "flight_mix.wav")
sf.write(wav, mix, SR, subtype="PCM_24")
json.dump({"fps": FPS, "events": [{"t": round(t, 3), "kind": "contact" if c else "stop"} for _, t, c in moves]
           + [{"t": round(t_of(f), 3), "kind": "cut"} for f in TL["cuts"]]},
          open(os.path.join(OUT, "cues.json"), "w"), indent=1)
print("wrote", wav)
if len(sys.argv) > 1:
    vid = sys.argv[1]
    dst = vid.replace(".mp4", "_sound.mp4")
    os.system(f'ffmpeg -y -v error -i "{vid}" -i "{wav}" -c:v copy -c:a aac -b:a 256k -shortest "{dst}"')
    print("wrote", dst)
