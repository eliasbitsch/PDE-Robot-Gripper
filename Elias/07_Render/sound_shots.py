# Sound for the xray, grid and robot previews: cinematic only (review 25 Nov: whir, ticks and noise sounded like a drill),
# effects from earshot's bank (made in code, not recordings), sub drone and the accelerating pulse made here; every cue comes from the shot's timeline (hero_v1.py writes them).
# run with earshot's venv:  C:/git/_ext/earshot/.venv/Scripts/python.exe sound_shots.py xray|grid|robot [video.mp4]
import json, os, sys
import numpy as np
import soundfile as sf
from scipy.signal import butter, resample_poly, sosfilt
from earshot.forge import bank

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "sound")
SHOT = sys.argv[1]
TL = json.load(open(os.path.join(HERE, "out", f"{SHOT}_timeline.json")))
KIN = json.load(open(os.path.join(HERE, "..", "02_CAD_Body", "out", "kinematics.json")))
SR, FPS = 48000, TL["fps"]
LEN = TL["end"] / FPS
N = int(SR * LEN)
T = np.arange(N) / SR
mix = np.zeros((N, 2))
rng = np.random.default_rng(11)
cues = []


def t_of(frame):
    return (frame - 1) / FPS


def place(y, t_onset, gain_db, align=0.0, pan=0.0):
    y = y if y.ndim == 2 else np.stack([y, y], 1)
    y = y * np.array([min(1.0, 1 - pan), min(1.0, 1 + pan)])
    i = int(round((t_onset - align) * SR))
    a, b = max(i, 0), min(i + len(y), N)
    if b > a:
        mix[a:b] += y[a - i:b - i] * 10 ** (gain_db / 20)


def sfx(sid, t, gain_db, at="onset", pan=0.0, kind=None, **kn):
    y, m = bank.render(sid, checks=False, **kn)
    place(y, t, gain_db, align=m["peak_s"] if at == "peak" else m["onset_s"], pan=pan)
    if kind:
        cues.append({"t": round(t, 3), "kind": kind})


def lp(x, hz, order=4):
    return sosfilt(butter(order, hz, "lowpass", fs=SR, output="sos"), x)


def bp(x, lo, hi):
    return sosfilt(butter(2, [lo, hi], "bandpass", fs=SR, output="sos"), x)


def drone(gain_db, swell):
    """sub bed in D: D1 + D2 + A2 sines with slow beating, plus dark filtered air; swell(T) shapes it"""
    y = (np.sin(2 * np.pi * 36.71 * T) + 0.6 * np.sin(2 * np.pi * 73.42 * T + 1) * (0.8 + 0.2 * np.sin(2 * np.pi * 0.13 * T))
         + 0.3 * np.sin(2 * np.pi * 110.0 * T + 2) * (0.7 + 0.3 * np.sin(2 * np.pi * 0.21 * T)))
    y *= swell * np.clip(T / 1.2, 0, 1) * np.clip((LEN - T) / 0.8, 0, 1)
    place(np.stack([y, np.roll(y, 40)], 1), 0.0, gain_db)


def clack(t, big=False, gain=0.0):
    """jaws on the part: layered metal clack"""
    sfx("pro_pop_in/deep", t, -12 + gain)
    sfx("pro_impact/deep" if big else "pro_impact/short", t, (-6 if big else -11) + gain, kind="contact")
    if big:
        sfx("pro_sub_drop/soft", t, -12 + gain)


def jaw_moves(spans):
    for k, (f0, f1, a, b) in enumerate(spans):
        t0, t1, closing = t_of(f0), t_of(f1), b < a
        d = min(t1 - t0, 0.9)
        sfx("swipe/velvet", t1, -20, at="peak", dur=d + 0.3, apex=d)
        if closing:
            clack(t1, big=k == len(spans) - 1)
        else:
            sfx("pro_pop_in/airy", t1, -20, kind="stop")


if SHOT == "xray":
    w0, w1 = t_of(TL["wipe"][0]), t_of(TL["wipe"][1])
    drone(-16, 0.6 + 0.4 * np.clip((T - w0) / 2, 0, 1))
    sfx("pro_impact/deep", 0.15, -8, kind="open")
    sfx("whoosh_big/deep", 0.5 * (t_of(1) + w0), -20, at="peak", dur=2.0, apex=1.1)   # the push-in
    sfx("pro_whoosh_big/reverse", w0, -15, at="peak", dur=1.2, apex=0.9)
    sfx("pro_whoosh_big/tonal", 0.5 * (w0 + w1), -15, at="peak", dur=w1 - w0, apex=(w1 - w0) * 0.55, kind="wipe")
    sfx("shimmer/m1", w0 + 0.2, -19)
    sfx("pro_shimmer/m0", w1 - 0.3, -18)
    sfx("pro_impact/air", w1, -14, kind="wipe_end")
    jaw_moves(TL["jaws"])
elif SHOT == "grid":
    land, fly0 = t_of(TL["land"]), t_of(TL["fly0"])
    drone(-17, 0.55 + 0.45 * np.clip((T - fly0) / (land - fly0), 0, 1))
    for k, f in enumerate(TL["drops"]):                # every part lands in the grid: small taps, a little random
        if k % 2 == 0:
            sfx(("pro_pop_in/round", "pro_pop_in/airy")[k // 2 % 2], t_of(f + 6), -26 + rng.uniform(-3, 2),
                pan=rng.uniform(-0.4, 0.4))
    sfx("whoosh_big/swell", t_of(TL["fly0"] - 15), -16, at="peak", dur=1.6, apex=0.9, kind="tilt")
    sfx("riser/shepard", land, -24, at="peak", dur=min(land - fly0, 3.6))
    for k, (f0, f1, screw, size) in enumerate(TL["flights"]):
        pan = rng.uniform(-0.5, 0.5)
        big = size > 0.06
        if big or k % 2 == 0:                          # not every screw needs its own swish
            sfx("swish/blade" if not big else "whoosh_fast/tight", t_of((f0 + f1) / 2), -24 if big else -31,
                at="peak", pan=pan)
        if big:
            sfx("pro_pop_in/deep", t_of(f1), -16, pan=pan)
            sfx("pro_impact/short", t_of(f1), -16)
    sfx("pro_impact/deep", land, -6, kind="assembled")
    sfx("sub_drop/pro", land, -12)
    sfx("pro_shimmer/m4", land + 0.1, -21)
elif SHOT == "robot":
    sp = np.array(TL["speed"])
    pk0, pk1, back = (t_of(f) for f in TL["pick"])
    drone(-15, 0.5 + 0.5 * np.clip((T - pk0) / 2, 0, 1) * np.clip((t_of(TL["pick"][2]) + 1 - T) / 1.5, 0.3, 1))
    # the gripper flies onto the flange and snaps on
    sfx("whoosh_fast/heavy", t_of(TL["seat"]), -12, at="peak")
    sfx("impact/classic", t_of(TL["seat"]), -8, kind="seat")
    # cable draws, plug goes on, nut turned home (two turns of ratchet)
    sfx("swipe/glide", t_of(TL["cable"][1]), -20, at="peak", dur=1.0, apex=0.6)
    sfx("pro_pop_in/deep", t_of(TL["plug"]), -12, kind="plug")
    for f in TL["dip_cuts"]:
        sfx("whoosh_fast/reverse", t_of(f), -16, at="peak")
        sfx("pro_impact/air", t_of(f) + 0.1, -15, kind="cut")
    r0, r1 = (t_of(f) for f in TL["rise"])
    sfx("whoosh_big/deep", 0.5 * (r0 + r1), -12, at="peak", dur=r1 - r0, apex=(r1 - r0) * 0.6, kind="rise")
    sfx("pro_sub_drop/low", r1, -12)
    for f in TL["blinks"]:
        sfx("pro_pop_in/round", t_of(f + 8), -18)
    # pick-and-place: a kick on every cycle start and hats between them, so the pulse speeds up with the robot
    starts, cyc = TL["cycle_starts"], TL["cycles"]
    for k, (s0, Tc) in enumerate(zip(starts, cyc)):
        sfx("impact/punch", t_of(s0), -9 + 0.4 * k)
        sfx("sub_drop/punch", t_of(s0), -14)
        for j in (1, 2, 3):
            if j == 2:                                   # off-beat: a short sub pulse instead of a hat
                sfx("pro_impact/air", t_of(s0 + j * Tc / 4), -19)
    for k, f in enumerate(TL["jump_cuts"][1:-1]):
        sfx(("whoosh_fast/tight", "whoosh_fast/crisp", "swish/zing")[k % 3], t_of(f), -13, at="peak",
            pan=(-0.3, 0.3)[k % 2], kind="cut")
    sfx("whoosh_fast/heavy", t_of(TL["jump_cuts"][0]), -10, at="peak", kind="cut")
    for k, f in enumerate(TL["grips"]):
        clack(t_of(f), gain=-2)
    for f in TL["releases"]:
        sfx("pro_pop_in/round", t_of(f), -13, kind="release")

    sfx("riser/tension", pk1, -16, at="peak", dur=2.6)
    sfx("impact/deep", pk1, -5, kind="final")
    sfx("pro_impact/rumble", pk1, -12)
    sfx("logo_sting/soft_bloom", back, -18)
    jc = TL["jump_cuts"][-1]
    sfx("whoosh_big/swell", t_of(jc), -14, at="peak")

# master: high-pass, limiter, true-peak ceiling -1.5 dBTP before AAC
mix = sosfilt(butter(2, 30, "highpass", fs=SR, output="sos"), mix, axis=0)
mix *= np.clip((LEN - T) / 0.3, 0, 1)[:, None]
mix /= np.abs(mix).max()
DRIVE = 2.4
mix = np.tanh(mix * DRIVE) / np.tanh(DRIVE)
mix = lp(mix.T, 16000, 6).T
mix *= 10 ** (-1.5 / 20) / np.abs(resample_poly(mix, 4, 1, axis=0)).max()
os.makedirs(OUT, exist_ok=True)
wav = os.path.join(OUT, f"{SHOT}_mix.wav")
sf.write(wav, mix, SR, subtype="PCM_24")
json.dump({"fps": FPS, "events": sorted(cues, key=lambda c: c["t"])}, open(os.path.join(OUT, f"{SHOT}_cues.json"), "w"), indent=1)
print("wrote", wav)
if len(sys.argv) > 2:
    vid = sys.argv[2]
    dst = vid.replace(".mp4", "_sound.mp4")
    os.system(f'ffmpeg -y -v error -i "{vid}" -i "{wav}" -c:v copy -c:a aac -b:a 256k -shortest "{dst}"')
    print("wrote", dst)
