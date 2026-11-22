# Music for the flight preview: a licensed library track (Mixkit Stock Music Free License: free in videos, no
# attribution, not redistributable on its own), trimmed, faded with the picture and set to -14.5 LUFS, true peak -3 dBTP before AAC (the sources clip, AAC overshoots).
# python music_flight.py <mixkit id> <start s> <tag>   ->  out/flight_preview_<tag>.mp4
import json, os, subprocess, sys, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
mid, start, tag = sys.argv[1], float(sys.argv[2]), sys.argv[3]
TL = json.load(open(os.path.join(OUT, "flight_timeline.json")))
length = TL["end"] / TL["fps"]
fade0 = (TL["end_fade"][0] - 1) / TL["fps"]

src = os.path.join(OUT, "music", f"mixkit_{mid}.mp3")
if not os.path.exists(src):
    os.makedirs(os.path.dirname(src), exist_ok=True)
    req = urllib.request.Request(f"https://assets.mixkit.co/music/{mid}/{mid}.mp3", headers={"User-Agent": "Mozilla/5.0"})
    open(src, "wb").write(urllib.request.urlopen(req).read())

af = (f"atrim={start}:{start + length},asetpts=PTS-STARTPTS,afade=t=in:d=0.4,"
      f"afade=t=out:st={fade0 - 0.5:.2f}:d={length - fade0 + 0.5:.2f},loudnorm=I=-14.5:TP=-3:LRA=11")
dst = os.path.join(OUT, f"flight_preview_{tag}.mp4")
subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", os.path.join(OUT, "flight_preview.mp4"), "-i", src, "-filter:a", af,
                "-map", "0:v", "-map", "1:a", "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-ar", "48000",
                "-shortest", dst], check=True)
print("wrote", dst)
