"""Hero film: frames -> opening and closing mp4 with sound. python finalize_hero.py <frame-dir prefix, e.g. eevee_>

opening = flight + assemble, sound out/sound/opening_mix_private.wav (private music: class presentation only)
closing = explode + robot, sound out/sound/closing_mix.wav (effects only)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
PRE = sys.argv[1] if len(sys.argv) > 1 else "eevee_"
FF = ["ffmpeg", "-y", "-loglevel", "error"]


def scene(name):
    d = os.path.join(OUT, PRE + name)
    dst = os.path.join(OUT, f"_{PRE}{name}.mp4")
    subprocess.run(FF + ["-framerate", "30", "-i", os.path.join(d, "f_%04d.png"), "-c:v", "libx264", "-crf", "17",
                         "-preset", "medium", "-pix_fmt", "yuv420p", dst], check=True)
    return dst


def film(parts, sound, dst):
    lst = os.path.join(OUT, "_concat.txt")
    open(lst, "w").write("".join(f"file '{p}'\n" for p in parts))
    tmp = dst + ".video.mp4"
    subprocess.run(FF + ["-f", "concat", "-safe", "0", "-i", lst, "-c", "copy", tmp], check=True)
    subprocess.run(FF + ["-i", tmp, "-i", sound, "-map", "0:v", "-map", "1:a", "-c:v", "copy", "-c:a", "aac", "-b:a", "256k",
                         "-shortest", dst], check=True)
    os.remove(tmp)
    print("wrote", dst)


if __name__ == "__main__":
    scenes = {n: scene(n) for n in ("flight", "assemble", "explode", "robot")}
    film([scenes["flight"], scenes["assemble"]], os.path.join(OUT, "sound", "opening_mix_private.wav"),
         os.path.join(OUT, "hero_1_opening_1080p_PRIVATE.mp4"))
    film([scenes["explode"], scenes["robot"]], os.path.join(OUT, "sound", "closing_mix.wav"),
         os.path.join(OUT, "hero_2_abschluss_1080p.mp4"))
