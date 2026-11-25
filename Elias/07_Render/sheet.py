# Contact sheet of review stills: python sheet.py <folder> [cols]  ->  <folder>.png with the frame number on each tile
import os
import sys

from PIL import Image, ImageDraw

folder = sys.argv[1].rstrip("/\\")
cols = int(sys.argv[2]) if len(sys.argv) > 2 else 3
files = sorted(f for f in os.listdir(folder) if f.endswith(".png"))
ims = [Image.open(os.path.join(folder, f)).convert("RGB") for f in files]
w, h = ims[0].size
rows = (len(ims) + cols - 1) // cols
sheet = Image.new("RGB", (cols * w, rows * h), (20, 20, 20))
for i, (f, im) in enumerate(zip(files, ims)):
    x, y = (i % cols) * w, (i // cols) * h
    sheet.paste(im, (x, y))
    ImageDraw.Draw(sheet).text((x + 8, y + 6), f.split("_")[-1].split(".")[0], fill=(255, 200, 0))
sheet.save(folder + ".png")
print(folder + ".png")
