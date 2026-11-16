"""Robot links (ABB STEP) and part B (FH STEP) as STL for MuJoCo. Run with FreeCAD: fc.sh export_meshes.py
Both are third-party data, so meshes/ is gitignored. Gripper meshes come from 02_CAD_Body/out/stl (build.py)."""
import os
import sys

import Part

HERE = os.path.dirname(os.path.abspath(sys.argv[0]))   # fc.sh passes the script path, no __file__
sys.path.insert(0, os.path.join(HERE, "..", "06_Cell"))
import cell_params as CP  # noqa: E402

OUT = os.path.join(HERE, "meshes")
os.makedirs(OUT, exist_ok=True)
for i, n in enumerate(CP.ROBOT["links"]):
    s = Part.read(CP.ROBOT["files"].format(n))
    m = s.tessellate(0.8)                                 # 0.8 mm chord: smooth enough to look at, small files
    import Mesh
    Mesh.Mesh([[m[0][a], m[0][b], m[0][c]] for a, b, c in m[1]]).write(os.path.join(OUT, f"robot_link{i}.stl"))
    bb = s.BoundBox
    print(f"link{i}: {len(m[1])} tris, x {bb.XMin:.0f}..{bb.XMax:.0f} z {bb.ZMin:.0f}..{bb.ZMax:.0f}")

part_b = os.path.join(HERE, "..", "..", "3D-Daten der Bauteile-20260904", "Bauteil_B.STEP")
s = Part.read(part_b)
m = s.tessellate(0.3)
import Mesh  # noqa: E402
Mesh.Mesh([[m[0][a], m[0][b], m[0][c]] for a, b, c in m[1]]).write(os.path.join(OUT, "part_b.stl"))
print("part_b:", round(s.Volume), "mm3", s.BoundBox)
