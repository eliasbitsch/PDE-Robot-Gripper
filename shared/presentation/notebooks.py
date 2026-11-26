"""Start or stop every presentation notebook at once (read-only marimo run, fixed ports) and open an overview page.

python notebooks.py start   (or double-click start_notebooks.cmd)  ->  notebooks.html in the browser
python notebooks.py stop    (or stop_notebooks.cmd)
Start it a few minutes before the talk: each notebook computes once at start, afterwards it opens at once.
"""
import os
import socket
import subprocess
import sys
import time
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
NOTEBOOKS = [  # (port, path, title, what)
    (2741, "shared/presentation/class_presentation.py", "Design journey", "main line of the talk"),
    (2742, "Viktoriia/01_Concept/v1_concept.py", "V1 Concept", "requirements, morphological box, VDI 2225"),
    (2743, "Viktoriia/02_Sizing/v2_sizing.py", "V2 Sizing", "grip force, e-stop, gears, rail, bearings"),
    (2744, "Viktoriia/04_Analytical/v4_jaw.py", "V4 Jaw", "beam proof of the jaw vs FEM"),
    (2745, "Elias/05_FEM/review_viktoriia_notebook.py", "Review V2 / V4", "checked against simulation and FEM"),
    (2746, "Viktoriia/06_Review/review_elias_response.py", "Response", "Viktoriia's answer to the review"),
    (2747, "Elias/04_Simulation/validation_notebook.py", "Validation", "hand calculation vs simulation, robustness"),
    (2748, "Elias/05_FEM/fem_notebook.py", "Jaw FEM", "gmsh + CalculiX, convergence"),
    (2749, "Elias/05_FEM/fem_train_notebook.py", "Jaw train FEM", "slide + holder + jaw"),
    (2750, "Elias/01_Robot/robot_notebook.py", "Robot", "selection, payload, cycle time, placement"),
]


def listening(port):
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


def page():
    rows = "".join(f'<a href="http://localhost:{p}" target="_blank"><b>{t}</b><span>{w}</span><i>:{p}</i></a>'
                   for p, _, t, w in NOTEBOOKS)
    html = f"""<!doctype html><html><head><meta charset="utf-8"><title>Gripper notebooks</title><style>
body{{font:16px system-ui,sans-serif;background:#f6f4ef;color:#1c1917;margin:40px auto;max-width:760px}}
h1{{font-weight:600}} a{{display:grid;grid-template-columns:200px 1fr 60px;gap:12px;padding:14px 18px;margin:8px 0;
background:#fff;border:1px solid #e7e5e4;border-radius:8px;color:inherit;text-decoration:none}}
a:hover{{border-color:#C2410C}} span{{color:#78716c}} i{{color:#a8a29e;font-style:normal;text-align:right}}
</style></head><body><h1>Robot gripper - notebooks</h1>{rows}
<p style="color:#78716c">Started by start_notebooks.cmd; a notebook that is still computing shows a spinner - wait a moment.</p></body></html>"""
    out = os.path.join(HERE, "notebooks.html")
    open(out, "w", encoding="utf-8").write(html)
    return out


def start():
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    for port, path, title, _ in NOTEBOOKS:
        if listening(port):
            continue
        full = os.path.join(ROOT, *path.split("/"))
        subprocess.Popen([sys.executable, "-m", "marimo", "run", full, "--headless", "--port", str(port), "--no-token"],
                         cwd=os.path.dirname(full), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)
        print("starting", title, port)
    for _ in range(60):                                # wait until the servers answer (they compute after that)
        if all(listening(p) for p, *_ in NOTEBOOKS):
            break
        time.sleep(1)
    webbrowser.open(page())


def stop():
    for port, _, title, _ in NOTEBOOKS:
        subprocess.run(["powershell", "-NoProfile", "-Command",
                        f"Get-NetTCPConnection -LocalPort {port} -State Listen -ErrorAction SilentlyContinue | "
                        "ForEach-Object { Stop-Process -Id $_.OwningProcess -Force }"], capture_output=True)
    print("stopped")


if __name__ == "__main__":
    {"start": start, "stop": stop}[sys.argv[1] if len(sys.argv) > 1 else "start"]()
