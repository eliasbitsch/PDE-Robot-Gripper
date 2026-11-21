"""Download the NVIDIA SimReady assets the cell uses (CC-BY-4.0, (c) NVIDIA) into 06_Cell/vendor/simready.

python fetch.py   (needs huggingface_hub; the assets stay out of the repo, see ../vendor/SOURCES.md)
"""
import os

from huggingface_hub import snapshot_download

HERE = os.path.dirname(os.path.abspath(__file__))
DST = os.path.join(HERE, "..", "vendor", "simready")
ASSETS = ["conveyor_belt_a05", "warehouse_conveyorBelt_a06", "warehouse_conveyorBelt_a29", "wire_machineGuard_a01", "wire_machineGuard_a03", "wire_machineGuard_a04",
          "environment_floor_a01", "digitalSystem_touchTerminal_standing_a01"]

if __name__ == "__main__":
    snapshot_download(repo_id="nvidia/simready-assets", repo_type="dataset", local_dir=DST,
                      allow_patterns=[f"{n}/**" for n in ASSETS],
                      ignore_patterns=["**/source_assets/**", "**/.thumbs/**", "**/runnables/**"])
    print("ok", DST)
