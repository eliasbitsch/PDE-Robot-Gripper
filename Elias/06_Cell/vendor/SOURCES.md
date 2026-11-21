# Vendor CAD (not committed)

| Folder / file | Content | Source |
|---|---|---|
| `irb1300/` | ABB IRB 1300 Type A 7&12-1400, OmniCore, STEP per joint | abb.com IRB 1300 page, "CAD Models, STEP ... joint" (9AKK107992A7467) |
| `irb1300_datasheet.pdf` | IRB 1300 data sheet | 9AKK107991A0383 |
| `irb1600/` | ABB IRB 1600 X-145, IRC5, STEP per joint (comparison) | abb.com IRB 1600 page (9AKK107045A3821) |
| `simready/` | NVIDIA SimReady assets `warehouse_conveyorBelt_a06` (flat belt, adapted by `simready/derived.py`), `wire_machineGuard_a01/a03/a04`, `environment_floor_a01`: looks of the cell in MuJoCo (belt, fence, floor); `conveyor_belt_a05`, `warehouse_conveyorBelt_a29`, `digitalSystem_touchTerminal_standing_a01` downloaded but unused. CC-BY-4.0, (c) NVIDIA. Get them with `simready/fetch.py`, convert with `simready/usd2obj.py` (Blender) | https://huggingface.co/datasets/nvidia/simready-assets |
