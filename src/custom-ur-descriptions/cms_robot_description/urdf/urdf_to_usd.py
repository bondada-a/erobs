#!/usr/bin/env python3
"""Headless URDF -> USD batch conversion for Isaac Sim 4.5.0.

Run with Isaac Sim's bundled python, NOT system python:
    ~/isaacsim/python.sh urdf_to_usd.py

Converts every *_isaac.urdf next to this script into a .usd under usd/.
Import settings mirror docs/isaac_sim_integration.md (fix base, stiffness
drive, self-collision on, collisions-from-visuals on). Joint-drive gains are
NOT applied here — see isaac_sim_joint_params.yaml for that pass.
"""
import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})  # must come before omni imports

import omni.kit.commands  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from pxr import Usd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "usd")
os.makedirs(OUT, exist_ok=True)

cfg = _urdf.ImportConfig()
cfg.fix_base = True                 # anchor to world (docs: Fix Base Link ON)
cfg.merge_fixed_joints = True       # collapse frame-only links (zivid_optical_frame, map)
cfg.import_inertia_tensor = True
cfg.self_collision = True           # docs: self-collision enabled
cfg.convex_decomp = False
cfg.make_default_prim = True
cfg.create_physics_scene = True
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
cfg.distance_scale = 1.0            # URDF is already in meters

urdfs = sorted(glob.glob(os.path.join(HERE, "*_isaac.urdf")))
print(f"[urdf_to_usd] {len(urdfs)} configs -> {OUT}")

for path in urdfs:
    name = os.path.splitext(os.path.basename(path))[0]
    dest = os.path.join(OUT, f"{name}.usd")
    # parse_urdf returns (status, robot_model); import_robot writes the stage
    status, robot = omni.kit.commands.execute(
        "URDFParseFile", urdf_path=path, import_config=cfg
    )
    omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=path,
        urdf_robot=robot,
        import_config=cfg,
        dest_path=dest,
    )
    ok = os.path.exists(dest) and Usd.Stage.Open(dest)
    print(f"  {'OK ' if ok else 'FAIL'}  {name}.usd")

app.close()
