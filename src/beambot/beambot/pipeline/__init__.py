"""Unified vision-task pipeline (issue #88).

One pipeline — settle -> move-to-scan -> DETECT -> COMPUTE-GOAL -> EXECUTE ->
terminal — with two name-keyed plugin registries (detectors, goal_computers)
and ONE motion executor that dispatches on a closed MotionTarget union.

Importing this package registers the built-in plugins (their decorators fire
on import). The server imports `beambot.pipeline` explicitly so a missing
import surfaces as a loud unknown-key error, not a silent no-op.
"""

from beambot.pipeline.motion_target import (
    CartesianTarget,
    JointTarget,
    snap_j6,
)
from beambot.pipeline.registry import (
    DETECTORS,
    GOAL_COMPUTERS,
    get_detector,
    get_goal_computer,
    register_detector,
    register_goal_computer,
)

# Import plugin modules for their registration side effects.
from beambot.pipeline import detectors as _detectors  # noqa: F401
from beambot.pipeline import goal_computers as _goal_computers  # noqa: F401

__all__ = [
    "CartesianTarget",
    "JointTarget",
    "snap_j6",
    "DETECTORS",
    "GOAL_COMPUTERS",
    "get_detector",
    "get_goal_computer",
    "register_detector",
    "register_goal_computer",
]
