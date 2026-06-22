"""Plugin registries for the vision-task pipeline (issue #88).

Two module-level dicts populated by decorators. A detector or goal_computer is
added by writing one function and decorating it — zero handler edits, zero
.action edits. Unknown keys fail loudly with the available list (the soft-schema
footgun the design review flagged: a typo must not silently pick the wrong
plugin or no-op).

A Detector is      detect(ctx) -> detection (PoseStamped for marker/sample_roi).
A GoalComputer is  compute(detection, ctx) -> MotionTarget | None
                   (None = detect_only short-circuit / cache-only).
`ctx` is the VisionTaskContext the server builds (see vision_task_stages.py).
"""

DETECTORS: dict = {}
GOAL_COMPUTERS: dict = {}


def register_detector(name: str):
    """Register a detector under `name`. Use as @register_detector("marker")."""

    def deco(fn):
        if name in DETECTORS:
            raise ValueError(f"detector '{name}' already registered")
        DETECTORS[name] = fn
        return fn

    return deco


def register_goal_computer(name: str):
    """Register a goal computer. Use as @register_goal_computer("approach_pose")."""

    def deco(fn):
        if name in GOAL_COMPUTERS:
            raise ValueError(f"goal_computer '{name}' already registered")
        GOAL_COMPUTERS[name] = fn
        return fn

    return deco


def get_detector(name: str):
    """Look up a detector, raising with the available keys on a miss."""
    try:
        return DETECTORS[name]
    except KeyError:
        raise KeyError(
            f"unknown detector '{name}'. Registered: {sorted(DETECTORS)}"
        ) from None


def get_goal_computer(name: str):
    """Look up a goal computer, raising with the available keys on a miss."""
    try:
        return GOAL_COMPUTERS[name]
    except KeyError:
        raise KeyError(
            f"unknown goal_computer '{name}'. Registered: {sorted(GOAL_COMPUTERS)}"
        ) from None
