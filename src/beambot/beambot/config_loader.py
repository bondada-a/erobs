"""Shared loader for the beamline YAML config.

Single source of truth: the BEAMBOT_BEAMLINE_CONFIG environment variable.
Set it to the absolute path of your beamline's YAML before launching anything
that touches the robot.

Refusing to default-load avoids the silent-CMS-fallback failure mode: a
deployment at a different beamline must declare itself, or nothing starts.
"""

import os
import threading
from typing import Tuple

import yaml


_ENV_VAR = "BEAMBOT_BEAMLINE_CONFIG"

# Process-lifetime memo of the parsed beamline config. The beamline is selected
# once at startup (via BEAMBOT_BEAMLINE_CONFIG) and never changes mid-run, so we
# parse the YAML exactly once and hand the same (dict, path) back to every
# caller. This is load-bearing for performance: helpers like arm_joint_names()
# are reached from ~500 Hz /joint_states callbacks; re-parsing the YAML on every
# access thrashed the GIL and starved MTC planning (the goal->motion latency
# bug). Double-checked locking mirrors spincoater._get_sample_model.
#
# The cached dict is SHARED — treat it as READ-ONLY. Mutating it (or any of its
# sub-dicts) leaks into every other caller in the process. The one historical
# runtime mutation (the cup_profile override) now lives on the orchestrator /
# MoveItLifecycleManager instead of being written back into this dict.
_config_cache: Tuple[dict, str] | None = None
_config_cache_lock = threading.Lock()


class BeamlineConfigError(RuntimeError):
    """Raised when the beamline config cannot be located or parsed."""


def get_beamline_config_path() -> str:
    """Return the absolute path to the beamline YAML, or raise.

    Reads the BEAMBOT_BEAMLINE_CONFIG environment variable. Errors out with
    an actionable message if unset or pointing at a missing file — the robot
    must not start without an explicit beamline declaration.
    """
    raw = os.environ.get(_ENV_VAR, "").strip()
    if not raw:
        raise BeamlineConfigError(
            f"{_ENV_VAR} environment variable is not set.\n"
            f"Set it to the absolute path of your beamline's YAML config, e.g.:\n"
            f"    export {_ENV_VAR}=/path/to/your_beamline.yaml\n"
            f"For CMS:\n"
            f"    export {_ENV_VAR}=$(realpath src/beambot/config/cms_beamline.yaml)"
        )

    path = os.path.abspath(os.path.expanduser(raw))
    if not os.path.isfile(path):
        raise BeamlineConfigError(
            f"{_ENV_VAR} points at a file that does not exist: {path}"
        )
    return path


def load_beamline_config() -> Tuple[dict, str]:
    """Return (parsed_yaml_dict, absolute_path_loaded_from), parsed once.

    The YAML is parsed on the first call and memoized for the process lifetime
    (see _config_cache). Every later call — from any of the ~30 helpers below,
    at any frequency — returns the same cached objects without touching disk.

    The path is returned alongside the dict so callers can resolve sibling
    paths declared inside the YAML (poses_file, scene_file) relative to it.

    IMPORTANT: the returned dict is shared and must be treated as READ-ONLY.

    Failures are NOT cached: if the env var is unset or the file is unreadable
    the exception propagates and _config_cache stays None, so a later call (once
    the env is fixed) retries cleanly. This preserves the try/except fallbacks
    in arm_joint_names() and friends.
    """
    global _config_cache
    if _config_cache is None:
        with _config_cache_lock:
            if _config_cache is None:
                _config_cache = _load_beamline_config_uncached()
    return _config_cache


def _load_beamline_config_uncached() -> Tuple[dict, str]:
    """Read and parse the beamline YAML from disk (no caching)."""
    path = get_beamline_config_path()
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        raise BeamlineConfigError(f"Failed to parse {path}: {e}") from e
    if not isinstance(data, dict):
        raise BeamlineConfigError(f"{path}: expected a YAML mapping at root")
    return data, path


def reset_beamline_config_cache() -> None:
    """Clear the memoized config so the next load() re-parses from disk.

    For test isolation (a test that points the env var at a different YAML) or
    an explicit in-process reload. Not used on the normal robot path — the
    beamline is fixed for the process lifetime.
    """
    global _config_cache
    with _config_cache_lock:
        _config_cache = None


_DEFAULT_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def arm_joint_names() -> list[str]:
    """Ordered list of arm joint names from the active beamline YAML.

    Order is significant: pose values in the registry are positional and
    must align with this list. Falls back to the standard UR 6-DOF order
    if the YAML is unset (test paths, isolated module imports).
    """
    try:
        config, _ = load_beamline_config()
        joints = config.get("robot", {}).get("arm_joints")
        if joints:
            return list(joints)
    except Exception:
        pass
    return list(_DEFAULT_ARM_JOINTS)


def gripper_tip_frame(gripper: str, default: str = "flange") -> str:
    """Return the TF tip frame for a configured gripper.

    Reads `grippers.<name>.tip_frame` from the active beamline YAML. Returns
    `default` on any read failure (missing env var, missing key, etc.) — the
    fallback "flange" is the safe choice because IK still resolves at the
    arm flange when no gripper is detected.
    """
    try:
        config, _ = load_beamline_config()
        return config.get("grippers", {}).get(gripper, {}).get("tip_frame", default)
    except Exception:
        return default


def gripper_z_offset(gripper: str, default: float = 0.0) -> float:
    """Return the default Z offset (meters) for a configured gripper.

    Used by vision approach to push or pull the IK target along the gripper's
    Z axis (e.g. -0.02 for Hand-E to clear finger thickness).
    """
    try:
        config, _ = load_beamline_config()
        val = config.get("grippers", {}).get(gripper, {}).get("z_offset", default)
        return float(val)
    except Exception:
        return default


def z_offset_for_tip_frame(tip_frame: str, default: float = 0.0) -> float:
    """Return the z_offset associated with a tip frame.

    Looks across grippers.* entries for one whose tip_frame matches,
    then returns its z_offset. The vision pipeline detects the active
    gripper by probing TF for one of these frames, so the lookup is
    naturally tip-frame-keyed even though the YAML keys grippers by
    name.
    """
    try:
        config, _ = load_beamline_config()
        for gconf in config.get("grippers", {}).values():
            if gconf.get("tip_frame") == tip_frame:
                return float(gconf.get("z_offset", default))
        return default
    except Exception:
        return default


def configured_tip_frames() -> list[str]:
    """All gripper tip frames declared in the active beamline YAML.

    Used by stages that auto-detect the active gripper by probing TF for
    one of the known tip frames. Excludes "flange" (the no-gripper case)
    since detection there means "no gripper attached" — the caller falls
    back to flange explicitly when no tip frame matches.
    """
    try:
        config, _ = load_beamline_config()
        frames = []
        for gconf in config.get("grippers", {}).values():
            tf = gconf.get("tip_frame")
            if tf and tf != "flange" and tf not in frames:
                frames.append(tf)
        return frames
    except Exception:
        return []


def moveit_config_package(default: str = "cms_moveit_config") -> str:
    """Return the MoveIt config package name from the active beamline YAML.

    Used by stages that load joint_limits.yaml across gripper configs.
    Falls back to the provided default so module-level imports succeed
    in test paths where the env var is unset.
    """
    try:
        config, _ = load_beamline_config()
        return config.get("robot", {}).get("moveit_config_package", default)
    except Exception:
        return default


def description_package(default: str = "cms_robot_description") -> str:
    """Return the robot description package from the active beamline YAML.

    Used by the GUI 3D viewer to locate URDF/mesh resources. Falls back
    to the provided default when the env var is unset.
    """
    try:
        config, _ = load_beamline_config()
        return config.get("robot", {}).get("description_package", default)
    except Exception:
        return default


def gripper_urdf_file(gripper: str, default: str = "ur_standalone.urdf") -> str:
    """Return the URDF filename for a gripper as used by the GUI 3D viewer.

    The viewer loads `<urdf_file>` from the URDF source dir to render the
    arm + tool together. Per-beamline because mount layout differs.
    """
    try:
        config, _ = load_beamline_config()
        return config.get("grippers", {}).get(gripper, {}).get("urdf_file", default)
    except Exception:
        return default


def resolve_beamline_path(rel_or_abs: str, config_path: str) -> str:
    """Resolve a path declared inside a beamline YAML.

    Absolute paths are returned as-is. Relative paths are walked upward from
    the config's directory until the workspace root is found (a directory
    containing 'src/'), then joined to the relative path. Falls back to
    joining against the config's directory if no workspace root is found,
    so configs outside a colcon workspace still work.
    """
    if not rel_or_abs:
        return ""
    expanded = os.path.expanduser(rel_or_abs)
    if os.path.isabs(expanded):
        return os.path.abspath(expanded)

    config_dir = os.path.dirname(os.path.realpath(config_path))
    candidate = config_dir
    for _ in range(10):
        if os.path.isdir(os.path.join(candidate, "src")):
            return os.path.join(candidate, expanded)
        parent = os.path.dirname(candidate)
        if parent == candidate:
            break
        candidate = parent

    return os.path.join(config_dir, expanded)
