"""Parse orchestrator task scripts and resolve named poses."""

import json
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import yaml


DRY_RUN_SUPPORTED_TYPES = {"moveto", "end_effector"}
SUPPORTED_TASK_TYPES = {
    "moveto",
    "end_effector",
    "tool_exchange",
    "vision_task",
    "vision_moveto",
    "vision_scan",
    "pick_sample",
    "place_sample",
    "pick_spincoater",
    "place_spincoater",
    "pipettor",
}


def _load_poses_registry(
    poses_file: str, on_warning: Callable[[str], None] | None
) -> dict[str, Any]:
    path = Path(poses_file)
    if not path.is_file():
        return {}
    try:
        with path.open() as stream:
            return yaml.safe_load(stream) or {}
    except Exception as error:  # noqa: BLE001 - unavailable registry is non-fatal
        if on_warning:
            on_warning(f"Failed to read poses registry: {error}")
        return {}


def parse_task_script(
    full_json: str,
    *,
    dry_run: bool,
    grippers: Mapping[str, Any],
    poses_file: str,
    on_info: Callable[[str], None] | None = None,
    on_warning: Callable[[str], None] | None = None,
) -> tuple[str, list[dict[str, Any]], str, bool]:
    """Parse task JSON and resolve missing named poses from YAML."""
    if not full_json:
        raise ValueError("Goal missing required full_json")

    try:
        script = json.loads(full_json)
    except json.JSONDecodeError as error:
        raise ValueError(f"Invalid JSON: {error}") from error

    if not isinstance(script, Mapping):
        raise ValueError("Task script root must be an object")
    if "start_gripper" not in script:
        raise ValueError("Task script missing 'start_gripper'")
    if "tasks" not in script:
        raise ValueError("Task script missing 'tasks'")

    start_gripper = script["start_gripper"]
    if not isinstance(start_gripper, str):
        raise ValueError("start_gripper must be a string")
    if start_gripper not in grippers:
        available = ", ".join(grippers)
        raise ValueError(f"Unknown gripper: {start_gripper} (available: {available})")

    tasks = script["tasks"]
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("tasks must be a non-empty list")
    allowed = ", ".join(sorted(SUPPORTED_TASK_TYPES))
    for index, task in enumerate(tasks):
        if not isinstance(task, Mapping):
            raise ValueError(f"tasks[{index}] must be an object")
        task_type = task.get("task_type")
        if not isinstance(task_type, str) or task_type not in SUPPORTED_TASK_TYPES:
            raise ValueError(f"tasks[{index}].task_type must be one of: {allowed}")

    if dry_run:
        unsupported = [
            (index, task.get("task_type", "?"))
            for index, task in enumerate(tasks)
            if task.get("task_type", "") not in DRY_RUN_SUPPORTED_TYPES
        ]
        if unsupported:
            bad = ", ".join(
                f"step {index + 1} ({task_type})" for index, task_type in unsupported
            )
            allowed = ", ".join(sorted(DRY_RUN_SUPPORTED_TYPES))
            raise ValueError(
                f"Dry-run not supported for: {bad}. "
                f"v1 supports only: {allowed}. "
                "Run without dry_run (or with use_mock_hardware) for full task types."
            )

    poses = script.get("poses", {})
    pose_keys_needed = set()
    for task in tasks:
        target = task.get("target", "")
        if target and target not in poses:
            pose_keys_needed.add(target)
        for key in task.get("scan_positions", []):
            if key not in poses:
                pose_keys_needed.add(key)
        for field in (
            "scan_pose",
            "approach_pose",
            "target_pose",
            "place_pose",
            "pickup_pose",
        ):
            value = task.get(field)
            if value and value not in poses:
                pose_keys_needed.add(value)

    if pose_keys_needed:
        registry = _load_poses_registry(poses_file, on_warning)
        resolved = [key for key in pose_keys_needed if key in registry]
        poses.update((key, registry[key]) for key in resolved)
        if resolved and on_info:
            on_info(f"Auto-resolved {len(resolved)} pose(s) from registry: {resolved}")

    return start_gripper, tasks, json.dumps(poses), dry_run
