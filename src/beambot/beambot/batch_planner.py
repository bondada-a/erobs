"""Batch planner -- groups consecutive batchable tasks for optimized execution.

Simple tasks (moveto, end_effector) can be grouped into a single MTC Task
with multiple stages, reducing planning overhead (~1.5s per task saved).

Tasks that require runtime decisions (vision, tool exchange, pipettor) are
batch breakers and always execute individually via their action servers.
"""

from typing import Dict, Any, List, Tuple

# Task types that can be batched into a single MTC Task.
# These support the add_to_task() pattern for stage composition.
BATCHABLE_TYPES = {"moveto", "end_effector"}

# Task types that break batching (require runtime decisions or special handling).
# - tool_exchange: Changes robot kinematics (requires MoveIt restart)
# - vision_moveto/pick_sample/place_sample: Require runtime marker detection
# - vision_scan: Batch scans all markers (robot moves during execution)
# - pipettor: Uses separate action server (not MTC-based)
BATCH_BREAKERS = {"tool_exchange", "vision_moveto", "vision_scan", "pick_sample", "place_sample", "pipettor"}


def group_into_batches(
    tasks: List[Dict[str, Any]],
    enabled: bool = True,
) -> List[Tuple[str, List[Dict[str, Any]]]]:
    """Group consecutive batchable tasks into batches.

    Simple tasks (moveto, end_effector) are grouped together.
    All other tasks become single-task batches and execute via
    their respective action servers.

    Args:
        tasks: List of task dictionaries from the JSON script.
        enabled: When False, every task becomes a single-task batch
                 (disables batching optimization entirely).

    Returns:
        List of (batch_type, tasks) tuples where:
        - batch_type: "batched" for batchable tasks, "single" otherwise
        - tasks: List of task dicts in this batch
    """
    if not enabled:
        return [("single", [task]) for task in tasks]

    batches = []
    current_batch: List[Dict[str, Any]] = []
    current_type = None

    for task in tasks:
        task_type = task.get("task_type", "")

        if task_type in BATCHABLE_TYPES:
            if current_type == "batched":
                current_batch.append(task)
            else:
                if current_batch:
                    batches.append((current_type, current_batch))
                current_batch = [task]
                current_type = "batched"
        else:
            if current_batch:
                batches.append((current_type, current_batch))
            batches.append(("single", [task]))
            current_batch = []
            current_type = None

    if current_batch:
        batches.append((current_type, current_batch))

    return batches
