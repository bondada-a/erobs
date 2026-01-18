# Coding Conventions

**Analysis Date:** 2026-01-17

## Naming Patterns

**Files:**
- snake_case for all Python files (e.g., `base_action_server.py`, `vision_stages.py`)
- Test files prefixed with `test_` (e.g., `test_contour_detection.py`)
- Launch files: `*_bringup.launch.py` or `*_test.launch.py`

**Functions:**
- snake_case for all functions (e.g., `initialize_stages()`, `create_task_template()`)
- Private methods prefixed with single underscore (e.g., `_execute()`, `_goal_callback()`)
- Callback methods suffixed with `_callback` (e.g., `_execute_callback()`, `_cancel_callback()`)
- No special prefix for async functions

**Variables:**
- snake_case for variables and parameters (e.g., `node_name`, `marker_ids`)
- UPPER_SNAKE_CASE for module constants (e.g., `VELOCITY_SCALING`, `DEFAULT_ARM_GROUP`)
- Private instance variables prefixed with `self._` (e.g., `self._executing`, `self._lock`)

**Types:**
- PascalCase for classes (e.g., `BaseActionServer`, `MTCOrchestratorServer`)
- PascalCase for dataclasses (e.g., `CircleDetectionParams`, `ContourDetectionParams`)
- No I prefix for interfaces (Python convention)

## Code Style

**Formatting:**
- 4 spaces per indentation level (Python standard)
- Double quotes for strings (consistent throughout codebase)
- Line length: Varies by package (88-225+), pragmatically relaxed
- Shebang: `#!/usr/bin/env python3` for executable scripts

**Linting:**
- flake8 with Black-compatible settings (E203, W503 ignored)
- pylint with relaxed complexity thresholds for robotics (max-statements=70)
- Config locations:
  - `src/vision/zivid-python-samples/.flake8`
  - `src/end_effectors/ros2_epick_gripper/.flake8`

**Type Hints:**
- Consistently used for function signatures
- Return types always specified with `->`
- Complex types from `typing` module (List, Dict, Optional, Tuple, Any)

**Example:**
```python
def detect_markers(
    client,
    node: Node,
    marker_ids: List[int],
    dictionary: str = "aruco4x4_50",
    timeout: float = 45.0
) -> List[Tuple[int, Pose]]:
```

## Import Organization

**Order:**
1. Standard library (json, threading, time)
2. Third-party (yaml, numpy, cv2)
3. ROS2 (rclpy, moveit, geometry_msgs)
4. Local/package (beambot.stages, beambot_interfaces)

**Grouping:**
- Blank line between groups
- Alphabetical within each group (generally followed)
- Type imports with regular imports (no separate section)

**Example from `orchestrator.py`:**
```python
import json
import threading
import time
from typing import Dict, Any, List, Tuple

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from beambot_interfaces.action import MTCExecution, MoveToAction
from beambot.core.moveit_lifecycle_manager import MoveItLifecycleManager
```

## Error Handling

**Patterns:**
- Try/except/finally with logging at catch sites
- Specific exception types preferred (but broad `except Exception` common)
- Return False for stage failures, True for success

**Error Types:**
- Throw on invalid input (after logging)
- Return boolean for expected failures (planning failed, detection failed)
- Log with context before handling: `self.get_logger().error(f"Failed: {e}")`

**Example:**
```python
try:
    result = self._execute(goal_handle)
    return result
except Exception as e:
    self.get_logger().error(f"Exception during execution: {e}")
    goal_handle.abort()
    return error_result
finally:
    with self._lock:
        self._executing = False
```

## Logging

**Framework:**
- ROS2 `self.get_logger()` exclusively (no print statements)
- Levels: debug, info, warn, error

**Patterns:**
- f-strings with context: `f"Step {i}: {task_type}"`
- Log at service boundaries (action accept/complete)
- Log state transitions (gripper change, MoveIt restart)

**Example:**
```python
self.get_logger().info(f"Pick/place: gripper_group={goal.gripper_group}")
self.get_logger().warn("Rejecting goal: server busy")
self.get_logger().error(f"Unknown gripper: {gripper_name}")
```

## Comments

**When to Comment:**
- Explain *why*, not what
- Document business logic and constraints
- Note workarounds and their reasons

**JSDoc/TSDoc (Python Docstrings):**
- Google-style with Args, Returns, Raises sections
- Required for public API functions
- Optional for internal if signature is self-explanatory

**Module Docstrings:**
```python
"""Base class for MTC action servers.

Provides goal lifecycle management, concurrent execution prevention,
and standard error handling for all MTC action servers.
"""
```

**Function Docstrings:**
```python
def joints_from_degrees(degrees: List[float]) -> Dict[str, float]:
    """Convert joint angles from degrees to radians dict.

    Args:
        degrees: List of 6 joint angles in degrees

    Returns:
        Dictionary mapping joint names to radian values
    """
```

**TODO Comments:**
- Format: `# TODO: description`
- Link to issue if exists: `# TODO: Fix race condition (issue #123)`

## Function Design

**Size:**
- Keep under 50-100 lines (relaxed for robotics complexity)
- Extract helpers for repeated logic
- Complex functions acceptable if well-documented

**Parameters:**
- Use object destructuring for many parameters
- Default values for optional parameters
- Type hints on all parameters

**Return Values:**
- Explicit return statements
- Boolean for success/failure in stages
- Result objects for action server responses

## Module Design

**Exports:**
- Named exports preferred (Python standard)
- `__init__.py` for package-level exports
- Factory functions for polymorphic types (e.g., `get_camera()`)

**Dataclasses:**
- Used for structured parameter groups
- Immutable by default (no explicit frozen=True, but treated as such)

**Example:**
```python
@dataclass
class CircleDetectionParams:
    """Parameters for Hough circle detection."""
    min_radius: int = 15
    max_radius: int = 100
    blur_kernel: int = 5
```

## ROS2-Specific Conventions

**Node Names:**
- Servers: `beambot_{function}_server` (e.g., `beambot_moveto_server`)
- Orchestrator: `beambot_orchestrator`

**Action Names:**
- Format: `beambot_{function}` (e.g., `beambot_moveto`, `beambot_execution`)

**Topics/Services:**
- Namespaced under `/beambot/` for internal topics
- Camera topics under `/zivid/`

**Callback Groups:**
- ReentrantCallbackGroup for concurrent action handling
- MutuallyExclusiveCallbackGroup when needed

---

*Convention analysis: 2026-01-17*
*Update when patterns change*
