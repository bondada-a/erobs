# Coding Conventions

**Analysis Date:** 2026-01-27

## Naming Patterns

**Files:**
- `snake_case.py` - All Python modules (`base_action_server.py`, `pick_place_stages.py`)
- `*_server.py` - Action server implementations
- `*_stages.py` - Stage composition classes
- `*.test.ts` / `*_test.cpp` - Test files (C++)
- `PascalCase.action` - ROS2 action definitions

**Functions:**
- `snake_case` - All functions (`initialize_stages()`, `run()`, `detect_markers()`)
- No special prefix for async functions
- `_private_method` - Private methods with leading underscore

**Variables:**
- `snake_case` - Variables (`current_gripper`, `batch_tasks`)
- `UPPER_SNAKE_CASE` - Constants (`DEFAULT_RETRY_COUNT`, `BATCHABLE_TYPES`)
- `_private` - Private members (`_executing`, `_lock`, `_stages`)

**Types:**
- `PascalCase` - Classes (`BaseActionServer`, `PickPlaceStages`, `DetectionResult`)
- `PascalCase` - Dataclasses (`GripperDetection`, `CircleDetectionParams`)
- No I prefix for interfaces

## Code Style

**Formatting:**
- 4 space indentation (PEP 8)
- 88 character line length (Black-compatible)
- Double quotes for strings
- Trailing commas in multi-line collections

**Linting:**
- Flake8 with extended rules (E, F, W, C, B590)
- Ignores: W503, E203, E501 (Black compatibility)
- Config: `.flake8` in `src/end_effectors/ros2_epick_gripper/`

**C++ Style:**
- 4 space indentation
- 100 character line length
- Google C++ Style base with modifications
- clang-format config in driver packages

## Import Organization

**Order:**
1. Standard library (`import os`, `from typing import`)
2. Third-party packages (`import yaml`, `from moveit`)
3. ROS packages (`import rclpy`, `from geometry_msgs`)
4. Local imports (`from beambot.stages`)

**Grouping:**
- Blank line between groups
- Alphabetical within groups (not strictly enforced)

**Path Aliases:**
- None (direct relative imports used)

## Error Handling

**Patterns:**
- Try/except at stage level, return False on failure
- Action servers catch exceptions, set result.success = False
- Log error with context before returning

**Error Types:**
- `json.JSONDecodeError` - Invalid JSON input
- `KeyError` - Missing configuration keys
- `RuntimeError` - MTC planning failures
- Logged with `self.logger.error()` or `self.get_logger().error()`

**Async:**
- Futures with timeout checks
- Polling loops for service availability

## Logging

**Framework:**
- ROS 2 logging (`self.get_logger()` or `self.logger`)
- Levels: debug, info, warn, error

**Patterns:**
- Log at entry/exit of major operations
- Include context: task type, target, gripper name
- f-string formatting: `self.logger.info(f"Planning move to: {target}")`

**Where:**
- Action servers log goal receipt and result
- Stages log planning attempts and outcomes
- Orchestrator logs batch grouping and dispatch

## Comments

**When to Comment:**
- Explain "why" for non-obvious code
- Document C++ equivalent for ported stages
- Mark known issues with TODO

**Docstrings:**
- Google-style triple-quoted docstrings
- Required for public classes and methods
- Include Args, Returns, Examples sections

**TODO Comments:**
- Format: `# TODO: description`
- Referenced in CLAUDE.md for tracking

## Function Design

**Size:**
- Keep under ~100 lines
- Extract helpers for complex logic
- Single responsibility per function

**Parameters:**
- Use goal objects (ROS2 action pattern)
- Optional parameters with defaults
- Type hints for all parameters

**Return Values:**
- Boolean for success/failure
- Dataclasses for complex returns (`DetectionResult`)
- Explicit return statements

## Module Design

**Exports:**
- No `__all__` (all public by convention)
- Import from module directly

**Barrel Files:**
- Not used (ROS2 package pattern)

**Initialization:**
- `__init__.py` present but minimal
- Resource registration for ament

---

*Convention analysis: 2026-01-27*
*Update when patterns change*
