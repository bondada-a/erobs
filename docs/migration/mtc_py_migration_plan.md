# MTC_PY Migration Plan: C++ to Python

> **Version**: 2.0
> **Date**: December 2, 2025
> **Status**: Ready for Implementation

---

## Executive Summary

This document outlines the comprehensive plan for migrating the EROBS MoveIt Task Constructor (MTC) implementation from C++ (`mtc_pipeline`) to Python (`mtc_py`). The goal is to create an **exact behavioral mirror** of the existing C++ implementation, enabling side-by-side testing and establishing a baseline for future Python-specific enhancements.

### Key Findings

1. **MTC Python bindings already exist** in the repository at `src/moveit_task_constructor/core/python/`
2. **All required MTC features are available** in Python (Pick, Place, GenerateGraspPose, Fallbacks, etc.)
3. **Architecture is migration-friendly** - Action server pattern allows parallel C++/Python implementations
4. **MTC RViz plugin works regardless** of Python or C++ backend
5. **Hybrid node approach required** - MTC needs `rclcpp.Node`, action servers use `rclpy`

---

## Clarification Responses (Finalized Decisions)

| # | Question | Decision | Rationale |
|---|----------|----------|-----------|
| 1 | Action definitions | **Copy to mtc_py** | Enables eventual deletion of mtc_pipeline without dependency issues |
| 2 | Action server naming | **Add `_py` suffix** | e.g., `mtc_moveto_py` - allows side-by-side testing, easy cleanup later |
| 3 | Node type | **Hybrid approach** | `rclcpp.Node` for MTC, `rclpy` for action servers (required by MTC internals) |
| 4 | Custom stage | **Keep in C++** | Keep PipettorActionServer in C++, call via action client from Python |
| 5 | MoveIt lifecycle | **Most reliable method** | Use subprocess with proper process management |
| 6 | Testing environment | **URSim** | Behaves like real robot, good for validation |
| 7 | Config files | **Copy to mtc_py** | Independent package, no symlink issues |
| 8 | Phase order | **Orchestrator after Phase 3** | Enables early system testing before vision/pipettor |
| 9 | Error messages | **Similar meaning OK** | Pythonic phrasing acceptable as long as clear |
| 10 | Scaling factors | **Hardcoded 20%** | Exact parity with C++ implementation |

---

## Table of Contents

1. [Current Architecture Analysis](#1-current-architecture-analysis)
2. [Component Mapping](#2-component-mapping)
3. [Package Structure](#3-package-structure)
4. [Implementation Phases](#4-implementation-phases) *(Updated order)*
5. [Technical Details](#5-technical-details)
6. [Testing Strategy](#6-testing-strategy)
7. [Risk Assessment](#7-risk-assessment)

---

## 1. Current Architecture Analysis

### 1.1 mtc_pipeline Package Overview

The current C++ implementation consists of:

```
src/mtc_pipeline/
├── action/                    # ROS 2 action definitions
│   ├── MTCExecution.action
│   ├── MoveToAction.action
│   ├── EndEffectorAction.action
│   ├── PickPlaceAction.action
│   ├── ToolExchangeAction.action
│   ├── VisionMoveToAction.action
│   ├── VisionPickPlaceAction.action
│   └── PipettorAction.action
├── config/
│   ├── grippers.yaml         # Gripper configurations
│   └── obstacles.yaml        # Planning scene obstacles
├── include/mtc_pipeline/
│   ├── base_stages.hpp       # Core MTC utilities
│   ├── move_to_stages.hpp
│   ├── end_effector_stages.hpp
│   ├── pick_place_stages.hpp
│   ├── tool_exchange_stages.hpp
│   ├── vision_stages.hpp
│   ├── vision_pick_place_stages.hpp
│   ├── pipettor_stages.hpp
│   ├── pipettor_operation_stage.hpp  # Custom MTC stage
│   ├── base_action_server.hpp
│   ├── mtc_orchestrator_action_server.hpp
│   ├── gripper_utils.hpp
│   ├── gripper_config_registry.hpp
│   └── core/
│       ├── moveit_lifecycle_manager.hpp
│       └── ur_tool_interface.hpp
├── src/
│   ├── stages/               # Stage implementations
│   └── action_servers/       # Action server implementations
└── launch/
    └── mtc_bringup.launch.py
```

### 1.2 Architecture Pattern

```
┌─────────────────────────────────────────────────────────────────┐
│                    MTCOrchestratorActionServer                   │
│  - Parses JSON task definitions                                  │
│  - Manages MoveIt lifecycle (launch/shutdown)                    │
│  - Routes to appropriate action server                           │
└─────────────────────┬───────────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│ MoveToServer  │ │PickPlaceServer│ │ VisionServer  │  ... (7 servers)
│               │ │               │ │               │
│ MoveToStages  │ │PickPlaceStages│ │ VisionStages  │  ... (7 stage classes)
└───────────────┘ └───────────────┘ └───────────────┘
        │             │             │
        └─────────────┼─────────────┘
                      ▼
              ┌───────────────┐
              │  BaseStages   │  (shared utilities)
              └───────────────┘
```

### 1.3 Critical Technical Discovery: rclcpp vs rclpy

**The MTC Python demos use `import rclcpp`** - this is a pybind11 wrapper around the C++ rclcpp library, NOT the same as `rclpy`.

```python
# MTC demo pattern (from demo/scripts/cartesian.py)
import rclcpp
from moveit.task_constructor import core, stages

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

task = core.Task()
task.loadRobotModel(node)  # Requires rclcpp.Node, NOT rclpy.Node!

pipeline = core.PipelinePlanner(node)  # Also requires rclcpp.Node
```

**Why this matters:**
- MTC's C++ code expects `rclcpp::Node::SharedPtr`
- The `rclcpp` Python module creates nodes backed by actual C++ objects
- `rclpy.Node` is pure Python and CANNOT be used with MTC

**Our solution - Hybrid Architecture:**
```
┌─────────────────────────────────────────────────────────────────┐
│                    Python Action Server (rclpy)                  │
│  - Receives goals via ROS 2 action interface                     │
│  - Handles threading and lifecycle                               │
└─────────────────────┬───────────────────────────────────────────┘
                      │ Creates internally
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    MTC Node (rclcpp Python bindings)             │
│  - Used for Task creation and planning                           │
│  - Passed to PipelinePlanner, CartesianPath, etc.                │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Component Mapping

### 2.1 Stage Classes Mapping

| C++ Class | Python Equivalent | Notes |
|-----------|-------------------|-------|
| `BaseStages` | `base_stages.py` | Planner factories, utilities |
| `MoveToStages` | `move_to_stages.py` | Relative + target moves |
| `EndEffectorStages` | `end_effector_stages.py` | Gripper open/close |
| `PickPlaceStages` | `pick_place_stages.py` | 10-step sequence |
| `ToolExchangeStages` | `tool_exchange_stages.py` | Magnetic holder ops |
| `VisionStages` | `vision_stages.py` | Zivid + TF2 |
| `VisionPickPlaceStages` | `vision_pick_place_stages.py` | Vision-guided pick/place |
| `PipettorStages` | **Keep C++ (via action)** | Call existing C++ server |
| `PipettorOperationStage` | **Keep C++ (via action)** | Custom stage stays in C++ |

### 2.2 Action Server Mapping

| C++ Server | Python Equivalent | Action Name |
|------------|-------------------|-------------|
| `MoveToActionServer` | `move_to_server.py` | `mtc_moveto_py` |
| `EndEffectorActionServer` | `end_effector_server.py` | `mtc_endeffector_py` |
| `PickPlaceActionServer` | `pick_place_server.py` | `mtc_pickplace_py` |
| `ToolExchangeActionServer` | `tool_exchange_server.py` | `mtc_toolexchange_py` |
| `VisionMoveToActionServer` | `vision_server.py` | `mtc_vision_py` |
| `VisionPickPlaceActionServer` | `vision_pick_place_server.py` | `mtc_vision_pickplace_py` |
| `PipettorActionServer` | **Keep C++ server** | `mtc_pipettor` (unchanged) |
| `MTCOrchestratorActionServer` | `orchestrator.py` | `mtc_execute_py` |

### 2.3 MTC Python API Mapping

| C++ Type | Python Import | Usage |
|----------|---------------|-------|
| `mtc::Task` | `core.Task` | Task container |
| `mtc::stages::CurrentState` | `stages.CurrentState` | Initial state |
| `mtc::stages::MoveTo` | `stages.MoveTo` | Target moves |
| `mtc::stages::MoveRelative` | `stages.MoveRelative` | Direction moves |
| `mtc::stages::Connect` | `stages.Connect` | Path planning |
| `mtc::stages::ModifyPlanningScene` | `stages.ModifyPlanningScene` | Scene updates |
| `mtc::stages::GenerateGraspPose` | `stages.GenerateGraspPose` | Grasp generation |
| `mtc::stages::Pick` | `stages.Pick` | Pick container |
| `mtc::stages::Place` | `stages.Place` | Place container |
| `mtc::SerialContainer` | `core.SerialContainer` | Sequential stages |
| `mtc::Fallbacks` | `core.Fallbacks` | Alternative approaches |
| `mtc::solvers::PipelinePlanner` | `core.PipelinePlanner` | OMPL planner |
| `mtc::solvers::CartesianPath` | `core.CartesianPath` | Cartesian planner |
| `mtc::solvers::JointInterpolationPlanner` | `core.JointInterpolationPlanner` | Joint space |

---

## 3. Package Structure

### 3.1 Proposed mtc_py Directory Structure

```
src/mtc_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── mtc_py                    # ament resource marker
├── action/                       # COPIED from mtc_pipeline
│   ├── MTCExecution.action
│   ├── MoveToAction.action
│   ├── EndEffectorAction.action
│   ├── PickPlaceAction.action
│   ├── ToolExchangeAction.action
│   ├── VisionMoveToAction.action
│   └── VisionPickPlaceAction.action
│   # NOTE: PipettorAction.action NOT copied - use C++ server
├── config/                       # COPIED from mtc_pipeline
│   ├── grippers.yaml
│   └── obstacles.yaml
├── mtc_py/
│   ├── __init__.py
│   ├── stages/
│   │   ├── __init__.py
│   │   ├── base_stages.py        # Core utilities, planners
│   │   ├── move_to_stages.py     # MoveTo operations
│   │   ├── end_effector_stages.py # Gripper control
│   │   ├── pick_place_stages.py  # Pick and place sequence
│   │   ├── tool_exchange_stages.py # Tool docking
│   │   ├── vision_stages.py      # Vision-guided motion
│   │   └── vision_pick_place_stages.py # Vision pick/place
│   ├── actions/
│   │   ├── __init__.py
│   │   ├── base_action_server.py # Template pattern (rclpy)
│   │   ├── move_to_server.py
│   │   ├── end_effector_server.py
│   │   ├── pick_place_server.py
│   │   ├── tool_exchange_server.py
│   │   ├── vision_server.py
│   │   ├── vision_pick_place_server.py
│   │   └── orchestrator.py       # Main coordinator
│   ├── core/
│   │   ├── __init__.py
│   │   ├── mtc_node.py           # rclcpp.Node wrapper for MTC
│   │   ├── moveit_lifecycle_manager.py
│   │   └── ur_tool_interface.py
│   └── utils/
│       ├── __init__.py
│       ├── gripper_utils.py
│       ├── gripper_config_registry.py
│       ├── obstacle_loader.py
│       └── transforms.py         # TF2 utilities
├── launch/
│   └── mtc_py_bringup.launch.py
├── scripts/
│   ├── move_to_server_node.py
│   ├── end_effector_server_node.py
│   ├── pick_place_server_node.py
│   ├── tool_exchange_server_node.py
│   ├── vision_server_node.py
│   ├── vision_pick_place_server_node.py
│   └── orchestrator_node.py
└── test/
    ├── test_base_stages.py
    ├── test_move_to_stages.py
    ├── test_action_servers.py
    └── integration/
        └── test_full_pipeline.py
```

### 3.2 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mtc_py</name>
  <version>0.1.0</version>
  <description>Python MTC motion planning pipeline (mirror of mtc_pipeline)</description>
  <maintainer email="abondada@bnl.gov">abondada</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- ROS 2 core -->
  <depend>rclpy</depend>
  <exec_depend>python3-yaml</exec_depend>

  <!-- MoveIt and MTC -->
  <depend>moveit_task_constructor_core</depend>
  <depend>moveit_ros_planning_interface</depend>

  <!-- TF2 -->
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <!-- Vision (for later phases) -->
  <depend>zivid_interfaces</depend>

  <!-- Pipettor (calls C++ server via action) -->
  <depend>mtc_pipeline</depend>

  <!-- Geometry -->
  <depend>geometry_msgs</depend>
  <depend>shape_msgs</depend>
  <depend>moveit_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Note**: Using `ament_cmake` with `ament_cmake_python` to support both action generation and Python package installation.

### 3.3 CMakeLists.txt (Hybrid Package)

```cmake
cmake_minimum_required(VERSION 3.8)
project(mtc_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCExecution.action"
  "action/MoveToAction.action"
  "action/EndEffectorAction.action"
  "action/PickPlaceAction.action"
  "action/ToolExchangeAction.action"
  "action/VisionMoveToAction.action"
  "action/VisionPickPlaceAction.action"
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/move_to_server_node.py
  scripts/end_effector_server_node.py
  scripts/pick_place_server_node.py
  scripts/tool_exchange_server_node.py
  scripts/vision_server_node.py
  scripts/vision_pick_place_server_node.py
  scripts/orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 4. Implementation Phases (Updated Order)

**Key Change**: Orchestrator moved to Phase 4 (after Complex Stages) to enable early system testing.

### Phase 0: Package Scaffold (Day 1)
**Goal**: Create empty package structure that builds

- [ ] Create directory structure
- [ ] Write package.xml
- [ ] Write CMakeLists.txt
- [ ] Create `__init__.py` files
- [ ] Copy action definitions from mtc_pipeline
- [ ] Copy config files from mtc_pipeline
- [ ] Verify `colcon build` succeeds

**Deliverables**: Empty but buildable mtc_py package with action types

---

### Phase 1: Foundation Layer (Days 2-4)
**Goal**: Core utilities that all stages depend on

#### 1.1 mtc_node.py (rclcpp wrapper)
```python
"""MTC Node wrapper - manages rclcpp.Node for MTC operations."""

import rclcpp
from typing import Optional

class MTCNode:
    """Wrapper around rclcpp.Node for MTC operations.

    MTC requires rclcpp.Node (C++ backed), not rclpy.Node.
    This class manages the rclcpp context for MTC operations.
    """

    _instance: Optional['MTCNode'] = None
    _initialized: bool = False

    def __init__(self, name: str = "mtc_py"):
        if not MTCNode._initialized:
            rclcpp.init()
            MTCNode._initialized = True
        self._node = rclcpp.Node(name)

    @property
    def node(self):
        """Get the underlying rclcpp.Node for MTC operations."""
        return self._node

    @classmethod
    def get_instance(cls, name: str = "mtc_py") -> 'MTCNode':
        """Get or create singleton MTCNode instance."""
        if cls._instance is None:
            cls._instance = cls(name)
        return cls._instance

    def shutdown(self):
        """Shutdown rclcpp context."""
        if MTCNode._initialized:
            rclcpp.shutdown()
            MTCNode._initialized = False
            MTCNode._instance = None
```

#### 1.2 gripper_utils.py
```python
"""Gripper helper functions matching gripper_utils.hpp"""

def get_group_name(gripper_type: str) -> str:
    """Get MoveIt group name for gripper type.

    Args:
        gripper_type: Gripper identifier (e.g., "hande", "epick")

    Returns:
        MoveIt group name or empty string for no gripper
    """
    if not gripper_type or gripper_type in ("none", "pipettor"):
        return ""
    return f"{gripper_type}_gripper"


def get_state_name(gripper_type: str, open: bool) -> str:
    """Get SRDF state name for gripper position.

    Args:
        gripper_type: Gripper identifier
        open: True for open position, False for closed

    Returns:
        SRDF state name (e.g., "hande_open", "vacuum_on")
    """
    if gripper_type == "epick":
        return "vacuum_off" if open else "vacuum_on"
    return f"{gripper_type}_{'open' if open else 'closed'}"
```

#### 1.3 base_stages.py
```python
"""Core MTC utilities - Python equivalent of base_stages.hpp/cpp"""

import math
from typing import Dict, Tuple, Optional
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header

from mtc_py.core.mtc_node import MTCNode

# Direction vectors matching C++ implementation
DIRECTION_VECTORS: Dict[str, Tuple[float, float, float]] = {
    "forward":  ( 1.0,  0.0,  0.0), "x":  ( 1.0,  0.0,  0.0),
    "backward": (-1.0,  0.0,  0.0), "-x": (-1.0,  0.0,  0.0),
    "right":    ( 0.0,  1.0,  0.0), "y":  ( 0.0,  1.0,  0.0),
    "left":     ( 0.0, -1.0,  0.0), "-y": ( 0.0, -1.0,  0.0),
    "up":       ( 0.0,  0.0, -1.0), "z":  ( 0.0,  0.0, -1.0),
    "down":     ( 0.0,  0.0,  1.0), "-z": ( 0.0,  0.0,  1.0),
}

# UR5e default joint names
DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]

# Hardcoded scaling factors (matching C++ at 20%)
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2


class BaseStages:
    """Base class providing MTC utilities for all stage implementations."""

    def __init__(self, rclpy_node, arm_group: str = "ur_manipulator",
                 gripper_group: str = "", ik_frame: str = "tool0"):
        """Initialize base stages.

        Args:
            rclpy_node: The rclpy node (for logging)
            arm_group: MoveIt planning group for arm
            gripper_group: MoveIt planning group for gripper
            ik_frame: Frame for IK calculations
        """
        self.rclpy_node = rclpy_node  # For logging
        self.mtc_node = MTCNode.get_instance()  # For MTC operations
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.ik_frame = ik_frame
        self.logger = rclpy_node.get_logger()

    def create_task_template(self, name: str) -> core.Task:
        """Create a new MTC task with standard configuration."""
        task = core.Task()
        task.name = name
        task.loadRobotModel(self.mtc_node.node)

        # Add current state as first stage
        task.add(stages.CurrentState("current_state"))
        return task

    def make_pipeline_planner(self) -> core.PipelinePlanner:
        """Create OMPL pipeline planner."""
        planner = core.PipelinePlanner(self.mtc_node.node)
        planner.planner = "RRTConnectkConfigDefault"
        planner.goal_joint_tolerance = 1e-4
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        return planner

    def make_cartesian_planner(self) -> core.CartesianPath:
        """Create Cartesian path planner."""
        planner = core.CartesianPath()
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        planner.step_size = 0.01
        return planner

    def make_joint_interpolation_planner(self) -> core.JointInterpolationPlanner:
        """Create joint interpolation planner (for gripper)."""
        planner = core.JointInterpolationPlanner()
        planner.max_velocity_scaling_factor = VELOCITY_SCALING
        planner.max_acceleration_scaling_factor = ACCELERATION_SCALING
        return planner

    def create_relative_move_stage(
        self,
        label: str,
        direction: str,
        distance: float,
        planner
    ) -> stages.MoveRelative:
        """Create a MoveRelative stage for directional movement.

        Args:
            label: Stage name
            direction: Direction string (e.g., "forward", "up", "x")
            distance: Distance in meters
            planner: Planner instance

        Returns:
            Configured MoveRelative stage

        Raises:
            ValueError: If direction is unknown
        """
        if direction not in DIRECTION_VECTORS:
            raise ValueError(f"Unknown direction: {direction}. "
                           f"Valid: {list(DIRECTION_VECTORS.keys())}")

        vec = DIRECTION_VECTORS[direction]

        stage = stages.MoveRelative(label, planner)
        stage.group = self.arm_group

        # Create direction vector
        header = Header(frame_id=self.ik_frame)
        direction_vec = Vector3Stamped(
            header=header,
            vector=Vector3(
                x=vec[0] * distance,
                y=vec[1] * distance,
                z=vec[2] * distance
            )
        )
        stage.setDirection(direction_vec)

        return stage

    def load_plan_execute(self, task: core.Task) -> bool:
        """Plan and execute the task.

        Args:
            task: Configured MTC task

        Returns:
            True if successful, False otherwise
        """
        try:
            # Plan (limit to 1 solution for efficiency)
            if not task.plan(max_solutions=1):
                self.logger.error("Planning failed - no valid solution found")
                return False

            # Execute best solution
            if task.solutions:
                task.execute(task.solutions[0])
                self.logger.info("Task executed successfully")
                return True
            else:
                self.logger.error("No solutions available after planning")
                return False

        except Exception as e:
            self.logger.error(f"Task execution failed: {e}")
            return False

    @staticmethod
    def joints_from_degrees(degrees: list) -> dict:
        """Convert joint angles from degrees to radians dict.

        Args:
            degrees: List of 6 joint angles in degrees

        Returns:
            Dictionary mapping joint names to radian values
        """
        return {
            name: math.radians(deg)
            for name, deg in zip(DEFAULT_JOINT_NAMES, degrees)
        }
```

#### 1.4 base_action_server.py
```python
"""Base action server template - Python equivalent of base_action_server.hpp

Uses rclpy for action server (standard ROS 2 Python pattern).
MTC operations use rclcpp.Node internally via MTCNode.
"""

import threading
from typing import TypeVar, Generic, Callable, Type
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

ActionT = TypeVar('ActionT')


class BaseActionServer(Node, Generic[ActionT]):
    """Template base class for MTC action servers.

    Handles:
    - Goal lifecycle management
    - Concurrent execution prevention
    - Worker thread pattern for non-blocking execution
    """

    def __init__(
        self,
        node_name: str,
        action_name: str,
        action_type: Type[ActionT],
    ):
        """Initialize action server.

        Args:
            node_name: ROS node name
            action_name: Action server name (e.g., "mtc_moveto_py")
            action_type: Action type class
        """
        super().__init__(node_name)

        self._executing = False
        self._lock = threading.Lock()
        self._action_type = action_type

        # Stages instance (created by subclass)
        self._stages = None

        # Create action server with reentrant callback group
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(f"{node_name} started on '{action_name}'")

    def initialize_stages(self):
        """Initialize stages - must be called after construction.

        Override in subclass to create specific stages instance.
        """
        raise NotImplementedError("Subclass must implement initialize_stages()")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle cancel requests - not supported (can't safely abort mid-motion)."""
        self.get_logger().warn("Cancel not supported - cannot safely abort mid-motion")
        return CancelResponse.REJECT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the goal with concurrency protection."""
        with self._lock:
            if self._executing:
                self.get_logger().warn("Rejecting goal: server busy")
                result = self._action_type.Result()
                result.success = False
                result.error_message = "Server busy"
                goal_handle.abort()
                return result
            self._executing = True

        try:
            result = self._execute(goal_handle)

            if result.success:
                goal_handle.succeed()
                self.get_logger().info("Goal succeeded")
            else:
                goal_handle.abort()
                self.get_logger().error(f"Goal failed: {result.error_message}")

            return result

        except Exception as e:
            self.get_logger().error(f"Exception during execution: {e}")
            result = self._action_type.Result()
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

        finally:
            with self._lock:
                self._executing = False

    def _execute(self, goal_handle: ServerGoalHandle):
        """Execute goal - override in subclass.

        Args:
            goal_handle: The goal handle with request data

        Returns:
            Action result
        """
        raise NotImplementedError("Subclass must implement _execute()")
```

**Deliverables**:
- Working `mtc_node.py` (rclcpp wrapper)
- Working `gripper_utils.py`
- Working `base_stages.py` with planners and utilities
- Working `base_action_server.py` template

---

### Phase 2: Simple Action Servers (Days 5-7)
**Goal**: Implement simplest action servers first

#### 2.1 move_to_stages.py
```python
"""MoveTo stages - Python equivalent of move_to_stages.hpp/cpp"""

import json
from moveit.task_constructor import core, stages
from mtc_py.stages.base_stages import BaseStages
from mtc_py.action import MoveToAction


class MoveToStages(BaseStages):
    """Handles MoveTo action: relative moves, joint poses, named states."""

    def run(self, goal: MoveToAction.Goal) -> bool:
        """Execute MoveTo action.

        Args:
            goal: MoveToAction goal with target, direction, distance, etc.

        Returns:
            True if successful
        """
        task = self.create_task_template("MoveTo Task")

        # Select planner based on planning_type
        if goal.planning_type == "cartesian":
            planner = self.make_cartesian_planner()
        else:
            planner = self.make_pipeline_planner()

        # Case 1: Relative move (direction + distance)
        if goal.direction and goal.distance != 0.0:
            stage = self.create_relative_move_stage(
                f"move_{goal.direction}_{goal.distance:.3f}m",
                goal.direction,
                goal.distance,
                planner
            )
            task.add(stage)
            self.logger.info(f"Planning relative move: {goal.direction} {goal.distance}m")

        # Case 2: Target-based move
        elif goal.target:
            poses = json.loads(goal.poses_json) if goal.poses_json else {}

            move_stage = stages.MoveTo(f"move_to_{goal.target}", planner)
            move_stage.group = self.arm_group

            # Check if target is a defined joint pose
            if goal.target in poses:
                joint_values = poses[goal.target]
                if isinstance(joint_values, list):
                    move_stage.setGoal(self.joints_from_degrees(joint_values))
                    self.logger.info(f"Planning move to joint pose: {goal.target}")
                else:
                    self.logger.error(f"Invalid pose format for {goal.target}")
                    return False
            else:
                # Assume it's a named SRDF state
                move_stage.setGoal(goal.target)
                self.logger.info(f"Planning move to named state: {goal.target}")

            task.add(move_stage)

        else:
            self.logger.error("No valid move target specified")
            return False

        return self.load_plan_execute(task)
```

#### 2.2 move_to_server.py
```python
"""MoveToAction server - handles MoveTo goals via MTC."""

from mtc_py.actions.base_action_server import BaseActionServer
from mtc_py.stages.move_to_stages import MoveToStages
from mtc_py.action import MoveToAction


class MoveToActionServer(BaseActionServer[MoveToAction]):
    """Action server for MoveTo operations."""

    def __init__(self):
        super().__init__(
            node_name="mtc_moveto_server_py",
            action_name="mtc_moveto_py",
            action_type=MoveToAction,
        )
        self.initialize_stages()

    def initialize_stages(self):
        """Create MoveToStages instance."""
        self._stages = MoveToStages(self)

    def _execute(self, goal_handle):
        """Execute MoveTo goal."""
        result = MoveToAction.Result()
        goal = goal_handle.request

        if self._stages is None:
            result.success = False
            result.error_message = "Stages not initialized"
            return result

        try:
            result.success = self._stages.run(goal)
            if not result.success:
                result.error_message = "Motion planning or execution failed"
        except Exception as e:
            result.success = False
            result.error_message = str(e)

        return result
```

#### 2.3 end_effector_stages.py
```python
"""EndEffector stages - Python equivalent of end_effector_stages.hpp/cpp"""

from moveit.task_constructor import core, stages
from mtc_py.stages.base_stages import BaseStages
from mtc_py.utils.gripper_utils import get_group_name, get_state_name
from mtc_py.action import EndEffectorAction


class EndEffectorStages(BaseStages):
    """Handles gripper open/close operations."""

    def run(self, goal: EndEffectorAction.Goal) -> bool:
        """Execute EndEffector action.

        Args:
            goal: EndEffectorAction goal with gripper type and open/close

        Returns:
            True if successful
        """
        gripper_group = get_group_name(goal.gripper)
        if not gripper_group:
            self.logger.info(f"No gripper group for type: {goal.gripper} - no-op")
            return True  # No-op success for "none" gripper

        task = self.create_task_template("EndEffector Task")
        planner = self.make_joint_interpolation_planner()

        # Determine target state
        target_state = get_state_name(goal.gripper, goal.open)

        # Create MoveTo stage for gripper
        action_name = "open" if goal.open else "close"
        stage = stages.MoveTo(f"gripper_{action_name}", planner)
        stage.group = gripper_group
        stage.setGoal(target_state)

        task.add(stage)

        self.logger.info(f"Planning gripper {action_name} ({target_state})")
        return self.load_plan_execute(task)
```

#### 2.4 Entry Point Scripts

```python
#!/usr/bin/env python3
# scripts/move_to_server_node.py
"""MoveToAction server node entry point."""

import rclpy
from mtc_py.actions.move_to_server import MoveToActionServer


def main(args=None):
    rclpy.init(args=args)
    node = MoveToActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Deliverables**:
- Working `move_to_stages.py` and `move_to_server.py`
- Working `end_effector_stages.py` and `end_effector_server.py`
- Entry point scripts for both servers

---

### Phase 3: Complex Stages (Days 8-12)
**Goal**: Pick/place and tool exchange

#### 3.1 pick_place_stages.py
```python
"""PickPlace stages - Python equivalent of pick_place_stages.hpp/cpp

Implements 10-step pick and place sequence:
1. Open gripper
2. Approach pick (with wrist constraint)
3. Move to pick (with wrist constraint)
4. Close gripper
5. Retreat from pick (with wrist constraint)
6. Approach place
7. Move to place
8. Open gripper (release)
9. Retreat from place
10. (Optional) Close gripper
"""

import json
from moveit.task_constructor import core, stages
from moveit_msgs.msg import Constraints, OrientationConstraint
from mtc_py.stages.base_stages import BaseStages
from mtc_py.utils.gripper_utils import get_group_name, get_state_name
from mtc_py.action import PickPlaceAction


class PickPlaceStages(BaseStages):
    """Handles pick and place operations with wrist constraints."""

    def create_wrist_constraint(self) -> Constraints:
        """Create constraint to keep wrist level during pick.

        Returns:
            Constraints message for wrist orientation
        """
        constraints = Constraints()

        orientation = OrientationConstraint()
        orientation.header.frame_id = "base_link"
        orientation.link_name = "wrist_3_link"
        orientation.orientation.w = 1.0  # Identity quaternion
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 3.14159  # Free rotation around Z
        orientation.weight = 1.0

        constraints.orientation_constraints.append(orientation)
        return constraints

    def run(self, goal: PickPlaceAction.Goal) -> bool:
        """Execute 10-step pick and place sequence.

        Args:
            goal: PickPlaceAction goal with poses and gripper type

        Returns:
            True if successful
        """
        poses = json.loads(goal.poses_json) if goal.poses_json else {}
        task = self.create_task_template("Pick and Place")

        pipeline = self.make_pipeline_planner()
        gripper_planner = self.make_joint_interpolation_planner()

        gripper_group = get_group_name(goal.gripper)
        wrist_constraint = self.create_wrist_constraint()

        def add_gripper_stage(name: str, open: bool):
            """Add gripper open/close stage."""
            if not gripper_group:
                return
            state = get_state_name(goal.gripper, open)
            stage = stages.MoveTo(name, gripper_planner)
            stage.group = gripper_group
            stage.setGoal(state)
            task.add(stage)

        def add_move(name: str, target: str, with_constraint: bool = False):
            """Add movement stage."""
            stage = stages.MoveTo(name, pipeline)
            stage.group = self.arm_group

            if target in poses:
                stage.setGoal(self.joints_from_degrees(poses[target]))
            else:
                stage.setGoal(target)

            if with_constraint:
                stage.path_constraints = wrist_constraint

            task.add(stage)

        # ========== PICK SEQUENCE (with wrist constraint) ==========
        self.logger.info("Building pick sequence...")

        # 1. Open gripper
        add_gripper_stage("1_open_gripper", True)

        # 2. Approach pick position
        add_move("2_pick_approach", goal.pick_approach, with_constraint=True)

        # 3. Move to pick target
        add_move("3_pick_target", goal.pick_target, with_constraint=True)

        # 4. Close gripper (grasp)
        add_gripper_stage("4_close_gripper", False)

        # 5. Retreat from pick
        add_move("5_pick_retreat", goal.pick_approach, with_constraint=True)

        # ========== PLACE SEQUENCE (no constraint) ==========
        self.logger.info("Building place sequence...")

        # 6. Approach place position
        add_move("6_place_approach", goal.place_approach)

        # 7. Move to place target
        add_move("7_place_target", goal.place_target)

        # 8. Open gripper (release)
        add_gripper_stage("8_release", True)

        # 9. Retreat from place
        add_move("9_place_retreat", goal.place_approach)

        self.logger.info("Executing pick and place task...")
        return self.load_plan_execute(task)
```

#### 3.2 tool_exchange_stages.py
```python
"""ToolExchange stages - Python equivalent of tool_exchange_stages.hpp/cpp

Handles magnetic tool holder operations for gripper swapping.
"""

import json
from moveit.task_constructor import core, stages
from mtc_py.stages.base_stages import BaseStages
from mtc_py.action import ToolExchangeAction

# Dock configuration
DOCK_SPACING = 0.10  # meters between dock positions
DOCK_APPROACH_DISTANCE = 0.05  # meters
DOCK_INSERT_DISTANCE = 0.05  # meters
DOCK_RETREAT_DISTANCE = 0.10  # meters


class ToolExchangeStages(BaseStages):
    """Handles tool exchange at magnetic holder."""

    DOCK_MAP = {"hande": 0, "epick": 1, "pipettor": 2}

    def run(self, goal: ToolExchangeAction.Goal) -> bool:
        """Execute tool exchange sequence.

        Args:
            goal: ToolExchangeAction goal with gripper type and dock/undock

        Returns:
            True if successful
        """
        poses = json.loads(goal.poses_json) if goal.poses_json else {}
        task = self.create_task_template("Tool Exchange")

        pipeline = self.make_pipeline_planner()
        cartesian = self.make_cartesian_planner()

        dock_index = self.DOCK_MAP.get(goal.gripper_type, 0)

        if goal.is_dock:
            self.logger.info(f"Docking {goal.gripper_type} at position {dock_index}")
            self._add_dock_sequence(task, goal, poses, pipeline, cartesian)
        else:
            self.logger.info(f"Undocking {goal.gripper_type} from position {dock_index}")
            self._add_undock_sequence(task, goal, poses, pipeline, cartesian)

        return self.load_plan_execute(task)

    def _add_dock_sequence(self, task, goal, poses, pipeline, cartesian):
        """Add stages for docking (putting tool away)."""
        # Move to pre-dock position
        stage = stages.MoveTo("pre_dock", pipeline)
        stage.group = self.arm_group
        if goal.approach_pose in poses:
            stage.setGoal(self.joints_from_degrees(poses[goal.approach_pose]))
        else:
            stage.setGoal(goal.approach_pose)
        task.add(stage)

        # Insert into dock (cartesian)
        insert = self.create_relative_move_stage(
            "insert_dock", "forward", DOCK_INSERT_DISTANCE, cartesian
        )
        task.add(insert)

        # Retreat from dock
        retreat = self.create_relative_move_stage(
            "retreat_dock", "backward", DOCK_RETREAT_DISTANCE, cartesian
        )
        task.add(retreat)

    def _add_undock_sequence(self, task, goal, poses, pipeline, cartesian):
        """Add stages for undocking (picking up tool)."""
        # Move to pre-undock position
        stage = stages.MoveTo("pre_undock", pipeline)
        stage.group = self.arm_group
        if goal.approach_pose in poses:
            stage.setGoal(self.joints_from_degrees(poses[goal.approach_pose]))
        else:
            stage.setGoal(goal.approach_pose)
        task.add(stage)

        # Approach dock (cartesian)
        approach = self.create_relative_move_stage(
            "approach_dock", "forward", DOCK_APPROACH_DISTANCE, cartesian
        )
        task.add(approach)

        # Retreat with tool
        retreat = self.create_relative_move_stage(
            "retreat_with_tool", "backward", DOCK_RETREAT_DISTANCE, cartesian
        )
        task.add(retreat)
```

**Deliverables**:
- Working `pick_place_stages.py` with 10-step sequence
- Working `tool_exchange_stages.py` with dock/undock
- Corresponding action servers and entry scripts

---

### Phase 4: Orchestrator, Launch & Testing (Days 13-18)
**Goal**: Enable early system testing

#### 4.1 orchestrator.py
```python
"""MTC Orchestrator - Python equivalent of mtc_orchestrator_action_server.cpp

Coordinates multi-step robot tasks with gripper/MoveIt management.
"""

import json
import subprocess
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from mtc_py.action import (
    MTCExecution, MoveToAction, EndEffectorAction,
    PickPlaceAction, ToolExchangeAction
)
from mtc_py.utils.gripper_config_registry import GripperConfigRegistry
from mtc_py.core.moveit_lifecycle_manager import MoveItLifecycleManager
from mtc_py.core.ur_tool_interface import URToolInterface


class MTCOrchestrator(Node):
    """Central coordinator for MTC operations."""

    def __init__(self):
        super().__init__('mtc_orchestrator_py')

        self._executing = False
        self._lock = threading.Lock()

        # Load gripper configurations
        self._gripper_registry = GripperConfigRegistry(self)

        # Core components
        self._moveit_manager = MoveItLifecycleManager(self)
        self._tool_interface = URToolInterface(self)

        # Action clients (with _py suffix)
        callback_group = ReentrantCallbackGroup()
        self._clients = {
            'moveto': ActionClient(self, MoveToAction, 'mtc_moveto_py',
                                   callback_group=callback_group),
            'endeffector': ActionClient(self, EndEffectorAction, 'mtc_endeffector_py',
                                        callback_group=callback_group),
            'pickplace': ActionClient(self, PickPlaceAction, 'mtc_pickplace_py',
                                      callback_group=callback_group),
            'toolexchange': ActionClient(self, ToolExchangeAction, 'mtc_toolexchange_py',
                                         callback_group=callback_group),
        }

        # Main action server
        self._action_server = ActionServer(
            self,
            MTCExecution,
            'mtc_execute_py',
            execute_callback=self._execute,
            callback_group=callback_group,
        )

        self.get_logger().info("MTC Orchestrator (Python) started")

    def _execute(self, goal_handle):
        """Execute multi-step task sequence."""
        with self._lock:
            if self._executing:
                return self._abort_busy(goal_handle)
            self._executing = True

        try:
            return self._execute_impl(goal_handle)
        finally:
            with self._lock:
                self._executing = False

    def _execute_impl(self, goal_handle):
        """Implementation of task execution."""
        result = MTCExecution.Result()
        goal = goal_handle.request

        # Parse JSON
        try:
            full_data = json.loads(goal.full_json)
            tasks = full_data.get('tasks', [])
            poses_json = json.dumps(full_data.get('poses', {}))
            start_gripper = full_data.get('start_gripper', 'none')
        except json.JSONDecodeError as e:
            result.success = False
            result.error_message = f"JSON parse error: {e}"
            goal_handle.abort()
            return result

        # Setup MoveIt
        gripper_config = self._gripper_registry.get_config(start_gripper)
        if gripper_config:
            self._moveit_manager.ensure_running(gripper_config.moveit_package)
            self._tool_interface.set_voltage(gripper_config.tool_voltage)

        # Execute tasks
        result.total_steps = len(tasks)
        for i, task in enumerate(tasks):
            result.completed_steps = i

            # Publish feedback
            feedback = MTCExecution.Feedback()
            feedback.current_step = i + 1
            feedback.current_action = task.get('type', 'unknown')
            feedback.progress_percentage = (i / len(tasks)) * 100
            goal_handle.publish_feedback(feedback)

            # Execute
            success = self._execute_task(task, poses_json, goal.robot_ip)
            if not success:
                result.success = False
                result.error_message = f"Task {i+1} failed: {task.get('type')}"
                goal_handle.abort()
                return result

        result.success = True
        result.completed_steps = len(tasks)
        goal_handle.succeed()
        return result

    def _execute_task(self, task: Dict[str, Any], poses_json: str, robot_ip: str) -> bool:
        """Route task to appropriate action server."""
        task_type = task.get('type', '')

        handlers = {
            'moveto': self._call_moveto,
            'endeffector': self._call_endeffector,
            'pickplace': self._call_pickplace,
            'toolexchange': self._call_toolexchange,
        }

        handler = handlers.get(task_type)
        if handler:
            return handler(task, poses_json)
        else:
            self.get_logger().error(f"Unknown task type: {task_type}")
            return False

    def _call_moveto(self, task: Dict, poses_json: str) -> bool:
        """Call MoveTo action server."""
        goal = MoveToAction.Goal()
        goal.target = task.get('target', '')
        goal.direction = task.get('direction', '')
        goal.distance = float(task.get('distance', 0.0))
        goal.planning_type = task.get('planning_type', 'pipeline')
        goal.poses_json = poses_json
        return self._send_and_wait(self._clients['moveto'], goal, 'MoveTo')

    def _call_endeffector(self, task: Dict, poses_json: str) -> bool:
        """Call EndEffector action server."""
        goal = EndEffectorAction.Goal()
        goal.gripper = task.get('gripper', '')
        goal.open = task.get('open', True)
        return self._send_and_wait(self._clients['endeffector'], goal, 'EndEffector')

    def _call_pickplace(self, task: Dict, poses_json: str) -> bool:
        """Call PickPlace action server."""
        goal = PickPlaceAction.Goal()
        goal.gripper = task.get('gripper', '')
        goal.pick_approach = task.get('pick_approach', '')
        goal.pick_target = task.get('pick_target', '')
        goal.place_approach = task.get('place_approach', '')
        goal.place_target = task.get('place_target', '')
        goal.poses_json = poses_json
        return self._send_and_wait(self._clients['pickplace'], goal, 'PickPlace')

    def _call_toolexchange(self, task: Dict, poses_json: str) -> bool:
        """Call ToolExchange action server."""
        goal = ToolExchangeAction.Goal()
        goal.gripper_type = task.get('gripper_type', '')
        goal.is_dock = task.get('is_dock', True)
        goal.approach_pose = task.get('approach_pose', '')
        goal.poses_json = poses_json
        return self._send_and_wait(self._clients['toolexchange'], goal, 'ToolExchange')

    def _send_and_wait(self, client, goal, name: str, timeout: float = 120.0) -> bool:
        """Send goal and wait for result."""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{name} server not available")
            return False

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{name} goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        return result_future.result().result.success

    def _abort_busy(self, goal_handle):
        """Abort with server busy message."""
        result = MTCExecution.Result()
        result.success = False
        result.error_message = "Server busy"
        goal_handle.abort()
        return result
```

#### 4.2 Launch File
```python
# launch/mtc_py_bringup.launch.py
"""Launch all mtc_py action servers."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtc_py',
            executable='move_to_server_node.py',
            name='mtc_moveto_server_py',
            output='screen',
        ),
        Node(
            package='mtc_py',
            executable='end_effector_server_node.py',
            name='mtc_endeffector_server_py',
            output='screen',
        ),
        Node(
            package='mtc_py',
            executable='pick_place_server_node.py',
            name='mtc_pickplace_server_py',
            output='screen',
        ),
        Node(
            package='mtc_py',
            executable='tool_exchange_server_node.py',
            name='mtc_toolexchange_server_py',
            output='screen',
        ),
        Node(
            package='mtc_py',
            executable='orchestrator_node.py',
            name='mtc_orchestrator_py',
            output='screen',
        ),
    ])
```

**Deliverables**:
- Working orchestrator with MoveIt lifecycle management
- Working launch file for core servers
- Integration tests with URSim

---

### Phase 5: Vision Stages (Days 19-23)
**Goal**: Zivid integration with TF2

#### 5.1 vision_stages.py
```python
"""Vision stages - integrates Zivid camera with MTC planning."""

import json
from typing import Optional
import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

from moveit.task_constructor import core, stages
from mtc_py.stages.base_stages import BaseStages
from zivid_interfaces.srv import GetArUcoPose
from mtc_py.action import VisionMoveToAction


class VisionStages(BaseStages):
    """Handles vision-guided motion with Zivid camera."""

    def __init__(self, rclpy_node, **kwargs):
        super().__init__(rclpy_node, **kwargs)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, rclpy_node)

        # Zivid service client
        self.aruco_client = rclpy_node.create_client(
            GetArUcoPose,
            '/zivid_camera/get_aruco_pose'
        )

    def get_aruco_pose(self, marker_id: int) -> Optional[PoseStamped]:
        """Call Zivid service to get ArUco marker pose."""
        if not self.aruco_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("Zivid ArUco service not available")
            return None

        request = GetArUcoPose.Request()
        request.marker_id = marker_id

        future = self.aruco_client.call_async(request)
        rclpy.spin_until_future_complete(self.rclpy_node, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            return future.result().pose
        return None

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """Transform pose to target frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return do_transform_pose_stamped(pose, transform)
        except TransformException as e:
            self.logger.error(f"TF2 transform failed: {e}")
            return None

    def run(self, goal: VisionMoveToAction.Goal) -> bool:
        """Execute vision-guided move."""
        # Get marker pose from Zivid
        marker_pose = self.get_aruco_pose(goal.marker_id)
        if marker_pose is None:
            self.logger.error(f"Failed to detect marker {goal.marker_id}")
            return False

        # Transform to base frame
        base_pose = self.transform_pose(marker_pose, "base_link")
        if base_pose is None:
            return False

        # Apply offset
        base_pose.pose.position.x += goal.offset_x
        base_pose.pose.position.y += goal.offset_y
        base_pose.pose.position.z += goal.offset_z

        # Create MTC task
        task = self.create_task_template("Vision MoveTo")
        planner = self.make_pipeline_planner()

        stage = stages.MoveTo("vision_move", planner)
        stage.group = self.arm_group
        stage.setGoal(base_pose)
        task.add(stage)

        return self.load_plan_execute(task)
```

**Deliverables**:
- Working `vision_stages.py` with Zivid integration
- Working `vision_pick_place_stages.py`
- TF2 utilities

---

### Phase 6: Pipettor Integration (Days 24-26)
**Goal**: Integrate with existing C++ pipettor server

The pipettor stages use a custom MTC stage (`PipettorOperationStage`) that is complex to port to Python. Per the decision to **keep it in C++**, the Python orchestrator will call the existing C++ `PipettorActionServer` via action client.

```python
# In orchestrator.py, add pipettor client
from mtc_pipeline.action import PipettorAction  # Import from C++ package

# In __init__:
self._clients['pipettor'] = ActionClient(
    self, PipettorAction, 'mtc_pipettor',  # Note: NO _py suffix
    callback_group=callback_group
)

# In _execute_task:
elif task_type == 'pipettor':
    return self._call_pipettor(task, poses_json)
```

**Deliverables**:
- Orchestrator integration with C++ pipettor server
- End-to-end pipettor workflow testing

---

### Phase 7: Full Testing & Documentation (Days 27-30)
**Goal**: Complete validation and documentation

- [ ] Full integration tests with URSim
- [ ] Parity testing (C++ vs Python outputs)
- [ ] Performance benchmarking
- [ ] Documentation updates
- [ ] README for mtc_py package

---

## 5. Technical Details

### 5.1 Hybrid Node Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     Python Process                                │
├──────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────┐    ┌─────────────────────────────┐  │
│  │     rclpy.Node          │    │     rclcpp.Node             │  │
│  │  (Action Servers)       │    │  (MTC Operations)           │  │
│  │                         │    │                             │  │
│  │  - Receives goals       │───▶│  - Task.loadRobotModel()   │  │
│  │  - Publishes feedback   │    │  - PipelinePlanner()        │  │
│  │  - Returns results      │◀───│  - task.plan() / execute()  │  │
│  └─────────────────────────┘    └─────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### 5.2 MoveIt Lifecycle Management

Using subprocess for reliability (matching C++ pattern):

```python
class MoveItLifecycleManager:
    """Manages MoveIt process lifecycle via subprocess."""

    def __init__(self, node):
        self.node = node
        self._process: Optional[subprocess.Popen] = None
        self._current_package: Optional[str] = None

    def ensure_running(self, moveit_package: str):
        """Ensure MoveIt is running with correct config."""
        if self._current_package == moveit_package and self._is_running():
            return

        self.stop()
        self._start(moveit_package)
        self._current_package = moveit_package

    def _start(self, package: str):
        """Launch MoveIt via ros2 launch."""
        cmd = ['ros2', 'launch', package, 'moveit.launch.py']
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Wait for MoveIt to be ready
        time.sleep(5.0)

    def stop(self):
        """Stop MoveIt process."""
        if self._process:
            self._process.terminate()
            self._process.wait(timeout=10)
            self._process = None
```

### 5.3 Testing with URSim

```bash
# Terminal 1: Start URSim
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=ursim

# Terminal 2: Start MoveIt
ros2 launch ur_zivid_hande_moveit_config moveit.launch.py

# Terminal 3: Start mtc_py servers
ros2 launch mtc_py mtc_py_bringup.launch.py

# Terminal 4: Test MoveTo action
ros2 action send_goal /mtc_moveto_py mtc_py/action/MoveToAction \
  "{target: 'home', poses_json: '{}'}"
```

---

## 6. Testing Strategy

### 6.1 Test Matrix

| Level | Focus | Environment | Tools |
|-------|-------|-------------|-------|
| Unit | Stage functions | Mocked | pytest |
| Integration | Action servers | URSim | ros2 action CLI |
| System | Orchestrator workflows | URSim | mtc_gui / scripts |
| Parity | C++ vs Python | URSim | Custom comparison |

### 6.2 Unit Test Example

```python
# test/test_gripper_utils.py
import pytest
from mtc_py.utils.gripper_utils import get_group_name, get_state_name


def test_get_group_name_hande():
    assert get_group_name("hande") == "hande_gripper"


def test_get_group_name_none():
    assert get_group_name("none") == ""
    assert get_group_name("") == ""


def test_get_state_name_epick():
    assert get_state_name("epick", True) == "vacuum_off"
    assert get_state_name("epick", False) == "vacuum_on"


def test_get_state_name_hande():
    assert get_state_name("hande", True) == "hande_open"
    assert get_state_name("hande", False) == "hande_closed"
```

---

## 7. Risk Assessment

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| rclcpp/rclpy interaction issues | High | Medium | Tested pattern from MTC demos |
| Custom stage not portable | High | N/A | **Decision: Keep in C++** |
| Performance degradation | Medium | Medium | Profile hot paths |
| MTC Python bindings bugs | Medium | Low | Bindings already in production |
| URSim vs real robot differences | Medium | Low | Test on real robot before deployment |

---

## Appendix A: File Checklist

### Phase 0: Package Scaffold
- [ ] `src/mtc_py/package.xml`
- [ ] `src/mtc_py/CMakeLists.txt`
- [ ] `src/mtc_py/resource/mtc_py`
- [ ] `src/mtc_py/mtc_py/__init__.py`
- [ ] `src/mtc_py/action/` (copied from mtc_pipeline)
- [ ] `src/mtc_py/config/` (copied from mtc_pipeline)

### Phase 1: Foundation
- [ ] `mtc_py/core/__init__.py`
- [ ] `mtc_py/core/mtc_node.py`
- [ ] `mtc_py/utils/__init__.py`
- [ ] `mtc_py/utils/gripper_utils.py`
- [ ] `mtc_py/stages/__init__.py`
- [ ] `mtc_py/stages/base_stages.py`
- [ ] `mtc_py/actions/__init__.py`
- [ ] `mtc_py/actions/base_action_server.py`

### Phase 2: Simple Servers
- [ ] `mtc_py/stages/move_to_stages.py`
- [ ] `mtc_py/stages/end_effector_stages.py`
- [ ] `mtc_py/actions/move_to_server.py`
- [ ] `mtc_py/actions/end_effector_server.py`
- [ ] `scripts/move_to_server_node.py`
- [ ] `scripts/end_effector_server_node.py`

### Phase 3: Complex Stages
- [ ] `mtc_py/stages/pick_place_stages.py`
- [ ] `mtc_py/stages/tool_exchange_stages.py`
- [ ] `mtc_py/actions/pick_place_server.py`
- [ ] `mtc_py/actions/tool_exchange_server.py`
- [ ] `scripts/pick_place_server_node.py`
- [ ] `scripts/tool_exchange_server_node.py`

### Phase 4: Orchestrator & Launch
- [ ] `mtc_py/core/moveit_lifecycle_manager.py`
- [ ] `mtc_py/core/ur_tool_interface.py`
- [ ] `mtc_py/utils/gripper_config_registry.py`
- [ ] `mtc_py/actions/orchestrator.py`
- [ ] `scripts/orchestrator_node.py`
- [ ] `launch/mtc_py_bringup.launch.py`

### Phase 5: Vision
- [ ] `mtc_py/utils/transforms.py`
- [ ] `mtc_py/stages/vision_stages.py`
- [ ] `mtc_py/stages/vision_pick_place_stages.py`
- [ ] `mtc_py/actions/vision_server.py`
- [ ] `mtc_py/actions/vision_pick_place_server.py`
- [ ] `scripts/vision_server_node.py`
- [ ] `scripts/vision_pick_place_server_node.py`

### Phase 6: Pipettor Integration
- [ ] Orchestrator update for pipettor action client

### Phase 7: Testing
- [ ] `test/test_gripper_utils.py`
- [ ] `test/test_base_stages.py`
- [ ] `test/test_move_to_stages.py`
- [ ] `test/integration/test_action_servers.py`
- [ ] `test/integration/test_orchestrator.py`

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-02 | Claude | Initial comprehensive plan |
| 2.0 | 2025-12-02 | Claude | Updated with user decisions, phase reordering, rclcpp clarification |

---

*End of MTC_PY Migration Plan Document - Version 2.0*
