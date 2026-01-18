# Architecture

**Analysis Date:** 2026-01-17

## Pattern Overview

**Overall:** ROS2 Action Server Hierarchy with MTC Orchestration

**Key Characteristics:**
- Multi-layer robotic orchestration using ROS2 action servers
- Central orchestrator dispatches JSON task scripts to specialized servers
- MoveIt Task Constructor (MTC) for hierarchical motion planning
- Batching optimization for consecutive simple tasks (~1.5s savings per task)
- Beamline-agnostic via YAML configuration
- Optional Bluesky integration for experiment control

## Layers

**Orchestration Layer (`src/beambot/beambot/action_servers/`):**
- Purpose: Central coordination, task dispatch, MoveIt lifecycle management
- Contains: MTCOrchestratorServer, task routing logic, gripper state tracking
- Key file: `orchestrator.py`
- Depends on: Action clients to specialized servers, MoveIt lifecycle manager
- Used by: Bluesky/Ophyd, GUI client, direct action calls

**Action Server Layer (7 Specialized Servers):**
- Purpose: Handle specific task types with ROS2 action pattern
- Contains: Goal acceptance, execution, result handling for each task type
- Key files: `move_to_server.py`, `end_effector_server.py`, `pick_place_server.py`, `tool_exchange_server.py`, `vision_server.py`, `vision_pick_place_server.py`, `pipettor_server.py`
- Depends on: Stage implementations (MTC pipeline)
- Used by: Orchestrator (via action clients)

**Stage Composition Layer (`src/beambot/beambot/stages/`):**
- Purpose: Build MTC pipelines with planning and execution stages
- Contains: MTC stage definitions, planner factories, task templates
- Key files: `base_stages.py`, `move_to_stages.py`, `pick_place_stages.py`, `vision_stages.py`
- Depends on: MoveIt Task Constructor, MoveIt planning interface
- Used by: Action servers

**Vision Layer (`src/beambot/beambot/camera/`):**
- Purpose: Camera-agnostic abstraction for object detection
- Contains: Zivid camera wrapper, ArUco/circle/contour detection
- Key files: `__init__.py` (factory), `zivid.py`
- Depends on: OpenCV, Zivid ROS2 driver, TF2
- Used by: VisionStages, VisionPickPlaceStages

**Motion Planning Layer (MTC + MoveIt):**
- Purpose: Low-level robotic motion execution
- Contains: MoveIt configuration, IK solvers, OMPL planning
- Key files: `src/custom-ur-descriptions/ur5e_moveit_configs/*/`
- Depends on: MoveIt 2, OMPL, ros2_control
- Used by: Stage implementations

**Bluesky Integration Layer (`src/bluesky_ros/`):**
- Purpose: Bridge Bluesky experiment control with ROS2 orchestrator
- Contains: Ophyd device wrapper, async action handling
- Key file: `mtc_ophyd_device.py`
- Depends on: Bluesky, Ophyd, ROS2 action client
- Used by: Bluesky RunEngine

## Data Flow

**JSON Task Execution (Primary Flow):**

1. User submits JSON task script (via Bluesky, GUI, or direct action call)
2. MTCOrchestratorServer parses JSON, loads beamline config
3. Orchestrator launches/reuses MoveIt for current gripper
4. For each task in sequence:
   - Simple tasks (moveto, end_effector): Batched into single MTC execution
   - Complex tasks (pick_place, vision_*): Sent to specialized action server
5. Action server creates MTC task with appropriate stages
6. MTC plans trajectory via OMPL/Cartesian planner
7. UR driver executes trajectory
8. Result propagates back through layers to user

**Vision-Guided Pick (Example):**

1. VisionPickPlaceAction goal received
2. VisionPickPlaceStages triggers Zivid capture
3. Camera wrapper detects markers/circles/contours
4. TF2 transforms detection from camera frame to robot base
5. First MTC task: Open → approach → detect → grasp → close → retreat
6. Second MTC task: Place approach → place → open → retreat

**State Management:**
- Gripper state tracked in orchestrator (`_current_gripper`)
- MoveIt lifecycle managed per gripper (restarts on tool exchange)
- Task batching state cleared on batch-breaking operations

## Key Abstractions

**BaseActionServer (`src/beambot/beambot/action_servers/base_action_server.py`):**
- Purpose: Common action server infrastructure
- Pattern: Template Method (goal lifecycle, concurrent execution prevention)
- Examples: All 7 specialized servers extend this

**BaseStages (`src/beambot/beambot/stages/base_stages.py`):**
- Purpose: MTC pipeline building utilities
- Pattern: Factory (planner creation), Template (task template)
- Provides: `create_task_template()`, `make_pipeline_planner()`, `make_cartesian_planner()`

**Camera Factory (`src/beambot/beambot/camera/__init__.py`):**
- Purpose: Camera-agnostic detection interface
- Pattern: Factory (`get_camera()` returns camera implementation)
- Examples: ZividCamera (currently only implementation)

**MoveItLifecycleManager (`src/beambot/beambot/core/moveit_lifecycle_manager.py`):**
- Purpose: MoveIt subprocess management, controller activation
- Pattern: Lifecycle (start/stop MoveIt per gripper configuration)

## Entry Points

**Main Launch File:**
- Location: `src/beambot/launch/beambot_bringup.launch.py`
- Triggers: `ros2 launch beambot beambot_bringup.launch.py`
- Responsibilities: Start all action servers, configure beamline, optional vision/pipettor

**GUI Client:**
- Location: `src/mtc_gui/mtc_gui/mtc_gui_client.py`
- Triggers: `ros2 run mtc_gui mtc_gui_client`
- Responsibilities: Desktop interface for task creation and execution

**Bluesky Integration:**
- Location: `src/bluesky_ros/mtc_ophyd_device.py`
- Triggers: Bluesky RunEngine `yield from robot.set(json_task)`
- Responsibilities: Ophyd device wrapper for autonomous experiments

## Error Handling

**Strategy:** Exception bubbling with logging at each layer

**Patterns:**
- Action servers catch exceptions, log with context, abort goal
- Stages return boolean success/failure
- Orchestrator tracks partial progress, can pause/resume
- MoveIt failures propagate as planning/execution errors

**Recovery:**
- Pause/resume via `/beambot/pause` service
- Controller activation retry in MoveItLifecycleManager
- Vision detection retry logic (configurable count and delay)

## Cross-Cutting Concerns

**Logging:**
- ROS2 `self.get_logger()` throughout
- Levels: debug, info, warn, error
- Format: f-strings with context (e.g., `f"Step {i}: {task_type}"`)

**Validation:**
- JSON task parsing in orchestrator
- Gripper existence check before tool exchange
- Pose reference validation (TODO: needs improvement)

**Configuration:**
- Single beamline YAML loaded at startup
- Propagated to all action servers via constructor params
- Gripper-specific MoveIt configs selected dynamically

**Transforms:**
- TF2 for camera → robot base transforms
- Hand-eye calibration stored in XACRO

---

*Architecture analysis: 2026-01-17*
*Update when major patterns change*
