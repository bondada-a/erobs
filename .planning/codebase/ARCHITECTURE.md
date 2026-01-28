# Architecture

**Analysis Date:** 2026-01-27

## Pattern Overview

**Overall:** Hierarchical Task Execution with ROS2 Action Server Composition

**Key Characteristics:**
- Three-tier action server architecture (Orchestrator → Specialized Servers → MTC Stages)
- Configuration-driven deployment (single YAML for beamline-specific settings)
- Task batching for performance optimization (~1.5s saved per batched task)
- Camera-agnostic vision system (factory pattern)

## Layers

**Orchestration Layer:**
- Purpose: Central coordination, task dispatch, MoveIt lifecycle management
- Contains: `orchestrator.py` - Receives JSON task scripts, manages batching
- Location: `src/beambot/beambot/action_servers/orchestrator.py`
- Depends on: All specialized action servers, MoveIt lifecycle manager
- Used by: GUI client, Bluesky Ophyd device

**Action Server Layer:**
- Purpose: Task-specific execution via MTC pipelines
- Contains: 8 specialized servers (move_to, pick_place, vision, end_effector, tool_exchange, vision_pick_place, pipettor)
- Location: `src/beambot/beambot/action_servers/*_server.py`
- Depends on: Stage implementations, MTC core
- Used by: Orchestrator (dispatch), direct action clients

**Stage Layer:**
- Purpose: MTC stage composition and execution
- Contains: Reusable stage builders (move_to, pick_place, vision, gripper control)
- Location: `src/beambot/beambot/stages/*_stages.py`
- Depends on: `moveit.task_constructor`, base_stages utilities
- Used by: Action servers

**Vision Layer:**
- Purpose: Camera abstraction, detection methods
- Contains: Zivid wrapper with ArUco, circle, contour detection
- Location: `src/beambot/beambot/camera/zivid.py`
- Depends on: zivid_interfaces, cv_bridge, OpenCV
- Used by: Vision stages

**Hardware Layer:**
- Purpose: Robot descriptions, gripper drivers, MoveIt configs
- Contains: URDF/XACRO, ros2_control interfaces, MoveIt SRDF
- Location: `src/custom-ur-descriptions/`, `src/end_effectors/`
- Depends on: ur_robot_driver, ros2_control
- Used by: MoveIt, stage layer

## Data Flow

**Task Execution (Bluesky/GUI → Robot):**

1. User/AI writes JSON task script
2. Orchestrator receives `MTCExecution.Goal` with full_json
3. Parse JSON: extract tasks, poses, start_gripper
4. MoveIt Lifecycle Manager:
   - Get gripper config from beamline_config.yaml
   - Set tool voltage (UR secondary port 30002)
   - Launch MoveIt subprocess (gripper-specific launch)
   - Load collision obstacles
5. Task Grouping (if batching enabled):
   - Group consecutive moveto + end_effector tasks
   - Complex tasks via individual action servers
6. For each batch:
   - Batched: Create single MTC Task, add stages, plan once
   - Single: Dispatch to specialized action server
7. MTC Execution:
   - Pipeline planner queries IK (KDL)
   - OMPL plans collision-free trajectory
   - Execute on real robot via UR driver

**State Management:**
- Pause/Resume via ROS2 services (`/beambot/pause`, `/beambot/resume`)
- Execution state published on `/beambot/execution_state`
- Current gripper tracked after tool_exchange operations

## Key Abstractions

**BaseActionServer:**
- Purpose: Template for all action servers
- Location: `src/beambot/beambot/action_servers/base_action_server.py`
- Examples: All *_server.py inherit from this
- Pattern: Template Method (subclasses implement `initialize_stages()`)

**BaseStages:**
- Purpose: MTC utilities, planner factories, common operations
- Location: `src/beambot/beambot/stages/base_stages.py`
- Examples: `MoveToStages`, `PickPlaceStages`, `VisionStages`
- Pattern: Strategy (different stage implementations)

**Dual-Mode Stages:**
- Purpose: Support both standalone and batched execution
- Pattern: `add_to_task()` for batching, `run()` for standalone
- Examples: All stage classes support both modes

**Detection Factory:**
- Purpose: Camera-agnostic vision interface
- Location: `src/beambot/beambot/camera/zivid.py`
- Pattern: Factory (detection_type → detection method)

## Entry Points

**Primary Launch:**
- Location: `src/beambot/launch/beambot_bringup.launch.py`
- Triggers: `ros2 launch beambot beambot_bringup.launch.py`
- Responsibilities: Launch all 8 action servers, orchestrator, optionally camera

**GUI Client:**
- Location: `src/mtc_gui/mtc_gui/mtc_gui_client.py`
- Triggers: `ros2 run mtc_gui mtc_gui_client`
- Responsibilities: Task composition, camera view, execution monitoring

**Bluesky Integration:**
- Location: `src/bluesky_ros/mtc_ophyd_device.py`
- Triggers: `device.set(json_string)` in Bluesky plan
- Responsibilities: Ophyd device wrapper for action client

## Error Handling

**Strategy:** Exceptions bubble up to action server, logged, returned as failed result

**Patterns:**
- Stage failures return `False`, logged with context
- Action servers catch exceptions, set result.success = False
- Orchestrator handles per-task failures, can continue or abort batch
- Pause/resume for manual intervention

## Cross-Cutting Concerns

**Logging:**
- ROS 2 logging via `self.get_logger()` or `self.logger`
- Levels: debug, info, warn, error
- Context included (task type, target, gripper)

**Validation:**
- JSON parsed with try/except on JSONDecodeError
- Pose lookups validated before use
- Gripper states validated from config

**Configuration:**
- Single source of truth: `src/beambot/config/default_beamline.yaml`
- Loaded once at orchestrator startup
- Gripper configs, robot IP, vision settings

---

*Architecture analysis: 2026-01-27*
*Update when major patterns change*
