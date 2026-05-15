"""Main window: layout, menus, step list, execution controls, status log."""

import json
import time

from pathlib import Path

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSplitter,
    QLabel,
    QCheckBox,
    QComboBox,
    QLineEdit,
    QTextEdit,
    QPushButton,
    QProgressBar,
    QFileDialog,
    QMessageBox,
    QGroupBox,
    QDialog,
    QTabWidget,
    QListWidget,
    QListWidgetItem,
    QFrame,
    QSizePolicy,
)
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QFont
from action_msgs.msg import GoalStatus

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE
from .camera_panel import CameraPanel
from .pose_dialogs import PosesManagerDialog, SavePoseDialog
from .poses_panel import PosesPanel
from .chat_panel import ChatPanel
from .agent_bridge import AgentBridge
from .step_list_panel import StepListPanel, TASK_TYPE_CONFIG

try:
    from .visualization_panel import VisualizationPanel, WEBENGINE_AVAILABLE
except ImportError:
    WEBENGINE_AVAILABLE = False

# Default task step templates
TASK_DEFAULTS = {
    "moveto": {
        "task_type": "moveto",
        "target": "moveit_home",
        "planning_type": "joint",
    },
    "pick_sample": {
        "task_type": "pick_sample",
        "use_vision": True,
        "detection_type": "marker",
        "tag_id": 0,
        "scan_pose": "",
        "z_offset": 0.0,
    },
    "place_sample": {
        "task_type": "place_sample",
        "use_vision": True,
        "detection_type": "marker",
        "tag_id": 0,
        "scan_pose": "",
        "z_offset": 0.0,
    },
    "vision_scan": {
        "task_type": "vision_scan",
        "scan_positions": [],
        "scans_per_position": 3,
        "timeout": 10.0,
    },
    "tool_exchange": {
        "task_type": "tool_exchange",
        "operation": "load",
        "gripper": "hande",
        "dock_number": 3,
        "approach_pose": "load_approach",
    },
    "end_effector": {
        "task_type": "end_effector",
        "end_effector_type": "epick",
        "end_effector_action": "vacuum_on",
    },
    "vision_moveto": {
        "task_type": "vision_moveto",
        "detection_type": "marker",
        "tag_id": 0,
        "timeout": 10.0,
        "z_offset": 0.0,
        "marker_dictionary": "aruco4x4_50",
    },
    "pipettor": {"task_type": "pipettor", "operation": "SUCK", "volume_pct": 0.5},
}


def task_summary(step):
    """One-line summary for the task tree."""
    t = step.get("task_type", "?")
    if t == "moveto":
        d = step.get("direction", "")
        if d:
            return f"Relative {d} {step.get('distance', 0)}m"
        cart = step.get("cartesian_target")
        if cart:
            return f"Cartesian [{', '.join(f'{v:.3f}' for v in cart)}]"
        return f"Move to {step.get('target', '?')}"
    elif t == "pick_sample":
        if step.get("use_vision", True):
            return f"Vision pick ({step.get('detection_type', 'marker')}, tag {step.get('tag_id', 0)})"
        return f"Hardcoded pick -> {step.get('target_pose', '?')}"
    elif t == "place_sample":
        if step.get("use_vision", True):
            return f"Vision place ({step.get('detection_type', 'marker')}, tag {step.get('tag_id', 0)})"
        return f"Hardcoded place -> {step.get('target_pose', '?')}"
    elif t == "vision_scan":
        n = len(step.get("scan_positions", []))
        return f"Scan {n} positions ({step.get('scans_per_position', 3)}x each)"
    elif t == "tool_exchange":
        return f"{step.get('operation', '?')} {step.get('gripper', '?')} at dock {step.get('dock_number', '?')}"
    elif t == "end_effector":
        return f"{step.get('end_effector_type', '?')} {step.get('end_effector_action', '?')}"
    elif t == "vision_moveto":
        prefix = "[detect only] " if step.get("detect_only") else ""
        det = step.get("detection_type", "marker")
        s = f"{prefix}Detect ArUco {step.get('tag_id', 0)}"
        od = step.get("offset_direction", "")
        if od:
            s += f" +{od} {step.get('offset_distance', 0)}m"
        return s
    elif t == "pipettor":
        op = step.get("operation", "SUCK")
        if op in ("SUCK", "EXPEL"):
            return f"{op} {step.get('volume_pct', 0) * 100:.0f}%"
        elif op == "SET_LED":
            c = step.get("led_color", {})
            return f"LED ({c.get('r', 0):.1f},{c.get('g', 0):.1f},{c.get('b', 0):.1f})"
        return op
    return t


class MTCMainWindow(QMainWindow):
    def __init__(self, ros2: ROS2Bridge):
        super().__init__()
        self.ros2 = ros2
        self.config = {
            "start_gripper": "epick",
            "poses": {
                "home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            },
            "tasks": [],
        }
        self.current_json_file = None
        self.current_robot_pose = None
        # "human" or "agent" — tells _on_result whether to notify the bridge
        self._execution_initiator = "human"
        self._last_goal_was_dry_run = False

        # Load beamline YAML once, before _build_central uses fields from it.
        # Soft-fail: GUI can still open as a JSON inspector when no robot is
        # configured; operator can type the IP into the QLineEdit by hand.
        self._beamline_config, self._beamline_config_path = self._load_beamline_yaml()

        self.setWindowTitle("MTC GUI Client (beambot)")
        self.resize(1400, 900)

        self._build_menu()
        self._build_central()
        self._connect_signals()
        self._load_beamline_poses()

    # --- Menu ---

    def _build_menu(self):
        mb = self.menuBar()
        file_menu = mb.addMenu("File")
        load_action = file_menu.addAction("Load JSON", self._load_json)
        load_action.setShortcut("Ctrl+O")
        save_action = file_menu.addAction("Save JSON", self._save_json)
        save_action.setShortcut("Ctrl+S")
        file_menu.addSeparator()
        exit_action = file_menu.addAction("Exit", self.close)
        exit_action.setShortcut("Ctrl+Q")
        view_menu = mb.addMenu("View")
        self.dark_mode_action = view_menu.addAction("Dark Mode")
        self.dark_mode_action.setCheckable(True)
        self.dark_mode_action.setChecked(True)
        self.dark_mode_action.toggled.connect(self._toggle_dark_mode)

        help_menu = mb.addMenu("Help")
        help_menu.addAction("About", self._show_about)

    # --- Central Layout ---

    def _build_central(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)

        # Config bar (top)
        config_box = QGroupBox("Robot Configuration")
        config_layout = QHBoxLayout(config_box)
        config_layout.addWidget(QLabel("Robot IP:"))
        default_ip = self._beamline_config.get("robot", {}).get("ip", "")
        # Read-only: the IP comes from $BEAMBOT_BEAMLINE_CONFIG (YAML is the
        # single source of truth). Displayed here for operator awareness;
        # not editable from the GUI to avoid silently diverging from what
        # the orchestrator actually connects to.
        self.robot_ip_edit = QLineEdit(default_ip)
        self.robot_ip_edit.setMaximumWidth(150)
        self.robot_ip_edit.setReadOnly(True)
        self.robot_ip_edit.setToolTip(
            "Sourced from $BEAMBOT_BEAMLINE_CONFIG (robot.ip). "
            "Edit the YAML and restart to change."
        )
        config_layout.addWidget(self.robot_ip_edit)
        config_layout.addWidget(QLabel("Start Gripper:"))
        self.gripper_combo = QComboBox()
        self.gripper_combo.addItems(list(self._beamline_config.get("grippers", {}).keys()))
        # Changing gripper invalidates any cached dry-run plan (different SRDF).
        self.gripper_combo.currentTextChanged.connect(
            lambda _: self._set_plan_cached(False)
        )
        config_layout.addWidget(self.gripper_combo)
        test_btn = QPushButton("Test Server")
        test_btn.clicked.connect(self.ros2.test_server)
        config_layout.addWidget(test_btn)
        poses_btn = QPushButton("Manage Poses")
        poses_btn.clicked.connect(self._manage_poses)
        config_layout.addWidget(poses_btn)
        if ROS2_AVAILABLE:
            save_pose_btn = QPushButton("Save Current Pose")
            save_pose_btn.clicked.connect(self._save_current_pose)
            config_layout.addWidget(save_pose_btn)
        config_layout.addStretch()
        layout.addWidget(config_box)

        # Main splitter — three panes: sidebar | center (steps) | right tabs
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(self.main_splitter, stretch=1)

        self.main_splitter.addWidget(self._build_sidebar())
        self.main_splitter.addWidget(self._build_center_pane())
        self.main_splitter.addWidget(self._build_right_tabs())

        # Sidebar narrow, center pane gets the room, right tabs middling
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 3)
        self.main_splitter.setStretchFactor(2, 2)
        self.main_splitter.setSizes([240, 720, 440])

    # --- Layout sub-builders ---

    def _build_sidebar(self) -> QWidget:
        """Narrow left sidebar with TASKS | POSES | RUNS tabs."""
        self.left_tabs = QTabWidget()
        self.left_tabs.setMinimumWidth(220)
        self.left_tabs.setMaximumWidth(320)

        # --- TASKS tab: vertical add-task palette + templates section ---
        tasks_tab = QWidget()
        tasks_layout = QVBoxLayout(tasks_tab)
        tasks_layout.setContentsMargins(6, 6, 6, 6)
        tasks_layout.setSpacing(6)

        add_label = QLabel("ADD TASK")
        add_label.setFont(QFont("Sans", 8, QFont.Weight.Bold))
        add_label.setStyleSheet("color: #888888; letter-spacing: 1px;")
        tasks_layout.addWidget(add_label)

        # The vertical task palette doubles as our self.task_toolbar so
        # _execute / _on_result can still disable it during runs.
        self.task_toolbar = QListWidget()
        self.task_toolbar.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self.task_toolbar.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.task_toolbar.setFrameShape(QFrame.Shape.NoFrame)
        self.task_toolbar.setSpacing(2)
        self.task_toolbar.setStyleSheet(
            "QListWidget { background-color: transparent; border: none; }"
            "QListWidget::item { padding: 6px 4px; border-radius: 4px; }"
            "QListWidget::item:hover { background-color: rgba(255,255,255,0.06); }"
        )
        palette_items = [
            ("Move To", "moveto"),
            ("Pick Sample", "pick_sample"),
            ("Place Sample", "place_sample"),
            ("Tool Exchange", "tool_exchange"),
            ("End Effector", "end_effector"),
            ("Vision MoveTo", "vision_moveto"),
            ("Vision Scan", "vision_scan"),
            ("Pipettor", "pipettor"),
        ]
        item_height = 32
        for label, task_type in palette_items:
            cfg = TASK_TYPE_CONFIG.get(task_type, {})
            icon = cfg.get("icon", "+")
            item = QListWidgetItem(f"  {icon}   {label}")
            item.setData(Qt.ItemDataRole.UserRole, task_type)
            item.setSizeHint(QSize(0, item_height))
            self.task_toolbar.addItem(item)
        self.task_toolbar.itemClicked.connect(
            lambda it: self._add_task(it.data(Qt.ItemDataRole.UserRole))
        )
        # Pin the palette to exactly fit all items so nothing is hidden behind a scrollbar.
        palette_h = (
            item_height * len(palette_items)
            + 2 * self.task_toolbar.spacing() * len(palette_items)
            + 4
        )
        self.task_toolbar.setFixedHeight(palette_h)
        self.task_toolbar.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        tasks_layout.addWidget(self.task_toolbar)

        templates_label = QLabel("TEMPLATES")
        templates_label.setFont(QFont("Sans", 8, QFont.Weight.Bold))
        templates_label.setStyleSheet("color: #888888; letter-spacing: 1px;")
        tasks_layout.addSpacing(8)
        tasks_layout.addWidget(templates_label)

        templates_placeholder = QLabel("No saved templates yet.")
        templates_placeholder.setStyleSheet("color: #666666; font-size: 10px;")
        templates_placeholder.setWordWrap(True)
        tasks_layout.addWidget(templates_placeholder)

        tasks_layout.addStretch(1)

        self.left_tabs.addTab(tasks_tab, "Tasks")

        # --- POSES tab ---
        self.poses_panel = PosesPanel()
        self.poses_panel.poses_loaded.connect(self._on_poses_loaded)
        # Double-click a pose to append a Move To step targeting it.
        self.poses_panel.pose_activated.connect(
            lambda name, _values: self._add_moveto_for_pose(name)
        )
        self.left_tabs.addTab(self.poses_panel, "Poses")

        # --- RUNS tab (placeholder) ---
        runs_tab = QWidget()
        runs_layout = QVBoxLayout(runs_tab)
        runs_layout.setContentsMargins(6, 6, 6, 6)
        runs_placeholder = QLabel("Run history is not implemented yet.")
        runs_placeholder.setStyleSheet("color: #666666; font-size: 10px;")
        runs_placeholder.setWordWrap(True)
        runs_placeholder.setAlignment(Qt.AlignmentFlag.AlignTop)
        runs_layout.addWidget(runs_placeholder)
        runs_layout.addStretch(1)
        self.left_tabs.addTab(runs_tab, "Runs")

        return self.left_tabs

    def _build_center_pane(self) -> QWidget:
        """Center pane: execution toolbar (top), step list (middle), status log (bottom)."""
        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(4, 4, 4, 4)
        center_layout.setSpacing(4)

        # Top execution toolbar — Execute / Pause / Resume / Stop + progress
        exec_bar = QFrame()
        exec_bar.setFrameShape(QFrame.Shape.NoFrame)
        exec_layout = QHBoxLayout(exec_bar)
        exec_layout.setContentsMargins(4, 4, 4, 4)
        exec_layout.setSpacing(6)

        self.exec_btn = QPushButton("Execute")
        self.exec_btn.clicked.connect(self._execute)
        exec_layout.addWidget(self.exec_btn)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self.ros2.pause_task)
        exec_layout.addWidget(self.pause_btn)
        self.resume_btn = QPushButton("Resume")
        self.resume_btn.setEnabled(False)
        self.resume_btn.clicked.connect(self.ros2.resume_task)
        exec_layout.addWidget(self.resume_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.ros2.stop_execution)
        exec_layout.addWidget(self.stop_btn)

        self.dry_run_check = QCheckBox("Dry Run")
        self.dry_run_check.setToolTip(
            "Plan-only preview: animate the planned motion in the 3D view "
            "without moving the robot. v1 supports moveto + end_effector only.\n\n"
            "After a successful Dry Run, Execute will replay the previewed "
            "plan exactly — no re-planning."
        )
        exec_layout.addWidget(self.dry_run_check)

        self.plan_cached_label = QLabel("")
        self.plan_cached_label.setStyleSheet(
            "color: #2e7d32; font-size: 11px; font-weight: bold;"
        )
        exec_layout.addWidget(self.plan_cached_label)

        exec_layout.addSpacing(8)
        self.progress_bar = QProgressBar()
        exec_layout.addWidget(self.progress_bar, stretch=1)
        center_layout.addWidget(exec_bar)

        # Step list
        self.step_list = StepListPanel()
        self.step_list.item_double_clicked.connect(self._edit_task_by_index)
        # Drop a pose onto the step list to append a Move To step targeting it.
        self.step_list.pose_dropped.connect(self._add_moveto_for_pose)
        self.step_list.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        center_layout.addWidget(self.step_list, stretch=1)

        # Step reorder / remove controls — sit just under the step list
        step_ops_bar = QFrame()
        step_ops_bar.setFrameShape(QFrame.Shape.NoFrame)
        step_ops = QHBoxLayout(step_ops_bar)
        step_ops.setContentsMargins(4, 0, 4, 0)
        step_ops.setSpacing(4)
        self.up_step_btn = QPushButton("↑ Up")
        self.up_step_btn.clicked.connect(self._move_up)
        self.down_step_btn = QPushButton("↓ Down")
        self.down_step_btn.clicked.connect(self._move_down)
        self.remove_step_btn = QPushButton("✕ Remove")
        self.remove_step_btn.clicked.connect(self._remove_task)
        self.clear_steps_btn = QPushButton("⌫ Clear")
        self.clear_steps_btn.setToolTip("Remove all steps from the sequence")
        self.clear_steps_btn.clicked.connect(self._clear_tasks)
        for b in (
            self.up_step_btn,
            self.down_step_btn,
            self.remove_step_btn,
            self.clear_steps_btn,
        ):
            b.setFlat(True)
            step_ops.addWidget(b)
        step_ops.addStretch(1)
        center_layout.addWidget(step_ops_bar)

        # Status log (bottom)
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(140)
        self.status_log.setFont(QFont("Monospace", 9))
        center_layout.addWidget(self.status_log)

        return center

    def _build_right_tabs(self) -> QWidget:
        """Right panel: Chat / Camera / 3D View tabs (unchanged from before)."""
        right_tabs = QTabWidget()

        self.chat_panel = ChatPanel()
        self.agent_bridge = AgentBridge()
        right_tabs.addTab(self.chat_panel, "Chat")

        if ROS2_AVAILABLE:
            self.camera = CameraPanel(self.ros2)
            right_tabs.addTab(self.camera, "Camera")

        if WEBENGINE_AVAILABLE:
            self.viz_panel = VisualizationPanel(self.ros2)
            right_tabs.addTab(self.viz_panel, "3D View")

        return right_tabs

    # --- Signal Connections ---

    def _connect_signals(self):
        self.ros2.log.connect(self._log)
        self.ros2.joint_state_received.connect(self._on_joint_state)
        self.ros2.action_feedback_received.connect(self._on_feedback)
        self.ros2.action_result_received.connect(self._on_result)

        # 3D visualization panel
        if WEBENGINE_AVAILABLE and hasattr(self, "viz_panel"):
            self.ros2.joint_state_received.connect(self.viz_panel._on_joint_state)
            self.ros2.gripper_changed.connect(self.viz_panel.set_gripper)
            self.ros2.preview_trajectory_received.connect(
                self.viz_panel.play_trajectory
            )

        # Chat panel ↔ Agent bridge
        self.chat_panel.message_submitted.connect(self._on_chat_message)
        self.agent_bridge.response_received.connect(self.chat_panel.append_assistant)
        self.agent_bridge.tool_called.connect(self.chat_panel.append_tool_call)
        self.agent_bridge.thinking_changed.connect(self.chat_panel.set_thinking)
        self.agent_bridge.error_occurred.connect(self.chat_panel.append_error)
        self.agent_bridge.connected.connect(
            lambda n: self._log(f"Agent connected: {n} tools")
        )
        self.agent_bridge.connected.connect(
            lambda n: self.chat_panel.set_status(f"Connected ({n} tools)")
        )

        # Agent → GUI task queue (Plan/Run mode)
        self.agent_bridge.tasks_proposed.connect(self._on_tasks_proposed)
        self.agent_bridge.tasks_cleared.connect(self._on_agent_tasks_cleared)
        self.agent_bridge.execution_requested.connect(self._on_agent_execute_requested)

        # Mode toggle: chat panel ↔ bridge
        self.chat_panel.mode_change_requested.connect(self.agent_bridge.set_mode)
        self.agent_bridge.mode_changed.connect(self._on_agent_mode_changed)

        # Render execution result as a chat bubble for the agent's surface
        self.agent_bridge.execution_outcome.connect(
            self.chat_panel.append_execution_outcome
        )

        # Auto-connect agent on startup
        self.agent_bridge.connect_agent()

    # --- Task Management ---

    def _add_task(self, task_type):
        import copy

        step = copy.deepcopy(TASK_DEFAULTS.get(task_type, {"task_type": task_type}))
        self.config["tasks"].append(step)
        self._refresh_tree()
        self._log(f"Added {task_type}")

    def _add_moveto_for_pose(self, pose_name: str):
        """Append a Move To step targeting the named pose (joint planning)."""
        if not pose_name:
            return
        import copy

        step = copy.deepcopy(TASK_DEFAULTS["moveto"])
        step["target"] = pose_name
        self.config["tasks"].append(step)
        self._refresh_tree()
        self._log(f"Added Move To '{pose_name}'")

    def _remove_task(self):
        indices = sorted(self.step_list.selected_indices(), reverse=True)
        if not indices:
            return
        for i in indices:
            if 0 <= i < len(self.config["tasks"]):
                self.config["tasks"].pop(i)
        self._refresh_tree()

    def _clear_tasks(self):
        n = len(self.config["tasks"])
        if n == 0:
            return
        reply = QMessageBox.question(
            self,
            "Clear sequence",
            f"Remove all {n} step{'s' if n != 1 else ''} from the sequence?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        self.config["tasks"].clear()
        self._refresh_tree()
        self._log(f"Cleared {n} step{'s' if n != 1 else ''}")

    # --- Agent-driven queue updates (Plan/Run mode) ---

    def _on_tasks_proposed(self, tasks: list, options: dict):
        """Agent emitted propose_tasks. Populate the queue for human review."""
        replace = options.get("replace", True)
        if replace:
            self.config["tasks"] = list(tasks)
        else:
            self.config["tasks"].extend(tasks)
        # start_gripper must update the combobox — _execute snapshots from
        # gripper_combo.currentText() at dispatch time, so writing only to
        # config["start_gripper"] would be silently overwritten.
        sg = options.get("start_gripper")
        if sg:
            idx = self.gripper_combo.findText(sg)
            if idx >= 0:
                self.gripper_combo.setCurrentIndex(idx)
        if options.get("poses"):
            self.config.setdefault("poses", {}).update(options["poses"])
        self._refresh_tree()
        self._log(
            f"Agent proposed {len(tasks)} task(s) "
            f"({'replace' if replace else 'append'})"
        )

    def _on_agent_tasks_cleared(self):
        """Agent emitted clear_proposed_tasks."""
        n = len(self.config["tasks"])
        self.config["tasks"] = []
        self._refresh_tree()
        self._log(f"Agent cleared the task queue ({n} step(s) removed)")

    def _on_agent_execute_requested(self):
        """Agent emitted execute_queue (Run mode). Reuse the human Execute path."""
        if self.ros2._current_goal_handle is not None:
            self.agent_bridge.notify_execution_complete(
                False, "Another goal is already executing", 0, 0
            )
            return
        if not self.config["tasks"]:
            self.agent_bridge.notify_execution_complete(
                False, "Task queue is empty", 0, 0
            )
            return
        self._execution_initiator = "agent"
        self._log(f"Agent dispatched execution ({len(self.config['tasks'])} task(s))")
        self._execute()

    def _on_agent_mode_changed(self, mode: str):
        """Bridge confirmed the new mode. Reset chat and update the panel."""
        self.chat_panel.clear_chat()
        self.chat_panel.set_mode_label(mode)
        self.chat_panel.set_status(f"Mode: {mode}")
        self.chat_panel.append_assistant(
            f"Switched to {mode.upper()} mode. Conversation reset."
        )
        self._log(f"Agent mode → {mode}")

    def _move_up(self):
        indices = self.step_list.selected_indices()
        if len(indices) != 1:
            return
        idx = indices[0]
        if idx <= 0:
            return
        tasks = self.config["tasks"]
        tasks[idx], tasks[idx - 1] = tasks[idx - 1], tasks[idx]
        self._refresh_tree()
        self.step_list.set_current_row(idx - 1)

    def _move_down(self):
        indices = self.step_list.selected_indices()
        if len(indices) != 1:
            return
        idx = indices[0]
        tasks = self.config["tasks"]
        if idx >= len(tasks) - 1:
            return
        tasks[idx], tasks[idx + 1] = tasks[idx + 1], tasks[idx]
        self._refresh_tree()
        self.step_list.set_current_row(idx + 1)

    def _edit_task_by_index(self, idx):
        if idx < 0 or idx >= len(self.config["tasks"]):
            return
        step = self.config["tasks"][idx]

        from .task_forms import open_task_form

        result = open_task_form(step, idx, self.config.get("poses", {}), self)
        if result is not None:
            self.config["tasks"][idx] = result
            self._refresh_tree()
            self._log(f"Updated step {idx + 1}")

    def _refresh_tree(self):
        self.step_list.refresh(self.config["tasks"], task_summary)
        # Any task list change invalidates the cached dry-run plan on the
        # orchestrator (different goal JSON = different cache key). Mirror
        # that on the client so the operator doesn't see "plan cached" for
        # a task they just edited.
        self._set_plan_cached(False)

    _PLAN_CACHED_GREEN = "color: #2e7d32; font-size: 11px; font-weight: bold;"
    _PLAN_CACHED_RED = "color: #c62828; font-size: 11px; font-weight: bold;"

    def _set_plan_cached(self, cached: bool, reason: str = ""):
        """Update the 'Plan cached' indicator next to the Dry Run checkbox."""
        if cached:
            self.plan_cached_label.setStyleSheet(self._PLAN_CACHED_GREEN)
            self.plan_cached_label.setText(
                "✓ Plan cached — Execute will replay preview"
            )
        elif reason:
            self.plan_cached_label.setStyleSheet(self._PLAN_CACHED_RED)
            self.plan_cached_label.setText(f"⚠ {reason}")
            from PyQt6.QtCore import QTimer

            QTimer.singleShot(8000, self._clear_plan_cache_warning)
        else:
            self.plan_cached_label.setText("")

    def _clear_plan_cache_warning(self):
        self.plan_cached_label.setText("")
        self.plan_cached_label.setStyleSheet(self._PLAN_CACHED_GREEN)

    # --- Execution ---

    # Task types previewable in v1 dry-run (must match orchestrator's
    # DRY_RUN_SUPPORTED_TYPES). Mirrored client-side so the operator gets
    # an immediate, friendly message instead of a goal rejection.
    _DRY_RUN_SUPPORTED = {"moveto", "end_effector"}

    def _execute(self):
        if not self.config["tasks"]:
            QMessageBox.warning(self, "Warning", "No tasks defined")
            return

        dry_run = self.dry_run_check.isChecked()
        if dry_run:
            unsupported = [
                (i + 1, t.get("task_type", "?"))
                for i, t in enumerate(self.config["tasks"])
                if t.get("task_type", "") not in self._DRY_RUN_SUPPORTED
            ]
            if unsupported:
                bad = "\n  - ".join(f"step {n}: {tt}" for n, tt in unsupported)
                QMessageBox.warning(
                    self,
                    "Dry Run Unavailable",
                    f"Dry-run preview only supports moveto + end_effector in v1.\n\n"
                    f"Unsupported steps:\n  - {bad}\n\n"
                    f"Uncheck 'Dry Run' to execute, or remove these steps to preview.",
                )
                return

        self.config["start_gripper"] = self.gripper_combo.currentText()
        self._last_goal_was_dry_run = dry_run
        self.exec_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.pause_btn.setEnabled(not dry_run)
        self.task_toolbar.setEnabled(False)
        self.progress_bar.setValue(0)
        self.step_list.start_execution(len(self.config["tasks"]))
        self.ros2.execute_task(json.dumps(self.config), dry_run=dry_run)

    def _on_feedback(self, progress, step, action, gripper, msg):
        self.progress_bar.setValue(int(progress))
        self.step_list.update_step(step, progress, action)
        self._log(f"[{progress:.0f}%] Step {step}: {action} | {gripper} | {msg}")

    def _on_result(self, status, error_msg, completed, total):
        self.exec_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.pause_btn.setEnabled(False)
        self.resume_btn.setEnabled(False)
        self.task_toolbar.setEnabled(True)
        was_dry_run = self._last_goal_was_dry_run

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.progress_bar.setValue(100)
            self.step_list.finish_execution("success", completed)
            if was_dry_run:
                self._log(
                    f"Dry-run preview complete: {completed}/{total} steps planned"
                )
                # Plan is now cached on the orchestrator. Mirror that in the
                # GUI so the operator knows Execute will replay the preview.
                self._set_plan_cached(True)
            else:
                self._log(f"Task completed: {completed}/{total} steps")
                # A successful execute drops the cache server-side; keep
                # the GUI in sync.
                self._set_plan_cached(False)
        elif status == GoalStatus.STATUS_CANCELED:
            self.step_list.finish_execution("cancelled", completed)
            self._log(f"Task cancelled: {error_msg}")
        else:
            self.step_list.finish_execution("failed", completed)
            # Friendly message for cached-plan invalidation refusals.
            if error_msg.startswith("CACHE_"):
                # Format: "CACHE_<REASON>: <human message>"
                reason = (
                    error_msg.split(":", 1)[1].strip()
                    if ":" in error_msg
                    else error_msg
                )
                self._log(f"Cached plan invalid: {reason}")
                self._set_plan_cached(False, "Cached plan invalid — run Dry Run again")
                QMessageBox.information(
                    self,
                    "Cached Plan Invalid",
                    f"The previewed plan can no longer be executed:\n\n{reason}\n\n"
                    "Click Dry Run to preview the plan from the current state, "
                    "then Execute.",
                )
            else:
                self._log(f"Task failed: {error_msg} ({completed}/{total} steps)")

        # Notify the agent bridge if this run was agent-initiated; resolves
        # the pending execute_queue future so the agent can continue.
        if self._execution_initiator == "agent":
            success = status == GoalStatus.STATUS_SUCCEEDED
            self.agent_bridge.notify_execution_complete(
                success, error_msg, completed, total
            )
        self._execution_initiator = "human"

    def _on_joint_state(self, pose):
        self.current_robot_pose = pose

    # --- JSON I/O ---

    def _load_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load JSON", "", "JSON (*.json);;All (*)"
        )
        if not path:
            return
        try:
            with open(path) as f:
                self.config = json.load(f)
            if "start_gripper" in self.config:
                idx = self.gripper_combo.findText(self.config["start_gripper"])
                if idx >= 0:
                    self.gripper_combo.setCurrentIndex(idx)
            self.current_json_file = path
            self._refresh_tree()
            self._log(f"Loaded {path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load: {e}")

    def _save_json(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save JSON", "", "JSON (*.json);;All (*)"
        )
        if not path:
            return
        try:
            self.config["start_gripper"] = self.gripper_combo.currentText()
            with open(path, "w") as f:
                json.dump(self.config, f, indent=2)
            self._log(f"Saved to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save: {e}")

    # --- Logging ---

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.status_log.append(f"[{ts}] {msg}")

    def _toggle_dark_mode(self, enabled):
        from .main import toggle_dark_mode

        toggle_dark_mode(self._app(), enabled)

    def _app(self):
        from PyQt6.QtWidgets import QApplication

        return QApplication.instance()

    def _show_about(self):
        QMessageBox.about(
            self,
            "About",
            (
                "MTC GUI Client (PyQt6)\n\n"
                "Task sequence builder for beambot orchestrator.\n"
                "Communicates via ROS2 ActionClient.\n\n"
                "Action server: beambot_execution"
            ),
        )

    # --- Pose Management ---

    def _load_beamline_yaml(self) -> tuple[dict, str | None]:
        """Read $BEAMBOT_BEAMLINE_CONFIG once at startup.

        Returns ({}, None) instead of raising so the GUI still opens for
        operators who only want to inspect a JSON file. Hardware-touching
        consumers (action servers, MCP) fail loudly; this UI surface
        deliberately doesn't.
        """
        import os
        import yaml
        raw = os.environ.get("BEAMBOT_BEAMLINE_CONFIG", "").strip()
        if not raw:
            self._pending_log = (
                "BEAMBOT_BEAMLINE_CONFIG not set; robot IP and pose "
                "registry will be empty until you set it and restart."
            )
            return {}, None
        path = os.path.abspath(os.path.expanduser(raw))
        if not os.path.isfile(path):
            self._pending_log = f"BEAMBOT_BEAMLINE_CONFIG points at missing file: {path}"
            return {}, None
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                self._pending_log = f"{path}: expected a YAML mapping at root"
                return {}, None
            return data, path
        except Exception as e:
            self._pending_log = f"Failed to parse {path}: {e}"
            return {}, None

    def _load_beamline_poses(self):
        """Push poses from the loaded beamline config into the poses panel."""
        if self._beamline_config_path:
            self.poses_panel.load_from_beamline_config(self._beamline_config_path)
        if hasattr(self, "_pending_log"):
            self._log(self._pending_log)
            del self._pending_log

    def _on_poses_loaded(self, poses: dict):
        """Merge beamline poses into the working config."""
        self.config.setdefault("poses", {}).update(poses)
        self._log(f"Loaded {len(poses)} poses from beamline config")

    def _manage_poses(self):
        dlg = PosesManagerDialog(self.config.get("poses", {}), self)
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.result is not None:
            self.config["poses"] = dlg.result
            self._log(f"Updated poses ({len(dlg.result)} poses)")

    def _save_current_pose(self):
        if self.current_robot_pose is None:
            QMessageBox.warning(
                self, "No Pose", "No robot pose available. Is the robot connected?"
            )
            return
        dlg = SavePoseDialog(
            self.current_robot_pose, self.config, self.current_json_file, self
        )
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.result is not None:
            r = dlg.result
            name = r["pose_name"]
            values = r["pose_values"]
            if r["action"] == "add_to_config":
                self.config.setdefault("poses", {})[name] = values
                self._log(f"Added pose '{name}' to config")
            elif r["action"] == "save_to_current" and self.current_json_file:
                self.config.setdefault("poses", {})[name] = values
                self.config["start_gripper"] = self.gripper_combo.currentText()
                with open(self.current_json_file, "w") as f:
                    json.dump(self.config, f, indent=2)
                self._log(f"Saved pose '{name}' to {self.current_json_file}")
            elif r["action"] == "save_to_new":
                self.config.setdefault("poses", {})[name] = values
                path = r.get("file_path")
                if path:
                    with open(path, "w") as f:
                        json.dump(self.config, f, indent=2)
                    self.current_json_file = path
                    self._log(f"Saved pose '{name}' to {path}")

    def _on_chat_message(self, text):
        self.chat_panel.append_user(text)
        self.agent_bridge.send_message(text)

    def closeEvent(self, event):
        self.agent_bridge.disconnect()
        self.ros2.shutdown()
        event.accept()
