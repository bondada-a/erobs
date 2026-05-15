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
    QToolBar,
    QLabel,
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
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from action_msgs.msg import GoalStatus

from .ros2_bridge import ROS2Bridge, ROS2_AVAILABLE
from .camera_panel import CameraPanel
from .pose_dialogs import PosesManagerDialog, SavePoseDialog
from .poses_panel import PosesPanel
from .chat_panel import ChatPanel
from .agent_bridge import AgentBridge
from .step_list_panel import StepListPanel

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
        file_menu.addAction("Load JSON", self._load_json, "Ctrl+O")
        file_menu.addAction("Save JSON", self._save_json, "Ctrl+S")
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close, "Ctrl+Q")
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
        self.robot_ip_edit = QLineEdit("192.168.1.101")
        self.robot_ip_edit.setMaximumWidth(150)
        config_layout.addWidget(self.robot_ip_edit)
        config_layout.addWidget(QLabel("Start Gripper:"))
        self.gripper_combo = QComboBox()
        self.gripper_combo.addItems(["epick", "hande", "2fg7", "pipettor", "none"])
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

        # Main splitter (task editor left, camera right — camera added later)
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(self.main_splitter, stretch=1)

        # Left panel — tabbed (Tasks | Poses)
        self.left_tabs = QTabWidget()

        # --- Tasks tab ---
        tasks_tab = QWidget()
        left_layout = QVBoxLayout(tasks_tab)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # Task toolbar
        self.task_toolbar = QToolBar()
        self.task_toolbar.setMovable(False)
        task_toolbar = self.task_toolbar
        for label, task_type in [
            ("Add MoveTo", "moveto"),
            ("Add Pick", "pick_sample"),
            ("Add Place", "place_sample"),
            ("Add Tool Exchange", "tool_exchange"),
            ("Add End Effector", "end_effector"),
            ("Add Vision MoveTo", "vision_moveto"),
            ("Add Vision Scan", "vision_scan"),
            ("Add Pipettor", "pipettor"),
        ]:
            action = task_toolbar.addAction(label)
            action.triggered.connect(lambda checked, tt=task_type: self._add_task(tt))

        task_toolbar.addSeparator()
        task_toolbar.addAction("Remove", self._remove_task)
        task_toolbar.addAction("Up", self._move_up)
        task_toolbar.addAction("Down", self._move_down)
        left_layout.addWidget(task_toolbar)

        # Step list panel (replaces old QTreeWidget)
        self.step_list = StepListPanel()
        self.step_list.item_double_clicked.connect(self._edit_task_by_index)
        left_layout.addWidget(self.step_list, stretch=1)

        # Execution bar
        exec_box = QGroupBox("Execution")
        exec_layout = QHBoxLayout(exec_box)
        self.exec_btn = QPushButton("Execute")
        self.exec_btn.clicked.connect(self._execute)
        exec_layout.addWidget(self.exec_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.ros2.stop_execution)
        exec_layout.addWidget(self.stop_btn)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self.ros2.pause_task)
        exec_layout.addWidget(self.pause_btn)
        self.resume_btn = QPushButton("Resume")
        self.resume_btn.setEnabled(False)
        self.resume_btn.clicked.connect(self.ros2.resume_task)
        exec_layout.addWidget(self.resume_btn)
        self.progress_bar = QProgressBar()
        exec_layout.addWidget(self.progress_bar, stretch=1)
        left_layout.addWidget(exec_box)

        # Status log
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(180)
        self.status_log.setFont(QFont("Monospace", 9))
        left_layout.addWidget(self.status_log)

        self.left_tabs.addTab(tasks_tab, "Tasks")

        # --- Poses tab ---
        self.poses_panel = PosesPanel()
        self.poses_panel.poses_loaded.connect(self._on_poses_loaded)
        self.left_tabs.addTab(self.poses_panel, "Poses")

        self.main_splitter.addWidget(self.left_tabs)

        # Right panel — tabbed (Chat | Camera)
        right_tabs = QTabWidget()

        # Chat panel (always available)
        self.chat_panel = ChatPanel()
        self.agent_bridge = AgentBridge()
        right_tabs.addTab(self.chat_panel, "Chat")

        # Camera panel (only if ROS2 available)
        if ROS2_AVAILABLE:
            self.camera = CameraPanel(self.ros2)
            right_tabs.addTab(self.camera, "Camera")

        # 3D visualization panel
        if WEBENGINE_AVAILABLE:
            self.viz_panel = VisualizationPanel(self.ros2)
            right_tabs.addTab(self.viz_panel, "3D View")

        self.main_splitter.addWidget(right_tabs)
        self.main_splitter.setStretchFactor(0, 3)
        self.main_splitter.setStretchFactor(1, 1)

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

        # Auto-connect agent on startup
        self.agent_bridge.connect_agent()

    # --- Task Management ---

    def _add_task(self, task_type):
        import copy

        step = copy.deepcopy(TASK_DEFAULTS.get(task_type, {"task_type": task_type}))
        self.config["tasks"].append(step)
        self._refresh_tree()
        self._log(f"Added {task_type}")

    def _remove_task(self):
        indices = sorted(self.step_list.selected_indices(), reverse=True)
        if not indices:
            return
        for i in indices:
            if 0 <= i < len(self.config["tasks"]):
                self.config["tasks"].pop(i)
        self._refresh_tree()

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

    # --- Execution ---

    def _execute(self):
        if not self.config["tasks"]:
            QMessageBox.warning(self, "Warning", "No tasks defined")
            return
        self.config["start_gripper"] = self.gripper_combo.currentText()
        self.exec_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.pause_btn.setEnabled(True)
        self.task_toolbar.setEnabled(False)
        self.progress_bar.setValue(0)
        self.step_list.start_execution(len(self.config["tasks"]))
        self.ros2.execute_task(json.dumps(self.config))

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

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.progress_bar.setValue(100)
            self.step_list.finish_execution("success", completed)
            self._log(f"Task completed: {completed}/{total} steps")
        elif status == GoalStatus.STATUS_CANCELED:
            self.step_list.finish_execution("cancelled", completed)
            self._log(f"Task cancelled: {error_msg}")
        else:
            self.step_list.finish_execution("failed", completed)
            self._log(f"Task failed: {error_msg} ({completed}/{total} steps)")

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

    def _load_beamline_poses(self):
        """Load poses from the beamline config's poses_file on startup."""
        # Resolve workspace root (directory containing src/)
        here = Path(__file__).resolve()
        workspace_root = None
        for parent in here.parents:
            if (parent / "src" / "beambot").exists():
                workspace_root = parent
                break
        if workspace_root is None:
            return

        config_path = (
            workspace_root / "src" / "beambot" / "config" / "default_beamline.yaml"
        )
        if config_path.exists():
            self.poses_panel.load_from_beamline_config(config_path)

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
