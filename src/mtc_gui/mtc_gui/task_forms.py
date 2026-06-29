"""Task edit form dialogs — base class + all 8 task type forms."""

import copy

from PyQt6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QFormLayout,
    QDialogButtonBox,
    QScrollArea,
    QWidget,
    QLabel,
    QLineEdit,
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QSpinBox,
    QGroupBox,
    QHBoxLayout,
    QTextEdit,
    QMessageBox,
    QGridLayout,
    QPushButton,
    QSizePolicy,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


# --- Beamline-driven enumerations -----------------------------------------
# Read fresh on each dialog open so the YAML is the single source. Soft-fails
# to empty lists if BEAMBOT_BEAMLINE_CONFIG isn't set — operator sees an
# empty dropdown and the GUI's startup banner already explains why.


def _configured_grippers() -> list[str]:
    """All gripper names declared in the active beamline YAML."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        return list(config.get("grippers", {}).keys())
    except Exception:
        return []


def _grippers_with_states() -> list[str]:
    """Grippers whose YAML config declares a non-empty `states:` block.

    Excludes grippers like `none` and `pipettor` that have no end-effector
    actions, so the EndEffectorForm only offers grippers that can act.
    """
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        return [
            name
            for name, gconf in config.get("grippers", {}).items()
            if gconf.get("states")
        ]
    except Exception:
        return []


def _end_effector_actions() -> list[str]:
    """Union of all `states.grasp` + `states.release` action names across grippers."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        actions: list[str] = []
        for gconf in config.get("grippers", {}).values():
            states = gconf.get("states") or {}
            for key in ("grasp", "release"):
                val = states.get(key)
                if val and val not in actions:
                    actions.append(val)
        return actions
    except Exception:
        return []


def _vision_target_grid(target_name: str) -> dict:
    """Load grid config for a vision target (rows, cols)."""
    try:
        from beambot.config_loader import load_beamline_config

        config, _ = load_beamline_config()
        targets = config.get("vision_targets", {})
        return targets.get(target_name, {}).get("grid", {})
    except Exception:
        return {}


# --- Dispatch ---


def open_task_form(step, step_index, poses, parent=None, current_pose=None,
                   preview_cb=None, end_preview_cb=None, viz_widget=None):
    """Open the edit dialog for a task step. Returns edited step dict or None if cancelled."""
    task_type = step.get("task_type", "")
    form_cls = _FORMS.get(task_type)
    if not form_cls:
        QMessageBox.warning(parent, "Unknown", f"No form for task type: {task_type}")
        return None
    if form_cls is MoveToForm:
        dialog = form_cls(step, step_index, poses, parent, current_pose=current_pose,
                          preview_cb=preview_cb, end_preview_cb=end_preview_cb,
                          viz_widget=viz_widget)
    else:
        dialog = form_cls(step, step_index, poses, parent)
    if dialog.exec() == QDialog.DialogCode.Accepted:
        return dialog.result
    return None


# --- Base Form ---


class BaseTaskForm(QDialog):
    TITLE = "Edit Step"

    def __init__(self, step, step_index, poses, parent=None):
        super().__init__(parent)
        self.step = copy.deepcopy(step)
        self.poses = poses
        self.result = None

        self.setWindowTitle(f"{self.TITLE} (Step {step_index + 1})")
        self.setMinimumWidth(480)
        self.setModal(True)

        outer = QVBoxLayout(self)

        # Title
        title = QLabel(self.TITLE)
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        outer.addWidget(title)

        # Scrollable form area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form_widget = QWidget()
        self.form = QFormLayout(form_widget)
        scroll.setWidget(form_widget)
        content_row = QHBoxLayout()
        content_row.addWidget(scroll, stretch=1)
        outer.addLayout(content_row, stretch=1)
        self._content_row = content_row

        self.build_form()

        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        outer.addWidget(buttons)

    def build_form(self):
        pass

    def collect_values(self):
        return self.step

    def _on_save(self):
        try:
            self.result = self.collect_values()
            self.accept()
        except (ValueError, KeyError) as e:
            QMessageBox.critical(self, "Invalid Input", str(e))

    # --- Helpers ---

    def add_combo(self, label, key, options):
        combo = QComboBox()
        combo.addItems(options)
        current = str(self.step.get(key, options[0] if options else ""))
        idx = combo.findText(current)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        self.form.addRow(label, combo)
        return combo

    def add_text(self, label, key, default=""):
        edit = QLineEdit(str(self.step.get(key, default)))
        self.form.addRow(label, edit)
        return edit

    def add_float(
        self, label, key, default=0.0, min_val=-999.0, max_val=999.0, decimals=4
    ):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setValue(float(self.step.get(key, default)))
        self.form.addRow(label, spin)
        return spin

    def add_int(self, label, key, default=0, min_val=0, max_val=999):
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(int(self.step.get(key, default)))
        self.form.addRow(label, spin)
        return spin

    def add_check(self, label, key, default=False):
        check = QCheckBox()
        check.setChecked(bool(self.step.get(key, default)))
        self.form.addRow(label, check)
        return check

    def add_section(self, title):
        group = QGroupBox(title)
        layout = QFormLayout(group)
        self.form.addRow(group)
        return layout

    def add_hint(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color: gray; font-size: 10px;")
        lbl.setWordWrap(True)
        self.form.addRow(lbl)

    def add_offset_row(self, section, prefix, keys):
        """Add X/Y/Z offset fields in a horizontal row."""
        row = QHBoxLayout()
        spins = {}
        for axis, key in zip(["X:", "Y:", "Z:"], keys):
            row.addWidget(QLabel(axis))
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(4)
            spin.setValue(float(self.step.get(key, 0.0)))
            spin.setMaximumWidth(90)
            row.addWidget(spin)
            spins[key] = spin
        section.addRow(prefix, row)
        return spins


# --- Concrete Forms ---


class MoveToForm(BaseTaskForm):
    TITLE = "MoveTo Configuration"

    _MODES = ["Named target", "Relative move", "Cartesian target", "Joint values"]

    _JOINT_NAMES = [
        "Shoulder Pan", "Shoulder Lift", "Elbow",
        "Wrist 1", "Wrist 2", "Wrist 3",
    ]
    _JOINT_PRESETS = {
        "Home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "Straight Up": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
        "All Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self, step, step_index, poses, parent=None, current_pose=None,
                 preview_cb=None, end_preview_cb=None, viz_widget=None):
        self._current_pose = current_pose
        self._step_index = step_index
        self._preview_cb = preview_cb
        self._end_preview_cb = end_preview_cb
        self._viz_widget = viz_widget
        super().__init__(step, step_index, poses, parent)

    def _detect_mode(self):
        """Match backend precedence: relative > cartesian > joint > named."""
        if self.step.get("direction") and float(self.step.get("distance", 0)) != 0:
            return "Relative move"
        if self.step.get("cartesian_target"):
            return "Cartesian target"
        target = self.step.get("target", "")
        if target and self.poses.get(target) and len(self.poses[target]) == 6:
            return "Joint values"
        return "Named target"

    def build_form(self):
        # Mode selector
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(self._MODES)
        self.mode_combo.setCurrentText(self._detect_mode())
        self.form.addRow("Move mode:", self.mode_combo)

        self._preview = QLabel("")
        self._preview.setStyleSheet("color: gray; font-size: 10px;")
        self.form.addRow(self._preview)

        # --- Named target section ---
        self._named_group = QGroupBox("Named Target")
        named_layout = QFormLayout(self._named_group)
        self.form.addRow(self._named_group)
        self.target = QLineEdit(str(self.step.get("target", "moveit_home")))
        named_layout.addRow("Target:", self.target)
        hint = QLabel("Named state (moveit_home) or pose name from the pose manager")
        hint.setStyleSheet("color: gray; font-size: 10px;")
        named_layout.addRow(hint)

        # --- Planning type (cartesian mode only) ---
        self._planning_group = QGroupBox()
        self._planning_group.setFlat(True)
        self._planning_group.setStyleSheet("QGroupBox{border:none;}")
        pl = QFormLayout(self._planning_group)
        self.form.addRow(self._planning_group)
        self.planning = QComboBox()
        self.planning.addItems(["joint", "cartesian"])
        current_pt = str(self.step.get("planning_type", "joint"))
        idx = self.planning.findText(current_pt)
        if idx >= 0:
            self.planning.setCurrentIndex(idx)
        pl.addRow("Planning Type:", self.planning)

        # --- Relative move section ---
        self._rel_group = QGroupBox("Relative Move")
        rel = QFormLayout(self._rel_group)
        self.form.addRow(self._rel_group)
        self.direction = QComboBox()
        self.direction.addItems(["forward", "backward", "left", "right", "up", "down"])
        self.direction.setCurrentText(self.step.get("direction", "forward"))
        rel.addRow("Direction:", self.direction)
        self.distance = QDoubleSpinBox()
        self.distance.setRange(0, 2.0)
        self.distance.setDecimals(3)
        self.distance.setValue(float(self.step.get("distance", 0.05)))
        rel.addRow("Distance (m):", self.distance)

        # --- Cartesian target section ---
        self._cart_group = QGroupBox("Cartesian Target")
        cart = QFormLayout(self._cart_group)
        self.form.addRow(self._cart_group)
        existing = self.step.get("cartesian_target", [])
        self.cart_x = QDoubleSpinBox()
        self.cart_x.setRange(-2, 2)
        self.cart_x.setDecimals(4)
        self.cart_y = QDoubleSpinBox()
        self.cart_y.setRange(-2, 2)
        self.cart_y.setDecimals(4)
        self.cart_z = QDoubleSpinBox()
        self.cart_z.setRange(-2, 2)
        self.cart_z.setDecimals(4)
        pos_row = QHBoxLayout()
        for lbl, spin, i in [
            ("X:", self.cart_x, 0),
            ("Y:", self.cart_y, 1),
            ("Z:", self.cart_z, 2),
        ]:
            pos_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            pos_row.addWidget(spin)
        cart.addRow("Position (m):", pos_row)

        self.cart_r = QDoubleSpinBox()
        self.cart_r.setRange(-360, 360)
        self.cart_r.setDecimals(2)
        self.cart_p = QDoubleSpinBox()
        self.cart_p.setRange(-360, 360)
        self.cart_p.setDecimals(2)
        self.cart_yaw = QDoubleSpinBox()
        self.cart_yaw.setRange(-360, 360)
        self.cart_yaw.setDecimals(2)
        rot_row = QHBoxLayout()
        for lbl, spin, i in [
            ("R:", self.cart_r, 3),
            ("P:", self.cart_p, 4),
            ("Y:", self.cart_yaw, 5),
        ]:
            rot_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            rot_row.addWidget(spin)
        cart.addRow("Orientation (deg):", rot_row)

        self.frame_id = QComboBox()
        self.frame_id.addItems(["base_link", "flange"])
        self.frame_id.setCurrentText(self.step.get("frame_id", "base_link"))
        cart.addRow("Frame:", self.frame_id)

        # --- Joint values section ---
        self._joint_group = QGroupBox("Joint Values")
        jl = QFormLayout(self._joint_group)
        self.form.addRow(self._joint_group)

        # Load existing joint values if editing a joint-mode step
        existing_jv = []
        target = self.step.get("target", "")
        if target and self.poses.get(target) and len(self.poses[target]) == 6:
            existing_jv = self.poses[target]

        self.joint_spins = []
        for i, jname in enumerate(self._JOINT_NAMES):
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(2)
            spin.setSuffix(" deg")
            if i < len(existing_jv):
                spin.setValue(existing_jv[i])
            jl.addRow(f"{jname}:", spin)
            self.joint_spins.append(spin)

        # Presets row
        preset_row = QHBoxLayout()
        for pname, pvals in self._JOINT_PRESETS.items():
            btn = QPushButton(pname)
            btn.clicked.connect(lambda checked, v=pvals: self._apply_joint_preset(v))
            preset_row.addWidget(btn)
        jl.addRow("Presets:", preset_row)

        # Read current pose button
        self._read_pose_btn = QPushButton("Read current pose")
        self._read_pose_btn.clicked.connect(self._read_current_pose)
        jl.addRow(self._read_pose_btn)

        # Wire mode switching
        self.mode_combo.currentTextChanged.connect(self._apply_mode)
        self._apply_mode(self.mode_combo.currentText())

        # Wire live preview callbacks
        for spin in self.joint_spins:
            spin.valueChanged.connect(self._emit_preview)
        self.target.textChanged.connect(self._emit_preview)
        self._emit_preview()

        # Embed the 3D viewer to the right of the form
        if self._viz_widget is not None:
            self._viz_widget.setParent(None)
            self._content_row.addWidget(self._viz_widget, stretch=1)
            self._viz_widget.show()
            self.setMinimumWidth(900)
            self.setMinimumHeight(520)

    def _apply_joint_preset(self, values):
        for spin, val in zip(self.joint_spins, values):
            spin.setValue(val)

    def _read_current_pose(self):
        if self._current_pose is None:
            QMessageBox.warning(
                self, "No Pose", "No robot pose available. Is the robot connected?"
            )
            return
        for spin, val in zip(self.joint_spins, self._current_pose):
            spin.setValue(val)

    def _apply_mode(self, mode):
        self._named_group.setVisible(mode == "Named target")
        self._rel_group.setVisible(mode == "Relative move")
        self._cart_group.setVisible(mode == "Cartesian target")
        self._planning_group.setVisible(mode == "Cartesian target")
        self._joint_group.setVisible(mode == "Joint values")
        previews = {
            "Named target": f"Will execute: move to '{self.target.text()}'",
            "Relative move": "Will execute: relative move in selected direction",
            "Cartesian target": "Will execute: move to XYZ coordinates",
            "Joint values": "Will execute: move to explicit joint angles",
        }
        self._preview.setText(previews.get(mode, ""))
        self._emit_preview()

    def _emit_preview(self, _=None):
        """Push the current goal pose to the 3D viewer preview callback."""
        if not self._preview_cb:
            return
        mode = self.mode_combo.currentText()
        if mode == "Joint values":
            self._preview_cb([s.value() for s in self.joint_spins])
        elif mode == "Named target":
            pose = self.poses.get(self.target.text())
            if isinstance(pose, (list, tuple)) and len(pose) == 6:
                self._preview_cb(list(pose))
            elif self._end_preview_cb:
                self._end_preview_cb()
        else:
            if self._end_preview_cb:
                self._end_preview_cb()

    def done(self, r):
        if self._end_preview_cb:
            self._end_preview_cb()
        if self._viz_widget is not None:
            self._viz_widget.setParent(None)
        super().done(r)

    def collect_values(self):
        s = {**self.step}
        mode = self.mode_combo.currentText()

        # Always strip all branches, then add back only the active one
        for k in (
            "target",
            "direction",
            "distance",
            "cartesian_target",
            "frame_id",
            "planning_type",
            "_inline_joint_pose",
        ):
            s.pop(k, None)

        if mode == "Named target":
            s["target"] = self.target.text()
            s["planning_type"] = "joint"
        elif mode == "Relative move":
            s["direction"] = self.direction.currentText()
            s["distance"] = self.distance.value()
        elif mode == "Cartesian target":
            s["planning_type"] = self.planning.currentText()
            x, y, z = self.cart_x.value(), self.cart_y.value(), self.cart_z.value()
            cart = [x, y, z]
            r, p, yaw = self.cart_r.value(), self.cart_p.value(), self.cart_yaw.value()
            if r != 0 or p != 0 or yaw != 0:
                cart.extend([r, p, yaw])
            s["cartesian_target"] = cart
            s["frame_id"] = self.frame_id.currentText()
        elif mode == "Joint values":
            # Reuse existing inline name or generate one from step index
            target = self.step.get("target", "")
            if not (target and self.poses.get(target) and len(self.poses[target]) == 6):
                target = f"moveto_joints_{self._step_index + 1}"
            s["target"] = target
            s["planning_type"] = "joint"
            values = [spin.value() for spin in self.joint_spins]
            s["_inline_joint_pose"] = {"name": target, "values": values}
        return s


class SampleForm(BaseTaskForm):
    """Shared form for pick_sample and place_sample."""

    def __init__(self, step, step_index, poses, parent=None):
        self.mode = "pick" if step.get("task_type") == "pick_sample" else "place"
        self.TITLE = "Pick Sample" if self.mode == "pick" else "Place Sample"
        super().__init__(step, step_index, poses, parent)

    def build_form(self):
        self.use_vision = self.add_check("Use Vision:", "use_vision", True)

        # Vision fields
        self.vision_group = QGroupBox("Vision Mode")
        vl = QFormLayout(self.vision_group)
        self.form.addRow(self.vision_group)

        self.det_type = QComboBox()
        self.det_type.addItems(["marker", "sample_roi"])
        self.det_type.setCurrentText(self.step.get("detection_type", "marker"))
        vl.addRow("Detection Type:", self.det_type)

        self.tag_id = QSpinBox()
        self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        vl.addRow("Tag ID:", self.tag_id)

        self.scan_pose = QLineEdit(self.step.get("scan_pose", ""))
        vl.addRow("Scan Pose:", self.scan_pose)

        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1)
        self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        vl.addRow("Z Offset (m):", self.z_offset)

        self.marker_offsets = self.add_offset_row(
            vl,
            "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"],
        )

        self.offset_dir = QComboBox()
        self.offset_dir.addItems(
            ["", "forward", "backward", "left", "right", "up", "down"]
        )
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        vl.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0)
        self.offset_dist.setDecimals(4)
        self.offset_dist.setValue(float(self.step.get("offset_distance", 0.0)))
        vl.addRow("Flange Distance (m):", self.offset_dist)

        # Hardcoded fields
        self.hardcoded_group = QGroupBox("Hardcoded Mode")
        hl = QFormLayout(self.hardcoded_group)
        self.form.addRow(self.hardcoded_group)

        self.approach_pose = QLineEdit(self.step.get("approach_pose", ""))
        hl.addRow("Approach Pose:", self.approach_pose)
        self.target_pose = QLineEdit(self.step.get("target_pose", ""))
        hl.addRow("Target Pose:", self.target_pose)

        # Toggle visibility
        def toggle(checked):
            self.vision_group.setVisible(checked)
            self.hardcoded_group.setVisible(not checked)

        self.use_vision.stateChanged.connect(
            lambda state: toggle(state == Qt.CheckState.Checked.value)
        )
        toggle(self.use_vision.isChecked())

    def collect_values(self):
        s = {**self.step}
        s["use_vision"] = self.use_vision.isChecked()
        if s["use_vision"]:
            s["detection_type"] = self.det_type.currentText()
            s["tag_id"] = self.tag_id.value()
            s["scan_pose"] = self.scan_pose.text()
            s["z_offset"] = self.z_offset.value()
            for key, spin in self.marker_offsets.items():
                v = spin.value()
                if v != 0.0:
                    s[key] = v
                else:
                    s.pop(key, None)
            od = self.offset_dir.currentText()
            if od:
                s["offset_direction"] = od
                s["offset_distance"] = self.offset_dist.value()
            else:
                s.pop("offset_direction", None)
                s.pop("offset_distance", None)
            s.pop("approach_pose", None)
            s.pop("target_pose", None)
        else:
            s["approach_pose"] = self.approach_pose.text()
            s["target_pose"] = self.target_pose.text()
            for key in (
                "detection_type",
                "tag_id",
                "sample_index",
                "scan_pose",
                "z_offset",
                "marker_offset_x",
                "marker_offset_y",
                "marker_offset_z",
                "offset_direction",
                "offset_distance",
            ):
                s.pop(key, None)
        return s


class VisionScanForm(BaseTaskForm):
    TITLE = "Vision Scan Configuration"

    def build_form(self):
        self.add_hint(
            "Scans markers from multiple positions and caches averaged poses.\n"
            "Subsequent vision_moveto tasks use the cache for faster detection."
        )
        self.form.addRow(QLabel("Scan Positions (one pose name per line):"))
        self.positions_text = QTextEdit()
        self.positions_text.setMaximumHeight(120)
        existing = self.step.get("scan_positions", [])
        if existing:
            self.positions_text.setPlainText("\n".join(existing))
        self.form.addRow(self.positions_text)
        self.spp = self.add_int("Scans Per Position:", "scans_per_position", 3, 1, 20)
        self.timeout = self.add_float("Timeout (s):", "timeout", 10.0, 1.0, 120.0, 1)

    def collect_values(self):
        raw = self.positions_text.toPlainText()
        positions = [p.strip() for p in raw.split("\n") if p.strip()]
        return {
            **self.step,
            "scan_positions": positions,
            "scans_per_position": self.spp.value(),
            "timeout": self.timeout.value(),
        }


class ToolExchangeForm(BaseTaskForm):
    TITLE = "Tool Exchange Configuration"

    def build_form(self):
        self.operation = self.add_combo("Operation:", "operation", ["load", "dock"])
        self.gripper = self.add_combo("Gripper:", "gripper", _configured_grippers())
        self.dock_num = self.add_int("Dock Number:", "dock_number", 3, 1, 10)
        self.approach = self.add_text(
            "Approach Pose:", "approach_pose", "load_approach"
        )
        self.add_hint("Auto convention: load -> load_approach, dock -> dock_approach")

        def update_approach(text):
            if text == "load":
                self.approach.setText("load_approach")
            elif text == "dock":
                self.approach.setText("dock_approach")

        self.operation.currentTextChanged.connect(update_approach)

    def collect_values(self):
        return {
            **self.step,
            "operation": self.operation.currentText(),
            "gripper": self.gripper.currentText(),
            "dock_number": self.dock_num.value(),
            "approach_pose": self.approach.text(),
        }


class EndEffectorForm(BaseTaskForm):
    TITLE = "End Effector Configuration"

    def build_form(self):
        self.type_combo = self.add_combo(
            "Type:", "end_effector_type", _grippers_with_states()
        )
        self.action_combo = self.add_combo(
            "Action:", "end_effector_action", _end_effector_actions()
        )

    def collect_values(self):
        return {
            **self.step,
            "end_effector_type": self.type_combo.currentText(),
            "end_effector_action": self.action_combo.currentText(),
        }


class VisionMoveToForm(BaseTaskForm):
    TITLE = "Vision MoveTo Configuration"

    def build_form(self):
        self.add_hint(
            "Detect object using Zivid camera and move gripper to detected location."
        )
        self.det_type = self.add_combo(
            "Detection Type:", "detection_type", ["marker", "sample_roi"]
        )

        # Marker options
        marker_sec = self.add_section("ArUco Marker Options")
        self.tag_id = QSpinBox()
        self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        marker_sec.addRow("Marker ID:", self.tag_id)
        self.marker_dict = QComboBox()
        self.marker_dict.addItems(
            [
                "aruco4x4_50",
                "aruco4x4_100",
                "aruco4x4_250",
                "aruco5x5_50",
                "aruco5x5_100",
                "aruco5x5_250",
                "aruco6x6_50",
                "aruco6x6_100",
                "aruco6x6_250",
            ]
        )
        self.marker_dict.setCurrentText(
            self.step.get("marker_dictionary", "aruco4x4_50")
        )
        marker_sec.addRow("Dictionary:", self.marker_dict)

        # Common options
        common = self.add_section("Common Options")
        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1)
        self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        common.addRow("Z Offset (m):", self.z_offset)
        self.timeout = QDoubleSpinBox()
        self.timeout.setRange(1.0, 120.0)
        self.timeout.setDecimals(1)
        self.timeout.setValue(float(self.step.get("timeout", 10.0)))
        common.addRow("Timeout (s):", self.timeout)
        self.detect_only = QCheckBox()
        self.detect_only.setChecked(bool(self.step.get("detect_only", False)))
        common.addRow("Detect Only:", self.detect_only)

        # Offsets
        self.marker_offsets = self.add_offset_row(
            common,
            "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"],
        )
        self.offset_dir = QComboBox()
        self.offset_dir.addItems(
            ["", "forward", "backward", "left", "right", "up", "down"]
        )
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        common.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0)
        self.offset_dist.setDecimals(4)
        self.offset_dist.setValue(float(self.step.get("offset_distance", 0.0)))
        common.addRow("Flange Distance (m):", self.offset_dist)

    def collect_values(self):
        s = {**self.step}
        s["detection_type"] = self.det_type.currentText()
        s["tag_id"] = self.tag_id.value()
        s["marker_dictionary"] = self.marker_dict.currentText()
        s["z_offset"] = self.z_offset.value()
        s["timeout"] = self.timeout.value()
        s["detect_only"] = self.detect_only.isChecked()
        for key, spin in self.marker_offsets.items():
            v = spin.value()
            if v != 0.0:
                s[key] = v
            else:
                s.pop(key, None)
        od = self.offset_dir.currentText()
        if od:
            s["offset_direction"] = od
            s["offset_distance"] = self.offset_dist.value()
        else:
            s.pop("offset_direction", None)
            s.pop("offset_distance", None)
        return s


class PipettorForm(BaseTaskForm):
    TITLE = "Pipettor Configuration"

    def build_form(self):
        self.add_hint("SUCK=aspirate, EXPEL=dispense, EJECT_TIP, SET_LED")
        self.operation = self.add_combo(
            "Operation:", "operation", ["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"]
        )
        self.volume = self.add_float("Volume %:", "volume_pct", 0.5, 0.0, 1.0, 2)

        # LED color
        led = self.add_section("LED Color (for SET_LED)")
        led_color = self.step.get("led_color", {"r": 0.0, "g": 1.0, "b": 0.0})
        self.led_r = QDoubleSpinBox()
        self.led_r.setRange(0, 1)
        self.led_r.setDecimals(2)
        self.led_r.setValue(led_color.get("r", 0.0))
        led.addRow("Red:", self.led_r)
        self.led_g = QDoubleSpinBox()
        self.led_g.setRange(0, 1)
        self.led_g.setDecimals(2)
        self.led_g.setValue(led_color.get("g", 1.0))
        led.addRow("Green:", self.led_g)
        self.led_b = QDoubleSpinBox()
        self.led_b.setRange(0, 1)
        self.led_b.setDecimals(2)
        self.led_b.setValue(led_color.get("b", 0.0))
        led.addRow("Blue:", self.led_b)

        # Color preview
        self.color_preview = QLabel()
        self.color_preview.setFixedHeight(25)
        self._update_preview()
        led.addRow("Preview:", self.color_preview)

        self.led_r.valueChanged.connect(self._update_preview)
        self.led_g.valueChanged.connect(self._update_preview)
        self.led_b.valueChanged.connect(self._update_preview)

    def _update_preview(self):
        r = int(self.led_r.value() * 255)
        g = int(self.led_g.value() * 255)
        b = int(self.led_b.value() * 255)
        self.color_preview.setStyleSheet(
            f"background-color: rgb({r},{g},{b}); border: 1px solid gray;"
        )

    def collect_values(self):
        return {
            **self.step,
            "operation": self.operation.currentText(),
            "volume_pct": self.volume.value(),
            "led_color": {
                "r": self.led_r.value(),
                "g": self.led_g.value(),
                "b": self.led_b.value(),
            },
        }


class _GridSelectorForm(BaseTaskForm):
    """Base form with a clickable grid for selecting A1, A2, B1, etc."""

    TARGET_NAME = ""  # Override in subclasses

    def build_form(self):
        grid_cfg = _vision_target_grid(self.TARGET_NAME)
        self._rows = grid_cfg.get("rows", 1)
        self._cols = grid_cfg.get("cols", 1)

        self.add_hint(f"Select a position on the {self._rows}×{self._cols} grid.")

        # Grid of buttons
        grid_group = QGroupBox("Position")
        grid_layout = QGridLayout(grid_group)
        grid_layout.setSpacing(2)

        self._grid_buttons: dict[tuple[int, int], QPushButton] = {}
        self._selected_row = int(self.step.get("row", 0))
        self._selected_col = int(self.step.get("col", 0))

        # Column headers (1, 2, 3, ...)
        for c in range(self._cols):
            lbl = QLabel(str(c + 1))
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFont(QFont("Monospace", 8))
            lbl.setStyleSheet("color: #888;")
            grid_layout.addWidget(lbl, 0, c + 1)

        # Row headers (A, B, C, ...) and buttons
        for r in range(self._rows):
            row_lbl = QLabel(chr(ord("A") + r))
            row_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            row_lbl.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
            row_lbl.setStyleSheet("color: #aaa;")
            grid_layout.addWidget(row_lbl, r + 1, 0)

            for c in range(self._cols):
                btn = QPushButton(f"{chr(ord('A') + r)}{c + 1}")
                btn.setFixedSize(36, 28)
                btn.setFont(QFont("Monospace", 8))
                btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                btn.setCheckable(True)
                btn.clicked.connect(
                    lambda checked, row=r, col=c: self._select_cell(row, col)
                )
                grid_layout.addWidget(btn, r + 1, c + 1)
                self._grid_buttons[(r, c)] = btn

        self.form.addRow(grid_group)

        # Highlight initial selection
        self._update_selection()

        # Show selected label
        self._sel_label = QLabel(
            self._cell_name(self._selected_row, self._selected_col)
        )
        self._sel_label.setStyleSheet(
            "font-size: 13px; font-weight: bold; color: #5ce68a;"
        )
        self.form.addRow("Selected:", self._sel_label)

    def _cell_name(self, row, col):
        return f"{chr(ord('A') + row)}{col + 1}"

    def _select_cell(self, row, col):
        self._selected_row = row
        self._selected_col = col
        self._update_selection()
        self._sel_label.setText(self._cell_name(row, col))

    def _update_selection(self):
        for (r, c), btn in self._grid_buttons.items():
            if r == self._selected_row and c == self._selected_col:
                btn.setChecked(True)
                btn.setStyleSheet(
                    "background-color: #2a6b3a; color: white; border: 1px solid #5ce68a;"
                )
            else:
                btn.setChecked(False)
                btn.setStyleSheet("")

    def collect_values(self):
        return {
            **self.step,
            "row": self._selected_row,
            "col": self._selected_col,
            "position": self._cell_name(self._selected_row, self._selected_col),
        }


class PickupTipForm(_GridSelectorForm):
    TITLE = "Pickup Tip"
    TARGET_NAME = "tip_rack"

    def build_form(self):
        super().build_form()
        self.add_hint(
            "Tip rack: 8 rows (A-H) × 12 columns (1-12). "
            "Uses vision to align with marker, then moves to selected position."
        )


class PickupVialForm(_GridSelectorForm):
    TITLE = "Vial Rack"
    TARGET_NAME = "vial_rack"

    def build_form(self):
        super().build_form()

        # Pipettor operation after reaching vial
        op_group = QGroupBox("Pipettor Operation (optional)")
        op_layout = QFormLayout(op_group)

        self._vial_operation = QComboBox()
        self._vial_operation.addItems(["None", "SUCK", "EXPEL"])
        self._vial_operation.setCurrentText(self.step.get("pipettor_operation", "None"))
        op_layout.addRow("Operation:", self._vial_operation)

        self._vial_volume = QDoubleSpinBox()
        self._vial_volume.setRange(0.0, 1.0)
        self._vial_volume.setDecimals(2)
        self._vial_volume.setSingleStep(0.05)
        self._vial_volume.setValue(float(self.step.get("volume_pct", 0.5)))
        op_layout.addRow("Volume %:", self._vial_volume)

        def _toggle_volume(text):
            self._vial_volume.setEnabled(text != "None")

        self._vial_operation.currentTextChanged.connect(_toggle_volume)
        _toggle_volume(self._vial_operation.currentText())

        self.form.addRow(op_group)
        self.add_hint(
            "Vial rack: 2 rows (A-B) × 5 columns (1-5). "
            "Select SUCK to aspirate or EXPEL to dispense after insertion."
        )

    def collect_values(self):
        s = super().collect_values()
        op = self._vial_operation.currentText()
        if op != "None":
            s["pipettor_operation"] = op
            s["volume_pct"] = self._vial_volume.value()
        else:
            s.pop("pipettor_operation", None)
            s.pop("volume_pct", None)
        return s


class SpincoaterForm(BaseTaskForm):
    """Form for place_spincoater and pick_spincoater tasks."""

    @property
    def _is_pick(self):
        return self.step.get("task_type") == "pick_spincoater"

    @property
    def TITLE(self):
        return "Pick from Spincoater" if self._is_pick else "Place on Spincoater"

    def build_form(self):
        mode = "pickup" if self._is_pick else "placement"
        self.add_hint(
            f"Vision-guided {mode}: detects orientation via 2D capture, "
            f"corrects wrist angle, then {'picks' if self._is_pick else 'places'}."
        )
        self.scan_pose = self.add_text("Scan pose:", "scan_pose", "spincoater_scan")
        pose_key = "pickup_pose" if self._is_pick else "place_pose"
        self.target_pose = self.add_text(
            f"{'Pickup' if self._is_pick else 'Place'} pose:",
            pose_key,
            "spincoater_place",
        )
        self.forward_dist = self.add_float(
            "Forward distance (m):", "forward_distance", 0.003, 0.0, 0.05, 4
        )
        self.k_offset = self.add_float(
            "K offset (deg):", "k_offset", 0.0, -90.0, 90.0, 1
        )
        if not self._is_pick:
            self.release = self.add_check("Release vacuum:", "release", True)

    def collect_values(self):
        pose_key = "pickup_pose" if self._is_pick else "place_pose"
        result = {
            **self.step,
            "scan_pose": self.scan_pose.text(),
            pose_key: self.target_pose.text(),
            "forward_distance": self.forward_dist.value(),
            "k_offset": self.k_offset.value(),
        }
        if not self._is_pick:
            result["release"] = self.release.isChecked()
        return result


# --- Form dispatch map ---

_FORMS = {
    "moveto": MoveToForm,
    "pick_sample": SampleForm,
    "place_sample": SampleForm,
    "vision_scan": VisionScanForm,
    "tool_exchange": ToolExchangeForm,
    "end_effector": EndEffectorForm,
    "vision_moveto": VisionMoveToForm,
    "pipettor": PipettorForm,
    "pickup_tip": PickupTipForm,
    "pickup_vial": PickupVialForm,
    "place_spincoater": SpincoaterForm,
    "pick_spincoater": SpincoaterForm,
}
