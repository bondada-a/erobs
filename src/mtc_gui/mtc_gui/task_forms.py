"""Task edit form dialogs — base class + all 8 task type forms."""

import copy

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QDialogButtonBox, QScrollArea,
    QWidget, QLabel, QLineEdit, QComboBox, QCheckBox, QDoubleSpinBox,
    QSpinBox, QGroupBox, QHBoxLayout, QTextEdit, QSlider, QMessageBox,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor


# --- Dispatch ---

def open_task_form(step, step_index, poses, parent=None):
    """Open the edit dialog for a task step. Returns edited step dict or None if cancelled."""
    task_type = step.get("task_type", "")
    form_cls = _FORMS.get(task_type)
    if not form_cls:
        QMessageBox.warning(parent, "Unknown", f"No form for task type: {task_type}")
        return None
    dialog = form_cls(step, step_index, poses, parent)
    if dialog.exec() == QDialog.Accepted:
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
        outer.addWidget(scroll, stretch=1)

        self.build_form()

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
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

    def add_float(self, label, key, default=0.0, min_val=-999.0, max_val=999.0, decimals=4):
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

    def build_form(self):
        self.target = self.add_text("Target:", "target", "moveit_home")
        self.add_hint("Named state (moveit_home) or pose name from the pose manager")
        self.planning = self.add_combo("Planning Type:", "planning_type", ["joint", "cartesian"])

        # Relative move
        rel = self.add_section("Relative Move (optional)")
        self.direction = QComboBox()
        self.direction.addItems(["", "forward", "backward", "left", "right", "up", "down"])
        self.direction.setCurrentText(self.step.get("direction", ""))
        rel.addRow("Direction:", self.direction)
        self.distance = QDoubleSpinBox()
        self.distance.setRange(0, 2.0)
        self.distance.setDecimals(3)
        self.distance.setValue(float(self.step.get("distance", 0.0)))
        rel.addRow("Distance (m):", self.distance)

        # Cartesian target
        cart = self.add_section("Cartesian Target (optional)")
        existing = self.step.get("cartesian_target", [])
        self.cart_x = QDoubleSpinBox(); self.cart_x.setRange(-2, 2); self.cart_x.setDecimals(4)
        self.cart_y = QDoubleSpinBox(); self.cart_y.setRange(-2, 2); self.cart_y.setDecimals(4)
        self.cart_z = QDoubleSpinBox(); self.cart_z.setRange(-2, 2); self.cart_z.setDecimals(4)
        pos_row = QHBoxLayout()
        for lbl, spin, i in [("X:", self.cart_x, 0), ("Y:", self.cart_y, 1), ("Z:", self.cart_z, 2)]:
            pos_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            pos_row.addWidget(spin)
        cart.addRow("Position (m):", pos_row)

        self.cart_r = QDoubleSpinBox(); self.cart_r.setRange(-360, 360); self.cart_r.setDecimals(2)
        self.cart_p = QDoubleSpinBox(); self.cart_p.setRange(-360, 360); self.cart_p.setDecimals(2)
        self.cart_yaw = QDoubleSpinBox(); self.cart_yaw.setRange(-360, 360); self.cart_yaw.setDecimals(2)
        rot_row = QHBoxLayout()
        for lbl, spin, i in [("R:", self.cart_r, 3), ("P:", self.cart_p, 4), ("Y:", self.cart_yaw, 5)]:
            rot_row.addWidget(QLabel(lbl))
            if len(existing) > i:
                spin.setValue(existing[i])
            rot_row.addWidget(spin)
        cart.addRow("Orientation (deg):", rot_row)

        self.frame_id = QComboBox()
        self.frame_id.addItems(["base_link", "flange"])
        self.frame_id.setCurrentText(self.step.get("frame_id", "base_link"))
        cart.addRow("Frame:", self.frame_id)

        lbl = QLabel("Leave at 0 to use named target. XYZ only = straight-down orientation.")
        lbl.setStyleSheet("color: gray; font-size: 10px;")
        cart.addRow(lbl)

    def collect_values(self):
        s = {**self.step}
        s["target"] = self.target.text()
        s["planning_type"] = self.planning.currentText()

        if self.direction.currentText():
            s["direction"] = self.direction.currentText()
            s["distance"] = self.distance.value()
        else:
            s.pop("direction", None)
            s.pop("distance", None)

        # Cartesian: if any XYZ non-zero, include
        x, y, z = self.cart_x.value(), self.cart_y.value(), self.cart_z.value()
        if x != 0 or y != 0 or z != 0:
            cart = [x, y, z]
            r, p, yaw = self.cart_r.value(), self.cart_p.value(), self.cart_yaw.value()
            if r != 0 or p != 0 or yaw != 0:
                cart.extend([r, p, yaw])
            s["cartesian_target"] = cart
            s["frame_id"] = self.frame_id.currentText()
        else:
            s.pop("cartesian_target", None)
            s.pop("frame_id", None)
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
        self.det_type.addItems(["marker", "circle", "contour"])
        self.det_type.setCurrentText(self.step.get("detection_type", "marker"))
        vl.addRow("Detection Type:", self.det_type)

        self.tag_id = QSpinBox(); self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        vl.addRow("Tag ID:", self.tag_id)

        if self.mode == "pick":
            self.sample_idx = QSpinBox(); self.sample_idx.setRange(1, 99)
            self.sample_idx.setValue(int(self.step.get("sample_index", 1)))
            vl.addRow("Sample Index:", self.sample_idx)

        self.scan_pose = QLineEdit(self.step.get("scan_pose", ""))
        vl.addRow("Scan Pose:", self.scan_pose)

        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1); self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        vl.addRow("Z Offset (m):", self.z_offset)

        self.marker_offsets = self.add_offset_row(
            vl, "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"]
        )

        self.offset_dir = QComboBox()
        self.offset_dir.addItems(["", "forward", "backward", "left", "right", "up", "down"])
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        vl.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0); self.offset_dist.setDecimals(4)
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
        self.use_vision.stateChanged.connect(lambda state: toggle(state == Qt.Checked))
        toggle(self.use_vision.isChecked())

    def collect_values(self):
        s = {**self.step}
        s["use_vision"] = self.use_vision.isChecked()
        if s["use_vision"]:
            s["detection_type"] = self.det_type.currentText()
            s["tag_id"] = self.tag_id.value()
            if self.mode == "pick":
                s["sample_index"] = self.sample_idx.value()
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
            for key in ("detection_type", "tag_id", "sample_index", "scan_pose",
                        "z_offset", "marker_offset_x", "marker_offset_y",
                        "marker_offset_z", "offset_direction", "offset_distance"):
                s.pop(key, None)
        return s


class VisionScanForm(BaseTaskForm):
    TITLE = "Vision Scan Configuration"

    def build_form(self):
        self.add_hint("Scans markers from multiple positions and caches averaged poses.\n"
                      "Subsequent vision_moveto tasks use the cache for faster detection.")
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
        self.gripper = self.add_combo("Gripper:", "gripper",
                                      ["epick", "hande", "2fg7", "pipettor", "none"])
        self.dock_num = self.add_int("Dock Number:", "dock_number", 3, 1, 10)
        self.approach = self.add_text("Approach Pose:", "approach_pose", "load_approach")
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
        self.type_combo = self.add_combo("Type:", "end_effector_type",
                                         ["epick", "hande", "2fg7"])
        self.action_combo = self.add_combo("Action:", "end_effector_action",
                                           ["hande_open", "hande_closed",
                                            "vacuum_on", "vacuum_off"])

    def collect_values(self):
        return {
            **self.step,
            "end_effector_type": self.type_combo.currentText(),
            "end_effector_action": self.action_combo.currentText(),
        }


class VisionMoveToForm(BaseTaskForm):
    TITLE = "Vision MoveTo Configuration"

    def build_form(self):
        self.add_hint("Detect object using Zivid camera and move gripper to detected location.")
        self.det_type = self.add_combo("Detection Type:", "detection_type",
                                       ["marker", "circle", "contour"])

        # Marker options
        marker_sec = self.add_section("ArUco Marker Options")
        self.tag_id = QSpinBox(); self.tag_id.setRange(0, 999)
        self.tag_id.setValue(int(self.step.get("tag_id", 0)))
        marker_sec.addRow("Marker ID:", self.tag_id)
        self.marker_dict = QComboBox()
        self.marker_dict.addItems([
            "aruco4x4_50", "aruco4x4_100", "aruco4x4_250",
            "aruco5x5_50", "aruco5x5_100", "aruco5x5_250",
            "aruco6x6_50", "aruco6x6_100", "aruco6x6_250",
        ])
        self.marker_dict.setCurrentText(self.step.get("marker_dictionary", "aruco4x4_50"))
        marker_sec.addRow("Dictionary:", self.marker_dict)

        # Contour options
        contour_sec = self.add_section("Contour Options")
        self.sample_idx = QSpinBox(); self.sample_idx.setRange(1, 99)
        self.sample_idx.setValue(int(self.step.get("sample_index", 1)))
        contour_sec.addRow("Sample Index:", self.sample_idx)

        # Common options
        common = self.add_section("Common Options")
        self.z_offset = QDoubleSpinBox()
        self.z_offset.setRange(-0.1, 0.1); self.z_offset.setDecimals(4)
        self.z_offset.setValue(float(self.step.get("z_offset", 0.0)))
        common.addRow("Z Offset (m):", self.z_offset)
        self.timeout = QDoubleSpinBox()
        self.timeout.setRange(1.0, 120.0); self.timeout.setDecimals(1)
        self.timeout.setValue(float(self.step.get("timeout", 10.0)))
        common.addRow("Timeout (s):", self.timeout)
        self.detect_only = QCheckBox()
        self.detect_only.setChecked(bool(self.step.get("detect_only", False)))
        common.addRow("Detect Only:", self.detect_only)

        # Offsets
        self.marker_offsets = self.add_offset_row(
            common, "Marker Offsets (m):",
            ["marker_offset_x", "marker_offset_y", "marker_offset_z"]
        )
        self.offset_dir = QComboBox()
        self.offset_dir.addItems(["", "forward", "backward", "left", "right", "up", "down"])
        self.offset_dir.setCurrentText(self.step.get("offset_direction", ""))
        common.addRow("Flange Direction:", self.offset_dir)
        self.offset_dist = QDoubleSpinBox()
        self.offset_dist.setRange(0, 1.0); self.offset_dist.setDecimals(4)
        self.offset_dist.setValue(float(self.step.get("offset_distance", 0.0)))
        common.addRow("Flange Distance (m):", self.offset_dist)

    def collect_values(self):
        s = {**self.step}
        s["detection_type"] = self.det_type.currentText()
        s["tag_id"] = self.tag_id.value()
        s["sample_index"] = self.sample_idx.value()
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
        self.operation = self.add_combo("Operation:", "operation",
                                        ["SUCK", "EXPEL", "EJECT_TIP", "SET_LED"])
        self.volume = self.add_float("Volume %:", "volume_pct", 0.5, 0.0, 1.0, 2)

        # LED color
        led = self.add_section("LED Color (for SET_LED)")
        led_color = self.step.get("led_color", {"r": 0.0, "g": 1.0, "b": 0.0})
        self.led_r = QDoubleSpinBox(); self.led_r.setRange(0, 1); self.led_r.setDecimals(2)
        self.led_r.setValue(led_color.get("r", 0.0))
        led.addRow("Red:", self.led_r)
        self.led_g = QDoubleSpinBox(); self.led_g.setRange(0, 1); self.led_g.setDecimals(2)
        self.led_g.setValue(led_color.get("g", 1.0))
        led.addRow("Green:", self.led_g)
        self.led_b = QDoubleSpinBox(); self.led_b.setRange(0, 1); self.led_b.setDecimals(2)
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
}
