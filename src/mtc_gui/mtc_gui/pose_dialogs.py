"""Pose management dialogs: pose list manager, single pose editor, save current pose."""

import json
import math

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QDialogButtonBox,
    QTreeWidget, QTreeWidgetItem, QLabel, QLineEdit, QDoubleSpinBox,
    QPushButton, QGroupBox, QRadioButton, QButtonGroup, QFileDialog,
    QMessageBox, QHeaderView,
)
from PyQt5.QtCore import Qt


class PoseEditorDialog(QDialog):
    """Edit a single pose (6 joint angles in degrees)."""

    JOINT_NAMES = [
        "Shoulder Pan", "Shoulder Lift", "Elbow",
        "Wrist 1", "Wrist 2", "Wrist 3",
    ]
    PRESETS = {
        "Home": [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
        "Straight Up": [0.0, -90.0, 0.0, -90.0, 0.0, 0.0],
        "All Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    def __init__(self, name, values, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit Pose: {name}")
        self.setMinimumWidth(350)
        self.setModal(True)
        self.result = None

        layout = QVBoxLayout(self)

        # Joint value spinboxes
        form = QFormLayout()
        self.spins = []
        for i, jname in enumerate(self.JOINT_NAMES):
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(2)
            spin.setSuffix(" deg")
            spin.setValue(values[i] if i < len(values) else 0.0)
            form.addRow(f"{jname}:", spin)
            self.spins.append(spin)
        layout.addLayout(form)

        # Presets
        preset_group = QGroupBox("Presets")
        preset_layout = QHBoxLayout(preset_group)
        for pname, pvals in self.PRESETS.items():
            btn = QPushButton(pname)
            btn.clicked.connect(lambda checked, v=pvals: self._apply_preset(v))
            preset_layout.addWidget(btn)
        layout.addWidget(preset_group)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _apply_preset(self, values):
        for spin, val in zip(self.spins, values):
            spin.setValue(val)

    def _on_save(self):
        self.result = [spin.value() for spin in self.spins]
        self.accept()


class PosesManagerDialog(QDialog):
    """Manage the poses dictionary: list, add, edit, delete, import."""

    def __init__(self, poses, parent=None):
        super().__init__(parent)
        self.poses = dict(poses)  # work on a copy
        self.result = None

        self.setWindowTitle("Manage Poses")
        self.setMinimumSize(600, 450)
        self.setModal(True)

        layout = QVBoxLayout(self)

        # Pose tree
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Pose Name", "Joint Values (degrees)"])
        self.tree.header().setStretchLastSection(True)
        self.tree.setColumnWidth(0, 180)
        self.tree.itemDoubleClicked.connect(self._edit_pose)
        layout.addWidget(self.tree, stretch=1)

        # Buttons
        btn_row = QHBoxLayout()
        for label, slot in [
            ("Add", self._add_pose),
            ("Edit", self._edit_selected),
            ("Delete", self._delete_pose),
            ("Import JSON", self._import_json),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(slot)
            btn_row.addWidget(btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Dialog buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self._refresh()

    def _refresh(self):
        self.tree.clear()
        for name, values in sorted(self.poses.items()):
            vals_str = ", ".join(f"{v:.2f}" for v in values)
            item = QTreeWidgetItem([name, f"[{vals_str}]"])
            self.tree.addTopLevelItem(item)

    def _add_pose(self):
        dlg = PoseEditorDialog("new_pose", [0.0] * 6, self)
        if dlg.exec() == QDialog.Accepted:
            # Ask for name
            from PyQt5.QtWidgets import QInputDialog
            name, ok = QInputDialog.getText(self, "Pose Name", "Enter pose name:")
            if ok and name:
                self.poses[name] = dlg.result
                self._refresh()

    def _edit_selected(self):
        items = self.tree.selectedItems()
        if items:
            self._edit_pose(items[0])

    def _edit_pose(self, item):
        name = item.text(0)
        values = self.poses.get(name, [0.0] * 6)
        dlg = PoseEditorDialog(name, values, self)
        if dlg.exec() == QDialog.Accepted:
            self.poses[name] = dlg.result
            self._refresh()

    def _delete_pose(self):
        items = self.tree.selectedItems()
        if not items:
            return
        name = items[0].text(0)
        reply = QMessageBox.question(self, "Delete", f"Delete pose '{name}'?")
        if reply == QMessageBox.Yes:
            self.poses.pop(name, None)
            self._refresh()

    def _import_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import Poses JSON", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            imported = data.get("poses", data) if isinstance(data, dict) else {}
            count = 0
            for name, values in imported.items():
                if isinstance(values, list) and len(values) == 6:
                    self.poses[name] = values
                    count += 1
            self._refresh()
            QMessageBox.information(self, "Imported", f"Imported {count} pose(s)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Import failed: {e}")

    def _on_save(self):
        self.result = self.poses
        self.accept()


class SavePoseDialog(QDialog):
    """Save the current robot joint pose to the config or a file."""

    def __init__(self, current_pose, config, current_file, parent=None):
        super().__init__(parent)
        self.current_pose = current_pose
        self.config = config
        self.current_file = current_file
        self.result = None

        self.setWindowTitle("Save Current Pose")
        self.setMinimumWidth(400)
        self.setModal(True)

        layout = QVBoxLayout(self)

        # Display current pose
        pose_group = QGroupBox("Current Robot Pose")
        pose_layout = QFormLayout(pose_group)
        joints = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        for jname, val in zip(joints, current_pose):
            pose_layout.addRow(f"{jname}:", QLabel(f"{val:.2f} deg"))
        layout.addWidget(pose_group)

        # Pose name
        name_group = QGroupBox("Pose Name")
        name_layout = QFormLayout(name_group)
        self.name_edit = QLineEdit("new_pose")
        name_layout.addRow("Name:", self.name_edit)
        layout.addWidget(name_group)

        # Save options
        opts_group = QGroupBox("Save To")
        opts_layout = QVBoxLayout(opts_group)
        self.btn_group = QButtonGroup(self)
        self.radio_config = QRadioButton("Add to current config")
        self.radio_config.setChecked(True)
        self.btn_group.addButton(self.radio_config, 0)
        opts_layout.addWidget(self.radio_config)

        self.radio_file = QRadioButton(f"Save to: {current_file or '(no file loaded)'}")
        self.radio_file.setEnabled(current_file is not None)
        self.btn_group.addButton(self.radio_file, 1)
        opts_layout.addWidget(self.radio_file)

        self.radio_new = QRadioButton("Save to new file...")
        self.btn_group.addButton(self.radio_new, 2)
        opts_layout.addWidget(self.radio_new)
        layout.addWidget(opts_group)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _on_save(self):
        name = self.name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Error", "Pose name cannot be empty")
            return

        choice = self.btn_group.checkedId()
        if choice == 0:
            self.result = {"action": "add_to_config", "pose_name": name, "pose_values": self.current_pose}
        elif choice == 1:
            self.result = {"action": "save_to_current", "pose_name": name, "pose_values": self.current_pose}
        elif choice == 2:
            path, _ = QFileDialog.getSaveFileName(self, "Save Poses", "", "JSON (*.json)")
            if not path:
                return
            self.result = {
                "action": "save_to_new", "pose_name": name,
                "pose_values": self.current_pose, "file_path": path,
            }
        self.accept()
