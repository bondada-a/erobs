"""3D robot visualization panel using QWebEngineView + Three.js + urdf-loaders."""

import time
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from functools import partial

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, QUrl, QTimer

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False

GRIPPER_URDF_MAP = {
    "epick": "ur_with_zivid_epick.urdf",
    "hande": "ur_with_zivid_hande.urdf",
    "pipettor": "ur_with_zivid_pipettor.urdf",
    "2fg7": "ur_standalone.urdf",
    "none": "ur_standalone.urdf",
}

REQUIRED_PACKAGES = [
    "ur_description",
    "ur5e_robot_description",
    "zivid_description",
    "epick_description",
    "robotiq_hande_description",
    "pipette_description",
]


def _find_workspace_roots():
    """Find colcon workspace root(s). Returns list — may include worktree + main repo."""
    roots = []
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "src").is_dir() and (parent / "install").is_dir():
            roots.append(parent)
            break
        if (parent / "src" / "mtc_gui").is_dir():
            roots.append(parent)
            # Also check if we're in a worktree — the main repo is above .claude/worktrees/
            for p in parent.parents:
                if (p / "install").is_dir() and (p / "src").is_dir():
                    roots.append(p)
                    break
            break
    if not roots:
        roots.append(path.parents[3])
    return roots


def _resolve_packages():
    """Build a dict mapping ROS package names to their share directories on disk."""
    packages = {}

    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in REQUIRED_PACKAGES:
            try:
                packages[pkg] = get_package_share_directory(pkg)
            except Exception:
                pass
    except ImportError:
        pass

    workspace_roots = _find_workspace_roots()

    # Fill in anything ament_index didn't find
    candidates_per_pkg = {
        "ur_description": [Path("/opt/ros/jazzy/share/ur_description")],
        "ur5e_robot_description": [],
        "zivid_description": [Path("/opt/ros/jazzy/share/zivid_description")],
        "epick_description": [],
        "robotiq_hande_description": [],
        "pipette_description": [],
    }

    # Add workspace-relative paths for each root
    for ws in workspace_roots:
        candidates_per_pkg["ur5e_robot_description"].extend([
            ws / "install" / "ur5e_robot_description" / "share" / "ur5e_robot_description",
            ws / "src" / "custom-ur-descriptions" / "ur5e_robot_description",
        ])
        candidates_per_pkg["zivid_description"].insert(0,
            ws / "install" / "zivid_description" / "share" / "zivid_description")
        candidates_per_pkg["epick_description"].extend([
            ws / "install" / "epick_description" / "share" / "epick_description",
            ws / "src" / "end_effectors" / "ros2_epick_gripper" / "epick_description",
        ])
        candidates_per_pkg["robotiq_hande_description"].extend([
            ws / "install" / "robotiq_hande_description" / "share" / "robotiq_hande_description",
            ws / "src" / "end_effectors" / "robotiq_hande_description",
        ])
        candidates_per_pkg["pipette_description"].extend([
            ws / "install" / "pipette_description" / "share" / "pipette_description",
            ws / "src" / "end_effectors" / "pipettor" / "pipette_description",
        ])

    search_paths = candidates_per_pkg

    for pkg, candidates in search_paths.items():
        if pkg in packages:
            continue
        for candidate in candidates:
            if candidate.is_dir():
                packages[pkg] = str(candidate)
                break

    return packages


class _MeshRequestHandler(SimpleHTTPRequestHandler):
    """Serves mesh files and static resources for the 3D viewer."""

    def __init__(self, *args, package_map, resources_dir, urdf_dir, **kwargs):
        self.package_map = package_map
        self.resources_dir = resources_dir
        self.urdf_dir = urdf_dir
        super().__init__(*args, **kwargs)

    def translate_path(self, path):
        # Static resources (viewer.html, JS files)
        if path.startswith("/__static__/"):
            rel = path[len("/__static__/"):]
            return str(self.resources_dir / rel)

        # URDF files
        if path.startswith("/__urdf__/"):
            rel = path[len("/__urdf__/"):]
            return str(self.urdf_dir / rel)

        # Package mesh files: /package_name/rest/of/path
        parts = path.strip("/").split("/", 1)
        if len(parts) >= 1 and parts[0] in self.package_map:
            pkg_dir = self.package_map[parts[0]]
            rel = parts[1] if len(parts) > 1 else ""
            return str(Path(pkg_dir) / rel)

        return str(self.resources_dir / path.lstrip("/"))

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "public, max-age=3600")
        super().end_headers()

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


class _MeshServer:
    """Lightweight HTTP server for serving URDF meshes to the WebEngine."""

    def __init__(self, package_map, resources_dir, urdf_dir):
        handler = partial(
            _MeshRequestHandler,
            package_map=package_map,
            resources_dir=resources_dir,
            urdf_dir=urdf_dir,
        )
        self.server = HTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.server.shutdown()


class VisualizationPanel(QWidget):
    """3D robot visualization tab using Three.js + urdf-loaders in QWebEngineView."""

    def __init__(self, ros2_bridge=None, parent=None):
        super().__init__(parent)
        self.ros2 = ros2_bridge
        self._last_update = 0.0
        self._current_gripper = "epick"
        self._page_ready = False

        self._resources_dir = Path(__file__).parent / "resources"
        self._urdf_dir = None
        for ws in _find_workspace_roots():
            candidate = ws / "src" / "custom-ur-descriptions" / "ur5e_robot_description" / "urdf"
            if candidate.is_dir():
                self._urdf_dir = candidate
                break
        if self._urdf_dir is None:
            self._urdf_dir = Path(__file__).resolve().parents[2] / "custom-ur-descriptions" / "ur5e_robot_description" / "urdf"

        self._package_map = _resolve_packages()
        self._mesh_server = None
        self._setup_ui()
        self._start_server()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(4, 4, 4, 4)

        btn_reset = QPushButton("Reset View")
        btn_reset.setFixedHeight(24)
        btn_reset.clicked.connect(self._reset_view)
        toolbar.addWidget(btn_reset)

        btn_home = QPushButton("Home Pose")
        btn_home.setFixedHeight(24)
        btn_home.clicked.connect(self._home_pose)
        toolbar.addWidget(btn_home)

        btn_demo = QPushButton("Demo")
        btn_demo.setFixedHeight(24)
        btn_demo.setCheckable(True)
        btn_demo.toggled.connect(self._toggle_demo)
        toolbar.addWidget(btn_demo)

        toolbar.addStretch()
        self._status = QLabel("Initializing...")
        self._status.setStyleSheet("color: #888; font-size: 11px;")
        toolbar.addWidget(self._status)

        layout.addLayout(toolbar)

        if not WEBENGINE_AVAILABLE:
            fallback = QLabel("Install python3-pyqt5.qtwebengine for 3D visualization")
            fallback.setAlignment(Qt.AlignCenter)
            fallback.setStyleSheet("color: #888; font-size: 13px;")
            layout.addWidget(fallback)
            return

        self.web_view = QWebEngineView()
        self.web_view.setStyleSheet("background: #1e1e1e;")
        layout.addWidget(self.web_view, stretch=1)

    def _start_server(self):
        if not WEBENGINE_AVAILABLE:
            return

        self._mesh_server = _MeshServer(
            self._package_map, self._resources_dir, self._urdf_dir
        )
        self._mesh_server.start()
        port = self._mesh_server.port
        self._status.setText(f"Mesh server :{port}")

        url = f"http://127.0.0.1:{port}/__static__/viewer.html"
        self.web_view.loadFinished.connect(self._on_page_loaded)
        self.web_view.load(QUrl(url))

    def _on_page_loaded(self, ok):
        if not ok:
            self._status.setText("Failed to load viewer")
            return

        self._status.setText("Waiting for JS...")
        self._ready_poll = QTimer(self)
        self._ready_poll.timeout.connect(self._check_ready)
        self._ready_poll.start(100)

    def _check_ready(self):
        self.web_view.page().runJavaScript(
            "window._vizReady === true", self._on_ready_result
        )

    def _on_ready_result(self, ready):
        if ready:
            self._ready_poll.stop()
            self._page_ready = True
            self._status.setText("Ready")
            self._load_urdf(self._current_gripper)

    def _load_urdf(self, gripper):
        if not self._page_ready:
            return
        urdf_file = GRIPPER_URDF_MAP.get(gripper, "ur_standalone.urdf")
        port = self._mesh_server.port
        urdf_url = f"http://127.0.0.1:{port}/__urdf__/{urdf_file}"
        js = f"window.loadRobot('{urdf_url}', {port})"
        self.web_view.page().runJavaScript(js)

    def set_gripper(self, gripper_name):
        """Called when the gripper combo changes."""
        self._current_gripper = gripper_name.lower()
        self._load_urdf(self._current_gripper)

    def _on_joint_state(self, pose_deg):
        """Receive joint angles in degrees and forward to Three.js (throttled)."""
        if not self._page_ready:
            return
        now = time.monotonic()
        if now - self._last_update < 0.033:
            return
        self._last_update = now
        self.web_view.page().runJavaScript(f"window.updateJoints({pose_deg})")

    def _reset_view(self):
        if self._page_ready:
            self.web_view.page().runJavaScript("window.resetView()")

    def _home_pose(self):
        if self._page_ready:
            self.web_view.page().runJavaScript("window.setHomePose()")

    def _toggle_demo(self, checked):
        if self._page_ready:
            self.web_view.page().runJavaScript(
                f"window.demoMode({'true' if checked else 'false'})"
            )

    def closeEvent(self, event):
        if self._mesh_server:
            self._mesh_server.stop()
        super().closeEvent(event)
