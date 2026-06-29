"""Pose registry file I/O.

Format MUST stay byte-compatible with ``beambot_mcp_server._write_poses_file``
(``yaml.dump`` with ``default_flow_style=None, width=200``) so GUI- and
agent-written poses are indistinguishable on disk. Group comments are NOT
preserved on write; this matches MCP behavior and the current
``cms/poses.yaml`` has none.

Qt-free on purpose so the safety logic is unit-testable without a display.
"""

import os
import tempfile

import yaml


def read_poses(path):
    """Return the pose registry dict, keeping only 6-element list entries."""
    if not path or not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return {}
    return {k: v for k, v in data.items() if isinstance(v, list) and len(v) == 6}


def write_poses(path, poses):
    """Atomically write ``poses`` to ``path`` (temp file then os.replace)."""
    dir_path = os.path.dirname(os.path.realpath(path))
    os.makedirs(dir_path, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=dir_path, suffix=".yaml.tmp")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.dump(poses, f, default_flow_style=None, width=200)
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        raise
