"""Pure-Python tests for the Qt-free pose registry I/O (no display needed)."""

import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mtc_gui"))

from pose_io import read_poses, write_poses  # noqa: E402


def test_round_trip():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "poses.yaml")
        poses = {"home": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}
        write_poses(path, poses)
        assert read_poses(path) == poses


def test_merge_without_clobber():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "poses.yaml")
        write_poses(path, {"a": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]})
        existing = read_poses(path)
        existing["b"] = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        write_poses(path, existing)
        result = read_poses(path)
        assert "a" in result and "b" in result


def test_byte_format_flow_style():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "poses.yaml")
        write_poses(path, {"p": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]})
        with open(path) as f:
            text = f.read()
        assert "p: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]" in text


def test_delete():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "poses.yaml")
        write_poses(path, {"a": [1.0] * 6, "b": [2.0] * 6})
        poses = read_poses(path)
        poses.pop("a")
        write_poses(path, poses)
        result = read_poses(path)
        assert "a" not in result and "b" in result


def test_no_leftover_tmp_files():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "poses.yaml")
        write_poses(path, {"a": [1.0] * 6})
        leftovers = [f for f in os.listdir(d) if f.endswith(".yaml.tmp")]
        assert leftovers == []


if __name__ == "__main__":
    test_round_trip()
    test_merge_without_clobber()
    test_byte_format_flow_style()
    test_delete()
    test_no_leftover_tmp_files()
    print("All pose_io tests passed.")
