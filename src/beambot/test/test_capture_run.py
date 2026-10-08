"""Check capture-run cleanup without ROS or hardware."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import rclpy
import rclpy.action
import rclpy.node


@pytest.fixture
def capture_run():
    path = Path(__file__).resolve().parents[1] / "scripts" / "capture_octomap.py"
    spec = importlib.util.spec_from_file_location("_test_capture_octomap", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("case", ["server", "service", "exception", "interrupt", "success", "node"])
def test_run_cleans_up_on_return_and_exception(case, capture_run, monkeypatch, tmp_path):
    node, client = MagicMock(), MagicMock()
    constructor = MagicMock(return_value=node)
    send = MagicMock(return_value=(True, ""))
    error = KeyboardInterrupt() if case == "interrupt" else RuntimeError("test")
    if case in ("exception", "interrupt"):
        send.side_effect = error
    if case == "node":
        constructor.side_effect = error
    client.wait_for_server.return_value = case != "server"
    node.create_client.return_value.wait_for_service.return_value = case != "service"
    monkeypatch.setattr(rclpy, "init", MagicMock())
    monkeypatch.setattr(rclpy, "try_shutdown", MagicMock())
    monkeypatch.setattr(rclpy.node, "Node", constructor)
    monkeypatch.setattr(rclpy.action, "ActionClient", MagicMock(return_value=client))
    monkeypatch.setattr(capture_run, "_send_goal", send)
    monkeypatch.setattr(capture_run, "_save_map", MagicMock())
    run_json = tmp_path / "scan.json"
    run_json.write_text('{"tasks": [{"task_type": "moveto", "target": "home"}]}')
    args = SimpleNamespace(run_json=run_json, capture_prefix="oscan",
                           capture_service="/capture", no_save=False, save="map.bt")
    if case in ("exception", "interrupt", "node"):
        with pytest.raises(type(error)) as caught:
            capture_run.run(args)
        assert caught.value is error
    else:
        assert capture_run.run(args) == (0 if case == "success" else 1)
    assert node.destroy_node.call_count == (0 if case == "node" else 1)
    rclpy.try_shutdown.assert_called_once_with()
    assert capture_run._save_map.call_count == (1 if case == "success" else 0)
