"""Keep routine tests disconnected from ROS, hardware, and external services."""

import importlib.util
from pathlib import Path
import socket
import subprocess

import numpy.testing  # Load NumPy's CPU probe before subprocesses are blocked.
import pytest


def _blocked(*args, **kwargs):
    raise AssertionError("External activity is disabled in unit tests; use an isolated integration test")


def pytest_sessionstart(session):
    patch = pytest.MonkeyPatch()
    session.config._unit_safety_patch = patch
    session.config._unit_allow_loopback = False
    patch.setenv("BEAMBOT_BEAMLINE_CONFIG", str(
        Path(__file__).parent / "src/beambot/config/cms_drylab.yaml"
    ))
    for name in ("rclpy", "rclcpp"):
        if importlib.util.find_spec(name) is not None:
            module = __import__(name)
            patch.setattr(module, "init", _blocked)
            if name == "rclcpp":
                patch.setattr(module, "Node", _blocked)
            else:
                from rclpy.node import Node
                patch.setattr(Node, "__init__", _blocked)

    for method in ("connect", "connect_ex", "sendto", "bind"):
        original = getattr(socket.socket, method)

        def local_only(sock, *args, _original=original):
            address = args[-1]
            if sock.family != socket.AF_UNIX:
                if not session.config._unit_allow_loopback or address[0] not in ("127.0.0.1", "::1", "localhost"):
                    _blocked()
            return _original(sock, *args)

        patch.setattr(socket.socket, method, local_only)
    patch.setattr(subprocess.Popen, "__init__", _blocked)


@pytest.fixture(autouse=True)
def _loopback_scope(request):
    request.config._unit_allow_loopback = request.node.get_closest_marker("loopback") is not None
    yield
    request.config._unit_allow_loopback = False


def pytest_sessionfinish(session):
    session.config._unit_safety_patch.undo()
