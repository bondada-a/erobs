"""Routine tests must fail before opening robot/proxy connections or processes."""

import socket
import subprocess
import sys
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("address", [("127.0.0.1", 9090), ("192.0.2.10", 30002)])
def test_unmarked_network_connections_are_blocked(address):
    with pytest.raises(AssertionError, match="External activity is disabled"):
        socket.socket.connect(SimpleNamespace(family=socket.AF_INET), address)


def test_subprocess_startup_is_blocked():
    with pytest.raises(AssertionError, match="External activity is disabled"):
        with subprocess.Popen([sys.executable, "-c", "pass"]) as process:
            process.wait(timeout=5)
