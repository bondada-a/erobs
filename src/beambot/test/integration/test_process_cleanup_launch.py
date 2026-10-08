"""Exercise production shutdown against disposable process groups, never drivers."""

import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc

from beambot.utils.moveit_lifecycle_manager import MoveItLifecycleManager


def generate_test_description():
    return LaunchDescription([KeepAliveProc(), ReadyToTest()])


def _running(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1][0] != "Z"
    except FileNotFoundError:
        return False


class ProcessCleanupCase(unittest.TestCase):
    ignore_term = False

    def setUp(self):
        child_code = (
            "import os, signal; "
            + ("signal.signal(signal.SIGTERM, signal.SIG_IGN); " if self.ignore_term else "")
            + "print(os.getpid(), flush=True); signal.pause()"
        )
        parent_code = (
            "import subprocess, sys, signal; "
            "child = subprocess.Popen([sys.executable, '-c', sys.argv[1]], stdout=subprocess.PIPE, text=True); "
            "print(child.stdout.readline().strip(), flush=True); signal.pause()"
        )
        self.process = subprocess.Popen(
            [sys.executable, "-c", parent_code, child_code],
            start_new_session=True, stdout=subprocess.PIPE, text=True,
        )
        self.child_pid = None
        self.addCleanup(self.reap_processes)
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            self.assertTrue(selector.select(timeout=5), "Test child did not start")
        self.child_pid = int(self.process.stdout.readline())
        self.assertEqual(os.getpgid(self.child_pid), self.process.pid)
        self.assertNotEqual(self.process.pid, os.getpgrp())

        manager = MoveItLifecycleManager.__new__(MoveItLifecycleManager)
        manager._moveit_process = self.process
        manager._logger = Mock()
        manager._node = SimpleNamespace()
        manager._model_revision = "test"
        manager._drain_stale_execute_trajectory = Mock()
        start = time.monotonic()
        manager.kill_current_process()
        self.stop_sec = time.monotonic() - start

        deadline = time.monotonic() + 2
        while _running(self.child_pid) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsNotNone(self.process.poll())
        self.assertIsNone(manager._moveit_process)
        self.assertEqual(manager._model_revision, "")
        manager._drain_stale_execute_trajectory.assert_called_once_with()

    def reap_processes(self):
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=5)
        self.process.stdout.close()
        deadline = time.monotonic() + 5
        while True:
            try:
                pid, _ = os.waitpid(-self.process.pid, os.WNOHANG)
            except ChildProcessError:
                break
            if pid == 0:
                self.assertLess(time.monotonic(), deadline, "Test child could not be reaped")
                time.sleep(0.01)


class TestProcessCleanup(ProcessCleanupCase):
    def test_cooperative_child_stops(self):
        self.assertFalse(_running(self.child_pid))
        self.assertLess(self.stop_sec, 1.0, "Cooperative stop should not wait for the SIGKILL timeout")


class TestStubbornProcessCleanup(ProcessCleanupCase):
    ignore_term = True

    def test_child_outliving_launch_parent_stops(self):
        self.assertFalse(_running(self.child_pid))
        self.assertLess(self.stop_sec, 3.5, "SIGKILL fallback should fire at the 3 s budget")
