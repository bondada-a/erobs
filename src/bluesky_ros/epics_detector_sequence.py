#!/usr/bin/env python3
"""Run two robot tasks around one detector ON/OFF cycle.

Usage:
    python3 src/bluesky_ros/epics_detector_sequence.py
"""

import argparse
import math
import time
from pathlib import Path
from threading import Condition


TASKS_DIR = Path(__file__).parents[1] / "cms" / "tasks"
APPROACH_TASK = TASKS_DIR / "epick_tool_exchange_to_spincoater.json"
RETRACT_TASK = TASKS_DIR / "epick_spincoater_forward_retract.json"
DETECTOR_PV = "XF:11BMB-ES{Det:PIL2M}:cam1:DetectorState_RBV"
IDLE = 0
ACQUIRE = 1


def finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("must be finite")
    return value


def positive_float(text):
    value = finite_float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


class PVState:
    """Thread-safe state updated by PyEPICS monitor callbacks.

    >>> from threading import Timer
    >>> state = PVState("test")
    >>> state.on_connection(conn=True)
    >>> timer = Timer(0.01, state.on_value, kwargs={"value": 1})
    >>> timer.start()
    >>> state.wait_for(1, timeout=1)
    1
    >>> timer.join()
    """

    def __init__(self, pvname):
        self.pvname = pvname
        self.condition = Condition()
        self.connected = False
        self.value = None

    def on_connection(self, conn=None, **_):
        with self.condition:
            self.connected = bool(conn)
            self.condition.notify_all()

    def on_value(self, value=None, **_):
        with self.condition:
            self.value = value
            self.condition.notify_all()

    def wait_for(self, wanted, *, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if not self.connected:
                    raise ConnectionError(f"{self.pvname} disconnected")
                if self.value == wanted:
                    return self.value
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Expected detector value {wanted}, last value was {self.value}"
                    )
                self.condition.wait(remaining)


def run_task(robot, path):
    print(f"Running {path.name}")
    robot.set(str(path)).wait()


def main():
    parser = argparse.ArgumentParser(
        description="Run the EPICK spincoater tasks around a detector ON/OFF cycle."
    )
    parser.add_argument("--timeout", type=positive_float, default=300.0)
    parser.add_argument("--connection-timeout", type=positive_float, default=5.0)
    args = parser.parse_args()

    import epics
    import rclpy

    from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice

    state = PVState(DETECTOR_PV)
    detector = epics.PV(
        DETECTOR_PV,
        auto_monitor=True,
        connection_callback=state.on_connection,
    )
    if not detector.wait_for_connection(timeout=args.connection_timeout):
        raise ConnectionError(f"Could not connect to {DETECTOR_PV}")
    state.on_connection(conn=detector.connected)
    detector.add_callback(state.on_value, run_now=True)

    rclpy.init()
    robot = None
    try:
        robot = MTCExecutionDevice()
        run_task(robot, APPROACH_TASK)
        print(f"Waiting for {DETECTOR_PV} = Idle ({IDLE}), initial OFF")
        state.wait_for(IDLE, timeout=args.timeout)
        print(f"Waiting for {DETECTOR_PV} = Acquire ({ACQUIRE}), ON")
        state.wait_for(ACQUIRE, timeout=args.timeout)
        print(f"Waiting for {DETECTOR_PV} = Idle ({IDLE}), OFF")
        state.wait_for(IDLE, timeout=args.timeout)
        run_task(robot, RETRACT_TASK)
    except BaseException:
        if robot is not None:
            robot.cancel_goal()
        raise
    finally:
        detector.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
