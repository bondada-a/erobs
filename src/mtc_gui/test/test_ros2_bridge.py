import time

from PyQt6.QtCore import QCoreApplication

from mtc_gui.ros2_bridge import ROS2Bridge


_app = QCoreApplication.instance() or QCoreApplication([])


def test_missing_action_client_emits_one_result():
    bridge = ROS2Bridge()
    results = []
    bridge.action_result_received.connect(lambda *result: results.append(result))

    bridge.execute_task("{}")

    deadline = time.monotonic() + 2
    while not results and time.monotonic() < deadline:
        _app.processEvents()
        time.sleep(0.01)

    assert results == [(0, "Action client unavailable", 0, 0)]
