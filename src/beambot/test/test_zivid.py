"""Hardware-free checks for the Zivid camera wrapper."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from beambot.camera import zivid


class _Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _node(client=None):
    node = MagicMock()
    node.create_client.return_value = client
    stamp = SimpleNamespace(sec=1, nanosec=2)
    node.get_clock.return_value.now.return_value.to_msg.return_value = stamp
    return node


def test_capture_2d_uses_one_timeout_budget(monkeypatch):
    clock = _Clock()
    client = MagicMock()
    client.wait_for_service.side_effect = lambda *, timeout_sec: (
        clock.sleep(timeout_sec) or True
    )
    client.call_async.return_value = object()
    node = _node(client)
    subscription = object()
    node.create_subscription.return_value = subscription
    waited = []

    monkeypatch.setattr(zivid, "CvBridge", MagicMock)
    monkeypatch.setattr(zivid.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(zivid.time, "sleep", clock.sleep)
    monkeypatch.setattr(
        zivid,
        "_wait_for_future",
        lambda _future, timeout: waited.append(timeout) or False,
    )

    assert zivid.capture_2d(node, timeout=10.0) is None
    assert waited == [pytest.approx(7.5)]
    node.destroy_subscription.assert_called_once_with(subscription)
    node.destroy_client.assert_called_once_with(client)


def test_detect_markers_contains_service_exception(monkeypatch):
    clock = _Clock()
    future = MagicMock()
    future.result.side_effect = RuntimeError("driver failed")
    client = MagicMock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    node = _node()
    waited = []

    monkeypatch.setattr(zivid.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(zivid.time, "sleep", clock.sleep)
    monkeypatch.setattr(
        zivid,
        "_wait_for_future",
        lambda _future, timeout: waited.append(timeout) or True,
    )

    result = zivid.detect_markers(
        client, node, marker_ids=[7], timeout=5.0, settle_time=2.0
    )

    assert result.markers == []
    assert result.capture_stamp is None
    assert waited == [pytest.approx(3.0)]


def test_sample_roi_needs_image_but_not_point_cloud(monkeypatch):
    clock = _Clock()
    marker = SimpleNamespace(
        id=7,
        corners_in_pixel_coordinates=[
            SimpleNamespace(x=0, y=0),
            SimpleNamespace(x=10, y=0),
            SimpleNamespace(x=10, y=10),
            SimpleNamespace(x=0, y=10),
        ],
        pose=Pose(),
    )
    response = SimpleNamespace(
        success=True,
        detection_result=SimpleNamespace(detected_markers=[marker]),
    )
    future = MagicMock()
    future.result.return_value = response
    client = MagicMock()
    client.wait_for_service.return_value = True
    node = _node(client)
    subscription = object()
    image = object()
    callback = None

    def create_subscription(message_type, topic, received, qos):
        nonlocal callback
        callback = received
        return subscription

    def call_async(_request):
        callback(image)
        return future

    node.create_subscription.side_effect = create_subscription
    client.call_async.side_effect = call_async
    bridge = MagicMock()
    bridge.imgmsg_to_cv2.return_value = np.zeros((20, 20, 3), dtype=np.uint8)
    waited = []

    monkeypatch.setattr(zivid, "CvBridge", lambda: bridge)
    monkeypatch.setattr(zivid.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(zivid.time, "sleep", clock.sleep)
    monkeypatch.setattr(
        zivid,
        "_wait_for_future",
        lambda _future, timeout: waited.append(timeout) or True,
    )
    monkeypatch.setattr(
        zivid,
        "detect_sample_in_roi",
        lambda *_args, **_kwargs: {
            "pickup_px": (12, 12),
            "sample_size_mm": (1.0, 1.0),
        },
    )
    monkeypatch.setattr(
        zivid,
        "sample_roi_pickup_camera_xyz",
        lambda *_args, **_kwargs: (0.1, 0.2, 0.3),
    )

    result = zivid.detect_sample_roi(node, tag_id=7, timeout=5.0)

    assert result is not None
    pose, _stamp = result
    assert (pose.position.x, pose.position.y, pose.position.z) == (0.1, 0.2, 0.3)
    assert waited == [pytest.approx(3.0)]
    node.create_subscription.assert_called_once()
    assert node.create_subscription.call_args.args[:2] == (Image, zivid.IMAGE_TOPIC)
    node.destroy_subscription.assert_called_once_with(subscription)
    node.destroy_client.assert_called_once_with(client)
