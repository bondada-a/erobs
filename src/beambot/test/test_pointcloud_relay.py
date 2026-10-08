"""Offline relay checks without ROS nodes or Open3D."""

import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from beambot.mapping import pointcloud_relay


@pytest.fixture
def make_relay(monkeypatch):
    def factory(**overrides):
        params = {}
        logger = MagicMock()
        monkeypatch.setattr(pointcloud_relay.Node, "__init__", lambda self, name: None)
        monkeypatch.setattr(pointcloud_relay.Node, "declare_parameter",
                            lambda self, name, default: params.update({name: overrides.get(name, default)}))
        monkeypatch.setattr(pointcloud_relay.Node, "get_parameter",
                            lambda self, name: SimpleNamespace(value=params[name]))
        monkeypatch.setattr(pointcloud_relay.Node, "get_logger", lambda self: logger)
        monkeypatch.setattr(pointcloud_relay.Node, "create_subscription", lambda *args: MagicMock())
        monkeypatch.setattr(pointcloud_relay.Node, "create_publisher", lambda *args: MagicMock())
        return pointcloud_relay.PointCloudRelay()

    return factory


def test_pointcloud_conversion_preserves_xyz_and_metadata(make_relay, monkeypatch):
    cloud = MagicMock()
    cloud.remove_statistical_outlier.return_value = (cloud, [])
    cloud.voxel_down_sample.return_value = cloud
    monkeypatch.setattr(pointcloud_relay, "o3d", SimpleNamespace(
            geometry=SimpleNamespace(PointCloud=lambda: cloud),
            utility=SimpleNamespace(Vector3dVector=np.asarray),
        ), raising=False)
    relay = make_relay(voxel_size=0.01, noise_nb_neighbors=20, noise_std_ratio=2.0)
    xyz = np.array([[1, 2, 3], [np.nan, 4, 5], [6, 7, 8]], dtype=np.float32)
    expected = xyz[[0, 2]]
    header = Header(frame_id="camera")
    header.stamp.sec, header.stamp.nanosec = 42, 123

    for step in (12, 16):
        for names in (("x", "y", "z"), ("z", "y", "x")):
            fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(names)]
            data = np.zeros(len(xyz), dtype=point_cloud2.dtype_from_fields(fields, step))
            for i, name in enumerate(("x", "y", "z")):
                data[name] = xyz[:, i]
            for dense in (False, True):
                msg = PointCloud2(
                    header=header, height=1, width=len(xyz), fields=fields,
                    is_bigendian=sys.byteorder != "little", is_dense=dense,
                    point_step=step, row_step=len(xyz) * step, data=data.tobytes(),
                )
                for noise in (False, True):
                    cloud.reset_mock()
                    relay.enable_noise_filter = noise
                    result = relay._downsample_pointcloud(msg)
                    np.testing.assert_array_equal(cloud.points, expected.astype(np.float64))
                    assert cloud.points.dtype == np.float64
                    assert result == point_cloud2.create_cloud_xyz32(header, expected)
                    assert bytes(msg.data) == data.tobytes()
                    cloud.voxel_down_sample.assert_called_once_with(voxel_size=0.01)
                    if noise:
                        cloud.remove_statistical_outlier.assert_called_once_with(
                            nb_neighbors=20, std_ratio=2.0,
                        )
                    else:
                        cloud.remove_statistical_outlier.assert_not_called()

    relay.get_logger().error.assert_not_called()
    for points in (np.empty((0, 3), dtype=np.float32), np.full((2, 3), np.nan, dtype=np.float32)):
        msg = point_cloud2.create_cloud_xyz32(header, points)
        msg.is_dense = True
        cloud.reset_mock()
        assert relay._downsample_pointcloud(msg) is None
        cloud.voxel_down_sample.assert_not_called()


def test_startup_logs_only_active_processing(make_relay, monkeypatch):
    for available in (False, True):
        monkeypatch.setattr(pointcloud_relay, "OPEN3D_AVAILABLE", available)
        for downsampling in (False, True):
            for noise in (False, True):
                relay = make_relay(enable_downsampling=downsampling, enable_noise_filter=noise)
                logger = relay.get_logger()
                active = downsampling and available
                messages = [call.args[0] for call in logger.info.call_args_list]
                assert ("noise_filter(" in messages[-1]) == (noise and active)
                assert ("passthrough" in messages[-1]) == (not active)
                assert any("Noise filtering is inactive" in msg for msg in messages) == (noise and not active)
                assert logger.warning.call_count == int(downsampling and not available)
                if not active:
                    msg = PointCloud2(width=2, height=1)
                    relay.callback(msg)
                    relay.publisher.publish.assert_called_once_with(msg)
