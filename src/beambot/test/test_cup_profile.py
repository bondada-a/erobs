"""Check cup-profile launch wiring and real Xacro geometry without starting nodes."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from beambot import config_loader

xacro = pytest.importorskip("xacro")
pytest.importorskip("moveit_configs_utils")
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable


def test_selected_cup_reaches_both_models(monkeypatch, tmp_path):
    root = Path(__file__).resolve().parents[3] / "src/custom-ur-descriptions"
    monkeypatch.setenv("BEAMBOT_BEAMLINE_CONFIG", str(root.parent / "beambot/config/cms_drylab.yaml"))
    monkeypatch.setattr(config_loader, "_config_cache", None)
    spec = importlib.util.spec_from_file_location(
        "cup_profile_launch", root / "cms_moveit_config/launch/robot_bringup.launch.py",
    )
    bringup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bringup)
    get_share = bringup.get_package_share_path
    monkeypatch.setattr(bringup, "get_package_share_path", lambda package: (
        root / package if package in ("cms_moveit_config", "cms_robot_description")
        else get_share(package)
    ))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("BEAMBOT_EPICK_CUP_PROFILE", "3mm_dia_external_air")
    monkeypatch.delenv("BEAMBOT_EPICK_CUP_PROFILE", raising=False)
    context = LaunchContext()
    for action in bringup.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["cup_profile"] == "3mm_dia_external_air"
    context.launch_configurations.update(gripper="epick", use_mock_hardware="true")
    model = root / "cms_robot_description/urdf/ur_with_zivid_epick.xacro"

    def geometry(**mappings):
        doc = ET.fromstring(xacro.process_file(
            str(model), mappings={"name": "ur5e", **mappings},
        ).toxml())
        return [ET.tostring(node) for node in doc if node.tag in ("joint", "link")]

    default_geometry = geometry()
    assert default_geometry == geometry(cup_profile="3mm_dia_external_air")
    for profile in ("3mm_dia", "7mm_dia", "pen_vacuum", "default", "3mm_dia_external_air"):
        context.launch_configurations["cup_profile"] = profile
        actions = bringup.launch_setup(context)
        assert isinstance(actions[0], SetEnvironmentVariable)
        assert isinstance(actions[1], IncludeLaunchDescription)
        actions[0].execute(context)
        assert context.environment["BEAMBOT_EPICK_CUP_PROFILE"] == profile
        rsp_geometry = geometry()
        # An explicit MoveIt selection must win even over a stale environment.
        monkeypatch.setenv("BEAMBOT_EPICK_CUP_PROFILE", "3mm_dia_external_air")
        assert rsp_geometry == geometry(cup_profile=profile)
        if profile != "3mm_dia_external_air":
            assert rsp_geometry != default_geometry

    monkeypatch.setenv("BEAMBOT_EPICK_CUP_PROFILE", "missing_profile")
    with pytest.raises(xacro.XacroException, match="missing_profile"):
        geometry()
    context.launch_configurations["gripper"] = "none"
    assert not any(isinstance(action, SetEnvironmentVariable)
                   for action in bringup.launch_setup(context))
