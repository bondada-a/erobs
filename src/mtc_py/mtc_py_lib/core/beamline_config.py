"""Beamline configuration loader - Python equivalent of beamline_config.cpp.

Defines deployment-specific settings for each beamline (CMS, LIX, PDF, etc.)
Enables beamline-agnostic framework deployment.
"""

import os
from dataclasses import dataclass, field
from typing import Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class GripperEntry:
    """Configuration for a single gripper in beamline config."""
    moveit_package: str
    tool_voltage: int
    group_name: str = ""


@dataclass
class RobotConfig:
    """Robot-specific configuration."""
    model: str  # e.g., "ur5e", "ur3e"
    ip: str  # Default IP for this beamline
    arm_group: str = "ur_arm"
    ik_frame: str = "tool0"


@dataclass
class PlanningConfig:
    """Planning parameters."""
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2


@dataclass
class BeamlineConfig:
    """Complete beamline deployment configuration.

    Mirrors the C++ BeamlineConfig struct from beamline_config.hpp.
    """
    name: str
    robot: RobotConfig
    grippers: Dict[str, GripperEntry] = field(default_factory=dict)
    available_grippers: List[str] = field(default_factory=list)
    obstacle_config: str = "config/beamline_scene.yaml"
    planning: PlanningConfig = field(default_factory=PlanningConfig)


def load_beamline_config(yaml_file: str) -> BeamlineConfig:
    """Load beamline configuration from YAML file.

    Mirrors the C++ load_beamline_config() function.

    Args:
        yaml_file: Path to YAML file. If relative, resolved against
                   mtc_pipeline package share directory.

    Returns:
        BeamlineConfig populated from file

    Raises:
        RuntimeError: If file not found or invalid format
    """
    # Resolve path - if relative, use mtc_pipeline package share directory
    if yaml_file and not yaml_file.startswith('/'):
        try:
            package_share = get_package_share_directory("mtc_pipeline")
            resolved_path = os.path.join(package_share, yaml_file)
        except Exception as e:
            raise RuntimeError(
                f"Failed to resolve package path for: {yaml_file}"
            ) from e
    else:
        resolved_path = yaml_file

    # Check if file exists
    if not os.path.exists(resolved_path):
        raise RuntimeError(f"Beamline config file not found: {resolved_path}")

    # Load YAML
    try:
        with open(resolved_path, 'r') as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise RuntimeError(f"Failed to parse beamline config: {e}") from e

    # Validate required fields
    if not config.get("beamline"):
        raise RuntimeError("Missing 'beamline' field in config")

    if not config.get("robot"):
        raise RuntimeError("Missing 'robot' section in config")

    if not config.get("grippers"):
        raise RuntimeError("Missing 'grippers' section in config")

    # Parse robot configuration
    robot_data = config["robot"]
    robot = RobotConfig(
        model=robot_data["model"],
        ip=robot_data["ip"],
        arm_group=robot_data.get("arm_group", "ur_arm"),
        ik_frame=robot_data.get("ik_frame", "tool0"),
    )

    # Parse gripper configurations
    grippers: Dict[str, GripperEntry] = {}
    for name, gripper_data in config["grippers"].items():
        if not gripper_data.get("moveit_package"):
            raise RuntimeError(f"Missing 'moveit_package' for gripper: {name}")
        if "tool_voltage" not in gripper_data:
            raise RuntimeError(f"Missing 'tool_voltage' for gripper: {name}")

        grippers[name] = GripperEntry(
            moveit_package=gripper_data["moveit_package"],
            tool_voltage=gripper_data["tool_voltage"],
            group_name=gripper_data.get("group_name", ""),
        )

    # Parse available grippers (defaults to all configured grippers)
    available = config.get("available_grippers", list(grippers.keys()))

    # Parse workspace configuration
    workspace_data = config.get("workspace", {})
    obstacle_config = workspace_data.get(
        "obstacle_config", "config/beamline_scene.yaml"
    )

    # Parse planning parameters
    planning_data = config.get("planning", {})
    planning = PlanningConfig(
        velocity_scaling=planning_data.get("velocity_scaling", 0.2),
        acceleration_scaling=planning_data.get("acceleration_scaling", 0.2),
    )

    return BeamlineConfig(
        name=config["beamline"],
        robot=robot,
        grippers=grippers,
        available_grippers=available,
        obstacle_config=obstacle_config,
        planning=planning,
    )
