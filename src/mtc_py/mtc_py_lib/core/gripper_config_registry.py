"""Gripper Configuration Registry - loads gripper configs from YAML.

Python equivalent of gripper_config_registry.cpp.
Maps gripper names to MoveIt packages and tool voltage settings.
"""

import os
from dataclasses import dataclass
from typing import Dict, Optional, List
import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass
class GripperConfig:
    """Configuration for a single gripper type."""
    name: str
    moveit_package: str
    tool_voltage: int


class GripperConfigRegistry:
    """Registry of gripper configurations loaded from YAML.

    Loads gripper configurations from mtc_pipeline/config/grippers.yaml
    and provides lookup by gripper name.
    """

    def __init__(self, logger=None):
        """Initialize the registry and load configurations.

        Args:
            logger: Optional ROS logger for debug output
        """
        self._configs: Dict[str, GripperConfig] = {}
        self._logger = logger
        self._load_config()

    def _load_config(self):
        """Load gripper configurations from YAML file."""
        try:
            # Get path to grippers.yaml
            mtc_pipeline_share = get_package_share_directory("mtc_pipeline")
            config_path = os.path.join(mtc_pipeline_share, "config", "grippers.yaml")

            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            grippers = data.get("grippers", {})
            for name, config in grippers.items():
                self._configs[name] = GripperConfig(
                    name=name,
                    moveit_package=config.get("moveit_package", ""),
                    tool_voltage=config.get("tool_voltage", 0),
                )

            if self._logger:
                self._logger.info(
                    f"Loaded {len(self._configs)} gripper configurations: "
                    f"{list(self._configs.keys())}"
                )

        except Exception as e:
            if self._logger:
                self._logger.error(f"Failed to load gripper config: {e}")
            # Provide minimal fallback configs
            self._configs = {
                "none": GripperConfig("none", "ur_standalone_moveit_config", 0),
                "epick": GripperConfig("epick", "ur_zivid_epick_moveit_config", 24),
                "hande": GripperConfig("hande", "ur_zivid_hande_moveit_config", 24),
                "pipettor": GripperConfig("pipettor", "ur_zivid_pipettor_moveit_config", 24),
            }

    def get_config(self, gripper_name: str) -> Optional[GripperConfig]:
        """Get configuration for a gripper.

        Args:
            gripper_name: Name of the gripper (e.g., "epick", "hande")

        Returns:
            GripperConfig if found, None otherwise
        """
        return self._configs.get(gripper_name)

    def available_grippers(self) -> List[str]:
        """Get list of available gripper names.

        Returns:
            List of gripper names
        """
        return list(self._configs.keys())
