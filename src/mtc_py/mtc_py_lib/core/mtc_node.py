"""MTC Node wrapper - manages rclcpp.Node for MTC operations.

MTC requires rclcpp.Node (C++ backed), not rclpy.Node, because:
- MTC's C++ code expects rclcpp::Node::SharedPtr
- The rclcpp Python module creates actual C++ Node objects via pybind11
- rclpy.Node is pure Python and cannot be passed to C++ MTC code

OMPL Parameter Injection:
The Python rclcpp.Node binding doesn't expose declare_parameter(), so we
inject OMPL parameters via sys.argv BEFORE rclcpp.init(). This matches the
C++ base_stages.cpp behavior which declares these parameters on the node.
"""

import sys
from typing import Optional
import rclcpp


class MTCNode:
    """Wrapper around rclcpp.Node for MTC operations.

    MTC requires rclcpp.Node (C++ backed), not rclpy.Node.
    This class manages the rclcpp context for MTC operations.

    This is a singleton - only one MTC node context should exist per process.

    OMPL Configuration:
    The OMPL planning plugin parameters are injected via sys.argv before
    rclcpp.init() to ensure PipelinePlanner can find and use OMPL instead
    of falling back to CHOMP.
    """

    _instance: Optional['MTCNode'] = None
    _initialized: bool = False

    # OMPL parameters to inject (matches C++ base_stages.cpp)
    _OMPL_PARAMS = [
        ('ompl.planning_plugin', 'ompl_interface/OMPLPlanner'),
        ('ompl.request_adapters',
         'default_planner_request_adapters/AddTimeOptimalParameterization'),
    ]

    def __init__(self, name: str = "mtc_py"):
        """Initialize the MTC node.

        Args:
            name: Node name for the rclcpp node
        """
        if not MTCNode._initialized:
            # Inject OMPL parameters via sys.argv BEFORE rclcpp.init()
            # This is required because Python's rclcpp.Node doesn't have
            # declare_parameter() method (unlike C++ rclcpp::Node)
            self._inject_ompl_parameters()
            rclcpp.init()
            MTCNode._initialized = True

        # Create NodeOptions with automatically_declare_parameters_from_overrides
        # This is CRITICAL for MTC to pick up the OMPL parameters from sys.argv
        # Without this, parameters won't be propagated and time parameterization
        # will fail, causing "Time between points not strictly increasing" errors
        # See: https://github.com/moveit/moveit_task_constructor/issues/624
        options = rclcpp.NodeOptions()
        options.automatically_declare_parameters_from_overrides = True
        options.allow_undeclared_parameters = True

        self._node = rclcpp.Node(name, options)
        self._name = name

    @property
    def node(self):
        """Get the underlying rclcpp.Node for MTC operations.

        Returns:
            The rclcpp.Node instance
        """
        return self._node

    @classmethod
    def get_instance(cls, name: str = "mtc_py") -> 'MTCNode':
        """Get or create singleton MTCNode instance.

        Args:
            name: Node name (only used if creating new instance)

        Returns:
            The singleton MTCNode instance
        """
        if cls._instance is None:
            cls._instance = cls(name)
        return cls._instance

    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance (for testing)."""
        if cls._instance is not None:
            cls._instance.shutdown()
        cls._instance = None

    def _inject_ompl_parameters(self):
        """Inject OMPL parameters into sys.argv before rclcpp.init().

        Python's rclcpp.Node binding doesn't have declare_parameter(),
        so we inject parameters via command-line arguments. rclcpp.init()
        parses sys.argv and makes these parameters available to the node.

        This approach was validated by testing that PipelinePlanner(node, "ompl")
        successfully finds the OMPL plugin when parameters are passed this way.
        """
        # Check if --ros-args already in argv
        if '--ros-args' not in sys.argv:
            sys.argv.append('--ros-args')

        # Add each OMPL parameter
        for param_name, param_value in self._OMPL_PARAMS:
            param_arg = f'{param_name}:={param_value}'
            # Only add if not already present
            if not any(param_name in arg for arg in sys.argv):
                sys.argv.extend(['-p', param_arg])

    def shutdown(self):
        """Shutdown rclcpp context."""
        if MTCNode._initialized:
            try:
                rclcpp.shutdown()
            except Exception:
                pass  # Already shutdown
            MTCNode._initialized = False
            MTCNode._instance = None

    def __repr__(self) -> str:
        return f"MTCNode(name='{self._name}', initialized={MTCNode._initialized})"
