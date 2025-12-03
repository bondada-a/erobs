"""UR Tool Interface - manages low-level robot tool operations.

Python equivalent of ur_tool_interface.cpp.
Handles:
- Tool voltage setting via raw socket (port 30002)
- External control restart via dashboard service
"""

import socket
from typing import Optional

from rclpy.node import Node
from std_srvs.srv import Trigger


class URToolInterface:
    """Interface for UR robot tool operations.

    Manages tool voltage and external control program lifecycle.
    """

    # UR secondary interface port for URScript commands
    UR_SECONDARY_PORT = 30002
    SOCKET_TIMEOUT = 2.0  # seconds

    def __init__(self, node: Node, robot_ip: str = ""):
        """Initialize the tool interface.

        Args:
            node: ROS node for service calls and logging
            robot_ip: Robot IP address (can be set later via set_robot_ip)
        """
        self._node = node
        self._logger = node.get_logger()
        self._robot_ip = robot_ip

    def set_robot_ip(self, robot_ip: str):
        """Set the robot IP address.

        Args:
            robot_ip: Robot IP address
        """
        self._robot_ip = robot_ip

    @property
    def robot_ip(self) -> str:
        """Get the current robot IP address."""
        return self._robot_ip

    def set_tool_voltage(self, voltage: int) -> bool:
        """Set tool voltage via raw socket.

        Uses raw socket because this runs BEFORE MoveIt/ROS services are available.
        Connects to UR secondary interface (port 30002) and sends URScript command.

        Args:
            voltage: Tool voltage (0 or 24)

        Returns:
            True if successful, False on failure
        """
        if not self._robot_ip:
            self._logger.error("Robot IP not set")
            return False

        try:
            # Create socket with timeout
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.SOCKET_TIMEOUT)

            # Connect to UR secondary interface
            self._logger.info(
                f"Connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}"
            )
            sock.connect((self._robot_ip, self.UR_SECONDARY_PORT))

            # Send URScript command
            cmd = f"set_tool_voltage({voltage})\n"
            sock.sendall(cmd.encode())

            sock.close()
            self._logger.info(f"Tool voltage set to {voltage}V")
            return True

        except socket.timeout:
            self._logger.error(
                f"Timeout connecting to {self._robot_ip}:{self.UR_SECONDARY_PORT}"
            )
            return False
        except socket.error as e:
            self._logger.error(f"Socket error: {e}")
            return False
        except Exception as e:
            self._logger.error(f"Failed to set tool voltage: {e}")
            return False

    def restart_external_control(self) -> bool:
        """Restart UR external_control program via dashboard service.

        The tool voltage command stops the external_control program,
        so we need to restart it before robot can execute trajectories.

        Calls /dashboard_client/play service.

        Returns:
            True if successful, False on failure
        """
        try:
            # Create service client
            client = self._node.create_client(
                Trigger, "/dashboard_client/play"
            )

            # Wait for service
            if not client.wait_for_service(timeout_sec=5.0):
                self._logger.error("Dashboard play service not available")
                return False

            # Call service
            self._logger.info("Calling /dashboard_client/play...")
            request = Trigger.Request()
            future = client.call_async(request)

            # Wait for result with timeout (using rclpy's efficient waiting)
            import rclpy
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            if not future.done():
                self._logger.error("Dashboard play command timeout")
                return False

            result = future.result()
            if not result.success:
                self._logger.error(
                    f"Failed to restart external_control: {result.message}"
                )
                return False

            self._logger.info("External control program restarted")
            return True

        except Exception as e:
            self._logger.error(f"Failed to restart external_control: {e}")
            return False
