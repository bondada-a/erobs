"""Print stages - simple message printing (no MTC needed)."""


class PrintStages:
    """Handles print action - simple console output."""

    def __init__(self, node):
        """Initialize with ROS 2 node for logging."""
        self.logger = node.get_logger()

    def run(self, goal) -> bool:
        """Execute print action. Returns True always."""
        message = goal.message if hasattr(goal, 'message') else str(goal)
        self.logger.info(f"MESSAGE: {message}")
        return True
