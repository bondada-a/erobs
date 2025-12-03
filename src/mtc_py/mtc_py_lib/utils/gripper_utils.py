"""Gripper helper functions matching gripper_utils.hpp.

Provides utilities for mapping gripper types to MoveIt group names
and SRDF state names.
"""


def get_group_name(gripper_type: str) -> str:
    """Get MoveIt group name for gripper type.

    Args:
        gripper_type: Gripper identifier (e.g., "hande", "epick")

    Returns:
        MoveIt group name or empty string for no gripper

    Examples:
        >>> get_group_name("hande")
        'hande_gripper'
        >>> get_group_name("none")
        ''
        >>> get_group_name("pipettor")
        ''
    """
    if not gripper_type or gripper_type in ("none", "pipettor"):
        return ""
    return f"{gripper_type}_gripper"


def get_state_name(gripper_type: str, open: bool) -> str:
    """Get SRDF state name for gripper position.

    Args:
        gripper_type: Gripper identifier
        open: True for open position, False for closed

    Returns:
        SRDF state name (e.g., "hande_open", "vacuum_on")

    Examples:
        >>> get_state_name("hande", True)
        'hande_open'
        >>> get_state_name("hande", False)
        'hande_closed'
        >>> get_state_name("epick", True)
        'vacuum_off'
        >>> get_state_name("epick", False)
        'vacuum_on'
    """
    if gripper_type == "epick":
        return "vacuum_off" if open else "vacuum_on"
    return f"{gripper_type}_{'open' if open else 'closed'}"


def get_end_effector_action(gripper_type: str, open: bool) -> str:
    """Get the action name for end effector commands.

    This matches the end_effector_action field in EndEffectorAction.

    Args:
        gripper_type: Gripper identifier
        open: True for open position, False for closed

    Returns:
        Action name for the end effector command
    """
    return get_state_name(gripper_type, open)
