"""Unit conversion functions for Dynamixel motors."""

import math
from typing import Dict

from .constants import POSITION_RESOLUTION, CENTER_POSITION, JointConfig


def ticks_to_radians(ticks: int) -> float:
    """Convert Dynamixel position ticks to radians.

    Dynamixel XM series motors use 4096 ticks per revolution.
    Position 2048 is center (0 radians).
    Range: 0-4095 maps to approximately -pi to +pi.

    Args:
        ticks: Raw position value from motor (0-4095 for single turn)

    Returns:
        Position in radians
    """
    offset = ticks - CENTER_POSITION
    radians = (offset / POSITION_RESOLUTION) * 2 * math.pi
    return radians


def radians_to_ticks(radians: float) -> int:
    """Convert radians to Dynamixel position ticks.

    Args:
        radians: Position in radians

    Returns:
        Position ticks for motor command
    """
    offset = (radians / (2 * math.pi)) * POSITION_RESOLUTION
    ticks = int(round(CENTER_POSITION + offset))
    return ticks


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return math.degrees(radians)


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return math.radians(degrees)


def validate_joint_limits(
    joint_index: int, radians: float, joints_config: Dict[int, JointConfig]
) -> bool:
    """Check if position is within joint limits.

    Args:
        joint_index: Index of the joint (0-5)
        radians: Target position in radians
        joints_config: Joint configuration dictionary

    Returns:
        True if within limits, False otherwise
    """
    if joint_index not in joints_config:
        return False

    config = joints_config[joint_index]
    degrees = radians_to_degrees(radians)
    return config.min_degrees <= degrees <= config.max_degrees
