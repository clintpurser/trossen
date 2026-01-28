"""Dynamixel SDK driver package for ViperX-300s arm."""

from .constants import (
    PROTOCOL_VERSION,
    DEFAULT_BAUD_RATE,
    DEFAULT_USB_PORT,
    DEFAULT_PROFILE_VELOCITY,
    DEFAULT_PROFILE_ACCELERATION,
    VIPERX_300S_JOINTS,
    ALL_ARM_MOTOR_IDS,
    PRIMARY_MOTOR_IDS,
    GRIPPER_MOTOR_ID,
    NUM_JOINTS,
)
from .conversions import (
    ticks_to_radians,
    radians_to_ticks,
    radians_to_degrees,
    degrees_to_radians,
    validate_joint_limits,
)
from .driver import DynamixelDriver, DynamixelError, get_driver, close_driver

__all__ = [
    "PROTOCOL_VERSION",
    "DEFAULT_BAUD_RATE",
    "DEFAULT_USB_PORT",
    "DEFAULT_PROFILE_VELOCITY",
    "DEFAULT_PROFILE_ACCELERATION",
    "VIPERX_300S_JOINTS",
    "ALL_ARM_MOTOR_IDS",
    "PRIMARY_MOTOR_IDS",
    "GRIPPER_MOTOR_ID",
    "NUM_JOINTS",
    "ticks_to_radians",
    "radians_to_ticks",
    "radians_to_degrees",
    "degrees_to_radians",
    "validate_joint_limits",
    "DynamixelDriver",
    "DynamixelError",
    "get_driver",
    "close_driver",
]
