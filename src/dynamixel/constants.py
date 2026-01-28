"""Dynamixel motor constants for ViperX-300s."""

from typing import Dict, List, NamedTuple

# Protocol version for XM series motors
PROTOCOL_VERSION = 2.0

# Default communication settings
DEFAULT_BAUD_RATE = 1_000_000  # 1 Mbps
DEFAULT_USB_PORT = "/dev/ttyUSB0"

# Control Table Addresses (XM540-W270 and XM430-W350, Protocol 2.0)
ADDR_OPERATING_MODE = 11          # 1 byte
ADDR_TORQUE_ENABLE = 64           # 1 byte
ADDR_PROFILE_ACCELERATION = 108   # 4 bytes
ADDR_PROFILE_VELOCITY = 112       # 4 bytes
ADDR_GOAL_POSITION = 116          # 4 bytes
ADDR_MOVING = 122                 # 1 byte
ADDR_MOVING_STATUS = 123          # 1 byte
ADDR_PRESENT_VELOCITY = 128       # 4 bytes
ADDR_PRESENT_POSITION = 132       # 4 bytes

# Data lengths
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
LEN_MOVING = 1

# Motor resolution (XM series)
POSITION_RESOLUTION = 4096  # ticks per revolution
CENTER_POSITION = 2048      # center value (0 degrees)

# Operating modes
POSITION_CONTROL_MODE = 3
EXTENDED_POSITION_MODE = 4
PWM_CONTROL_MODE = 16


class JointConfig(NamedTuple):
    """Configuration for a single joint."""

    name: str
    motor_ids: List[int]  # Primary motor ID first, shadow motor if dual
    min_degrees: float
    max_degrees: float


# ViperX-300s Joint Configuration
# For dual-motor joints (shoulder, elbow), both motors receive the same command
VIPERX_300S_JOINTS: Dict[int, JointConfig] = {
    0: JointConfig("waist", [1], -180.0, 180.0),
    1: JointConfig("shoulder", [2, 3], -111.0, 111.0),      # dual motor (extended from -101/101)
    2: JointConfig("elbow", [4, 5], -106.0, 97.0),          # dual motor (extended from -101/92)
    3: JointConfig("forearm_roll", [6], -180.0, 180.0),
    4: JointConfig("wrist_angle", [7], -107.0, 130.0),
    5: JointConfig("wrist_rotate", [8], -180.0, 180.0),
}

# Motor IDs
ALL_ARM_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8]  # All arm motors (excluding gripper)
PRIMARY_MOTOR_IDS = [1, 2, 4, 6, 7, 8]         # One per joint (for reading)
GRIPPER_MOTOR_ID = 9                           # Gripper motor

# Number of DOF for the arm
NUM_JOINTS = 6

# Default motion profile values
DEFAULT_PROFILE_VELOCITY = 100      # 0-1023, 0 = no limit
DEFAULT_PROFILE_ACCELERATION = 50   # 0-32767
