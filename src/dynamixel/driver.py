"""Dynamixel SDK driver for ViperX-300s arm."""

import threading
from typing import Dict, List, Optional

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

from .constants import (
    PROTOCOL_VERSION,
    ADDR_TORQUE_ENABLE,
    ADDR_GOAL_POSITION,
    ADDR_PRESENT_POSITION,
    ADDR_MOVING,
    ADDR_PROFILE_VELOCITY,
    ADDR_PROFILE_ACCELERATION,
    ALL_ARM_MOTOR_IDS,
    PRIMARY_MOTOR_IDS,
    GRIPPER_MOTOR_ID,
    VIPERX_300S_JOINTS,
    NUM_JOINTS,
)
from .conversions import ticks_to_radians, radians_to_ticks


class DynamixelError(Exception):
    """Base exception for Dynamixel communication errors."""

    pass


# Module-level singleton storage for drivers by port
_driver_instances: Dict[str, "DynamixelDriver"] = {}
_driver_lock = threading.Lock()


def get_driver(port: str, baud_rate: int) -> "DynamixelDriver":
    """Get or create a singleton driver instance for a port.

    This ensures only one driver exists per serial port, preventing
    conflicts between arm and gripper components.
    """
    global _driver_instances
    with _driver_lock:
        if port not in _driver_instances:
            driver = DynamixelDriver(port, baud_rate)
            driver.open()
            _driver_instances[port] = driver
        return _driver_instances[port]


def close_driver(port: str) -> None:
    """Close and remove a driver instance."""
    global _driver_instances
    with _driver_lock:
        if port in _driver_instances:
            try:
                _driver_instances[port].close()
            except Exception:
                pass
            del _driver_instances[port]


class DynamixelDriver:
    """Driver for communicating with Dynamixel motors on ViperX-300s.

    This driver handles:
    - Serial port management
    - Synchronized reading/writing for multiple motors
    - Dual-motor joint handling (shoulder, elbow)
    - Thread-safe operations
    """

    def __init__(self, port: str, baud_rate: int):
        """Initialize the Dynamixel driver.

        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0")
            baud_rate: Communication baud rate (typically 1000000)
        """
        self._port_name = port
        self._baud_rate = baud_rate
        self._port_handler: Optional[PortHandler] = None
        self._packet_handler: Optional[PacketHandler] = None
        self._lock = threading.RLock()  # Use RLock for reentrant locking
        self._is_open = False

    @property
    def is_open(self) -> bool:
        """Check if the driver connection is open."""
        return self._is_open

    def open(self) -> None:
        """Open connection to motors.

        Raises:
            DynamixelError: If port cannot be opened or configured
        """
        with self._lock:
            if self._is_open:
                return

            self._port_handler = PortHandler(self._port_name)
            self._packet_handler = PacketHandler(PROTOCOL_VERSION)

            if not self._port_handler.openPort():
                raise DynamixelError(f"Failed to open port {self._port_name}")

            if not self._port_handler.setBaudRate(self._baud_rate):
                self._port_handler.closePort()
                raise DynamixelError(f"Failed to set baud rate {self._baud_rate}")

            self._is_open = True

    def close(self) -> None:
        """Close connection to motors."""
        with self._lock:
            if not self._is_open:
                return

            if self._port_handler:
                self._port_handler.closePort()

            self._is_open = False

    def _check_open(self) -> None:
        """Verify driver is open, raise if not."""
        if not self._is_open:
            raise DynamixelError("Driver not open")

    def enable_torque(self, motor_ids: Optional[List[int]] = None) -> None:
        """Enable torque on specified motors.

        Args:
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            motor_ids = ALL_ARM_MOTOR_IDS

        with self._lock:
            self._check_open()
            for motor_id in motor_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
                )
                if result != COMM_SUCCESS:
                    raise DynamixelError(
                        f"Failed to enable torque on motor {motor_id}: "
                        f"{self._packet_handler.getTxRxResult(result)}"
                    )

    def disable_torque(self, motor_ids: Optional[List[int]] = None) -> None:
        """Disable torque on specified motors.

        Args:
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            motor_ids = ALL_ARM_MOTOR_IDS

        with self._lock:
            self._check_open()
            for motor_id in motor_ids:
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 0
                )
                if result != COMM_SUCCESS:
                    raise DynamixelError(
                        f"Failed to disable torque on motor {motor_id}: "
                        f"{self._packet_handler.getTxRxResult(result)}"
                    )

    def read_joint_positions(self) -> List[float]:
        """Read current positions of all joints.

        Returns:
            List of 6 joint positions in radians

        Raises:
            DynamixelError: If communication fails
        """
        with self._lock:
            self._check_open()

            # Read positions individually for reliability
            # read4ByteTxRx returns (data, result, error)
            positions = []
            for joint_idx in range(NUM_JOINTS):
                config = VIPERX_300S_JOINTS[joint_idx]
                primary_id = config.motor_ids[0]

                data, result, error = self._packet_handler.read4ByteTxRx(
                    self._port_handler, primary_id, ADDR_PRESENT_POSITION
                )
                if result != COMM_SUCCESS or error != 0:
                    raise DynamixelError(
                        f"Failed to read position for motor {primary_id}: "
                        f"{self._packet_handler.getTxRxResult(result)}"
                    )

                positions.append(ticks_to_radians(data))

            return positions

    def write_joint_positions(self, positions: List[float]) -> None:
        """Write goal positions to all joints.

        For dual-motor joints (shoulder, elbow), both motors receive
        the same position command to stay synchronized.

        Args:
            positions: List of 6 target positions in radians

        Raises:
            ValueError: If wrong number of positions provided
            DynamixelError: If communication fails
        """
        if len(positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} positions, got {len(positions)}")

        with self._lock:
            self._check_open()

            # Write positions individually for reliability
            for joint_idx, position in enumerate(positions):
                config = VIPERX_300S_JOINTS[joint_idx]
                ticks = radians_to_ticks(position)

                # Write to all motors for this joint (handles dual-motor joints)
                for motor_id in config.motor_ids:
                    result, error = self._packet_handler.write4ByteTxRx(
                        self._port_handler, motor_id, ADDR_GOAL_POSITION, ticks
                    )
                    if result != COMM_SUCCESS:
                        raise DynamixelError(
                            f"Failed to write position for motor {motor_id}: "
                            f"{self._packet_handler.getTxRxResult(result)}"
                        )

    def is_moving(self) -> bool:
        """Check if any motor is currently moving.

        Returns:
            True if any motor is moving, False otherwise
        """
        with self._lock:
            self._check_open()

            # Read moving status from each motor individually
            # read1ByteTxRx returns (data, result, error)
            for motor_id in PRIMARY_MOTOR_IDS:
                data, result, error = self._packet_handler.read1ByteTxRx(
                    self._port_handler, motor_id, ADDR_MOVING
                )
                if result != COMM_SUCCESS or error != 0:
                    # Skip motors that fail to respond
                    continue
                if data:
                    return True

            return False

    def set_profile_velocity(
        self, velocity: int, motor_ids: Optional[List[int]] = None
    ) -> None:
        """Set profile velocity for smooth motion.

        Args:
            velocity: Velocity value (0 = no limit, higher = faster)
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            motor_ids = ALL_ARM_MOTOR_IDS

        with self._lock:
            self._check_open()
            for motor_id in motor_ids:
                result, error = self._packet_handler.write4ByteTxRx(
                    self._port_handler, motor_id, ADDR_PROFILE_VELOCITY, velocity
                )
                if result != COMM_SUCCESS:
                    raise DynamixelError(
                        f"Failed to set velocity on motor {motor_id}"
                    )

    def set_profile_acceleration(
        self, acceleration: int, motor_ids: Optional[List[int]] = None
    ) -> None:
        """Set profile acceleration for smooth motion.

        Args:
            acceleration: Acceleration value (0-32767)
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            motor_ids = ALL_ARM_MOTOR_IDS

        with self._lock:
            self._check_open()
            for motor_id in motor_ids:
                result, error = self._packet_handler.write4ByteTxRx(
                    self._port_handler, motor_id, ADDR_PROFILE_ACCELERATION, acceleration
                )
                if result != COMM_SUCCESS:
                    raise DynamixelError(
                        f"Failed to set acceleration on motor {motor_id}"
                    )

    # Gripper-specific methods

    def enable_gripper_torque(self) -> None:
        """Enable torque on the gripper motor."""
        self.enable_torque([GRIPPER_MOTOR_ID])

    def disable_gripper_torque(self) -> None:
        """Disable torque on the gripper motor."""
        self.disable_torque([GRIPPER_MOTOR_ID])

    def read_gripper_position(self) -> float:
        """Read current gripper position.

        Returns:
            Gripper position in radians
        """
        with self._lock:
            self._check_open()

            # read4ByteTxRx returns (data, result, error)
            data, result, error = self._packet_handler.read4ByteTxRx(
                self._port_handler, GRIPPER_MOTOR_ID, ADDR_PRESENT_POSITION
            )
            if result != COMM_SUCCESS or error != 0:
                raise DynamixelError(
                    f"Failed to read gripper position: "
                    f"{self._packet_handler.getRxPacketError(error)}"
                )

            return ticks_to_radians(data)

    def write_gripper_position(self, position: float) -> None:
        """Write goal position to gripper.

        Args:
            position: Target position in radians
        """
        with self._lock:
            self._check_open()

            ticks = radians_to_ticks(position)
            result, error = self._packet_handler.write4ByteTxRx(
                self._port_handler, GRIPPER_MOTOR_ID, ADDR_GOAL_POSITION, ticks
            )
            if result != COMM_SUCCESS:
                raise DynamixelError(
                    f"Failed to write gripper position: "
                    f"{self._packet_handler.getTxRxResult(result)}"
                )

    def is_gripper_moving(self) -> bool:
        """Check if gripper is currently moving.

        Returns:
            True if gripper is moving, False otherwise
        """
        with self._lock:
            self._check_open()

            # read1ByteTxRx returns (data, result, error)
            data, result, error = self._packet_handler.read1ByteTxRx(
                self._port_handler, GRIPPER_MOTOR_ID, ADDR_MOVING
            )
            if result != COMM_SUCCESS or error != 0:
                raise DynamixelError(
                    f"Failed to read gripper moving status: "
                    f"{self._packet_handler.getRxPacketError(error)}"
                )

            return bool(data)
