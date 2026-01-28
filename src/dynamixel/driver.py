"""Dynamixel SDK driver for ViperX-300s arm."""

import threading
import time
from typing import Dict, List, Optional

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# Retry configuration for transient port errors
MAX_RETRIES = 5
RETRY_DELAY = 0.1  # 100ms - longer delay to let port settle

from .constants import (
    PROTOCOL_VERSION,
    ADDR_OPERATING_MODE,
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
    POSITION_CONTROL_MODE,
)
from .conversions import ticks_to_radians, radians_to_ticks


class DynamixelError(Exception):
    """Base exception for Dynamixel communication errors."""

    pass


# Module-level singleton storage for drivers by port
_driver_instances: Dict[str, "DynamixelDriver"] = {}
_driver_lock = threading.Lock()

import logging
_logger = logging.getLogger("dynamixel.driver")


def get_driver(port: str, baud_rate: int) -> "DynamixelDriver":
    """Get or create a singleton driver instance for a port.

    This ensures only one driver exists per serial port, preventing
    conflicts between arm and gripper components.
    """
    global _driver_instances
    with _driver_lock:
        if port not in _driver_instances:
            _logger.info(f"Creating new driver instance for {port}")
            driver = DynamixelDriver(port, baud_rate)
            driver.open()
            _driver_instances[port] = driver
            _logger.info(f"Driver created: id={id(driver)}, lock_id={id(driver._lock)}")
        else:
            driver = _driver_instances[port]
            _logger.info(f"Returning existing driver: id={id(driver)}, lock_id={id(driver._lock)}")
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
        self._lock = threading.Lock()  # Use Lock to prevent concurrent access
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

    def _clear_port(self) -> None:
        """Clear any stale port state by resetting the is_using flag and flushing."""
        if self._port_handler is not None:
            # Log current state
            is_using = getattr(self._port_handler, 'is_using_', 'N/A')
            _logger.debug(f"Clearing port, current is_using_={is_using}")

            # Try to clear the SDK's internal is_using flag if it exists
            try:
                self._port_handler.is_using_ = False
            except AttributeError:
                _logger.warning("is_using_ attribute not found on port handler")

            # Also try to clear the port buffer
            try:
                self._port_handler.clearPort()
            except AttributeError:
                _logger.debug("clearPort() not available")
            except Exception as e:
                _logger.warning(f"clearPort() failed: {e}")

            # Small delay to let the port settle
            time.sleep(0.005)

            # Verify it was cleared
            is_using_after = getattr(self._port_handler, 'is_using_', 'N/A')
            _logger.debug(f"After clearing, is_using_={is_using_after}")

    def enable_torque(self, motor_ids: Optional[List[int]] = None) -> None:
        """Enable torque on specified motors.

        Args:
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            motor_ids = ALL_ARM_MOTOR_IDS

        with self._lock:
            self._check_open()
            self._clear_port()
            for motor_id in motor_ids:
                # Retry logic for write operations too
                for attempt in range(MAX_RETRIES):
                    result, error = self._packet_handler.write1ByteTxRx(
                        self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
                    )
                    if result == COMM_SUCCESS:
                        break
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
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
            self._clear_port()
            for motor_id in motor_ids:
                for attempt in range(MAX_RETRIES):
                    result, error = self._packet_handler.write1ByteTxRx(
                        self._port_handler, motor_id, ADDR_TORQUE_ENABLE, 0
                    )
                    if result == COMM_SUCCESS:
                        break
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
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
        _logger.debug(f"read_joint_positions called, driver_id={id(self)}, acquiring lock...")
        with self._lock:
            _logger.debug(f"Lock acquired, port is_using_={getattr(self._port_handler, 'is_using_', 'N/A')}")
            self._check_open()
            self._clear_port()  # Clear any stale port state

            # Read positions individually for reliability
            # read4ByteTxRx returns (data, result, error)
            positions = []
            for joint_idx in range(NUM_JOINTS):
                config = VIPERX_300S_JOINTS[joint_idx]
                primary_id = config.motor_ids[0]

                # Retry logic for transient errors
                last_error = None
                for attempt in range(MAX_RETRIES):
                    _logger.debug(f"Reading motor {primary_id}, attempt {attempt + 1}, is_using_={getattr(self._port_handler, 'is_using_', 'N/A')}")
                    data, result, error = self._packet_handler.read4ByteTxRx(
                        self._port_handler, primary_id, ADDR_PRESENT_POSITION
                    )
                    if result == COMM_SUCCESS and error == 0:
                        break
                    last_error = self._packet_handler.getTxRxResult(result)
                    _logger.warning(f"Motor {primary_id} read failed: {last_error}, is_using_={getattr(self._port_handler, 'is_using_', 'N/A')}")
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
                    raise DynamixelError(
                        f"Failed to read position for motor {primary_id}: {last_error}"
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
            self._clear_port()

            # Write positions individually for reliability
            for joint_idx, position in enumerate(positions):
                config = VIPERX_300S_JOINTS[joint_idx]
                ticks = radians_to_ticks(position)

                # Write to all motors for this joint (handles dual-motor joints)
                for motor_id in config.motor_ids:
                    for attempt in range(MAX_RETRIES):
                        result, error = self._packet_handler.write4ByteTxRx(
                            self._port_handler, motor_id, ADDR_GOAL_POSITION, ticks
                        )
                        if result == COMM_SUCCESS:
                            break
                        if attempt < MAX_RETRIES - 1:
                            self._clear_port()
                            time.sleep(RETRY_DELAY)
                    else:
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
            self._clear_port()

            # Read moving status from each motor individually
            # read1ByteTxRx returns (data, result, error)
            for motor_id in PRIMARY_MOTOR_IDS:
                # Retry logic for transient errors
                for attempt in range(MAX_RETRIES):
                    data, result, error = self._packet_handler.read1ByteTxRx(
                        self._port_handler, motor_id, ADDR_MOVING
                    )
                    if result == COMM_SUCCESS and error == 0:
                        break
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
                    # Skip motors that fail to respond after retries
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
            self._clear_port()
            for motor_id in motor_ids:
                for attempt in range(MAX_RETRIES):
                    result, error = self._packet_handler.write4ByteTxRx(
                        self._port_handler, motor_id, ADDR_PROFILE_VELOCITY, velocity
                    )
                    if result == COMM_SUCCESS:
                        break
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
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
            self._clear_port()
            for motor_id in motor_ids:
                for attempt in range(MAX_RETRIES):
                    result, error = self._packet_handler.write4ByteTxRx(
                        self._port_handler, motor_id, ADDR_PROFILE_ACCELERATION, acceleration
                    )
                    if result == COMM_SUCCESS:
                        break
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
                    raise DynamixelError(
                        f"Failed to set acceleration on motor {motor_id}"
                    )

    def reboot_motor(self, motor_id: int) -> None:
        """Reboot a motor to clear hardware errors.

        Args:
            motor_id: Motor ID to reboot
        """
        with self._lock:
            self._check_open()
            self._clear_port()

            for attempt in range(MAX_RETRIES):
                result, error = self._packet_handler.reboot(
                    self._port_handler, motor_id
                )
                if result == COMM_SUCCESS:
                    _logger.info(f"Rebooted motor {motor_id} to clear hardware errors")
                    # Give the motor time to reboot
                    time.sleep(0.5)
                    return
                if attempt < MAX_RETRIES - 1:
                    self._clear_port()
                    time.sleep(RETRY_DELAY)

            _logger.warning(
                f"Failed to reboot motor {motor_id}: "
                f"{self._packet_handler.getTxRxResult(result)}"
            )

    def read_operating_mode(self, motor_id: int) -> int:
        """Read the operating mode of a motor.

        Args:
            motor_id: Motor ID to read from

        Returns:
            Operating mode value (3 = Position Control, 4 = Extended Position, 16 = PWM)
        """
        with self._lock:
            self._check_open()
            self._clear_port()

            last_error = None
            for attempt in range(MAX_RETRIES):
                data, result, error = self._packet_handler.read1ByteTxRx(
                    self._port_handler, motor_id, ADDR_OPERATING_MODE
                )
                if result == COMM_SUCCESS and error == 0:
                    return data
                # Hardware error - try to return last known mode (Position Control)
                if error != 0:
                    _logger.warning(
                        f"Hardware error reading operating mode for motor {motor_id}: "
                        f"{self._packet_handler.getRxPacketError(error)}. "
                        f"Assuming Position Control Mode."
                    )
                    return POSITION_CONTROL_MODE
                last_error = self._packet_handler.getTxRxResult(result)
                if attempt < MAX_RETRIES - 1:
                    self._clear_port()
                    time.sleep(RETRY_DELAY)

            raise DynamixelError(
                f"Failed to read operating mode for motor {motor_id}: {last_error}"
            )

    def set_operating_mode(self, motor_id: int, mode: int) -> None:
        """Set the operating mode of a motor.

        IMPORTANT: Torque must be disabled before changing operating mode.

        Args:
            motor_id: Motor ID to configure
            mode: Operating mode (3 = Position Control, 4 = Extended Position, 16 = PWM)
        """
        with self._lock:
            self._check_open()
            self._clear_port()

            for attempt in range(MAX_RETRIES):
                result, error = self._packet_handler.write1ByteTxRx(
                    self._port_handler, motor_id, ADDR_OPERATING_MODE, mode
                )
                if result == COMM_SUCCESS:
                    _logger.info(f"Set motor {motor_id} operating mode to {mode}")
                    return
                if attempt < MAX_RETRIES - 1:
                    self._clear_port()
                    time.sleep(RETRY_DELAY)

            raise DynamixelError(
                f"Failed to set operating mode for motor {motor_id}: "
                f"{self._packet_handler.getTxRxResult(result)}"
            )

    # Gripper-specific methods

    def enable_gripper_torque(self) -> None:
        """Enable torque on the gripper motor and set motion profile.

        This method clears any hardware errors, ensures the gripper is in
        Position Control Mode, and enables torque.
        """
        # Reboot the motor to clear any hardware errors (e.g., from overload)
        _logger.info(f"Rebooting gripper motor {GRIPPER_MOTOR_ID} to clear any errors")
        self.reboot_motor(GRIPPER_MOTOR_ID)

        # Disable torque to allow mode change if needed
        self.disable_torque([GRIPPER_MOTOR_ID])

        # Check current operating mode
        current_mode = self.read_operating_mode(GRIPPER_MOTOR_ID)
        _logger.info(f"Gripper motor {GRIPPER_MOTOR_ID} current operating mode: {current_mode}")

        # Set to Position Control Mode if not already
        if current_mode != POSITION_CONTROL_MODE:
            _logger.info(f"Setting gripper to Position Control Mode ({POSITION_CONTROL_MODE})")
            self.set_operating_mode(GRIPPER_MOTOR_ID, POSITION_CONTROL_MODE)

        # Set profile velocity and acceleration for smooth motion
        self.set_profile_velocity(100, [GRIPPER_MOTOR_ID])
        self.set_profile_acceleration(50, [GRIPPER_MOTOR_ID])

        # Enable torque
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
            self._clear_port()

            # Retry logic for transient errors
            last_error = None
            for attempt in range(MAX_RETRIES):
                data, result, error = self._packet_handler.read4ByteTxRx(
                    self._port_handler, GRIPPER_MOTOR_ID, ADDR_PRESENT_POSITION
                )
                if result == COMM_SUCCESS and error == 0:
                    break
                last_error = self._packet_handler.getRxPacketError(error)
                if attempt < MAX_RETRIES - 1:
                    self._clear_port()
                    time.sleep(RETRY_DELAY)
            else:
                raise DynamixelError(f"Failed to read gripper position: {last_error}")

            return ticks_to_radians(data)

    def write_gripper_position(self, position: float) -> None:
        """Write goal position to gripper.

        If the gripper has a hardware error (e.g., from overload), this method
        will automatically reboot the motor to clear the error and retry.

        Args:
            position: Target position in radians
        """
        ticks = radians_to_ticks(position)

        # Try to write position, with hardware error recovery
        for recovery_attempt in range(2):  # Allow one recovery attempt
            with self._lock:
                self._check_open()
                self._clear_port()

                for attempt in range(MAX_RETRIES):
                    result, error = self._packet_handler.write4ByteTxRx(
                        self._port_handler, GRIPPER_MOTOR_ID, ADDR_GOAL_POSITION, ticks
                    )
                    if result == COMM_SUCCESS and error == 0:
                        return  # Success
                    # Hardware error detected - need to reboot
                    if error != 0:
                        _logger.warning(
                            f"Gripper hardware error during position write: "
                            f"{self._packet_handler.getRxPacketError(error)}. "
                            f"Will reboot motor."
                        )
                        break  # Exit retry loop to do recovery
                    if attempt < MAX_RETRIES - 1:
                        self._clear_port()
                        time.sleep(RETRY_DELAY)
                else:
                    # All retries exhausted without hardware error
                    raise DynamixelError(
                        f"Failed to write gripper position: "
                        f"{self._packet_handler.getTxRxResult(result)}"
                    )

            # If we get here, we had a hardware error - try to recover
            if recovery_attempt == 0:
                _logger.info("Attempting gripper recovery: rebooting motor")
                self.reboot_motor(GRIPPER_MOTOR_ID)
                # Re-enable torque after reboot
                self.enable_torque([GRIPPER_MOTOR_ID])

        raise DynamixelError("Failed to write gripper position after recovery attempt")

    def is_gripper_moving(self) -> bool:
        """Check if gripper is currently moving.

        Returns:
            True if gripper is moving, False otherwise.
            Returns False if hardware error (e.g., overload from gripping object).
        """
        with self._lock:
            self._check_open()
            self._clear_port()

            # Retry logic for transient errors
            last_error = None
            for attempt in range(MAX_RETRIES):
                data, result, error = self._packet_handler.read1ByteTxRx(
                    self._port_handler, GRIPPER_MOTOR_ID, ADDR_MOVING
                )
                if result == COMM_SUCCESS and error == 0:
                    return bool(data)
                # Hardware error (e.g., overload from gripping) - gripper has stopped
                if error != 0:
                    _logger.warning(
                        f"Gripper hardware error during is_moving check: "
                        f"{self._packet_handler.getRxPacketError(error)}. "
                        f"Assuming gripper stopped."
                    )
                    return False
                last_error = self._packet_handler.getTxRxResult(result)
                if attempt < MAX_RETRIES - 1:
                    self._clear_port()
                    time.sleep(RETRY_DELAY)

            raise DynamixelError(
                f"Failed to read gripper moving status: {last_error}"
            )
