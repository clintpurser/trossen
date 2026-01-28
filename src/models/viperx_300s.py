"""Viam Arm component implementation for Trossen ViperX-300s."""

import sys
import os
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, KinematicsFileFormat, Pose
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes

# Add src to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dynamixel import (
    DynamixelError,
    DEFAULT_BAUD_RATE,
    VIPERX_300S_JOINTS,
    NUM_JOINTS,
    DEFAULT_PROFILE_VELOCITY,
    DEFAULT_PROFILE_ACCELERATION,
)
from dynamixel.driver import get_driver, close_driver
from dynamixel.conversions import (
    radians_to_degrees,
    degrees_to_radians,
    validate_joint_limits,
)
from kinematics import KINEMATICS_FILE


class Viperx300s(Arm, EasyResource):
    """Viam Arm component for Trossen ViperX-300s robotic arm.

    This component provides joint-level control of the ViperX-300s 6-DOF arm
    via direct Dynamixel SDK communication.

    For Cartesian motion (move_to_position, get_end_position), use Viam's
    motion service which will compute FK/IK using the kinematics file.

    Configuration attributes:
        usb_port (str, required): Serial port path (e.g., "/dev/ttyUSB0")
        baud_rate (int, optional): Communication baud rate, default 1000000
        profile_velocity (int, optional): Motion velocity, default 100
        profile_acceleration (int, optional): Motion acceleration, default 50
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("clint", "trossen"), "viperx-300s")

    def __init__(self, name: str):
        super().__init__(name)
        self._driver = None
        self._kinematics_data: Optional[bytes] = None
        self._usb_port: str = ""

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """Create a new ViperX-300s arm instance."""
        arm = cls(config.name)
        arm.reconfigure(config, dependencies)
        return arm

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        """Validate configuration and return dependencies.

        Args:
            config: The configuration for this resource

        Returns:
            Tuple of (required_dependencies, optional_dependencies)

        Raises:
            ValueError: If configuration is invalid
        """
        attrs = config.attributes.fields

        # usb_port is required
        if "usb_port" not in attrs:
            raise ValueError("usb_port attribute is required")

        usb_port = attrs["usb_port"].string_value
        if not usb_port:
            raise ValueError("usb_port must be a non-empty string")

        # baud_rate is optional, validate if provided
        if "baud_rate" in attrs:
            baud_rate = attrs["baud_rate"].number_value
            if baud_rate <= 0:
                raise ValueError("baud_rate must be a positive integer")

        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        """Reconfigure the arm with new settings."""
        attrs = config.attributes.fields

        # Extract configuration
        self._usb_port = attrs["usb_port"].string_value
        baud_rate = (
            int(attrs["baud_rate"].number_value)
            if "baud_rate" in attrs
            else DEFAULT_BAUD_RATE
        )
        profile_velocity = (
            int(attrs["profile_velocity"].number_value)
            if "profile_velocity" in attrs
            else DEFAULT_PROFILE_VELOCITY
        )
        profile_acceleration = (
            int(attrs["profile_acceleration"].number_value)
            if "profile_acceleration" in attrs
            else DEFAULT_PROFILE_ACCELERATION
        )

        # Get singleton driver instance (creates if needed)
        try:
            self._driver = get_driver(self._usb_port, baud_rate)
            self._driver.enable_torque()
            self._driver.set_profile_velocity(profile_velocity)
            self._driver.set_profile_acceleration(profile_acceleration)
            self.logger.info(
                f"ViperX-300s connected on {self._usb_port} at {baud_rate} baud"
            )
        except DynamixelError as e:
            self.logger.error(f"Failed to initialize arm: {e}")
            raise

        # Load kinematics data
        self._load_kinematics()

    def _load_kinematics(self) -> None:
        """Load kinematics JSON file into memory."""
        try:
            with open(KINEMATICS_FILE, "rb") as f:
                self._kinematics_data = f.read()
            self.logger.debug(f"Loaded kinematics from {KINEMATICS_FILE}")
        except FileNotFoundError:
            self.logger.warning(f"Kinematics file not found: {KINEMATICS_FILE}")
            self._kinematics_data = b"{}"

    async def get_end_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Pose:
        """Get end effector position.

        NOTE: This requires forward kinematics calculation. Use Viam's
        motion service which computes FK from joint positions + kinematics.
        """
        raise NotImplementedError(
            "get_end_position requires FK solver. Use Viam motion service."
        )

    async def move_to_position(
        self,
        pose: Pose,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        """Move to Cartesian position.

        NOTE: This requires inverse kinematics. Use Viam's motion service
        for Cartesian motion planning.
        """
        raise NotImplementedError(
            "move_to_position requires IK solver. Use Viam motion service."
        )

    async def get_joint_positions(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> JointPositions:
        """Get current joint positions.

        Returns:
            JointPositions with 6 values in degrees
        """
        if self._driver is None:
            raise RuntimeError("Arm not initialized")

        try:
            radians = self._driver.read_joint_positions()
            degrees = [radians_to_degrees(r) for r in radians]
            return JointPositions(values=degrees)
        except DynamixelError as e:
            self.logger.error(f"Failed to read joint positions: {e}")
            raise

    async def move_to_joint_positions(
        self,
        positions: JointPositions,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        """Move to specified joint positions.

        Args:
            positions: JointPositions with 6 values in degrees
        """
        if self._driver is None:
            raise RuntimeError("Arm not initialized")

        if len(positions.values) != NUM_JOINTS:
            raise ValueError(
                f"Expected {NUM_JOINTS} joint positions, got {len(positions.values)}"
            )

        # Convert degrees to radians
        radians = [degrees_to_radians(d) for d in positions.values]

        # Validate joint limits
        for idx, rad in enumerate(radians):
            if not validate_joint_limits(idx, rad, VIPERX_300S_JOINTS):
                config = VIPERX_300S_JOINTS[idx]
                raise ValueError(
                    f"Joint {config.name} position {positions.values[idx]:.1f}° "
                    f"out of range [{config.min_degrees}, {config.max_degrees}]"
                )

        try:
            self._driver.write_joint_positions(radians)
        except DynamixelError as e:
            self.logger.error(f"Failed to move joints: {e}")
            raise

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        """Stop all arm motion immediately.

        Reads current positions and writes them as goal to halt motion.
        """
        if self._driver is None:
            return

        try:
            current = self._driver.read_joint_positions()
            self._driver.write_joint_positions(current)
        except DynamixelError as e:
            self.logger.error(f"Failed to stop arm: {e}")
            raise

    async def is_moving(self) -> bool:
        """Check if arm is currently moving."""
        if self._driver is None:
            return False

        try:
            return self._driver.is_moving()
        except DynamixelError as e:
            self.logger.error(f"Failed to check moving status: {e}")
            raise

    async def get_kinematics(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        """Get kinematics file format and data.

        Returns:
            Tuple of (KinematicsFileFormat, kinematics_bytes)
        """
        if self._kinematics_data is None:
            self._load_kinematics()

        return (
            KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA,
            self._kinematics_data or b"{}",
        )

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        """Execute custom commands.

        Supported commands:
            {"enable_torque": true/false} - Enable/disable motor torque
            {"set_velocity": <int>} - Set profile velocity (0-1023)
            {"set_acceleration": <int>} - Set profile acceleration (0-32767)
        """
        if self._driver is None:
            raise RuntimeError("Arm not initialized")

        result: Dict[str, Any] = {}

        if "enable_torque" in command:
            if command["enable_torque"]:
                self._driver.enable_torque()
                result["torque"] = "enabled"
            else:
                self._driver.disable_torque()
                result["torque"] = "disabled"

        if "set_velocity" in command:
            velocity = int(command["set_velocity"])
            self._driver.set_profile_velocity(velocity)
            result["velocity"] = velocity

        if "set_acceleration" in command:
            accel = int(command["set_acceleration"])
            self._driver.set_profile_acceleration(accel)
            result["acceleration"] = accel

        return result

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> Sequence[Geometry]:
        """Get arm geometries.

        Returns empty list; geometries are defined in the kinematics file.
        """
        return []

    async def close(self) -> None:
        """Clean up resources."""
        if self._driver is not None:
            try:
                self._driver.disable_torque()
                self.logger.info("ViperX-300s arm closed")
            except Exception as e:
                self.logger.warning(f"Error during close: {e}")
            # Don't close the singleton driver - gripper may still need it
            self._driver = None
