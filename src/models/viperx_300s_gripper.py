"""Viam Gripper component implementation for Trossen ViperX-300s."""

import sys
import os
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.gripper import Gripper
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes

# Add src to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dynamixel import DynamixelDriver, DynamixelError, DEFAULT_BAUD_RATE
from dynamixel.conversions import degrees_to_radians, radians_to_degrees
from .viperx_300s import Viperx300s


# Gripper position constants (in degrees)
GRIPPER_OPEN_POSITION = 45.0    # Fully open
GRIPPER_CLOSE_POSITION = -20.0  # Fully closed


class Viperx300sGripper(Gripper, EasyResource):
    """Viam Gripper component for Trossen ViperX-300s robotic arm gripper.

    This component controls the gripper (motor ID 9) on the ViperX-300s.
    It shares the Dynamixel driver with the arm component.

    Configuration attributes:
        usb_port (str, required): Serial port path (e.g., "/dev/ttyUSB0")
            Must match the arm's usb_port to share the connection.
        open_position (float, optional): Open position in degrees, default 45
        close_position (float, optional): Closed position in degrees, default -20
    """

    MODEL: ClassVar[Model] = Model(
        ModelFamily("clint", "trossen"), "viperx-300s-gripper"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._driver: Optional[DynamixelDriver] = None
        self._owns_driver: bool = False  # Whether we created the driver
        self._usb_port: str = ""
        self._open_position: float = GRIPPER_OPEN_POSITION
        self._close_position: float = GRIPPER_CLOSE_POSITION

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """Create a new ViperX-300s gripper instance."""
        gripper = cls(config.name)
        gripper.reconfigure(config, dependencies)
        return gripper

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

        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        """Reconfigure the gripper with new settings."""
        attrs = config.attributes.fields

        # Close existing driver if we own it
        if self._driver is not None and self._owns_driver:
            try:
                self._driver.disable_gripper_torque()
                self._driver.close()
            except Exception as e:
                self.logger.warning(f"Error closing driver during reconfigure: {e}")

        # Extract configuration
        self._usb_port = attrs["usb_port"].string_value
        self._open_position = (
            float(attrs["open_position"].number_value)
            if "open_position" in attrs
            else GRIPPER_OPEN_POSITION
        )
        self._close_position = (
            float(attrs["close_position"].number_value)
            if "close_position" in attrs
            else GRIPPER_CLOSE_POSITION
        )

        # Try to get shared driver from arm component
        self._driver = Viperx300s.get_shared_driver(self._usb_port)

        if self._driver is not None:
            self._owns_driver = False
            self.logger.info(
                f"ViperX-300s gripper sharing driver on {self._usb_port}"
            )
        else:
            # Create our own driver if arm hasn't initialized yet
            baud_rate = (
                int(attrs["baud_rate"].number_value)
                if "baud_rate" in attrs
                else DEFAULT_BAUD_RATE
            )
            self._driver = DynamixelDriver(self._usb_port, baud_rate)
            self._owns_driver = True
            try:
                self._driver.open()
                self.logger.info(
                    f"ViperX-300s gripper created own driver on {self._usb_port}"
                )
            except DynamixelError as e:
                self.logger.error(f"Failed to initialize gripper: {e}")
                raise

        # Enable gripper torque
        try:
            self._driver.enable_gripper_torque()
        except DynamixelError as e:
            self.logger.error(f"Failed to enable gripper torque: {e}")
            raise

    async def open(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        """Open the gripper."""
        if self._driver is None:
            raise RuntimeError("Gripper not initialized")

        try:
            position_rad = degrees_to_radians(self._open_position)
            self._driver.write_gripper_position(position_rad)
        except DynamixelError as e:
            self.logger.error(f"Failed to open gripper: {e}")
            raise

    async def grab(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> bool:
        """Close the gripper to grab an object.

        Returns:
            True if an object was grabbed (gripper didn't fully close),
            False if gripper closed completely (no object).
        """
        if self._driver is None:
            raise RuntimeError("Gripper not initialized")

        try:
            position_rad = degrees_to_radians(self._close_position)
            self._driver.write_gripper_position(position_rad)

            # Wait for motion to complete and check final position
            # For now, just return True - proper grab detection would
            # require monitoring current/load or final position
            return True

        except DynamixelError as e:
            self.logger.error(f"Failed to grab: {e}")
            raise

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> None:
        """Stop gripper motion."""
        if self._driver is None:
            return

        try:
            # Read current position and write it back to stop
            current = self._driver.read_gripper_position()
            self._driver.write_gripper_position(current)
        except DynamixelError as e:
            self.logger.error(f"Failed to stop gripper: {e}")
            raise

    async def is_moving(self) -> bool:
        """Check if gripper is currently moving."""
        if self._driver is None:
            return False

        try:
            return self._driver.is_gripper_moving()
        except DynamixelError as e:
            self.logger.error(f"Failed to check gripper moving status: {e}")
            raise

    async def get_kinematics(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        """Get kinematics information (not applicable for gripper)."""
        return (b"", "")

    async def is_holding_something(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> bool:
        """Check if gripper is holding something.

        Returns True if gripper didn't fully close (object detected).
        This is a simple implementation - proper detection would require
        monitoring motor current/load.
        """
        # For now, always return False - proper implementation would
        # check if gripper position is greater than close_position
        return False

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        """Execute custom commands.

        Supported commands:
            {"get_position": true} - Get current gripper position in degrees
            {"set_position": <float>} - Set gripper position in degrees
            {"enable_torque": true/false} - Enable/disable gripper torque
        """
        if self._driver is None:
            raise RuntimeError("Gripper not initialized")

        result: Dict[str, Any] = {}

        if "get_position" in command:
            position_rad = self._driver.read_gripper_position()
            result["position"] = radians_to_degrees(position_rad)

        if "set_position" in command:
            position_deg = float(command["set_position"])
            position_rad = degrees_to_radians(position_deg)
            self._driver.write_gripper_position(position_rad)
            result["set_position"] = position_deg

        if "enable_torque" in command:
            if command["enable_torque"]:
                self._driver.enable_gripper_torque()
                result["torque"] = "enabled"
            else:
                self._driver.disable_gripper_torque()
                result["torque"] = "disabled"

        return result

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> Sequence[Geometry]:
        """Get gripper geometries."""
        return []

    async def close(self) -> None:
        """Clean up resources."""
        if self._driver is not None:
            try:
                self._driver.disable_gripper_torque()
                self.logger.info("ViperX-300s gripper closed")

                # Only close driver if we own it
                if self._owns_driver:
                    self._driver.close()

            except Exception as e:
                self.logger.warning(f"Error during close: {e}")

            self._driver = None
