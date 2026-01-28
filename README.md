# Trossen ViperX-300s Viam Module

This module provides Viam components for controlling the [Trossen ViperX-300s](https://www.trossenrobotics.com/viperx-300) 6-DOF robotic arm.

## Features

- Direct Dynamixel SDK control (no ROS dependency)
- Joint-level position control with limit validation
- Synchronized dual-motor joint handling (shoulder, elbow)
- SVA kinematics file for Viam motion service integration
- Separate gripper component

## Models

This module provides the following models:

- [`clint:trossen:viperx-300s`](clint_trossen_viperx-300s.md) - 6-DOF arm component
- `clint:trossen:viperx-300s-gripper` - Gripper component

## Hardware Setup

1. Connect the ViperX-300s arm to power (12V)
2. Connect the U2D2 USB adapter to your computer/robot
3. Note the serial port (typically `/dev/ttyUSB0` on Linux)

## Quick Start

### Arm Configuration

```json
{
  "name": "arm",
  "model": "clint:trossen:viperx-300s",
  "type": "arm",
  "namespace": "rdk",
  "attributes": {
    "usb_port": "/dev/ttyUSB0",
    "baud_rate": 1000000
  }
}
```

### Gripper Configuration

```json
{
  "name": "gripper",
  "model": "clint:trossen:viperx-300s-gripper",
  "type": "gripper",
  "namespace": "rdk",
  "attributes": {
    "usb_port": "/dev/ttyUSB0"
  },
  "depends_on": ["arm"]
}
```

## Usage

### Joint Control

```python
from viam.components.arm import Arm, JointPositions

arm = Arm.from_robot(robot, "arm")

# Read current positions (6 values in degrees)
positions = await arm.get_joint_positions()
print(positions.values)

# Move to home position
await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 0, 0, 0]))

# Check if moving
moving = await arm.is_moving()
```

### Gripper Control

```python
from viam.components.gripper import Gripper

gripper = Gripper.from_robot(robot, "gripper")

# Open gripper
await gripper.open()

# Close/grab
grabbed = await gripper.grab()
```

### Cartesian Motion (via Motion Service)

For Cartesian motion planning, use Viam's motion service which will compute FK/IK from the kinematics file:

```python
from viam.services.motion import Motion
from viam.proto.common import Pose, PoseInFrame

motion = Motion.from_robot(robot, "builtin")
arm_resource = Arm.get_resource_name("arm")

# Move to a Cartesian pose
target = Pose(x=300, y=0, z=400, o_x=0, o_y=0, o_z=1, theta=0)
await motion.move(
    component_name=arm_resource,
    destination=PoseInFrame(reference_frame="world", pose=target)
)
```

## Joint Limits

| Joint | Range |
|-------|-------|
| Waist | ±180° |
| Shoulder | -101° to 101° |
| Elbow | -101° to 92° |
| Forearm Roll | ±180° |
| Wrist Angle | -107° to 130° |
| Wrist Rotate | ±180° |
