# Model clint:trossen:viperx-300s

Viam Arm component for the Trossen ViperX-300s 6-DOF robotic arm. This component provides joint-level control via direct Dynamixel SDK communication.

For Cartesian motion (move_to_position, get_end_position), use Viam's motion service which computes FK/IK using the provided kinematics file.

## Configuration

```json
{
  "usb_port": "/dev/ttyUSB0",
  "baud_rate": 1000000,
  "profile_velocity": 100,
  "profile_acceleration": 50
}
```

### Attributes

| Name | Type | Inclusion | Description |
|------|------|-----------|-------------|
| `usb_port` | string | Required | Serial port path for U2D2 adapter (e.g., "/dev/ttyUSB0") |
| `baud_rate` | int | Optional | Communication baud rate (default: 1000000) |
| `profile_velocity` | int | Optional | Motion velocity 0-1023, 0=no limit (default: 100) |
| `profile_acceleration` | int | Optional | Motion acceleration 0-32767 (default: 50) |

### Example Configuration

```json
{
  "name": "arm",
  "model": "clint:trossen:viperx-300s",
  "type": "arm",
  "namespace": "rdk",
  "attributes": {
    "usb_port": "/dev/ttyUSB0",
    "baud_rate": 1000000,
    "profile_velocity": 100,
    "profile_acceleration": 50
  }
}
```

## Supported Methods

| Method | Supported | Notes |
|--------|-----------|-------|
| `get_joint_positions` | Yes | Returns 6 joint angles in degrees |
| `move_to_joint_positions` | Yes | Accepts 6 joint angles in degrees |
| `get_kinematics` | Yes | Returns SVA format kinematics |
| `stop` | Yes | Halts motion at current position |
| `is_moving` | Yes | Checks motor moving status |
| `get_end_position` | No | Use motion service for FK |
| `move_to_position` | No | Use motion service for IK |
| `get_geometries` | Partial | Returns empty list |

## DoCommand

The following custom commands are supported:

### Enable/Disable Torque

```json
{"enable_torque": false}
```

Disables motor torque, allowing manual positioning of the arm.

### Set Velocity

```json
{"set_velocity": 200}
```

Sets the profile velocity for subsequent moves (0-1023).

### Set Acceleration

```json
{"set_acceleration": 100}
```

Sets the profile acceleration for subsequent moves (0-32767).

## Joint Configuration

| Index | Joint | Motor ID(s) | Limits |
|-------|-------|-------------|--------|
| 0 | Waist | 1 | ±180° |
| 1 | Shoulder | 2, 3 | -101° to 101° |
| 2 | Elbow | 4, 5 | -101° to 92° |
| 3 | Forearm Roll | 6 | ±180° |
| 4 | Wrist Angle | 7 | -107° to 130° |
| 5 | Wrist Rotate | 8 | ±180° |

Note: Shoulder and elbow use dual motors that are synchronized automatically.
