// Package dynamixel provides low-level Dynamixel motor communication for the ViperX-300s arm.
package dynamixel

import "math"

// Protocol and communication constants.
const (
	DefaultBaudRate = 1000000

	// Control table addresses (XM series, Protocol 2.0)
	AddrOperatingMode       uint16 = 11
	AddrTorqueEnable        uint16 = 64
	AddrProfileAcceleration uint16 = 108
	AddrProfileVelocity     uint16 = 112
	AddrGoalPosition        uint16 = 116
	AddrMoving              uint16 = 122
	AddrPresentPosition     uint16 = 132

	// Position resolution
	TicksPerRevolution = 4096
	CenterPosition     = 2048

	// Operating modes
	PositionControlMode = 3
	ExtendedPositionMode = 4
	PWMControlMode = 16

	// Default motion profile
	DefaultProfileVelocity     = 100
	DefaultProfileAcceleration = 50
)

// NumJoints is the number of arm joints (excluding gripper).
const NumJoints = 6

// JointConfig defines a single joint's motor mapping and limits.
type JointConfig struct {
	Name       string
	MotorIDs   []int    // Motor IDs for this joint (some joints have 2 motors)
	MinRadians float64
	MaxRadians float64
}

// DegreesToRadians converts degrees to radians.
func DegreesToRadians(deg float64) float64 {
	return deg * math.Pi / 180.0
}

// ViperX300SJoints defines the joint configuration for the ViperX-300s arm.
// Limits are slightly wider than URDF to accommodate real-world positions.
// Forearm_roll and wrist_rotate use extended position mode (±360°).
var ViperX300SJoints = []JointConfig{
	{Name: "waist", MotorIDs: []int{1}, MinRadians: DegreesToRadians(-180), MaxRadians: DegreesToRadians(180)},
	{Name: "shoulder", MotorIDs: []int{2, 3}, MinRadians: DegreesToRadians(-110), MaxRadians: DegreesToRadians(75)},
	{Name: "elbow", MotorIDs: []int{4, 5}, MinRadians: DegreesToRadians(-106), MaxRadians: DegreesToRadians(97)},
	{Name: "forearm_roll", MotorIDs: []int{6}, MinRadians: DegreesToRadians(-360), MaxRadians: DegreesToRadians(360)},
	{Name: "wrist_angle", MotorIDs: []int{7}, MinRadians: DegreesToRadians(-110), MaxRadians: DegreesToRadians(130)},
	{Name: "wrist_rotate", MotorIDs: []int{8}, MinRadians: DegreesToRadians(-360), MaxRadians: DegreesToRadians(360)},
}

// AllArmMotorIDs contains all motor IDs for the arm (excluding gripper).
var AllArmMotorIDs = []int{1, 2, 3, 4, 5, 6, 7, 8}

// PrimaryMotorIDs contains one motor ID per joint (for reading positions).
var PrimaryMotorIDs = []int{1, 2, 4, 6, 7, 8}

// GripperMotorID is the motor ID for the gripper.
const GripperMotorID = 9

// Gripper position constants (in radians).
var (
	GripperOpenPosition  = DegreesToRadians(85.0)  // Fully open
	GripperClosePosition = DegreesToRadians(-45.0) // Fully closed
)
