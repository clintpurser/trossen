package dynamixel

import (
	"encoding/binary"
	"math"
)

// TicksToRadians converts Dynamixel encoder ticks to radians.
// The motor has 4096 ticks per revolution with center at 2048.
func TicksToRadians(ticks int) float64 {
	// Offset from center position, then convert to radians
	offset := float64(ticks - CenterPosition)
	return offset * (2.0 * math.Pi / float64(TicksPerRevolution))
}

// RadiansToTicks converts radians to Dynamixel encoder ticks.
func RadiansToTicks(radians float64) int {
	offset := radians * (float64(TicksPerRevolution) / (2.0 * math.Pi))
	return int(math.Round(offset)) + CenterPosition
}

// RadiansToDegrees converts radians to degrees.
func RadiansToDegrees(radians float64) float64 {
	return radians * 180.0 / math.Pi
}

// ValidateJointLimits checks if a position is within joint limits.
func ValidateJointLimits(jointIdx int, radians float64) bool {
	if jointIdx < 0 || jointIdx >= len(ViperX300SJoints) {
		return false
	}
	config := ViperX300SJoints[jointIdx]
	return radians >= config.MinRadians && radians <= config.MaxRadians
}

// BytesToInt32 converts 4 bytes (little-endian) to an int32.
func BytesToInt32(data []byte) int32 {
	if len(data) < 4 {
		return 0
	}
	return int32(binary.LittleEndian.Uint32(data))
}

// Int32ToBytes converts an int32 to 4 bytes (little-endian).
func Int32ToBytes(val int32) []byte {
	buf := make([]byte, 4)
	binary.LittleEndian.PutUint32(buf, uint32(val))
	return buf
}
