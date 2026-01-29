package dynamixel

import (
	"errors"
	"fmt"
	"strings"
	"sync"
	"time"

	protocol "github.com/haguro/go-dxl/protocol/v2"
	"go.bug.st/serial"
)

// ErrNotOpen is returned when operations are attempted on a closed driver.
var ErrNotOpen = errors.New("driver not open")

// Driver provides thread-safe communication with Dynamixel motors.
type Driver struct {
	port    serial.Port
	handler *protocol.Handler
	mu      sync.Mutex
	isOpen  bool
}

// NewDriver creates and opens a new Dynamixel driver.
func NewDriver(portName string, baudRate int) (*Driver, error) {
	mode := &serial.Mode{
		BaudRate: baudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open serial port %s: %w", portName, err)
	}

	// Set read timeout
	if err := port.SetReadTimeout(100 * time.Millisecond); err != nil {
		port.Close()
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	handler := protocol.NewHandler(port, 100*time.Millisecond)

	return &Driver{
		port:    port,
		handler: handler,
		isOpen:  true,
	}, nil
}

// Close closes the driver and releases resources.
func (d *Driver) Close() error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if !d.isOpen {
		return nil
	}

	d.isOpen = false
	return d.port.Close()
}

// checkOpen verifies the driver is open.
func (d *Driver) checkOpen() error {
	if !d.isOpen {
		return ErrNotOpen
	}
	return nil
}

// EnableTorque enables torque on the specified motors.
func (d *Driver) EnableTorque(motorIDs []int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	for _, id := range motorIDs {
		if err := d.handler.Write(byte(id), AddrTorqueEnable, 1); err != nil {
			// Ignore hardware errors (motor may have stale error flag)
			if isHardwareError(err) {
				continue
			}
			return fmt.Errorf("failed to enable torque on motor %d: %w", id, err)
		}
	}
	return nil
}

// DisableTorque disables torque on the specified motors.
func (d *Driver) DisableTorque(motorIDs []int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	for _, id := range motorIDs {
		if err := d.handler.Write(byte(id), AddrTorqueEnable, 0); err != nil {
			// Ignore hardware errors
			if isHardwareError(err) {
				continue
			}
			return fmt.Errorf("failed to disable torque on motor %d: %w", id, err)
		}
	}
	return nil
}

// ReadJointPositions reads current positions of all arm joints.
// Returns positions in radians.
func (d *Driver) ReadJointPositions() ([]float64, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return nil, err
	}

	positions := make([]float64, NumJoints)

	// Read from primary motor of each joint
	for i, config := range ViperX300SJoints {
		primaryID := config.MotorIDs[0]
		data, err := d.handler.Read(byte(primaryID), AddrPresentPosition, 4)
		if err != nil {
			return nil, fmt.Errorf("failed to read position from motor %d: %w", primaryID, err)
		}

		ticks := int(BytesToInt32(data))
		positions[i] = TicksToRadians(ticks)
	}

	return positions, nil
}

// WriteJointPositions writes goal positions to all arm joints.
// Positions are in radians.
// Note: Hardware errors (like data limit errors) are logged but not returned,
// matching the Python SDK behavior.
func (d *Driver) WriteJointPositions(positions []float64) error {
	if len(positions) != NumJoints {
		return fmt.Errorf("expected %d positions, got %d", NumJoints, len(positions))
	}

	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	// Write to each joint's motors
	for i, pos := range positions {
		config := ViperX300SJoints[i]
		ticks := RadiansToTicks(pos)
		data := Int32ToBytes(int32(ticks))

		// Write to all motors for this joint (handles dual-motor joints)
		for _, motorID := range config.MotorIDs {
			if err := d.handler.Write(byte(motorID), AddrGoalPosition, data...); err != nil {
				// Log hardware errors but continue (matches Python SDK behavior)
				// Hardware errors like "data limit error" happen when position
				// exceeds motor's EEPROM limits, but the motor will still move
				// to the closest valid position
				if isHardwareError(err) {
					// Silently ignore hardware errors like Python does
					continue
				}
				return fmt.Errorf("failed to write position to motor %d: %w", motorID, err)
			}
		}
	}

	return nil
}

// isHardwareError checks if the error is a Dynamixel hardware error
// (as opposed to a communication error)
func isHardwareError(err error) bool {
	if err == nil {
		return false
	}
	errStr := err.Error()
	// Hardware errors from Dynamixel contain these phrases
	return strings.Contains(errStr, "data limit error") ||
		strings.Contains(errStr, "processing error") ||
		strings.Contains(errStr, "hardware error") ||
		strings.Contains(errStr, "overload error") ||
		strings.Contains(errStr, "overheating error")
}

// IsMoving checks if any arm motor is currently moving.
func (d *Driver) IsMoving() (bool, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return false, err
	}

	for _, id := range PrimaryMotorIDs {
		data, err := d.handler.Read(byte(id), AddrMoving, 1)
		if err != nil {
			return false, fmt.Errorf("failed to read moving status from motor %d: %w", id, err)
		}
		if len(data) > 0 && data[0] != 0 {
			return true, nil
		}
	}

	return false, nil
}

// SetProfileVelocity sets the profile velocity for smooth motion.
func (d *Driver) SetProfileVelocity(velocity int, motorIDs []int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	data := Int32ToBytes(int32(velocity))
	for _, id := range motorIDs {
		if err := d.handler.Write(byte(id), AddrProfileVelocity, data...); err != nil {
			return fmt.Errorf("failed to set velocity on motor %d: %w", id, err)
		}
	}

	return nil
}

// SetProfileAcceleration sets the profile acceleration for smooth motion.
func (d *Driver) SetProfileAcceleration(acceleration int, motorIDs []int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	data := Int32ToBytes(int32(acceleration))
	for _, id := range motorIDs {
		if err := d.handler.Write(byte(id), AddrProfileAcceleration, data...); err != nil {
			return fmt.Errorf("failed to set acceleration on motor %d: %w", id, err)
		}
	}

	return nil
}

// Reboot reboots a motor to clear hardware errors.
func (d *Driver) Reboot(motorID int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	if err := d.handler.Reboot(byte(motorID)); err != nil {
		return fmt.Errorf("failed to reboot motor %d: %w", motorID, err)
	}

	// Give the motor time to reboot
	time.Sleep(500 * time.Millisecond)
	return nil
}

// ReadOperatingMode reads the operating mode of a motor.
func (d *Driver) ReadOperatingMode(motorID int) (int, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return 0, err
	}

	data, err := d.handler.Read(byte(motorID), AddrOperatingMode, 1)
	if err != nil {
		return 0, fmt.Errorf("failed to read operating mode from motor %d: %w", motorID, err)
	}

	if len(data) == 0 {
		return 0, fmt.Errorf("no data returned for motor %d", motorID)
	}

	return int(data[0]), nil
}

// SetOperatingMode sets the operating mode of a motor.
// Note: Torque must be disabled before changing operating mode.
func (d *Driver) SetOperatingMode(motorID int, mode int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	if err := d.handler.Write(byte(motorID), AddrOperatingMode, byte(mode)); err != nil {
		return fmt.Errorf("failed to set operating mode on motor %d: %w", motorID, err)
	}

	return nil
}

// Gripper methods

// EnableGripperTorque enables torque on the gripper motor.
func (d *Driver) EnableGripperTorque() error {
	return d.EnableTorque([]int{GripperMotorID})
}

// DisableGripperTorque disables torque on the gripper motor.
func (d *Driver) DisableGripperTorque() error {
	return d.DisableTorque([]int{GripperMotorID})
}

// ReadGripperPosition reads the current gripper position in radians.
func (d *Driver) ReadGripperPosition() (float64, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return 0, err
	}

	data, err := d.handler.Read(byte(GripperMotorID), AddrPresentPosition, 4)
	if err != nil {
		return 0, fmt.Errorf("failed to read gripper position: %w", err)
	}

	ticks := int(BytesToInt32(data))
	return TicksToRadians(ticks), nil
}

// WriteGripperPosition writes the goal position to the gripper in radians.
func (d *Driver) WriteGripperPosition(position float64) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return err
	}

	ticks := RadiansToTicks(position)
	data := Int32ToBytes(int32(ticks))

	if err := d.handler.Write(byte(GripperMotorID), AddrGoalPosition, data...); err != nil {
		return fmt.Errorf("failed to write gripper position: %w", err)
	}

	return nil
}

// IsGripperMoving checks if the gripper motor is currently moving.
func (d *Driver) IsGripperMoving() (bool, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if err := d.checkOpen(); err != nil {
		return false, err
	}

	data, err := d.handler.Read(byte(GripperMotorID), AddrMoving, 1)
	if err != nil {
		return false, fmt.Errorf("failed to read gripper moving status: %w", err)
	}

	return len(data) > 0 && data[0] != 0, nil
}

// Singleton driver management for sharing between arm and gripper components.

var (
	driverInstance *Driver
	driverMu       sync.Mutex
	driverRefCount int
)

// GetDriver returns a shared driver instance for the given port.
// Multiple calls with the same port will return the same driver.
func GetDriver(portName string, baudRate int) (*Driver, error) {
	fmt.Printf("[DEBUG] GetDriver: acquiring lock for port %s\n", portName)
	driverMu.Lock()
	defer driverMu.Unlock()
	fmt.Printf("[DEBUG] GetDriver: lock acquired, driverInstance=%v, refCount=%d\n", driverInstance != nil, driverRefCount)

	if driverInstance != nil {
		driverRefCount++
		fmt.Printf("[DEBUG] GetDriver: reusing existing driver, new refCount=%d\n", driverRefCount)
		return driverInstance, nil
	}

	fmt.Printf("[DEBUG] GetDriver: creating new driver for %s at %d baud\n", portName, baudRate)
	driver, err := NewDriver(portName, baudRate)
	if err != nil {
		fmt.Printf("[DEBUG] GetDriver: NewDriver failed: %v\n", err)
		return nil, err
	}

	driverInstance = driver
	driverRefCount = 1
	fmt.Printf("[DEBUG] GetDriver: new driver created successfully\n")
	return driver, nil
}

// ReleaseDriver decrements the reference count and closes the driver when no longer in use.
func ReleaseDriver() {
	fmt.Printf("[DEBUG] ReleaseDriver: acquiring lock\n")
	driverMu.Lock()
	defer driverMu.Unlock()
	fmt.Printf("[DEBUG] ReleaseDriver: lock acquired, driverInstance=%v, refCount=%d\n", driverInstance != nil, driverRefCount)

	if driverInstance == nil {
		fmt.Printf("[DEBUG] ReleaseDriver: no driver instance to release\n")
		return
	}

	driverRefCount--
	fmt.Printf("[DEBUG] ReleaseDriver: decremented refCount to %d\n", driverRefCount)
	if driverRefCount <= 0 {
		fmt.Printf("[DEBUG] ReleaseDriver: closing driver\n")
		driverInstance.Close()
		driverInstance = nil
		driverRefCount = 0
		fmt.Printf("[DEBUG] ReleaseDriver: driver closed and cleared\n")
	}
}
