// Package gripper provides the Viam gripper component for the Trossen ViperX-300s.
package gripper

import (
	"context"
	"sync"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/clintpurser/trossen/dynamixel"
)

// Model is the Viam model for the ViperX-300s gripper.
var Model = resource.NewModel("clint", "trossen", "viperx-300s-gripper")

func init() {
	resource.RegisterComponent(gripper.API, Model, resource.Registration[gripper.Gripper, *Config]{
		Constructor: NewViperX300sGripper,
	})
}

// Config is the configuration for the ViperX-300s gripper.
type Config struct {
	USBPort       string  `json:"usb_port"`
	BaudRate      int     `json:"baud_rate,omitempty"`
	OpenPosition  float64 `json:"open_position,omitempty"`
	ClosePosition float64 `json:"close_position,omitempty"`
}

// Validate validates the config.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.USBPort == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "usb_port")
	}
	return nil, nil, nil
}

// viperX300sGripper implements the gripper.Gripper interface.
type viperX300sGripper struct {
	resource.Named
	resource.AlwaysRebuild

	mu            sync.RWMutex
	driver        *dynamixel.Driver
	logger        logging.Logger
	openPosition  float64
	closePosition float64
}

// NewViperX300sGripper creates a new ViperX-300s gripper component.
func NewViperX300sGripper(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	config, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	g := &viperX300sGripper{
		Named:         conf.ResourceName().AsNamed(),
		logger:        logger,
		openPosition:  config.OpenPosition,
		closePosition: config.ClosePosition,
	}

	// Set defaults
	if g.openPosition == 0 {
		g.openPosition = dynamixel.GripperOpenPosition
	} else {
		g.openPosition = dynamixel.DegreesToRadians(g.openPosition)
	}
	if g.closePosition == 0 {
		g.closePosition = dynamixel.GripperClosePosition
	} else {
		g.closePosition = dynamixel.DegreesToRadians(g.closePosition)
	}

	baudRate := config.BaudRate
	if baudRate == 0 {
		baudRate = dynamixel.DefaultBaudRate
	}

	// Get shared driver instance
	driver, err := dynamixel.GetDriver(config.USBPort, baudRate)
	if err != nil {
		return nil, errors.Wrap(err, "failed to get Dynamixel driver")
	}
	g.driver = driver

	// Enable gripper torque
	if err := g.driver.EnableGripperTorque(); err != nil {
		return nil, errors.Wrap(err, "failed to enable gripper torque")
	}

	// Set profile for smooth motion
	if err := g.driver.SetProfileVelocity(100, []int{dynamixel.GripperMotorID}); err != nil {
		return nil, errors.Wrap(err, "failed to set gripper velocity")
	}
	if err := g.driver.SetProfileAcceleration(50, []int{dynamixel.GripperMotorID}); err != nil {
		return nil, errors.Wrap(err, "failed to set gripper acceleration")
	}

	logger.Infof("ViperX-300s gripper initialized on %s", config.USBPort)
	return g, nil
}

// Open opens the gripper.
func (g *viperX300sGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	return g.driver.WriteGripperPosition(g.openPosition)
}

// Grab closes the gripper to grab an object.
func (g *viperX300sGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()

	if err := g.driver.WriteGripperPosition(g.closePosition); err != nil {
		return false, err
	}

	// For now, always return true - proper grab detection would require
	// monitoring motor current/load
	return true, nil
}

// IsHoldingSomething returns whether the gripper is holding something.
func (g *viperX300sGripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	// For now, return unknown status - proper detection would require
	// monitoring motor current/load or comparing final position to close_position
	return gripper.HoldingStatus{
		IsHoldingSomething: false,
	}, nil
}

// Stop stops gripper motion.
func (g *viperX300sGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	current, err := g.driver.ReadGripperPosition()
	if err != nil {
		return err
	}
	return g.driver.WriteGripperPosition(current)
}

// IsMoving returns whether the gripper is currently moving.
func (g *viperX300sGripper) IsMoving(ctx context.Context) (bool, error) {
	g.mu.RLock()
	defer g.mu.RUnlock()

	return g.driver.IsGripperMoving()
}

// Geometries returns the geometries of the gripper.
func (g *viperX300sGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	// Create a simple box geometry for the gripper
	// Approximate dimensions: 100mm x 70mm x 50mm
	box, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{X: 100, Y: 70, Z: 50}, g.Name().ShortName())
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{box}, nil
}

// ModelFrame returns nil as grippers don't have a kinematic model.
func (g *viperX300sGripper) ModelFrame() referenceframe.Model {
	return nil
}

// Kinematics returns nil as grippers don't have a kinematic model.
func (g *viperX300sGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, nil
}

// CurrentInputs returns the current gripper position as referenceframe inputs.
func (g *viperX300sGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	g.mu.RLock()
	defer g.mu.RUnlock()

	pos, err := g.driver.ReadGripperPosition()
	if err != nil {
		return nil, err
	}
	return []referenceframe.Input{referenceframe.Input(pos)}, nil
}

// GoToInputs moves the gripper to the specified position.
func (g *viperX300sGripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	for _, step := range inputSteps {
		if err := ctx.Err(); err != nil {
			return err
		}
		if len(step) > 0 {
			g.mu.Lock()
			err := g.driver.WriteGripperPosition(float64(step[0]))
			g.mu.Unlock()
			if err != nil {
				return err
			}
		}
	}
	return nil
}

// DoCommand handles custom commands.
func (g *viperX300sGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	result := make(map[string]interface{})

	if _, ok := cmd["get_position"]; ok {
		g.mu.RLock()
		pos, err := g.driver.ReadGripperPosition()
		g.mu.RUnlock()
		if err != nil {
			return nil, err
		}
		result["position"] = dynamixel.RadiansToDegrees(pos)
	}

	if val, ok := cmd["set_position"]; ok {
		degrees, ok := val.(float64)
		if !ok {
			return nil, errors.New("set_position must be a number (degrees)")
		}
		radians := dynamixel.DegreesToRadians(degrees)
		g.mu.Lock()
		err := g.driver.WriteGripperPosition(radians)
		g.mu.Unlock()
		if err != nil {
			return nil, err
		}
		result["set_position"] = degrees
	}

	if _, ok := cmd["enable_torque"]; ok {
		enable, ok := cmd["enable_torque"].(bool)
		if !ok {
			return nil, errors.New("enable_torque must be a boolean")
		}
		g.mu.Lock()
		var err error
		if enable {
			err = g.driver.EnableGripperTorque()
			result["torque"] = "enabled"
		} else {
			err = g.driver.DisableGripperTorque()
			result["torque"] = "disabled"
		}
		g.mu.Unlock()
		if err != nil {
			return nil, err
		}
	}

	return result, nil
}

// Close closes the gripper and releases resources.
func (g *viperX300sGripper) Close(ctx context.Context) error {
	g.mu.Lock()
	defer g.mu.Unlock()

	if g.driver != nil {
		if err := g.driver.DisableGripperTorque(); err != nil {
			g.logger.Warnf("Failed to disable gripper torque on close: %v", err)
		}
		dynamixel.ReleaseDriver()
		g.driver = nil
	}

	g.logger.Info("ViperX-300s gripper closed")
	return nil
}
