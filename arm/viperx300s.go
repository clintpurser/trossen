// Package arm provides the Viam arm component for the Trossen ViperX-300s.
package arm

import (
	"context"
	_ "embed"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"

	"github.com/pkg/errors"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"github.com/clintpurser/trossen/dynamixel"
)

//go:embed viperx300s_kinematics.json
var kinematicsJSON []byte

// Model is the Viam model for the ViperX-300s arm.
var Model = resource.NewModel("clint", "trossen", "viperx-300s")

func init() {
	resource.RegisterComponent(arm.API, Model, resource.Registration[arm.Arm, *Config]{
		Constructor: NewViperX300s,
	})
}

// Config is the configuration for the ViperX-300s arm.
type Config struct {
	USBPort             string `json:"usb_port"`
	BaudRate            int    `json:"baud_rate,omitempty"`
	ProfileVelocity     int    `json:"profile_velocity,omitempty"`
	ProfileAcceleration int    `json:"profile_acceleration,omitempty"`
	Motion              string `json:"motion,omitempty"` // Motion service name for MoveToPosition (defaults to "builtin")
}

// Validate validates the config.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.USBPort == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "usb_port")
	}

	// Declare motion service dependency
	deps := []string{}
	opt := []string{}
	if c.Motion != "" {
		// Explicitly specified motion service is required
		deps = append(deps, motion.Named(c.Motion).String())
	} else {
		// Builtin motion service is optional
		opt = append(opt, motion.Named("builtin").String())
	}

	return deps, opt, nil
}

// viperX300s implements the arm.Arm interface for the ViperX-300s.
type viperX300s struct {
	resource.Named
	resource.AlwaysRebuild

	mu     sync.RWMutex
	driver *dynamixel.Driver
	model  referenceframe.Model
	logger logging.Logger
	motion motion.Service // Motion service for MoveToPosition

	usbPort             string
	baudRate            int
	profileVelocity     int
	profileAcceleration int
}

// NewViperX300s creates a new ViperX-300s arm component.
func NewViperX300s(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (arm.Arm, error) {
	config, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	v := &viperX300s{
		Named:               conf.ResourceName().AsNamed(),
		logger:              logger,
		usbPort:             config.USBPort,
		baudRate:            config.BaudRate,
		profileVelocity:     config.ProfileVelocity,
		profileAcceleration: config.ProfileAcceleration,
	}

	// Set defaults
	if v.baudRate == 0 {
		v.baudRate = dynamixel.DefaultBaudRate
	}
	if v.profileVelocity == 0 {
		v.profileVelocity = dynamixel.DefaultProfileVelocity
	}
	if v.profileAcceleration == 0 {
		v.profileAcceleration = dynamixel.DefaultProfileAcceleration
	}

	// Load kinematics model
	if err := v.loadKinematics(); err != nil {
		return nil, errors.Wrap(err, "failed to load kinematics")
	}

	// Get motion service from dependencies
	if config.Motion != "" {
		// Explicitly specified - must be available
		v.motion, err = motion.FromDependencies(deps, config.Motion)
		if err != nil {
			return nil, errors.Wrapf(err, "failed to get motion service %q", config.Motion)
		}
		logger.Infof("Using motion service %q for MoveToPosition", config.Motion)
	} else {
		// Try to get builtin motion service (optional)
		v.motion, err = motion.FromDependencies(deps, "builtin")
		if err != nil {
			logger.Debugf("couldn't get default motion: %v", err)
		} else {
			logger.Infof("Using builtin motion service for MoveToPosition")
		}
	}

	// Initialize driver
	logger.Info("Getting Dynamixel driver...")
	driver, err := dynamixel.GetDriver(v.usbPort, v.baudRate)
	if err != nil {
		return nil, errors.Wrap(err, "failed to initialize Dynamixel driver")
	}
	v.driver = driver
	logger.Info("Dynamixel driver acquired")

	// Enable torque and set motion profile
	logger.Info("Enabling torque...")
	if err := v.driver.EnableTorque(dynamixel.AllArmMotorIDs); err != nil {
		return nil, errors.Wrap(err, "failed to enable torque")
	}
	logger.Info("Setting profile velocity...")
	if err := v.driver.SetProfileVelocity(v.profileVelocity, dynamixel.AllArmMotorIDs); err != nil {
		return nil, errors.Wrap(err, "failed to set profile velocity")
	}
	logger.Info("Setting profile acceleration...")
	if err := v.driver.SetProfileAcceleration(v.profileAcceleration, dynamixel.AllArmMotorIDs); err != nil {
		return nil, errors.Wrap(err, "failed to set profile acceleration")
	}

	logger.Infof("ViperX-300s arm initialized on %s at %d baud", v.usbPort, v.baudRate)
	return v, nil
}

// loadKinematics loads the kinematics model from embedded JSON.
func (v *viperX300s) loadKinematics() error {
	model, err := referenceframe.UnmarshalModelJSON(kinematicsJSON, v.Name().ShortName())
	if err != nil {
		return errors.Wrap(err, "failed to parse kinematics JSON")
	}
	v.model = model
	return nil
}

// EndPosition returns the current end-effector position using forward kinematics.
func (v *viperX300s) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	joints, err := v.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return referenceframe.ComputeOOBPosition(v.model, joints)
}

// MoveToPosition moves the arm to the specified pose using motion planning.
// This delegates to the configured motion service for IK and path planning.
func (v *viperX300s) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	if v.motion == nil {
		return errors.New("MoveToPosition requires the motion service; configure 'motion' attribute or ensure builtin motion service is available")
	}

	// Use the motion service to plan and execute the move
	_, err := v.motion.Move(ctx, motion.MoveReq{
		ComponentName: v.Name().Name,
		Destination:   referenceframe.NewPoseInFrame(referenceframe.World, pose),
		Extra:         extra,
	})
	return err
}

// MoveToJointPositions moves the arm to the specified joint positions.
func (v *viperX300s) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	if len(positions) != dynamixel.NumJoints {
		return fmt.Errorf("expected %d joint positions, got %d", dynamixel.NumJoints, len(positions))
	}

	// Convert Input slice to float64 slice (Input is a float64 alias)
	radians := make([]float64, len(positions))
	for i, pos := range positions {
		radians[i] = float64(pos)
	}

	// Debug: log the positions being commanded
	v.logger.Infof("MoveToJointPositions called with %d positions", len(positions))
	for i, rad := range radians {
		config := dynamixel.ViperX300SJoints[i]
		ticks := dynamixel.RadiansToTicks(rad)
		v.logger.Infof("  Joint %d (%s): %.4f rad (%.2f°) -> %d ticks",
			i, config.Name, rad, dynamixel.RadiansToDegrees(rad), ticks)
	}

	// Validate joint limits
	for i, rad := range radians {
		if !dynamixel.ValidateJointLimits(i, rad) {
			config := dynamixel.ViperX300SJoints[i]
			return fmt.Errorf("joint %s position %.1f° out of range [%.1f, %.1f]",
				config.Name, dynamixel.RadiansToDegrees(rad),
				dynamixel.RadiansToDegrees(config.MinRadians),
				dynamixel.RadiansToDegrees(config.MaxRadians))
		}
	}

	v.mu.Lock()
	defer v.mu.Unlock()

	return v.driver.WriteJointPositions(radians)
}

// MoveThroughJointPositions moves the arm through a series of joint positions.
func (v *viperX300s) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]any) error {
	for _, pos := range positions {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := v.MoveToJointPositions(ctx, pos, extra); err != nil {
			return err
		}
	}
	return nil
}

// JointPositions returns the current joint positions.
func (v *viperX300s) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	v.mu.RLock()
	defer v.mu.RUnlock()

	radians, err := v.driver.ReadJointPositions()
	if err != nil {
		return nil, err
	}

	// Convert float64 slice to Input slice (Input is a float64 alias)
	inputs := make([]referenceframe.Input, len(radians))
	for i, rad := range radians {
		inputs[i] = referenceframe.Input(rad)
	}
	return inputs, nil
}

// Stop stops the arm by writing current positions as goals.
func (v *viperX300s) Stop(ctx context.Context, extra map[string]interface{}) error {
	v.mu.Lock()
	defer v.mu.Unlock()

	current, err := v.driver.ReadJointPositions()
	if err != nil {
		return err
	}
	return v.driver.WriteJointPositions(current)
}

// IsMoving returns whether the arm is currently moving.
func (v *viperX300s) IsMoving(ctx context.Context) (bool, error) {
	v.mu.RLock()
	defer v.mu.RUnlock()

	return v.driver.IsMoving()
}

// ModelFrame returns the kinematics model.
func (v *viperX300s) ModelFrame() referenceframe.Model {
	return v.model
}

// Kinematics returns the kinematics model.
func (v *viperX300s) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return v.model, nil
}

// CurrentInputs returns the current joint positions as referenceframe inputs.
func (v *viperX300s) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return v.JointPositions(ctx, nil)
}

// GoToInputs moves the arm through the specified joint position waypoints.
func (v *viperX300s) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	for _, step := range inputSteps {
		if err := ctx.Err(); err != nil {
			return err
		}

		// Convert Input slice to float64 slice
		radians := make([]float64, len(step))
		for i, input := range step {
			radians[i] = float64(input)
		}

		v.mu.Lock()
		err := v.driver.WriteJointPositions(radians)
		v.mu.Unlock()
		if err != nil {
			return err
		}
	}
	return nil
}

// Geometries returns the geometries of the arm in its current configuration.
func (v *viperX300s) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	inputs, err := v.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gifs, err := v.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gifs.Geometries(), nil
}

// viperX300sModelParts lists the 3D model parts for the ViperX-300s arm.
// These correspond to GLB files in arm/3d_models/viperx-300s/
// The names MUST match the link IDs in viperx300s_kinematics.json for the
// visualizer to properly associate models with frames.
var viperX300sModelParts = []string{
	"base_link",
	"shoulder_link",
	"upper_arm_link",
	"forearm_link",
	"wrist_link",
	"gripper_link",
}

// Get3DModels returns the 3D models of the arm.
func (v *viperX300s) Get3DModels(ctx context.Context, extra map[string]interface{}) (map[string]*commonpb.Mesh, error) {
	models := make(map[string]*commonpb.Mesh)

	moduleRoot := os.Getenv("VIAM_MODULE_ROOT")
	if moduleRoot == "" {
		// Fallback for local development
		moduleRoot = "."
	}

	for _, part := range viperX300sModelParts {
		path := filepath.Join(moduleRoot, "arm", "3d_models", "viperx-300s", part+".glb")
		glb, err := os.ReadFile(path)
		if err != nil {
			// Log but continue - model files may not exist yet
			v.logger.Debugf("Could not load 3D model %s: %v", part, err)
			continue
		}
		models[part] = &commonpb.Mesh{
			Mesh:        glb,
			ContentType: "model/gltf-binary",
		}
	}

	return models, nil
}

// DoCommand handles custom commands.
func (v *viperX300s) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	result := make(map[string]interface{})

	if _, ok := cmd["enable_torque"]; ok {
		enable, ok := cmd["enable_torque"].(bool)
		if !ok {
			return nil, errors.New("enable_torque must be a boolean")
		}
		v.mu.Lock()
		var err error
		if enable {
			err = v.driver.EnableTorque(dynamixel.AllArmMotorIDs)
			result["torque"] = "enabled"
		} else {
			err = v.driver.DisableTorque(dynamixel.AllArmMotorIDs)
			result["torque"] = "disabled"
		}
		v.mu.Unlock()
		if err != nil {
			return nil, err
		}
	}

	if val, ok := cmd["set_velocity"]; ok {
		velocity, ok := val.(float64)
		if !ok {
			return nil, errors.New("set_velocity must be a number")
		}
		v.mu.Lock()
		err := v.driver.SetProfileVelocity(int(velocity), dynamixel.AllArmMotorIDs)
		v.mu.Unlock()
		if err != nil {
			return nil, err
		}
		result["velocity"] = velocity
	}

	if val, ok := cmd["set_acceleration"]; ok {
		accel, ok := val.(float64)
		if !ok {
			return nil, errors.New("set_acceleration must be a number")
		}
		v.mu.Lock()
		err := v.driver.SetProfileAcceleration(int(accel), dynamixel.AllArmMotorIDs)
		v.mu.Unlock()
		if err != nil {
			return nil, err
		}
		result["acceleration"] = accel
	}

	return result, nil
}

// Close closes the arm and releases resources.
func (v *viperX300s) Close(ctx context.Context) error {
	v.mu.Lock()
	defer v.mu.Unlock()

	if v.driver != nil {
		if err := v.driver.DisableTorque(dynamixel.AllArmMotorIDs); err != nil {
			v.logger.Warnf("Failed to disable torque on close: %v", err)
		}
		dynamixel.ReleaseDriver()
		v.driver = nil
	}

	v.logger.Info("ViperX-300s arm closed")
	return nil
}

// MarshalJSON implements json.Marshaler for config debugging.
func (c *Config) MarshalJSON() ([]byte, error) {
	type Alias Config
	return json.Marshal(&struct {
		*Alias
	}{
		Alias: (*Alias)(c),
	})
}
