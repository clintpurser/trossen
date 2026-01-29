// Package main is the entry point for the Trossen ViperX-300s Viam module.
package main

import (
	"context"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"

	// Import packages to register components
	viperxArm "github.com/clintpurser/trossen/arm"
	viperxGripper "github.com/clintpurser/trossen/gripper"
)

func main() {
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("trossen"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	mod, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}

	// Register arm component
	if err := mod.AddModelFromRegistry(ctx, arm.API, viperxArm.Model); err != nil {
		return err
	}

	// Register gripper component
	if err := mod.AddModelFromRegistry(ctx, gripper.API, viperxGripper.Model); err != nil {
		return err
	}

	if err := mod.Start(ctx); err != nil {
		return err
	}
	defer mod.Close(ctx)

	<-ctx.Done()
	return nil
}
