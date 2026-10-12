package turbine

import (
	"context"
	"time"
)

type DifferentialPressureSensor interface {
	ReadPressure(ctx context.Context) (float64, error)
}

type RotorSpeedSensor interface {
	ReadRotorSpeed(ctx context.Context) (float64, error)
}

type ResistanceActuator interface {
	SetLoadResistance(ctx context.Context, rOhms float64) error
}

type TurbineMotorController struct {
	pressSensor DifferentialPressureSensor
	speedSensor RotorSpeedSensor
	resActuator ResistanceActuator
	pid         *PIDController
	sim         *MotorSimulation
}

func NewTurbineMotorController(
	pressSensor DifferentialPressureSensor,
	speedSensor RotorSpeedSensor,
	resActuator ResistanceActuator,
	pid *PIDController,
	sim *MotorSimulation,
) *TurbineMotorController {
	return &TurbineMotorController{
		pressSensor: pressSensor,
		speedSensor: speedSensor,
		resActuator: resActuator,
		pid:         pid,
		sim:         sim,
	}
}

func (c *TurbineMotorController) Step(ctx context.Context, now time.Time) (windSpeed, targetOmega, currentOmega, loadR float64, err error) {
	deltaP, err := c.pressSensor.ReadPressure(ctx)
	if err != nil {
		return 0, 0, 0, 0, err
	}

	currentOmega, err = c.speedSensor.ReadRotorSpeed(ctx)
	if err != nil {
		return 0, 0, 0, 0, err
	}

	windSpeed = c.sim.WindFromPressure(deltaP)

	if windSpeed < 2.5 {
		loadR = c.sim.cfg.MaxLoadR
		_ = c.resActuator.SetLoadResistance(ctx, loadR)
		c.pid.Reset()
		return windSpeed, 0, currentOmega, loadR, nil
	}

	targetOmega = c.sim.CalculateOptimalOmega(windSpeed)

	admittance := c.pid.Update(targetOmega, currentOmega, now)
	if admittance <= (1.0 / c.sim.cfg.MaxLoadR) {
		loadR = c.sim.cfg.MaxLoadR
	} else {
		loadR = 1.0 / admittance
	}

	if loadR < c.sim.cfg.MinLoadR {
		loadR = c.sim.cfg.MinLoadR
	} else if loadR > c.sim.cfg.MaxLoadR {
		loadR = c.sim.cfg.MaxLoadR
	}

	_ = c.resActuator.SetLoadResistance(ctx, loadR)
	return windSpeed, targetOmega, currentOmega, loadR, nil
}
