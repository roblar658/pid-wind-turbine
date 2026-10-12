package turbine_test

import (
	"math"
	"testing"

	"wind-resistance-turbine/internal/turbine"
)

func TestMotorNeverReverses(t *testing.T) {
	cfg := turbine.MotorTurbineConfig{
		AirDensity:   1.225,
		RotorRadius:  2.5,
		RotorInertia: 40.0,
		OptTSR:       7.0,
		MaxCp:        0.44,
		MotorKm:      2.0,
		InternalR:    0.5,
		MinLoadR:     0.1,
		MaxLoadR:     500.0,
	}
	sim := turbine.NewMotorSimulation(cfg, 0.0)

	for i := 0; i < 50; i++ {
		sim.Step(0.0, 0.1, 0.1)
		if sim.Omega < 0 {
			t.Fatalf("Rotor reversed! Omega = %v", sim.Omega)
		}
	}
}

func TestElectricalFormulas(t *testing.T) {
	cfg := turbine.MotorTurbineConfig{
		MotorKm:   2.0,
		InternalR: 1.0,
	}
	sim := turbine.NewMotorSimulation(cfg, 10.0)

	emf, current, torque, power := sim.CalculateElectrical(3.0)

	if math.Abs(emf-20.0) > 1e-3 {
		t.Errorf("expected EMF 20.0, got %v", emf)
	}
	if math.Abs(current-5.0) > 1e-3 {
		t.Errorf("expected Current 5.0, got %v", current)
	}
	if math.Abs(torque-10.0) > 1e-3 {
		t.Errorf("expected Torque 10.0, got %v", torque)
	}
	if math.Abs(power-75.0) > 1e-3 {
		t.Errorf("expected Power 75.0, got %v", power)
	}
}
