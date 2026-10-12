package turbine

import (
	"math"
)

type MotorTurbineConfig struct {
	AirDensity    float64
	RotorRadius   float64
	RotorInertia  float64
	OptTSR        float64
	MaxCp         float64
	MotorKm       float64
	InternalR     float64
	MinLoadR      float64
	MaxLoadR      float64
	MechFrictionB float64
}

type MotorSimulation struct {
	cfg   MotorTurbineConfig
	Omega float64
}

func NewMotorSimulation(cfg MotorTurbineConfig, initialOmega float64) *MotorSimulation {
	return &MotorSimulation{
		cfg:   cfg,
		Omega: initialOmega,
	}
}

func (m *MotorSimulation) WindFromPressure(deltaP float64) float64 {
	if deltaP <= 0 {
		return 0
	}
	return math.Sqrt((2.0 * deltaP) / m.cfg.AirDensity)
}

func (m *MotorSimulation) CalculateOptimalOmega(windSpeed float64) float64 {
	if windSpeed <= 0.2 {
		return 0
	}
	return (m.cfg.OptTSR * windSpeed) / m.cfg.RotorRadius
}

func (m *MotorSimulation) CalculateElectrical(loadResistance float64) (emf, current, torque, power float64) {
	if m.Omega <= 0 {
		return 0, 0, 0, 0
	}

	emf = m.cfg.MotorKm * m.Omega
	rTot := m.cfg.InternalR + loadResistance
	if rTot < 0.01 {
		rTot = 0.01
	}

	current = emf / rTot
	torque = m.cfg.MotorKm * current
	power = current * current * loadResistance

	return emf, current, torque, power
}

func (m *MotorSimulation) Step(windSpeed float64, loadResistance float64, dt float64) (emf, current, torque, power, aeroTorque float64) {
	area := math.Pi * m.cfg.RotorRadius * m.cfg.RotorRadius

	tsr := 0.0
	if windSpeed > 0.2 {
		tsr = (m.Omega * m.cfg.RotorRadius) / windSpeed
	}

	cp := 0.0
	if tsr > 0 && windSpeed > 0.5 {
		diff := (tsr - m.cfg.OptTSR) / 2.5
		cp = m.cfg.MaxCp * math.Exp(-0.5*diff*diff)
	}

	pAero := 0.5 * m.cfg.AirDensity * area * math.Pow(windSpeed, 3) * cp
	effectiveOmega := m.Omega
	if effectiveOmega < 0.3 {
		effectiveOmega = 0.3
	}
	aeroTorque = pAero / effectiveOmega

	emf, current, torque, power = m.CalculateElectrical(loadResistance)

	netTorque := aeroTorque - torque - (m.cfg.MechFrictionB * m.Omega)
	dOmegaDt := netTorque / m.cfg.RotorInertia

	m.Omega += dOmegaDt * dt
	if m.Omega < 0 {
		m.Omega = 0
	}

	return emf, current, torque, power, aeroTorque
}
