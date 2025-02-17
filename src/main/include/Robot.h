// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "Auto/Auto.h"
#include "Controls.h"
#include "GamEpiece/Gamepiece.h"
#include "GamEpiece/Calgae.h"
#include "GamEpiece/Wrist.h"
#include "Elevator.h"
#include "BlinkyBlinky.h"
#include "Hang.h"
#include "Alerts.h"

class Robot : public frc::TimedRobot {
  public:
	Robot();
	void RobotInit() override;
	void RobotPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	void SimulationInit() override;
	void SimulationPeriodic() override;
  private:
	void reset(Component::MatchMode mode);
	Component::MatchMode lastMode = Component::MatchMode::DISABLED;

	Limelight limelight;

	Drive *drive;
	Calgae *calgae;
	Wrist *wrist;
	Elevator *elevator;
	Controls *controls;
	Auto *auto_;
	Gamepiece* gamepiece;
	BlinkyBlinky* blinkyBlinky;
	Hang* hang;

	std::vector<Component*> allComponents; 
};