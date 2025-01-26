// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "Controls.h"
#include "GamEpiece.h"

class Robot : public frc::TimedRobot {
  public:
	Robot();
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
	Drive drive {&limelight};
	Gamepiece gamepiece;
	Controls controls {&drive, &gamepiece};
  Limelight limelight;

	std::vector<Component*> allComponents {
    	&drive, &gamepiece, &controls, &limelight
   };
};
