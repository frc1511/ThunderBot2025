// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Basic/Robot.h"

Robot::Robot() {}
void Robot::RobotPeriodic() {
	for (int index = 0; index < allComponents.size(), index++;) {
		allComponents[index]->sendFeedback();
	}
}

void Robot::AutonomousInit() {
    reset(Component::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {
	for (int index = 0; index < allComponents.size(), index++;) {
		allComponents[index]->autoProcess();
	}
}

void Robot::TeleopInit() {
    reset(Component::MatchMode::TELEOP);
}
void Robot::TeleopPeriodic() {
	for (int index = 0; index < allComponents.size(), index++;) {
		allComponents[index]->teleOpProcess();
	}
}

void Robot::DisabledInit() {
    reset(Component::MatchMode::DISABLED);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    reset(Component::MatchMode::TEST);
}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
