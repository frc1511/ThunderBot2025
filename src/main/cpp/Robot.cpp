// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
	auto_.autoSelectorInit();
	AddPeriodic([&] {
		for (Component* component : allComponents) {
			component->sendFeedback();
		}
	}, 20_ms);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    reset(Component::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {
	auto_.process();
	drive.process();
}

void Robot::TeleopInit() {
    reset(Component::MatchMode::TELEOP);
}
void Robot::TeleopPeriodic() {
	for (Component* component : allComponents) {
		component->process();
	}
}

void Robot::DisabledInit() {
    reset(Component::MatchMode::DISABLED);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    reset(Component::MatchMode::TEST);
	for (Component* component : allComponents) {
		component->doPersistantConfiguration();
	}
}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::reset(Component::MatchMode mode) {
	for (Component* component : allComponents) {
		component->callResetToMode(lastMode);
	}

	lastMode = mode;
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
