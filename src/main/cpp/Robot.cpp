// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot() {}
void Robot::RobotPeriodic() {
  // AddPeriodic([&] {
    for (Component* component : allComponents) {
	 	component->sendFeedback();
	}
  // }, 20_ms);
}

void Robot::AutonomousInit() {
    reset(Component::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {
	for (Component* component : allComponents) {
		component->process();
	}
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
		component->doPersistentConfiguration();
	}

}
void Robot::TestPeriodic() {
	elevator.goToPreset(Elevator::kGROUND);
	//elevator.manualMovement(0.05);
	elevator.process();
}

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
