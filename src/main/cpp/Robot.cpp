// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot() :
				lastMode(Component::MatchMode::DISABLED),
				limelight(),
				drive(nullptr),
				calgae(nullptr),
				wrist(nullptr),
				elevator(nullptr),
				controls(nullptr),
				auto_(nullptr),
				allComponents()
{
#ifdef ENABLE_DRIVE
	drive = new Drive(&limelight);
	allComponents.push_back(drive);
#endif
#ifdef ENABLE_GAMEPIECE
	calgae = new Calgae();
	wrist = new Wrist();
	allComponents.push_back(wrist);
	allComponents.push_back(calgae);
#endif
#ifdef ENABLE_ELEVATOR
	elevator = new Elevator();
	allComponents.push_back(elevator);
#endif
#ifdef ENABLE_AUTO
	auto_ = new Auto(drive);
#endif
	gamepiece = new Gamepiece(calgae, wrist, elevator);
	allComponents.push_back(gamepiece);
#ifdef ENABLE_BLINKY_BLINKY
	blinkyBlinky = new BlinkyBlinky(gamepiece);
	allComponents.push_back(blinkyBlinky);
#endif
	controls = new Controls(drive, gamepiece, calgae, wrist, elevator, blinkyBlinky);
}

void Robot::RobotInit() {
	if (auto_)
		auto_->autoSelectorInit();
}
void Robot::RobotPeriodic() {
	controls->sendFeedback();
	for (Component* component : allComponents) {
	 	component->sendFeedback();
	}
}

void Robot::AutonomousInit() {
    reset(Component::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {
	if (auto_)
		auto_->process();
	
	for (Component* component : allComponents)
		component->process();
}

void Robot::TeleopInit() {
    reset(Component::MatchMode::TELEOP);
}
void Robot::TeleopPeriodic() {
	controls->process();
	for (Component* component : allComponents) {
		component->process();
	}
}

void Robot::DisabledInit() {
    reset(Component::MatchMode::DISABLED);
}
void Robot::DisabledPeriodic() {
	#ifdef ENABLE_BLINKY_BLINKY
	blinkyBlinky->process();
	#endif
}

void Robot::TestInit() {
    reset(Component::MatchMode::TEST);
	for (Component* component : allComponents) {
		component->doPersistentConfiguration();
	}

	printf("Persistent Configurations Applied \n");
}

void Robot::TestPeriodic() {
//#define ELEVATOR_TESTING
#if defined(ENABLE_ELEVATOR) && defined(ELEVATOR_TESTING)
	static Elevator::Preset testPresets[] = {
		Elevator::kGROUND,
		Elevator::kL4,
		Elevator::kL1,
		Elevator::kL2,
		Elevator::kL3,
		Elevator::kGROUND,
		Elevator::kSTOP
	};
	static int curTestPos = 0;
	static frc::Timer stepWaitTimer;
	elevator.goToPreset(testPresets[curTestPos]);
	if (elevator.atPreset() && testPresets[curTestPos] != Elevator::kSTOP) {
		if (stepWaitTimer.IsRunning() && stepWaitTimer.Get() > 5_s) {
			stepWaitTimer.Stop();
			stepWaitTimer.Reset();
			curTestPos++;
		} else {
			stepWaitTimer.Start();
		}
	}
	//elevator.manualMovement(0.05);
	elevator.process();
#endif
}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

void Robot::reset(Component::MatchMode mode) {
	if (auto_)
		auto_->resetToMatchMode(lastMode, mode);
	controls->resetToMatchMode(lastMode, mode);
	for (Component* component : allComponents) {
		component->resetToMatchMode(lastMode, mode);
	}

	lastMode = mode;
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
