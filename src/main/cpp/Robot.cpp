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
				hang(nullptr),
				allComponents()
{
#ifdef ENABLE_DRIVE
	drive = new Drive(&limelight);
	allComponents.push_back(drive);
#endif

#ifdef ENABLE_CALGAE
	calgae = new Calgae();
	allComponents.push_back(calgae);
#endif
#ifdef ENABLE_WRIST
	wrist = new Wrist();
	allComponents.push_back(wrist);
#endif

#ifdef ENABLE_ELEVATOR
	elevator = new Elevator();
	allComponents.push_back(elevator);
#endif

	gamepiece = new Gamepiece(calgae, wrist, elevator);
	allComponents.push_back(gamepiece);

#ifdef ENABLE_AUTO
	auto_ = new Auto(drive, &limelight, gamepiece);
	allComponents.push_back(auto_);
#endif

#ifdef ENABLE_HANG
	hang = new Hang();
	hang->encoder.SetPosition(0);
	allComponents.push_back(hang);
#endif

#ifdef ENABLE_BLINKY_BLINKY
	blinkyBlinky = new BlinkyBlinky(gamepiece, hang);
	allComponents.push_back(blinkyBlinky);
#endif

	controls = new Controls(drive, gamepiece, blinkyBlinky, hang, &limelight);
}

void Robot::RobotInit() {
	Alert::startTimer();

	gamepiece->elevator->zeroMotors();

	if (auto_ != nullptr) {
		auto_->autoImportAutoAutos();
		auto_->autoSelectorInit();
	}

	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_FRONT, PreferencesLimelight::PIPELINE_EMPTY);
	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_BACK, PreferencesLimelight::PIPELINE_EMPTY);

	auto cam = frc::CameraServer::StartAutomaticCapture();
	if (!cam.IsConnected()) {
		frc::CameraServer::RemoveCamera(cam.GetName());
	}
}

void Robot::RobotPeriodic() {
	controls->sendFeedback();

	for (Component* component : allComponents) {
	 	component->sendFeedback();
	}
}

void Robot::AutonomousInit() {
    reset(Component::MatchMode::AUTO);

	ThunderLogger::StartLogging();

	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_FRONT, PreferencesLimelight::PIPELINE_APRILTAGS);
	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_BACK, PreferencesLimelight::PIPELINE_APRILTAGS);
}
void Robot::AutonomousPeriodic() {
	for (Component* component : allComponents)
		component->process();
}

void Robot::TeleopInit() {
    reset(Component::MatchMode::TELEOP);

	ThunderLogger::StartLogging();

	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_FRONT, PreferencesLimelight::PIPELINE_APRILTAGS);
	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_BACK, PreferencesLimelight::PIPELINE_APRILTAGS);
}
void Robot::TeleopPeriodic() {
	controls->process();

	for (Component* component : allComponents) {
		component->process();
	}
}

void Robot::DisabledInit() {
    reset(Component::MatchMode::DISABLED);

	ThunderLogger::StopLogging();

	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_FRONT, PreferencesLimelight::PIPELINE_EMPTY);
	LimelightHelpers::setPipelineIndex(PreferencesLimelight::LIMELIGHT_BACK, PreferencesLimelight::PIPELINE_EMPTY);
}
void Robot::DisabledPeriodic() {
	#ifdef ENABLE_BLINKY_BLINKY
	blinkyBlinky->process();
	#endif

	if (controls->shouldPersistentConfig()) {
		for (Component* component : allComponents) {
			component->doConfiguration(true);
		}

		printf("Persistent Configurations Applied \n");
	}
}

void Robot::TestInit() {
    reset(Component::MatchMode::TEST);
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
	// if (gamepiece->elevator != nullptr) {
	// 	gamepiece->elevator->manualMovement(0.05);
	// 	gamepiece->elevator->process();
	// }
	//// if (drive != nullptr) { // In case we ever need this again
	//	// drive->swerveModules.at(1)->setDriveMotor(1_mps);
	//	// drive->swerveModules.at(1)->turningMotor.Set(0.4);
	//// }
	// if (hang != nullptr) {
	// 	hang->setSolenoidState(Hang::SolenoidState::UP);
	// }
}

void Robot::SimulationInit() { }
void Robot::SimulationPeriodic() { }

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
