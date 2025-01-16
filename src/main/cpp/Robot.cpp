// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot() {}
void Robot::RobotPeriodic() {
  // AddPeriodic([&] {
  //   drive.sendDebugInfo();
  // }, 20_ms);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  drive.doPersistentConfiguration();
  drive.resetToMode();
}
void Robot::TeleopPeriodic() {
  controls.process();
  drive.process();
}

void Robot::DisabledInit() {
  drive.resetToMode();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {
  wpi::array<SwerveModule*,4>* swerveModules = drive.getSwerveModules();

  frc::SwerveModuleState state {};
  state.angle = frc::Rotation2d(30_deg);
  state.speed = 0_mps;
  for (SwerveModule* module : *swerveModules) {
    module->setState(state);
  }
}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
