#pragma once

#include <units/angle.h>
#include <units/velocity.h>
#include <units/current.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Basic/Component.h"
#include "Preferences.h"

class SwerveModule {
public:
    SwerveModule(int driveID, int turningID, int canCoderID, units::degree_t offset);
    void doPersistentConfiguration();
    void setState(frc::SwerveModuleState state);
    void sendDebugInfo(std::size_t moduleIndex); // Replaces sendFeedback();
    
    void zeroDriveEncoder();

    frc::SwerveModuleState getState();

    frc::SwerveModulePosition getPosition();

    void stop();
    void setTurningMotor(units::radian_t angle);
private:

    void setDriveMotor(units::meters_per_second_t velocity);

    units::radian_t getRawCANcoderRotation();
    units::radian_t getCANcoderRotation();

    units::turn_t getTurningMotorPosition();
    
    units::meters_per_second_t getDriveVelocity();
    units::meter_t getDrivePosition();

    // The drive motor.
    ctre::phoenix6::hardware::TalonFX driveMotor;
    // The turning motor.
    ctre::phoenix6::hardware::TalonFX turningMotor;

    // The absolute encoder.
    ctre::phoenix6::hardware::CANcoder canCoder;

    // The offset of the turning absolute encoder.
    units::radian_t absEncoderOffset;

    
    ctre::phoenix6::controls::PositionVoltage turnRequest;
    ctre::phoenix6::controls::VelocityVoltage driveRequest;
};