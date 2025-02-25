#include "Drive/SwerveModule.h"

#define ALL_SWERVE_MODULES_COUNTS_PER_REV 42
#define OPTIMIZATION_BYPASS false
SwerveModule::SwerveModule(int driveID, int turningID, int canCoderID, units::degree_t offset)
: driveMotor(driveID),
  turningMotor(turningID),
  canCoder(canCoderID),
  absEncoderOffset(offset),
  turnRequest(ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0)),
  driveRequest(ctre::phoenix6::controls::VelocityVoltage{(units::turns_per_second_t)0}.WithSlot(0))

{
    doConfiguration(false);
}

void SwerveModule::doConfiguration(bool persist)
{
    // Can Coder
    ctre::phoenix6::configs::MagnetSensorConfigs magnetConfig;
    magnetConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    canCoder.GetConfigurator().Apply(magnetConfig);

    // Turning Motor
    ctre::phoenix6::configs::Slot0Configs turningPIDConfig {};
    turningPIDConfig.kP = PreferencesTurnMotor::PID.Kp;
    turningPIDConfig.kI = PreferencesTurnMotor::PID.Ki;
    turningPIDConfig.kD = PreferencesTurnMotor::PID.Kd;
    turningPIDConfig.kV = PreferencesTurnMotor::PID.Kv;
    turningPIDConfig.kS = PreferencesTurnMotor::PID.Ks;

    turningMotor.GetConfigurator().Apply(turningPIDConfig);

    ctre::phoenix6::configs::MotorOutputConfigs turningMotorOutput {};
    turningMotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    turningMotor.GetConfigurator().Apply(turningMotorOutput);

    ctre::phoenix6::configs::CurrentLimitsConfigs turningCurrentLimit {};
    turningCurrentLimit.WithSupplyCurrentLimit(PreferencesTurnMotor::MAX_AMPERAGE);
    turningCurrentLimit.WithSupplyCurrentLimitEnable(true);
    turningCurrentLimit.WithStatorCurrentLimit(PreferencesTurnMotor::MAX_AMPERAGE);
    turningCurrentLimit.WithStatorCurrentLimitEnable(true);
    turningMotor.GetConfigurator().Apply(turningCurrentLimit);

    ctre::phoenix6::configs::FeedbackConfigs turningFeedback {};
    turningFeedback.FeedbackRemoteSensorID = canCoder.GetDeviceID();
    turningFeedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    turningFeedback.RotorToSensorRatio = 18.75;
    turningMotor.GetConfigurator().Apply(turningFeedback);

    ctre::phoenix6::configs::ClosedLoopRampsConfigs ramp {};
    ramp.WithVoltageClosedLoopRampPeriod(PreferencesSwerve::DRIVE_RAMP_TIME);

    driveMotor.GetConfigurator().Apply(ramp);

    // Drive Motor
    ctre::phoenix6::configs::Slot0Configs drivePIDConfig {};
    drivePIDConfig.kP = PreferencesDriveMotor::PID.Kp;
    drivePIDConfig.kI = PreferencesDriveMotor::PID.Ki;
    drivePIDConfig.kD = PreferencesDriveMotor::PID.Kd;
    drivePIDConfig.kV = PreferencesDriveMotor::PID.Kv;
    drivePIDConfig.kS = PreferencesDriveMotor::PID.Ks;

    driveMotor.GetConfigurator().Apply(drivePIDConfig);
    ctre::phoenix6::configs::MotorOutputConfigs driveMotorOutput {};
    driveMotorOutput.Inverted = true;

    setDriveMotorsNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    driveMotor.GetConfigurator().Apply(driveMotorOutput);

    ctre::phoenix6::configs::CurrentLimitsConfigs driveCurrentLimit {};
    driveCurrentLimit.WithSupplyCurrentLimit(PreferencesDriveMotor::MAX_AMPERAGE);
    driveCurrentLimit.WithSupplyCurrentLimitEnable(true);
    driveCurrentLimit.WithStatorCurrentLimit(PreferencesDriveMotor::MAX_AMPERAGE);
    driveCurrentLimit.WithStatorCurrentLimitEnable(true);
    driveMotor.GetConfigurator().Apply(driveCurrentLimit);
}

void SwerveModule::setState(frc::SwerveModuleState state)
{
    frc::SwerveModuleState currentState = getState();

    frc::SwerveModuleState optimizedState;

    // Turn off optimization in crater mode to help with configuration.
    optimizedState = state;
    if (!OPTIMIZATION_BYPASS) {
        /**
         * Optimize the target state by flipping motor directions and adjusting
         * rotations in order to turn the least amount of distance possible.
         */
        optimizedState.Optimize(currentState.angle);
    }

    // Cosine Compensation; Don't drive full bore when facing the wrong way
    optimizedState.speed *= (optimizedState.angle - currentState.angle).Cos();

    /**
     * Only handle turning when the robot is actually driving (Stops the modules
     * from snapping back to 0 when the robot comes to a stop).
     */
    // Rotate the swerve module to the desired angle.
    
    if (units::math::abs(optimizedState.speed) > 0.01_mps) {
        setTurningMotor(optimizedState.angle.Radians());
    }
    
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed);
}

void SwerveModule::setTurningMotor(units::radian_t angle)
{
    // Subtract the absolute rotation from the target rotation to get the angle to turn.
    units::radian_t angleDelta(angle - getCANcoderRotation());
    
    /**
     * Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
     * If the value is above π rad or below -π rad...
     */
    if(units::math::abs(angleDelta).value() > std::numbers::pi) {
        const int sign = std::signbit(angleDelta.value()) ? -1 : 1;
        
        // Subtract 2π rad, or add 2π rad depending on the sign.
        angleDelta = units::radian_t(angleDelta.value() - (2 * std::numbers::pi) * sign);
    }
    
    units::turn_t output = angleDelta;
    
    // Add the current relative rotation to get the position to reference.
    output += getTurningMotorPosition();

    // Set the PID reference to the desired position.
    turningMotor.SetControl(turnRequest.WithPosition(output));
}

void SwerveModule::setDriveMotor(units::meters_per_second_t velocity)
{
    const units::turns_per_second_t tps = units::turns_per_second_t(velocity.value() * PreferencesDriveMotor::METERS_TO_TURNS);
    // driveRequest.Acceleration = units::turns_per_second_squared_t((1 - accelReduction) * DRIVE_PREFERENCES.MAX_ACCEL.value() * SWERVE_PREFERENCE.DRIVE_MOTOR.METERS_TO_TURNS);
    driveRequest.WithAcceleration(units::turns_per_second_squared_t((1 - accelReduction) *  DrivePreferences::MAX_ACCEL.value() * PreferencesDriveMotor::METERS_TO_TURNS));
    driveMotor.SetControl(driveRequest.WithVelocity(tps));
}

void SwerveModule::setAccelerationReduction(double reduction) {
    accelReduction = reduction;
};

void SwerveModule::setDriveMotorsNeutralMode(ctre::phoenix6::signals::NeutralModeValue neutralMode) {
    ctre::phoenix6::configs::MotorOutputConfigs driveMotorOutput {};
    driveMotorOutput.WithNeutralMode(neutralMode);
    driveMotor.GetConfigurator().Apply(driveMotorOutput);
}

void SwerveModule::stop() {
    turningMotor.Set(0.0);
    driveMotor.Set(0.0);

    zeroDriveEncoder();
}

units::turn_t SwerveModule::getTurningMotorPosition()
{
    return turningMotor.GetPosition().GetValue();
}

frc::SwerveModuleState SwerveModule::getState()                                                              
{
    // The velocity and rotation of the swerve module.
    return { getDriveVelocity(), getCANcoderRotation() };
}

frc::SwerveModulePosition SwerveModule::getPosition()
{
    // The velocity and rotation of the swerve module.
    return { getDrivePosition(), getCANcoderRotation() };
}

void SwerveModule::zeroDriveEncoder()
{
    driveMotor.SetPosition(0_tr);
}

units::radian_t SwerveModule::getRawCANcoderRotation()
{
    return canCoder.GetAbsolutePosition().GetValue();
}

units::radian_t SwerveModule::getCANcoderRotation()
{
    units::radian_t rawRotation = getRawCANcoderRotation();
    return rawRotation - absEncoderOffset;
}

units::meters_per_second_t SwerveModule::getDriveVelocity()
{
    return units::meters_per_second_t(driveMotor.GetVelocity().GetValue().value() * PreferencesDriveMotor::TURNS_TO_METERS);
}

units::meter_t SwerveModule::getDrivePosition()
{
    return units::meter_t(driveMotor.GetPosition().GetValueAsDouble() * PreferencesDriveMotor::TURNS_TO_METERS);
}

void SwerveModule::sendDebugInfo(std::size_t moduleIndex)
{
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_RawCANcoderRotation_deg",      moduleIndex), units::degree_t(getRawCANcoderRotation()).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_CANcoderRotation_deg",         moduleIndex), units::degree_t(getCANcoderRotation()).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderOffset_deg",            moduleIndex), units::degree_t(absEncoderOffset).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderOffset_rad",            moduleIndex), units::radian_t(absEncoderOffset).value());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_DriveMotor_turns",             moduleIndex), driveMotor.GetPosition().GetValue().value());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_TurnMotorEncoderRotation_deg", moduleIndex), units::degree_t(getTurningMotorPosition()).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_DriveMotorEncoderDistance_m",  moduleIndex), getDrivePosition().value());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_DriveVelocity_mps",            moduleIndex), getDriveVelocity().value());
    
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_TurningMotorTempC",            moduleIndex), turningMotor.GetDeviceTemp().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_DriveMotorTemp_C",             moduleIndex), driveMotor.GetDeviceTemp().GetValueAsDouble());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_TurningProcessorTemp_C",       moduleIndex), turningMotor.GetProcessorTemp().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_DriveProcessorTemp_C",         moduleIndex), driveMotor.GetProcessorTemp().GetValueAsDouble());
    
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_CurrentDrive_A",               moduleIndex), driveMotor.GetSupplyCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_CurrentTurning_A",             moduleIndex), turningMotor.GetSupplyCurrent().GetValueAsDouble());
    
    // Hi Jeff!
}