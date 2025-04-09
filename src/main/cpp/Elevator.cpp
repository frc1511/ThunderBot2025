#include "Elevator.h"

Elevator::Elevator() {
    doConfiguration(false);

    pidController.DisableContinuousInput();
}

void Elevator::process() {
    units::volt_t motorOutput = 0_V;

    if (!sensorBroken) {
        if (atMinHeight()) { // if elevator at the lower limit switch
            zeroMotors(); // zero the encoders

            encoderZeroed = true;
        }
    } else if (!encoderZeroed && sensorBroken) {
        encoderZeroed = true;
    }


    if (!encoderZeroed && !sensorBroken) { // if elevator not at the lower limit switch
        motorOutput = -1_V; // move down slowly until at the lower limit switch
    } else if (manualControl) {
        motorOutput = manualMovementSpeed * 9_V;
    } else {
        motorOutput = computeSpeedForPreset();
    }

    if (atMinHeight() && motorOutput < 0_V) // stop moving when at either limit switch
        motorOutput = 0_V;

    if (atMaxHeight() && motorOutput > 0_V)
        motorOutput = 0_V;

    if (wristExists)
        if (wristIsUnsafe)
            motorOutput = 0_V;

    if (settings.pitMode && isDisabled) {
        motorOutput = 0_V;
    }

    units::volt_t voltage = motorOutput;

    leftSparkMax.SetVoltage(voltage);
    rightSparkMax.SetVoltage(voltage);

    // motorOutput += 0.05; // Temp Feedfoward
    // // motorOutput += 0.02; // No calgae feedfoward

    // frc::SmartDashboard::PutNumber ("Elevator Motor Output", motorOutput);

    // motorOutput = std::clamp(motorOutput, -PreferencesElevator::MAX_DOWN_VOLTS, PreferencesElevator::MAX_UP_VOLTS);
    // if (settings.pitMode)
    //     motorOutput = std::clamp(motorOutput, -PreferencesElevator::MAX_DOWN_PIT_SPEED, PreferencesElevator::MAX_UP_PIT_SPEED);


    // rightSparkMax.Set(motorOutput);
    // leftSparkMax.Set(motorOutput);
}

void Elevator::resetToMatchMode(MatchMode priorMode, MatchMode mode) { //resets motor config
    targetPreset = kSTOP;
    manualControl = false;
    manualMovementSpeed = 0;
    leftSparkMax.Set(0);
    rightSparkMax.Set(0);
    pidController.Reset(getPosition());
}

void Elevator::doConfiguration(bool persist) {
    rev::spark::SparkMaxConfig motorConfig {};

    motorConfig.Inverted(false);
    motorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    rightSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, persist ? rev::spark::SparkBase::PersistMode::kPersistParameters : rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    motorConfig.Inverted(true);
    leftSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, persist ? rev::spark::SparkBase::PersistMode::kPersistParameters : rev::spark::SparkBase::PersistMode::kNoPersistParameters);
}

void Elevator::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Elevator Position (rotations)",             getPosition().value());
    frc::SmartDashboard::PutNumber ("Elevator Left Motor Position (rotations)",  leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber ("Elevator Left Motor Current",               leftSparkMax.GetOutputCurrent());
    frc::SmartDashboard::PutNumber ("Elevator Left Motor Temperature C",         leftSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Position (rotations)", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Temperature C",        rightSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Current",              rightSparkMax.GetOutputCurrent());
    frc::SmartDashboard::PutBoolean("Elevator At Target Preset",                 atPreset());
    frc::SmartDashboard::PutNumber ("Elevator Target Position (rotations)",      Position[targetPreset].value());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit tripping",             getLowerLimit());
    frc::SmartDashboard::PutBoolean("Elevator Upper Limit tripping",             getUpperLimit());
    frc::SmartDashboard::PutNumber ("Elevator Manual Movement Speed",            manualMovementSpeed);
    frc::SmartDashboard::PutBoolean("Elevator Zeroed",                           encoderZeroed);

    frc::SmartDashboard::PutData("Elevator PID", &pidController); 
    
    //! REMOVE ME IN THE FUTURE, THIS BETTER NOT BE HERE AT BUCKEYE
    // These should be temporary
    // double kS = frc::SmartDashboard::GetNumber("Elevator kS", PreferencesElevator::PID.Ks);
    // double kG = frc::SmartDashboard::GetNumber("Elevator kG", PreferencesElevator::PID.Kg);
    // double kV = frc::SmartDashboard::GetNumber("Elevator kV", PreferencesElevator::PID.Kv_EVFF);
    // double kA = frc::SmartDashboard::GetNumber("Elevator kA", PreferencesElevator::PID.Ka_EVFF);
    // frc::SmartDashboard::PutNumber("Elevator kS", kS);
    // frc::SmartDashboard::PutNumber("Elevator kG", kG);
    // frc::SmartDashboard::PutNumber("Elevator kV", kV);
    // frc::SmartDashboard::PutNumber("Elevator kA", kA);
    // if (ffController.GetKv() != (units::unit_t<frc::ElevatorFeedforward::kv_unit>)kV || 
    //     ffController.GetKa() != (units::unit_t<frc::ElevatorFeedforward::ka_unit>)kA) { // If Updated
    //     pidController.Reset(getPosition());
    //     pidController.SetGoal(10_tr);
    // }
    // ffController.SetKs((units::volt_t)kS);
    // ffController.SetKg((units::volt_t)kG);
    // ffController.SetKv((units::unit_t<frc::ElevatorFeedforward::kv_unit>)kV);
    // ffController.SetKa((units::unit_t<frc::ElevatorFeedforward::ka_unit>)kA);
}

bool Elevator::atMaxHeight() {
    if (!sensorBroken) {
        return getUpperLimit();
    } else {
        if (getPosition() > Position[kNET]-(units::turn_t)PreferencesElevator::TARGET_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }
}

bool Elevator::atMinHeight() {
    if (!sensorBroken) {
        return getLowerLimit();
    } else {
        if (getPosition() < (units::turn_t)PreferencesElevator::TARGET_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }
}

bool Elevator::getLowerLimit() {
    return !lowerLimitSwitch.Get();
}

bool Elevator::getUpperLimit() {
    return !upperLimitSwitch.Get();
}

double Elevator::getPercentHeight() {
    double percentHeight = getPosition() / Position[Preset::kNET];

    return percentHeight;
}

Elevator::Preset Elevator::getCurrentPreset() {
    return targetPreset;
}

units::turn_t Elevator::getPosition() {
    return units::turn_t((leftEncoder.GetPosition() + rightEncoder.GetPosition()) / 2);
}

void Elevator::goToPreset(Preset target) {
    if (targetPreset != target) { // If we have a new preset
        pidController.Reset(getPosition());
        pidController.SetGoal(Position[target]);
        startDownPosition = getPosition().value();
    }

    targetPreset = target;
    manualControl = false;
}

void Elevator::updateLineupTargetVariable(std::optional<lineup_t> newLineup) {
    if (lineupTarget != std::nullopt) {
        if (lineupTarget != newLineup) {
            if (newLineup != std::nullopt) {
                if (ELEVATOR_BRANCH_OFFSETS.count(newLineup.value()) == 1) {
                    auto val = ELEVATOR_BRANCH_OFFSETS.find(newLineup.value());
                    auto branchOffsets = val->second;

                    Branch yayBranch = Branch::kL2;
                    if (targetPreset == Preset::kL2) {
                        // Do Nothing, by not throwing the offset
                    } else if (targetPreset == Preset::kL3) {
                        yayBranch = Branch::kL3;
                    } else if (targetPreset == Preset::kL4) {
                        yayBranch = Branch::kL4;
                    } else {
                        // Not going to a Branch Position
                        lineupTarget = newLineup;
                        return;
                    }

                    if (branchOffsets.count(yayBranch) == 1) {
                        units::turn_t target = branchOffsets.find(yayBranch)->second;
                        pidController.SetGoal(target);
                    }
                }
            }
        }
    }
    lineupTarget = newLineup;
}

bool Elevator::atPreset() { //detects if at preset
    if(manualControl || targetPreset == kSTOP) // if in manual control or stopped we are always at our preset
        return true;

    if (!encoderZeroed) // if we are at the bottom we are not at our preset
        return false;

    if (fabs(getPosition().value() - Position[targetPreset].value()) < PreferencesElevator::TARGET_TOLERANCE) { // If the diff from our preset is less than our tol, we at the preset
        return true;
    }
    // if we aren't at our preset, we aren't at our preset

    return false;
}

void Elevator::manualMovement(double speed) { // allows input of speed and turns on manual movement
    manualMovementSpeed = std::clamp(speed, -1.0, 1.0 );
    manualControl = true;
}

void Elevator::setSensorBroken(bool isBroken) {
    sensorBroken = isBroken;
}

units::volt_t Elevator::computeSpeedForPreset() {
    if (targetPreset == Preset::kSTOP) {
        return 0_V;
    }

    units::turn_t targetPosition = Position[targetPreset];

    units::volt_t pidVolts = (units::volt_t)pidController.Calculate(getPosition());
    units::meters_per_second_t vel = (units::meters_per_second_t)(double)pidController.GetSetpoint().velocity;
    units::volt_t ffVolts = ffController.Calculate(vel);
    frc::SmartDashboard::PutNumber("Elevator FF", ffVolts.value());
    frc::SmartDashboard::PutNumber("Elevator PID", pidVolts.value());
    frc::SmartDashboard::PutNumber("Elevator Profiled Setpoint Pos", pidController.GetSetpoint().position.value());
    frc::SmartDashboard::PutNumber("Elevator Profiled Setpoint Vel", pidController.GetSetpoint().velocity.value());
    units::volt_t output = pidVolts + ffVolts;
    frc::SmartDashboard::PutNumber("Elevator FF+PID Volts Out", output.value());
    output = std::clamp(output, PreferencesElevator::MAX_DOWN_VOLTS, PreferencesElevator::MAX_UP_VOLTS);

    // units::turn_t difference = targetPosition - getPosition();

    // if (fabs(difference) < PreferencesElevator::TARGET_TOLERANCE) {
    //     return 0;
    // }

    // bool isDirectionUp = difference > 0_tr;

    // double speedFactorUp = std::clamp(fabs(difference.value()) * 0.1, 0.1, 1.0);

    // if (isDirectionUp) {
    //     return PreferencesElevator::MAX_UP_VOLTS * speedFactorUp;
    // }

    // double diffFromStart = startDownPosition - getPosition().value();

    // double speedFactorDown = std::clamp(fabs(diffFromStart) * 0.2, 0.2, 1.0);

    // speedFactorDown *= std::clamp(fabs(difference.value()) * 0.1, 0.2, 1.0);

    return output;
}

void Elevator::zeroMotors() {
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
}

// Mason spread the love on 1/28/25 at 8:19:43 >:)
// This is false (mason)