#include "Elevator.h" 

void Elevator::process() {
    runMotorsToPreset();
}

void Elevator::doPersistentConfiguration() {
    rev::spark::SparkMaxConfig motorConfig {};

    motorConfig.Inverted(false);
    rightSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    motorConfig.Inverted(true);
    leftSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Elevator::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Elevator Left Motor Position (rotations)",  leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber ("Elevator Left Motor Tempature C",           leftSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Position (rotations)", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Tempature C",          rightSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutNumber ("Elevator Target Position (rotations)",      ElevatorPosition[targetPreset].value());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit tripping",             getLowerLimitSwitch());
    frc::SmartDashboard::PutBoolean("Elevator Upper Limit tripping",             getUpperLimitSwitch());
}

bool Elevator::atMaxHeight() {
    return getUpperLimitSwitch();
}

bool Elevator::atMinHeight() {
    return getLowerLimitSwitch();
}

bool Elevator::getLowerLimitSwitch() {
    return !lowerLimitSwitch.Get();
}

bool Elevator::getUpperLimitSwitch() {
    return !upperLimitSwitch.Get();
}

double Elevator::getPosition() {
    return (leftEncoder.GetPosition() + rightEncoder.GetPosition()) / 2;
}

void Elevator::goToPreset(ElevatorPreset target) {
    targetPreset = target;
}

void Elevator::runMotorsToPreset() {
    if (targetPreset == ElevatorPreset::kSTOP) {
        rightSparkMax.Set(0);
        leftSparkMax.Set(0);
        return;
    }

    units::turn_t position = ElevatorPosition[targetPreset];
    double PIDOutput = PIDController.Calculate((units::turn_t)getPosition(), position);
    
    if (atMinHeight() && PIDOutput < 0)
        PIDOutput = 0;
    if (atMaxHeight() && PIDOutput > 0)
        PIDOutput = 0;

    rightSparkMax.Set(PIDOutput);
    leftSparkMax.Set(PIDOutput);
}

// Mason spread the love on 1/28/25 at 8:19:43 >:)
// This is false (mason)