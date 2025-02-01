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
    frc::SmartDashboard::PutNumber ("Elevator Target Position (rotations)",      Position[targetPreset].value());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit tripping",             getLowerLimit());
    frc::SmartDashboard::PutBoolean("Elevator Upper Limit tripping",             getUpperLimit());
}

void Elevator::incrementPositionIndex() {
    tempPresetIndex = std::clamp(tempPresetIndex+1, int(Preset::kSTOP), int(Preset::kMAX)-1);
}

void Elevator::decrementPositionIndex() {
    tempPresetIndex = std::clamp(tempPresetIndex-1, int(Preset::kSTOP), int(Preset::kMAX)-1);
}

void Elevator::updateCurrentPreset() {
    goToPreset(Preset(tempPresetIndex));
}

bool Elevator::atMaxHeight() {
    return getUpperLimit();
}

bool Elevator::atMinHeight() {
    return getLowerLimit();
}

bool Elevator::getLowerLimit() {
    return !lowerLimitSwitch.Get();
}

bool Elevator::getUpperLimit() {
    return !upperLimitSwitch.Get();
}

double Elevator::getPosition() {
    return (leftEncoder.GetPosition() + rightEncoder.GetPosition()) / 2;
}

void Elevator::goToPreset(Preset target) {
    targetPreset = target;
}

void Elevator::runMotorsToPreset() {
    if (targetPreset == Preset::kSTOP) {
        rightSparkMax.Set(0);
        leftSparkMax.Set(0);
        return;
    }

    units::turn_t position = Position[targetPreset];
    double PIDOutput = PIDController.Calculate((units::turn_t)getPosition(), position);
    
    if (atMinHeight() && PIDOutput < 0)
        PIDOutput = 0;
    if (atMaxHeight() && PIDOutput > 0)
        PIDOutput = 0;

    PIDOutput = std::clamp(PIDOutput, -ELEVATOR_PREFERENCE.MAX_SPEED, ELEVATOR_PREFERENCE.MAX_SPEED);
    
    rightSparkMax.Set(PIDOutput);
    leftSparkMax.Set(PIDOutput);
}

// Mason spread the love on 1/28/25 at 8:19:43 >:)
// This is false (mason)