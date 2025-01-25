#include "Elevator.h" 

void Elevator::process() {
    runMotorsToPreset();
}

void Elevator::doPersistentConfiguration() {
    rev::spark::SparkMaxConfig motorConfig {};
    motorConfig.closedLoop.Pid(ELEVATOR_PREFERENCE.PID.Kp, ELEVATOR_PREFERENCE.PID.Ki, ELEVATOR_PREFERENCE.PID.Kd);

    rightSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    motorConfig.Inverted(true);
    leftSparkMax.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
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
    return ((leftEncoder.GetPosition() + rightEncoder.GetPosition()) / 2);
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

    double position = ElevatorPosition[targetPreset];

    rightPIDController.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
    leftPIDController.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
}