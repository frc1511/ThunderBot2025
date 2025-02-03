#include "Elevator.h" 
void Elevator::process() {
    double motorSpeed = 0;
    if (atMinHeight()) { // if elevator at the lower limit switch
        leftEncoder.SetPosition(0); // zero the encoders
        rightEncoder.SetPosition(0);
        encoderZeroed = true;
    }
    if (encoderZeroed == false) { // if elevator not at the lower limit switch
        motorSpeed = -0.05; // move down slowly until at the lower limit switch
    }
    else if(manualControl) {
        motorSpeed = manualMovementSpeed;
    }
    else {
        motorSpeed = computeSpeedForPreset();
    }
    if (atMinHeight() && motorSpeed < 0) // stop moving when at either limit switch
        motorSpeed = 0;

    if (atMaxHeight() && motorSpeed > 0)
        motorSpeed = 0;
    
    motorSpeed = std::clamp(motorSpeed, -ELEVATOR_PREFERENCE.MAX_SPEED, ELEVATOR_PREFERENCE.MAX_SPEED);
    
    rightSparkMax.Set(motorSpeed);
    leftSparkMax.Set(motorSpeed);
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
    //frc::SmartDashboard::PutNumber ("Elevator Left Motor Tempature C",           leftSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutNumber ("Elevator Right Motor Position (rotations)", rightEncoder.GetPosition());
    //frc::SmartDashboard::PutNumber ("Elevator Right Motor Tempature C",          rightSparkMax.GetMotorTemperature());
    frc::SmartDashboard::PutBoolean ("Elevator At Target Preset",      atPreset());
    frc::SmartDashboard::PutNumber ("Elevator Target Position (rotations)",      Position[targetPreset].value());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit tripping",             getLowerLimit());
    frc::SmartDashboard::PutBoolean("Elevator Upper Limit tripping",             getUpperLimit());
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
    manualControl = false;
}

bool Elevator::atPreset() {
    if(manualControl || targetPreset == kSTOP) // if in manual control or stopped we are always at our preset
        return true;
    if (!encoderZeroed) // if we are at the bottom we are not at our preset
        return false;
    if(getPosition() < (Position[targetPreset].value() + targetTolerance) && getPosition() > (Position[targetPreset].value() - targetTolerance)) { // if we are within 1 turn(this can be changed) of the preset, we are at our preset
        return true;
    }
    else { // if we arent at our preset, we arent at our preset
        return false;
    }
}

void Elevator::manualMovement(double speed) {
    manualMovementSpeed = speed;
    manualControl = true;
}

void Elevator::setSensorBroken(bool isBroken) { // this still needs to be implemented
    sensorBroken = isBroken;
}
double Elevator::computeSpeedForPreset() {
    double PIDOutput = 0;
    if (targetPreset == Preset::kSTOP) {
        return PIDOutput;
    }

    units::turn_t position = Position[targetPreset];
    PIDOutput = PIDController.Calculate((units::turn_t)getPosition(), position);
    return PIDOutput;
}

// Mason spread the love on 1/28/25 at 8:19:43 >:)
// This is false (mason)