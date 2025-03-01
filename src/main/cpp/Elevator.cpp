#include "Elevator.h" 
Elevator::Elevator(Wrist *wrist_)
: wrist(wrist_) {
    doConfiguration(false);
}

void Elevator::process() {
    double motorSpeed = 0;

    if (atMinHeight()) { // if elevator at the lower limit switch
        leftEncoder.SetPosition(0); // zero the encoders
        rightEncoder.SetPosition(0);

        encoderZeroed = true;
    }

    if (!encoderZeroed) { // if elevator not at the lower limit switch
        motorSpeed = -0.03; // move down slowly until at the lower limit switch
    } else if (manualControl) {
        motorSpeed = manualMovementSpeed;
    } else {
        motorSpeed = computeSpeedForPreset();
    }

    if (atMinHeight() && motorSpeed < 0) // stop moving when at either limit switch
        motorSpeed = 0;

    if (atMaxHeight() && motorSpeed > 0)
        motorSpeed = 0;

    if (wrist != nullptr)
        if (wrist->wristIsUnsafe())
            motorSpeed = 0;

    if (settings.pitMode && isDisabled) {
        motorSpeed = 0;
    }

    motorSpeed += 0.05; // Temp Feedfoward

    frc::SmartDashboard::PutNumber ("Elevator Motor Output", motorSpeed);

    motorSpeed = std::clamp(motorSpeed, -PreferencesElevator::MAX_DOWN_SPEED, PreferencesElevator::MAX_UP_SPEED);
    if (settings.pitMode)
        motorSpeed = std::clamp(motorSpeed, -PreferencesElevator::MAX_DOWN_PIT_SPEED, PreferencesElevator::MAX_UP_PIT_SPEED);

    rightSparkMax.Set(motorSpeed);
    leftSparkMax.Set(motorSpeed);
}

void Elevator::resetToMatchMode(MatchMode priorMode, MatchMode mode) { //resets motor config
    targetPreset = kSTOP;
    manualControl = false;
    manualMovementSpeed = 0;
    leftSparkMax.Set(0);
    rightSparkMax.Set(0);
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
        startDownPosition = getPosition().value();
    }

    targetPreset = target;
    manualControl = false;
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

void Elevator::setSensorBroken(bool isBroken) { // TODO: this still needs to be implemented
    sensorBroken = isBroken;
}

double Elevator::computeSpeedForPreset() { 
    if (targetPreset == Preset::kSTOP) {
        return 0;
    }

    units::turn_t targetPosition = Position[targetPreset];
    units::turn_t difference = targetPosition - getPosition();

    if (fabs(difference.value()) < PreferencesElevator::TARGET_TOLERANCE) {
        return 0;
    }

    bool isDirectionUp = difference > 0_tr;

    double speedFactorUp = std::clamp(fabs(difference.value()) * 0.1, 0.1, 1.0);

    if (isDirectionUp) {
        return PreferencesElevator::MAX_UP_SPEED * speedFactorUp;
    }

    double diffFromStart = startDownPosition - getPosition().value();

    double speedFactorDown = std::clamp(fabs(diffFromStart) * 0.2, 0.2, 1.0);

    speedFactorDown *= std::clamp(fabs(difference.value()) * 0.1, 0.2, 1.0);

    return -PreferencesElevator::MAX_DOWN_SPEED * speedFactorDown;
}

// Mason spread the love on 1/28/25 at 8:19:43 >:)
// This is false (mason)