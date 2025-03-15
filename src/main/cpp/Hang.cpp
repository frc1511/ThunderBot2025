#include "Hang.h"

Hang::Hang() {
    doConfiguration(false);
}

void Hang::doConfiguration(bool persist) {
    rev::spark::SparkMaxConfig motorConfig {};

    motorConfig.Inverted(true);
    motorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    motor.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, persist ? rev::spark::SparkBase::PersistMode::kPersistParameters : rev::spark::SparkBase::PersistMode::kNoPersistParameters);
}

void Hang::process() {
    updateRealSolenoidState();

    speed = 0;

    switch (currentMode) {
        case ControlMode::STOPPED:
            setMotorSpeed(0);

            desiredSolenoidState = SolenoidState::DOWN;
            solenoidAction = SolenoidAction::NONE;

            setSolenoidState(desiredSolenoidState);

            return; //* Early Return

        case ControlMode::GOING_DOWN:
            if (!isHung()) {
                // if (fastyFast) {
                speed = PreferencesHang::MAX_HANG_SPEED_DOWN;
                // } else {
                //     speed = PreferencesHang::HANG_SPEED_DOWN_SLOW;
                // }
            }

            desiredSolenoidState = SolenoidState::DOWN;
            solenoidAction = SolenoidAction::NONE;

            break;
        case ControlMode::GOING_UP:
            desiredSolenoidState = SolenoidState::UP;

            if (encoder.GetPosition() < PreferencesHang::MAX_POSITION && realSolenoidState == SolenoidState::UP) {
                speed = PreferencesHang::MAX_HANG_SPEED_UP;
            }

            break;

        default:
            break;
    }

    // At this point we know we want to move up

    SolenoidState solenoidOutput = desiredSolenoidState;

    if (solenoidAction == SolenoidAction::KEEP_UP) {
        solenoidOutput = SolenoidState::UP;
    }

    if (desiredSolenoidState == SolenoidState::UP &&
          realSolenoidState == SolenoidState::DOWN &&
          solenoidAction != SolenoidAction::CHECKING_UP_STATE && 
          solenoidAction != SolenoidAction::DISENGAGING) {
        solenoidAction = SolenoidAction::CHECKING_UP_STATE;
        disengageTimer.Restart();
    }

    if (solenoidAction == SolenoidAction::DISENGAGING) {
        solenoidOutput = SolenoidState::DOWN;
        speed = PreferencesHang::BACKTRACKING_SPEED;
        if (fabs(getMotorPosition() - backtrackingStart) >= PreferencesHang::BACKTRACKING_DISTANCE) {
            solenoidAction = SolenoidAction::CHECKING_UP_STATE;
            disengageTimer.Restart();
        }
    }

    if (solenoidAction == SolenoidAction::CHECKING_UP_STATE) {
        solenoidOutput = SolenoidState::UP;
        disengageTimer.Start();

        if (realSolenoidState == SolenoidState::UP) {
            solenoidAction = SolenoidAction::KEEP_UP;
            disengageTimer.Stop();
        } else if (disengageTimer.Get() > PreferencesHang::DISENGAGE_DURATION) { // We can't go up rn, :(
            solenoidAction = SolenoidAction::DISENGAGING;
            backtrackingStart = getMotorPosition();
            disengageTimer.Stop();
        }

        speed = 0;
    }

    setSolenoidState(solenoidOutput);

    setMotorSpeed(speed);
}

bool Hang::isHung() {
    return hangHungSensor.Get();
}

bool Hang::isSolenoidUp() {
    return solenoidUpSensor.Get();
}

void Hang::setControlMode(ControlMode controlMode) {    
    currentMode = controlMode;
}

void Hang::updateRealSolenoidState() {
    if (isSolenoidUp()) {
        realSolenoidState = SolenoidState::UP;
    } else {
        realSolenoidState = SolenoidState::DOWN;
    }
}

void Hang::setSolenoidState(SolenoidState state) {
    relay.Set(state == SolenoidState::DOWN ? frc::Relay::kOff : frc::Relay::kOn);
}

double Hang::getMotorPosition() {
    return encoder.GetPosition();
}

void Hang::setMotorSpeed(double speed) {
    motor.Set(speed);
}

void Hang::sendFeedback() {
    frc::SmartDashboard::PutString ("Hang Current Mode",    currentModeAsString());
    frc::SmartDashboard::PutString ("Hang Solenoid Action", currentSolenoidActionAsString());
    frc::SmartDashboard::PutNumber ("Hang Current Speed",   speed);
    frc::SmartDashboard::PutBoolean("Hang Hung",            isHung());
    frc::SmartDashboard::PutBoolean("Hang Solenoid Up",     isSolenoidUp());
    frc::SmartDashboard::PutNumber ("Hang Position",        getMotorPosition());
    frc::SmartDashboard::PutBoolean("Hang Fasty Fast",      fastyFast);
}

std::string Hang::currentModeAsString() {
    switch (currentMode) {
        case ControlMode::STOPPED:    return "Stopped";
        case ControlMode::GOING_DOWN: return "Going Down";
        case ControlMode::GOING_UP:   return "Going Up";
        default:                      return "Invalid Mode";
    }
}

std::string Hang::currentSolenoidActionAsString() {
    switch (solenoidAction) {
        case SolenoidAction::NONE:              return "None";
        case SolenoidAction::KEEP_UP:           return "Keep Up";
        case SolenoidAction::DISENGAGING:       return "Disengaging";
        case SolenoidAction::CHECKING_UP_STATE: return "Checking Up State";
        default:                                return "Invalid Mode";
    }
}