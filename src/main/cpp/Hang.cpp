#include "Hang.h"

Hang::Hang() {
    doConfiguration(false);
}

void Hang::doConfiguration(bool persist) { }

void Hang::process() {
    updateRealSolenoidState();

    return; //! REMOVE ME TO TEST HANG

    double speed = 0;

    switch (currentMode) {
        case ControlMode::STOPPED:
            setMotorSpeed(0);

            desiredSolenoidState = SolenoidState::DOWN;
            solenoidAction = SolenoidAction::NONE;

            setSolenoidState(desiredSolenoidState);

            return;
        case ControlMode::GOING_DOWN:
            if (!isHung()) {
                speed = PreferencesHang::MAX_HANG_SPEED_DOWN;
            }

            desiredSolenoidState = SolenoidState::DOWN;
            solenoidAction = SolenoidAction::NONE;

            return;
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
        if (getMotorPosition() - backtrackingStart >= PreferencesHang::BACKTRACKING_DISTANCE) {
            solenoidAction = SolenoidAction::CHECKING_UP_STATE;
        }
    }

    if (solenoidAction == SolenoidAction::CHECKING_UP_STATE) {
        solenoidOutput = SolenoidState::UP;
        disengageTimer.Start();

        if (realSolenoidState == SolenoidState::UP && disengageTimer.Get() > PreferencesHang::DISENGAGE_DURATION) {
            solenoidAction = SolenoidAction::KEEP_UP;
            disengageTimer.Stop();
        } else { // We can't go up rn, :(
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
    relay.Set(state == SolenoidState::DOWN ? frc::Relay::kOn : frc::Relay::kOff);
}

double Hang::getMotorPosition() {
    return encoder.GetPosition();
}

void Hang::setMotorSpeed(double speed) {
    motor.Set(speed);
}