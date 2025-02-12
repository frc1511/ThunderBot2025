#include "Hang.h"

Hang::Hang()
: encoder(motor.GetEncoder()) {}

void Hang::process() {
    double speed = 0;
    switch (currentMode) {
    case ControlMode::STOPPED:
        setMotorSpeed(0);
        return;
    case ControlMode::GOING_DOWN:
        if (!isHung() && solenoidState == SolenoidState::UP) {
            speed = HANG_PREFERENCE.MAX_HANG_SPEED_DOWN;
        }
    case ControlMode::GOING_UP:
        if (encoder.GetPosition() < HANG_PREFERENCE.MAX_POSITION && solenoidState == SolenoidState::UP) {
            speed = HANG_PREFERENCE.MAX_HANG_SPEED_UP;
        }
        break;
    default:
        break;
    }

    if (solenoidState == SolenoidState::DOWN) {
        solenoidState = SolenoidState::DISENGAGING;
    }
    if (solenoidState == SolenoidState::DISENGAGING) {
        speed = HANG_PREFERENCE.BACKTRACKING_SPEED;
    }

    setMotorSpeed(speed);
}

bool Hang::isHung() {
    return hangHungSensor.Get();
}

bool Hang::isSolenoidUp() {
    return solenoidUpSensor.Get();
}

void Hang::setMotorSpeed(double speed) {
    motor.Set(speed);
}