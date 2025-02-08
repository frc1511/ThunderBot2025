#include "Hang.h"
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Preferences.h"

Hang::Hang()
{

}

Hang::~Hang()
{

}

void Hang::doPersistentConfiguration() {
    rev::spark::SparkMaxConfig motorConfig {};
    
    motorConfig.Inverted(false);
    motorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    motor.Configure(motorConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Hang::process() {
    switch (MotorState) {
        case BACKTRACKING:
            if (isPawlOpen()) {
                backtrack();
            } else {
                MotorState = motorState::IDLE;
            }
            break;
        case AWAITING_CHECK:
            motor.Set(0);
            if (backtrackingCheckTimer.Get() >= 0.3_s) {
                if (atMaxHeight()) {
                    if (isPawlOpen()) {
                        backtrackingCheckTimer.Stop();
                        MotorState = motorState::BACKTRACKING;
                        targetEncoderRotation = getMotorPosition() + BACKTRACK_ROTATIONS;
                    } else {
                        backtrackingCheckTimer.Stop();
                        MotorState = motorState::IDLE;
                    }
                } else {
                    backtrackingCheckTimer.Stop();
                    MotorState = motorState::IDLE;
                }
            }
            break;
        case IDLE:
            motor.Set(0);
            if (atMaxRotations() && isPawlOpen()) {
                backtrackingCheckTimer.Restart();
                MotorState = motorState::AWAITING_CHECK;
            }
            break;
        case MOVING_FORWARD:
            if (atMaxRotations() && !isPawlOpen() && getMotorPosition() >= -PREFERENCE_HANG.MAX_POSTION) {
                motor.Set(-PREFERENCE_CONTROLS.MAX_HANG_UP_SPEED);
            } else {
                motor.Set(0);
            }
            break;
        case MOVING_BACKWARD:
            if (!isReflectiveSensorTripped()) {
                motor.Set(PREFERENCE_CONTROLS.MAX_HANG_DOWN_SPEED);
            } else {
                encoder.SetPosition(0);
                motor.Set(0);
            }
            break;
    }
}

void Hang::backtrack() {
    if (getMotorPosition() >= targetEncoderRotation) {
        MotorState = motorState::AWAITING_CHECK;
        backtrackingCheckTimer.Restart();
        setSolenoid(true);
    } else {
        motor.Set(-0.1);
    }
}

void Hang::setMotorState(motorState state) { // Motor is reversed
    MotorState = state;
}

void Hang::setMotorStateSafe(motorState state) {
    if (MotorState != motorState::AWAITING_CHECK && MotorState != motorState::BACKTRACKING) {
        setMotorState(state);
    }
}

void Hang::setSolenoid(bool onOff) {
    SolenoidStates currentState = getSolenoidState();
    SolenoidStates nextState = SolenoidStates::OFF;
    if (onOff) {
        if (currentState == SolenoidStates::ON) {
            nextState = SolenoidStates::ON;
        } else {
            nextState = SolenoidStates::OFF;
        }
    }
}

void Hang::setMotorSpeed(double speed) {
    if (isRelayOn() && isPawlOpen()) {
        if (!atMaxPosition()) {
            motor.Set(speed);
        }
        else if (speed > 0) {
            motor.Set(speed);
        } 
        else {
            motor.Set(0);
        }
    } else {
        motor.Set(0);
    }
}

double Hang::getMotorPosition()
{
    encoderPosition = encoder.GetPosition();
    return encoderPosition;
}

bool Hang::atMaxPosition() // for deep hang
{
    getMotorPosition() >= maxEncoderRotations;
}

bool Hang::atMinPosition() // for deep hang
{
    getMotorPosition() <= minEncoderRotations;
}

bool Hang::atMaxHeight() // for shallow
{

}

bool Hang::atMinHeight() // for shallow
{

}

// std::string Hang::getMotorModeString() {
//     std::string motorMode = "Coast";
//     if (motor.GetIdleMode() == rev::spark::SparkBaseConfig::IdleMode::kCoast) {
//         motorMode = "Brake";
//     }
//     return motorMode;
// }

void Hang::sendFeedback() {
    frc::SmartDashboard::PutNumber("Hang__Position", getMotorPosition());
    frc::SmartDashboard::PutNumber("Hang_MotorTempC", motor.GetMotorTemperature());
    frc::SmartDashboard::PutString("Hang_motorMode", getMotorModeString());
    frc::SmartDashboard::PutString("Hang_MotorState", getMotorStateString(MotorState));
    frc::SmartDashboard::PutNumber("Hang__Timer", backtrackingCheckTimer.Get().value());
}

void Hang::setSolenoids(Hang::SolenoidStates state) {
    solenoidRelay.Set((frc::Relay::Value)state);
}

std::string Hang::getMotorStateString(motorState state) {
    std::string backtrackStateString = "IDLE";
    switch (state)
    {
    case motorState::AWAITING_CHECK:
        backtrackStateString = "Awaiting Check";
        break;
    case motorState::BACKTRACKING:
        backtrackStateString = "Backtracking";
        break;
    case motorState::MOVING_FORWARD:
        backtrackStateString = "Moving Up";
        break;
    case motorState::MOVING_BACKWARD:
        backtrackStateString = "Moving Down";
        break;
    default:
        break;
    }
    return backtrackStateString;
}

Hang::SolenoidStates Hang::getSolenoidState() {
    return (SolenoidStates)solenoidRelay.Get();
}

std::string Hang::getSolenoidStateString() {
    std::string solenoidState = "Off";
    switch (getSolenoidState())
    {
    case SolenoidStates::ON:
        solenoidState = "ON";
        break;
    default:
        break;
    }
    return solenoidState;
}

bool Hang::isRelayOn() {
    SolenoidStates state = (SolenoidStates)solenoidRelay.Get();
    return (state == SolenoidStates::ON);
}

bool Hang::isReflectiveSensorTripped(){
    return !reflectiveSensor.Get();
}

bool Hang::isPawlOpen(){
    return beamBreakSensor.Get(); // Pawl will be open when true
}

std::string Hang::ConvertTemperatureToString(double temp_c) {
    double temp_f = temp_c * 1.8 + 32;
    std::string temperature = std::to_string(temp_c) + "C " + std::to_string(temp_f) + "F";
    return temperature;
}