#pragma once

#include <Basic/IOMap.h>
#include "Preferences.h"

#include <frc/DigitalInput.h>
#include <frc/Relay.h>
#include <frc/Timer.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include "Basic/Component.h"
#include <frc/DigitalInput.h>
#include <frc/Relay.h>


#define BACKTRACK_ROTATIONS -2.072

class Hang : public Component {
    public:
    Hang();
    ~Hang();
    void doPersistentConfiguration();

    void reset();
    void process();

    enum motorState {
        IDLE,
        BACKTRACKING,
        AWAITING_CHECK,
        MOVING_FORWARD,
        MOVING_BACKWARD
    };

    /**
     *  Moves the mechanism to the target position 
     */
    void setSolenoidOnOff(bool onOff);

    void setMotorState(motorState state);
    void setMotorStateSafe(motorState state);

    void setMotorSpeed(double speed);

    double getMotorPosition();

    bool atMaxPosition();
    bool atMinPosition();

    bool atMaxHeight();
    bool atMinHeight();

    bool isRelayOn();

    bool isReflectiveSensorTripped();
    bool isPawlOpen();

    void sendFeedback() override;

    enum SolenoidStates {
        ON = frc::Relay::Value::kOn,
        OFF = frc::Relay::Value::kOff,
    };

    void setSolenoidState(Hang::SolenoidStates state);

    std::string getSolenoidStateString();

    // std::string getMotorModeString();

    std::string ConvertTemperatureToString(double temp);


    motorState MotorState = motorState::IDLE;

private:
    rev::spark::SparkMax motor {1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder encoder = motor.GetEncoder();

    double encoderPosition;
    double maxEncoderRotations;
    double minEncoderRotations;

    double targetEncoderRotation = 0;

    frc::Relay solenoidRelay{0, frc::Relay::kBothDirections};

    frc::DigitalInput reflectiveSensor; // for shallow hang
    frc::DigitalInput beamBreakSensor; // for deep hang, beam break

    frc::Timer backtrackingCheckTimer;

    void backtrack();

    std::string getMotorStateString(motorState state);
    SolenoidStates getSolenoidState();
    void configureMotor();
};