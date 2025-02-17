#pragma once

#include "Basic/IOMap.h"
#include "Preferences.h"
#include "Basic/Component.h"

#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/Relay.h>
#include <frc/Timer.h>

class Hang : public Component {
  public:
    Hang();
    void doPersistentConfiguration();

    void process();

    bool isHung();
    bool isSolenoidUp();

    enum class ControlMode {
        STOPPED,
        GOING_UP,
        GOING_DOWN
    };

    enum class SolenoidState {
        UP,
        DOWN,
    };

    enum class SolenoidAction {
        NONE,
        KEEP_UP,
        DISENGAGING,
        CHECKING_UP_STATE
    };

  void setControlMode(ControlMode controlMode);

  private:
    void updateRealSolenoidState();
    void setSolenoidState(SolenoidState state);
    double getMotorPosition();
    
    ControlMode currentMode = ControlMode::STOPPED;
    
    SolenoidAction solenoidAction = SolenoidAction::NONE;
    SolenoidState realSolenoidState = SolenoidState::DOWN;
    SolenoidState desiredSolenoidState = SolenoidState::DOWN;

    frc::Relay relay {RELAY_HANG, frc::Relay::Direction::kBothDirections};

    rev::spark::SparkMax motor {CAN_HANG, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder encoder {motor.GetEncoder()};
    frc::DigitalInput solenoidUpSensor {DIO_HANG_SOLENOID_UP};
    frc::DigitalInput hangHungSensor {DIO_HANG_HUNG};

    double backtrackingStart = 0.0;

    frc::Timer disengageTimer;

    void setMotorSpeed(double speed);
};