#pragma once

#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/Relay.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Libraries/elasticlib.h"

#include "Basic/IOMap.h"
#include "Preferences.h"
#include "Basic/Component.h"

class Hang : public Component {
  public:
    Hang();
    void doConfiguration(bool persist);

    void process();

    void sendFeedback();

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

    bool fastyFast = false;

    bool deepHang = true;

  private:
    void updateRealSolenoidState();
    void setSolenoidState(SolenoidState state);
    double getMotorPosition();

    std::string currentModeAsString();
    std::string currentSolenoidActionAsString();

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

    double speed = 0;

    frc::Timer disengageTimer;

    void setMotorSpeed(double speed);

    bool hasDeployed = false;

    friend class Robot;
};