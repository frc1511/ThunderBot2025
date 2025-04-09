#pragma once

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

class Elevator : public Component {
  public:
    Elevator();
    void process() override;

    void resetToMatchMode(MatchMode priorMode, MatchMode mode) override;
    void doConfiguration(bool persist) override;
    void sendFeedback() override;

    bool getLowerLimit();
    bool getUpperLimit();

    enum Preset {
        kSTOP,
        kGROUND, 
        kPROCESSOR,
        kCORAL_STATION,
        kL1,
        kL2,
        kL3,
        kL4,
        kNET,
        kTRANSIT,
        kREEF_LOW,
        kREEF_HIGH,
        kCORAL_STATION_LOW,
        _enum_MAX,
    };

    void goToPreset(Preset preset);

    // If manual this returns true
    bool atPreset();

    void manualMovement(double speed);

    bool isDisabled = false;

    double getPercentHeight();

    Preset getCurrentPreset();

    void zeroMotors();

    void setSensorBroken(bool isBroken);

  private:
    bool atMaxHeight();
    bool atMinHeight();

    units::turn_t Position[Preset::_enum_MAX] {
        0_tr,     // Stopped (Does not move to 0 turns)
        0_tr,     // Ground
        0_tr,     // Processor
        14_tr,    // Coral Station
        8_tr,     // L1
        10_tr,    // L2
        25.35_tr, // L3
        58_tr,    // L4
        66.1_tr,  // Net
        1_tr,     // Transit
        11.16_tr, // Reef Low
        25.65_tr, // Reef High
        10.53_tr,  // Coral Station Low
    };

    Preset targetPreset = Elevator::Preset::kSTOP;
    double manualMovementSpeed = 0;
    bool manualControl = false;
    
    bool encoderZeroed = false;
    bool sensorBroken = false;

    units::turn_t getPosition();
    units::volt_t computeSpeedForPreset();

    rev::spark::SparkMax leftSparkMax {CAN_LEFT_ELEVATOR, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax rightSparkMax {CAN_RIGHT_ELEVATOR, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkRelativeEncoder leftEncoder = leftSparkMax.GetEncoder();
    rev::spark::SparkRelativeEncoder rightEncoder = rightSparkMax.GetEncoder();

    frc::DigitalInput lowerLimitSwitch {DIO_ELEVATOR_BOTTOM_LIMITSWITCH};
    frc::DigitalInput upperLimitSwitch {DIO_ELEVATOR_TOP_LIMITSWITCH};

    double startDownPosition = 0;

    bool wristExists = false;
    bool wristIsUnsafe = true;

    frc::ElevatorFeedforward ffController {
      (units::volt_t)PreferencesElevator::PID.Ks, (units::volt_t)PreferencesElevator::PID.Kg,
      (units::unit_t<frc::ElevatorFeedforward::kv_unit>)PreferencesElevator::PID.Kv_EVFF, (units::unit_t<frc::ElevatorFeedforward::ka_unit>)PreferencesElevator::PID.Ka_EVFF
    };

    frc::ProfiledPIDController<units::turns> pidController {
      PreferencesElevator::PID.Kp, PreferencesElevator::PID.Ki, PreferencesElevator::PID.Kd,
      frc::TrapezoidProfile<units::turns>::Constraints(PreferencesElevator::MAX_VEL, PreferencesElevator::MAX_ACCEL)
    };

    void updateLineupTargetVariable(std::optional<lineup_t> newLineup);
    std::optional<lineup_t> lineupTarget = std::nullopt;

    friend class Gamepiece;
    friend class Controls;
};