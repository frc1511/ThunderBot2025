#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Controls;

class Elevator : public Component {
  public:
    void process() override;

    void resetToMatchMode(MatchMode priorMode, MatchMode mode) override;
    void doPersistentConfiguration() override;
    void sendFeedback() override;

  private:
    bool atMaxHeight();
    bool atMinHeight();
    bool getLowerLimit();
    bool getUpperLimit();
  public:
    enum Preset { //need measurements for the height of these sections, RN we have guesstimates with no units. ~G
        kSTOP,
        kGROUND, 
        kPROCESSOR,
        kCORAL_STATION,
        kL1,
        kL2,
        kL3,
        kL4,
        kNET,
        _enum_MAX,
    };
    void goToPreset(Preset preset);
    // if manual this returns true
    bool atPreset();
    void manualMovement(double speed);
    void setSensorBroken(bool isBroken);

    double getPercentHeight();

    Preset getCurrentPreset();
  private:
    units::turn_t Position[Preset::_enum_MAX] {
        0_tr,   // Stopped (Does not move to 0 turns)
        20_tr,  // Ground
        200_tr, // Processor
        350_tr, // Coral Station
        300_tr, // L1
        400_tr, // L2
        500_tr, // L3
        600_tr, // L4
        600_tr  // Net
    };

    Preset targetPreset = Elevator::Preset::kSTOP;
    double manualMovementSpeed = 0;
    bool manualControl = false;
    
    const units::turn_t targetTolerance = 1_tr;
    bool sensorBroken = false;
    bool encoderZeroed = false;

    units::turn_t getPosition();
    double computeSpeedForPreset();

    rev::spark::SparkMax leftSparkMax {CAN_LEFT_ELEVATOR, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax rightSparkMax {CAN_RIGHT_ELEVATOR, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkRelativeEncoder leftEncoder = leftSparkMax.GetEncoder();
    rev::spark::SparkRelativeEncoder rightEncoder = rightSparkMax.GetEncoder();

    frc::DigitalInput lowerLimitSwitch {DIO_ELEVATOR_BOTTOM_LIMITSWITCH};
    frc::DigitalInput upperLimitSwitch {DIO_ELEVATOR_TOP_LIMITSWITCH};

    frc::ProfiledPIDController<units::turn> PIDController {
      ELEVATOR_PREFERENCE.PID.Kp, ELEVATOR_PREFERENCE.PID.Ki, ELEVATOR_PREFERENCE.PID.Kd,
      frc::TrapezoidProfile<units::turn>::Constraints(ELEVATOR_PREFERENCE.PID.MaxVel, ELEVATOR_PREFERENCE.PID.MaxAccel)
    };
    // If Elevator sensor is broken, have code that can do its best without the sensor/stop the robot 
    friend class Controls;
};