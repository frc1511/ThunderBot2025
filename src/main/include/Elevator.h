#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Elevator : public Component {
  public:
    void process();

    void doPersistentConfiguration();
    void sendFeedback();

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
        kMAX,
    };
    void goToPreset(Preset preset);
    // if manual this returns true
    bool atPreset();
    void manualMovement(double speed);
    void setSensorBroken(bool isBroken);
    
  private:
    units::turn_t Position[Preset::kMAX] {
        0_tr,     // Stopped (Does not move to 0 turns)
        20_tr, // Ground
        200_tr, // Processor
        350_tr, // Coral Station
        300_tr, // L1
        400_tr, // L2
        500_tr, // L3
        600_tr  // L4
    };

    Preset targetPreset = Elevator::Preset::kSTOP;
    double manualMovementSpeed = 0;
    bool manualControl = false;
    // IN TURNS
    const double targetTolerance = 1;
    bool sensorBroken = false;
    double getPosition();
    bool encoderZeroed = false;
    double computeSpeedForPreset();

    //Not sure about motors for now. ~G
    //Added placeholders for device ID
    rev::spark::SparkMax leftSparkMax {1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax rightSparkMax {2, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkRelativeEncoder leftEncoder = leftSparkMax.GetEncoder();
    rev::spark::SparkRelativeEncoder rightEncoder = rightSparkMax.GetEncoder();

    frc::DigitalInput lowerLimitSwitch {DIO_ELEVATOR_BOTTOM_LIMITSWITCH};
    frc::DigitalInput upperLimitSwitch {DIO_ELEVATOR_TOP_LIMITSWITCH};

    frc::ProfiledPIDController<units::turn> PIDController {
      ELEVATOR_PREFERENCE.PID.Kp, ELEVATOR_PREFERENCE.PID.Ki, ELEVATOR_PREFERENCE.PID.Kd,
      frc::TrapezoidProfile<units::turn>::Constraints(ELEVATOR_PREFERENCE.PID.MaxVel, ELEVATOR_PREFERENCE.PID.MaxAccel)
    };
    /* Elevator needs to be able to stop moving/change the way it moves to certain presets (like if we accidentaly meant to go to L1 but went to L3)
    Elevator needs to know if it has reached its preset or if it is at its set preset
    Elevator needs to be able to move without presets (manual movement)
    Elevator needs to be at/reach a set starting point when it starts/turns on
    If Elevator sensor is broken, have code that can do its best without the sensor/stop the robot */
};