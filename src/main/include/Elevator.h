#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DigitalInput.h>

class Elevator : public Component {
  public:
    void process();

    void doPersistentConfiguration();

    bool atMaxHeight();
    bool atMinHeight();
    bool getLowerLimitSwitch();
    bool getUpperLimitSwitch();

    enum ElevatorPreset { //need measurements for the height of these sections, RN we have guesstimates with no units. ~G
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

    void goToPreset(ElevatorPreset target);

  private:
    double getPosition();

    void runMotorsToPreset();

    double ElevatorPosition[ElevatorPreset::kMAX] {
        0,     // Stopped
        100.0, // Ground
        200.0, // Processor
        350.0, // Coral Station
        300.0, // L1
        400.0, // L2
        500.0, // L3
        600.0  // L4
    };
    ElevatorPreset targetPreset;

    //Not sure about motors for now. ~G
    //Added placeholders for device ID
    rev::spark::SparkMax leftSparkMax {1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax rightSparkMax {2, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkRelativeEncoder leftEncoder = leftSparkMax.GetEncoder();
    rev::spark::SparkRelativeEncoder rightEncoder = rightSparkMax.GetEncoder();

    frc::DigitalInput lowerLimitSwitch {DIO_ELEVATOR_BOTTOM_LIMITSWITCH};
    frc::DigitalInput upperLimitSwitch {DIO_ELEVATOR_TOP_LIMITSWITCH};

    rev::spark::SparkClosedLoopController leftPIDController {leftSparkMax.GetClosedLoopController()};
    rev::spark::SparkClosedLoopController rightPIDController {rightSparkMax.GetClosedLoopController()};
};