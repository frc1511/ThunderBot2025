#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/DigitalInput.h>

class Elevator : public Component {
  public:
    void process();

    void getDistanceTraveled();

    bool getAtMaxHeight();
    bool getAtMinHeight();

    enum ElevatorPositions { //needs measurements for the height of these sections
        kGROUND,
        kPROCESSOR,
        kCORAL_STATION,
        kL1,
        kL2,
        kL3,
        kL4,
        kMAX
    };

    void goToPosition(ElevatorPositions target);

    private:
        double getPosition();

        void setSpeed(double speed);

        double ElevatorPositionValues[ElevatorPositions::kMAX] {
            100.0, // Ground
            200.0, // Processor
            350.0, // Coral Station
            300.0, // L1
            400.0, // L2
            500.0, // L3
            600.0  // L4
        };
        enum ElevatorPositions targetPosition;

        enum ElevatorControlModes {
            kMANUAL, 
            kPRESET,
            kSTOP,
        };
        enum ElevatorControlModes controlMode = kMANUAL;

        double speed = 0.0;

        double minHeight = 0.0;
        double maxHeight = 0.0;

        double initialDistanceToTarget;
        double currentDistanceToTarget;
        double currentAbsoluteDistanceToTarget;

        //Not sure about motors for now. ~G
        //Added placeholders for device ID
        rev::spark::SparkMax leftSparkMax {1, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax rightSparkMax {2, rev::spark::SparkLowLevel::MotorType::kBrushless};

        rev::spark::SparkRelativeEncoder leftEncoder = leftSparkMax.GetEncoder();
        rev::spark::SparkRelativeEncoder rightEncoder = rightSparkMax.GetEncoder();
};