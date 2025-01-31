#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/ClosedLoopConfig.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Gamepiece : public Component {
  public:
    Gamepiece();
    ~Gamepiece();

    void doPersistentConfiguration();

    void resetToMatchMode(MatchMode mode);

    void sendFeedback();

    void process();


    enum MotorModes {
        kNONE,
        kCORAL_INTAKE,
        kCORAL_SHOOT,
        kALGAE_INTAKE,
        kALGAE_SHOOT
    };

    void setMotorMode(Gamepiece::MotorModes mode);

  private:
    bool coralRetroreflectiveTripped();

    bool algaeRetroreflectiveTripped();

    void stopMotors();

    // Set the speed of the motor (-1 to 1)
    void runMotors(double speed);

    Gamepiece::MotorModes motorMode = MotorModes::kNONE;

    enum MotorSpeeds {
        kSTOPPED,
        kCORAL,
        kALGAE,
        kMAX
    };
    Gamepiece::MotorSpeeds motorSpeed = MotorSpeeds::kSTOPPED;

    double presetIntakeSpeeds [MotorSpeeds::kMAX] = {
        0.0,
        0.25,
        0.25
    };

    double presetShooterSpeeds [MotorSpeeds::kMAX] = {
        0.0,
        -0.25,
        -0.25
    };

    rev::spark::SparkMax leftSparkMax {CAN_LEFT_CALGAE, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMaxConfig leftSparkMaxConfig {};
    rev::spark::SparkClosedLoopController leftPidController = leftSparkMax.GetClosedLoopController();
    rev::spark::SparkMax rightSparkMax {5, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMaxConfig rightSparkMaxConfig {};
    rev::spark::SparkClosedLoopController rightPidController = rightSparkMax.GetClosedLoopController();

    frc::DigitalInput coralRetroreflective {DIO_CORAL_RETROREFLECTIVE};
    frc::DigitalInput algaeRetroreflective {DIO_ALGAE_RETROREFLECTIVE};

};